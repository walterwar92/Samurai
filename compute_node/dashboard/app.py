"""
FastAPI factory + Socket.IO + state push loop + static SPA mount.

Использование (см. __main__.py):

    state = DashboardState()
    mqtt = MQTTHandlers(broker, port, robot_id, state, ...); mqtt.start()
    ros2 = ROS2Subscribers(node, state)  # опционально, если rclpy доступен
    app = create_app(state, mqtt, ros2)
    uvicorn.run(app, host='0.0.0.0', port=5000)

Маршруты:
  /api/v1/...     — все новые APIRouter'ы (см. routers/)
  /api/...        — DEPRECATED alias (rewrite + Deprecation header)
                    sunset 2026-12-31, фронт переезжает в C12
  /docs, /redoc   — OpenAPI документация (FastAPI build-in)
  /openapi.json   — схема для openapi-typescript-codegen (C12)
  /ws/h264        — H.264 TCP-прокси (camera router)
  /socket.io/*    — Socket.IO для legacy фронта (state push)
  /assets/*       — built React assets (vite output)
  /static/*       — прочая статика
  /, /dashboard, /admin, /3d — SPA fallback (отдают index.html)
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
from typing import Optional

import socketio
import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .mqtt_handlers import MQTTHandlers
from .routers import (
    actuators,
    camera,
    control,
    detection,
    fsm,
    maps,
    robot,
    samcan,
    sensors,
    system,
)
from .state import DashboardState

log = logging.getLogger(__name__)

# Sunset для deprecated /api/* endpoints — фронт должен мигрировать до этой даты.
DEPRECATION_SUNSET = '2026-12-31'


def _find_static_dir() -> Optional[str]:
    """Найти compute_node/static/ — относительно текущего файла или корня репо."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', 'static'),  # compute_node/static
        os.path.join(here, '..', '..', 'compute_node', 'static'),
    ]
    for c in candidates:
        c = os.path.abspath(c)
        if os.path.isdir(c):
            return c
    return None


def create_app(
    state: DashboardState,
    mqtt: Optional[MQTTHandlers] = None,
    ros2: Optional[object] = None,  # ROS2Subscribers — Optional чтобы не требовать rclpy
    enable_socketio: bool = True,
) -> object:
    """Собрать FastAPI приложение.

    Возвращает либо FastAPI (если enable_socketio=False), либо
    socketio.ASGIApp обёртку (если включен SocketIO для legacy фронта).
    """
    app = FastAPI(
        title='Samurai Dashboard',
        version='2.0',
        description='REST + WebSocket API для управления роботом.',
    )

    # Кладём синглтоны в app.state — routers подцепляют через DI (_deps.py).
    app.state.dashboard_state = state
    app.state.mqtt_handlers = mqtt
    app.state.ros2_subscribers = ros2

    # CORS — открываем (фронт может быть на другом порту в dev-режиме).
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_methods=['*'],
        allow_headers=['*'],
    )

    # ── Deprecated /api/* → /api/v1/* alias middleware ────────────────
    # Старые URL переписываются на v1, ответ помечается Deprecation header.
    @app.middleware('http')
    async def deprecated_v0_alias(request: Request, call_next):
        path = request.url.path
        if path.startswith('/api/') and not path.startswith('/api/v1/'):
            new_path = '/api/v1' + path[len('/api'):]
            request.scope['path'] = new_path
            response = await call_next(request)
            response.headers['Deprecation'] = f'true; sunset="{DEPRECATION_SUNSET}"'
            response.headers['Link'] = f'<{new_path}>; rel="successor-version"'
            return response
        return await call_next(request)

    # ── /api/v1/ routers ──────────────────────────────────────────────
    # robot
    app.include_router(robot.router, prefix='/api/v1/robot')
    app.include_router(robot.speed_router, prefix='/api/v1/speed_profile')
    app.include_router(robot.emergency_router, prefix='/api/v1/emergency_stop')
    # sensors
    app.include_router(sensors.router, prefix='/api/v1/sensors')
    app.include_router(sensors.battery_router, prefix='/api/v1/battery')
    app.include_router(sensors.temperature_router, prefix='/api/v1/temperature')
    # actuators
    app.include_router(actuators.router, prefix='/api/v1/actuators')
    app.include_router(actuators.led_router, prefix='/api/v1/led')
    # detection + fsm
    app.include_router(detection.router, prefix='/api/v1/detection')
    app.include_router(detection.balls_router, prefix='/api/v1/balls')
    app.include_router(fsm.router, prefix='/api/v1/fsm')
    # maps
    app.include_router(maps.router, prefix='/api/v1/map')
    app.include_router(maps.slam_map_router, prefix='/api/v1/slam_map')
    app.include_router(maps.zones_router, prefix='/api/v1/zones')
    app.include_router(maps.planned_path_router, prefix='/api/v1/planned_path')
    # control
    app.include_router(control.patrol_router, prefix='/api/v1/patrol')
    app.include_router(control.follow_me_router, prefix='/api/v1/follow_me')
    app.include_router(control.path_recorder_router, prefix='/api/v1/path_recorder')
    app.include_router(control.precision_router, prefix='/api/v1/precision_drive')
    app.include_router(control.calibration_router, prefix='/api/v1/calibration')
    app.include_router(control.mission_router, prefix='/api/v1/mission')
    app.include_router(control.explorer_router, prefix='/api/v1/explorer')
    app.include_router(control.tts_router, prefix='/api/v1/tts')
    app.include_router(control.obstacle_router, prefix='/api/v1/obstacle_avoidance')
    app.include_router(control.collision_guard_router, prefix='/api/v1/collision_guard')
    # system
    app.include_router(system.status_router, prefix='/api/v1/status')
    app.include_router(system.log_router, prefix='/api/v1/log')
    app.include_router(system.mqtt_status_router, prefix='/api/v1/mqtt/status')
    app.include_router(system.multi_robot_router, prefix='/api/v1/multi_robot')
    app.include_router(system.hardware_router, prefix='/api/v1/hardware')
    # camera (HTTP) + samcan (proxy)
    app.include_router(camera.router, prefix='/api/v1/camera')
    app.include_router(samcan.router, prefix='/api/v1/samcan')

    # WebSocket /ws/h264 (без /api префикса — middleware его не трогает)
    app.include_router(camera.ws_router)

    # ── SPA fallback ──────────────────────────────────────────────────
    static_dir = _find_static_dir()
    index_html = os.path.join(static_dir, 'index.html') if static_dir else None

    def _serve_spa() -> object:
        """index.html ссылается на хешированные assets — не кешируем HTML."""
        if index_html and os.path.isfile(index_html):
            return FileResponse(
                index_html,
                headers={'Cache-Control': 'no-cache, no-store, must-revalidate'},
            )
        return JSONResponse(
            {'error': 'Frontend not built — run npm run build in compute_node/frontend/'},
            status_code=404,
        )

    @app.get('/')
    async def serve_root():
        return _serve_spa()

    for spa_path in ('/dashboard', '/admin', '/3d'):
        app.add_api_route(spa_path, _serve_spa, methods=['GET'])

    # ── Static mounts ─────────────────────────────────────────────────
    if static_dir:
        assets_dir = os.path.join(static_dir, 'assets')
        if os.path.isdir(assets_dir):
            app.mount('/assets', StaticFiles(directory=assets_dir), name='assets')
        app.mount('/static', StaticFiles(directory=static_dir), name='static')

    # ── Socket.IO + state push loop (legacy фронт) ────────────────────
    if not enable_socketio:
        return app

    sio = socketio.AsyncServer(
        async_mode='asgi',
        cors_allowed_origins='*',
        logger=False,
        engineio_logger=False,
        ping_interval=10,
        ping_timeout=5,
    )

    @sio.event
    async def connect(sid, environ):
        pass

    @sio.event
    async def disconnect(sid):
        pass

    @sio.event
    async def send_command(sid, data):
        text = data.get('text', '') if isinstance(data, dict) else str(data)
        if mqtt is not None and text:
            mqtt.publish('voice_command', text, qos=1)

    @sio.event
    async def reset_sim(sid, data):
        # No-op в robot mode — раньше сбрасывал симулятор.
        pass

    async def _push_loop():
        last_json = ''
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            try:
                payload = state.legacy_socketio_state()
                payload_json = json.dumps(payload, separators=(',', ':'), default=str)
            except Exception as exc:
                log.exception('SocketIO push: state serialization failed: %s', exc)
                continue
            if payload_json == last_json:
                continue
            last_json = payload_json
            await sio.emit('state_update', payload)

    @app.on_event('startup')
    async def _start_push_loop():
        asyncio.create_task(_push_loop())

    return socketio.ASGIApp(sio, other_asgi_app=app)


# ── Standalone runner — для быстрого smoke-теста ──────────────────────
def run_standalone(host: str = '0.0.0.0', port: int = 5000):
    """Запуск без MQTT/ROS2 — endpoints отдадут пустой state, но всё работает."""
    state = DashboardState()
    app = create_app(state, mqtt=None, ros2=None, enable_socketio=False)
    uvicorn.run(app, host=host, port=port, log_level='info')
