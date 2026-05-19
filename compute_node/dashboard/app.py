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
import time
from typing import Optional

import socketio
import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

# Optional Sentry integration (#73). Only initialised when SENTRY_DSN env
# var is set — keeps a fresh dev clone free of any 3rd-party telemetry,
# but a single env var is enough to turn it on in production.
try:
    import sentry_sdk
    from sentry_sdk.integrations.fastapi import FastApiIntegration
    from sentry_sdk.integrations.logging import LoggingIntegration
    _HAS_SENTRY = True
except ImportError:
    _HAS_SENTRY = False

# slowapi is optional — without it, dashboard runs unchanged. With it,
# write-heavy POST/PUT/DELETE endpoints get a global rate limit so a runaway
# script or buggy client can't DoS the robot's command pipeline.
try:
    from slowapi import Limiter
    from slowapi.errors import RateLimitExceeded
    from slowapi.util import get_remote_address
    _HAS_SLOWAPI = True
except ImportError:
    Limiter = None  # type: ignore[assignment,misc]
    RateLimitExceeded = Exception  # type: ignore[assignment,misc]
    get_remote_address = None  # type: ignore[assignment]
    _HAS_SLOWAPI = False

from .mqtt_handlers import MQTTHandlers
from .routers import (
    actuators,
    camera,
    control,
    detection,
    fsm,
    maps,
    mps,
    robot,
    samcan,
    sensors,
    system,
)
from .state import DashboardState

log = logging.getLogger(__name__)

# Sunset для deprecated /api/* endpoints — фронт должен мигрировать до этой даты.
DEPRECATION_SUNSET = '2026-12-31'


async def _run_robot_live_state_tick(_state) -> None:
    """Один тик aggregator-loop'а. Вынесено на уровень модуля для
    юнит-тестируемости.

    Если нет подписчиков на /ws/robot/live_state — НЕ строим point и
    не зовём broadcast (избавляемся от лишнего lock + сериализации).
    """
    from .routers.robot import (
        build_live_state_point,
        robot_live_state_broker,
    )
    if not robot_live_state_broker.has_subscribers():
        return
    try:
        point = build_live_state_point(_state).model_dump()
    except Exception as exc:
        log.exception('robot live_state aggregator failed: %s', exc)
        return
    robot_live_state_broker.broadcast({'type': 'live_state', 'point': point})


def _maybe_init_sentry() -> None:
    """Wire up Sentry error reporting if SENTRY_DSN is set (#73).

    Default-disabled. Enable by exporting:
        SENTRY_DSN=https://...@sentry.io/123
        SENTRY_ENV=prod                  # optional, default 'dev'
        SENTRY_TRACES_SAMPLE_RATE=0.1    # optional perf sampling
    """
    if not _HAS_SENTRY:
        return
    dsn = os.environ.get('SENTRY_DSN', '').strip()
    if not dsn:
        return
    env = os.environ.get('SENTRY_ENV', 'dev')
    try:
        traces_sample_rate = float(os.environ.get('SENTRY_TRACES_SAMPLE_RATE', '0'))
    except ValueError:
        traces_sample_rate = 0.0
    try:
        sentry_sdk.init(
            dsn=dsn,
            environment=env,
            traces_sample_rate=traces_sample_rate,
            # Capture unhandled exceptions but suppress noisy INFO/DEBUG.
            integrations=[
                FastApiIntegration(),
                LoggingIntegration(level=logging.INFO, event_level=logging.ERROR),
            ],
            # Don't accidentally exfiltrate PII (joystick coordinates, MQTT
            # payloads, etc.) — Sentry's default sends some user-identifying
            # data; turn it off by default.
            send_default_pii=False,
        )
        log.info('Sentry initialised (env=%s, traces=%.2f)', env, traces_sample_rate)
    except Exception as exc:  # noqa: BLE001
        log.warning('Sentry init failed: %s', exc)


# Initialise once at import time (idempotent — sentry_sdk.init is safe to
# call again, but the env var is read once).
_maybe_init_sentry()


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

    # ── Opt-in Bearer auth ────────────────────────────────────────────
    # By default the dashboard runs without auth — Pi и laptop в одной
    # домашней сети, фронт ходит без токена. Когда роутер пробрасывает
    # порт наружу или dashboard смотрит в WAN, оператор задаёт
    # SAMURAI_DASHBOARD_TOKEN — и тогда любой запрос на /api/v1/*
    # требует `Authorization: Bearer <exact token>`.
    #
    # Сравнение через hmac.compare_digest — constant-time, без
    # timing-leak. exempt-список покрывает SPA-static, OpenAPI-doc,
    # SocketIO и H264 WS (для них auth — отдельный механизм или нет
    # клиентов которые умеют слать токен через handshake).
    _dashboard_token = os.environ.get('SAMURAI_DASHBOARD_TOKEN', '').strip()
    if _dashboard_token:
        import hmac

        _AUTH_EXEMPT_PREFIXES = (
            '/docs', '/redoc', '/openapi.json',
            '/assets/', '/static/',
            '/socket.io/', '/ws/h264',
        )
        _AUTH_EXEMPT_PATHS = {'/', '/dashboard', '/admin', '/3d'}
        _PROTECTED_PREFIX = '/api/'

        def _extract_bearer(header: str) -> Optional[str]:
            # Принимаем ровно `Bearer <token>` с одним пробелом —
            # `Bearer  token` (двойной пробел) отвергаем (regression
            # против MOIS finding `auth_bypass`).
            if not header.startswith('Bearer '):
                return None
            token = header[len('Bearer '):]
            if not token or token.startswith(' ') or token.endswith((' ', '\n', '\r', '\t')):
                return None
            return token

        @app.middleware('http')
        async def bearer_auth(request: Request, call_next):
            # CORS preflight никогда не несёт Authorization — пропускаем,
            # иначе ломаем cross-origin клиентов (mobile WebView, dev :3000).
            if request.method == 'OPTIONS':
                return await call_next(request)
            path = request.url.path
            if path in _AUTH_EXEMPT_PATHS or \
                    any(path.startswith(p) for p in _AUTH_EXEMPT_PREFIXES) or \
                    not path.startswith(_PROTECTED_PREFIX):
                return await call_next(request)

            authz = request.headers.get('authorization', '')
            received = _extract_bearer(authz)
            if received is None or not hmac.compare_digest(received, _dashboard_token):
                return JSONResponse(
                    status_code=401,
                    content={'error': 'unauthorized',
                             'detail': 'valid Bearer token required'},
                    headers={'WWW-Authenticate': 'Bearer realm="samurai"'},
                )
            return await call_next(request)

        log.info('Bearer auth active on %s* (token len=%d)',
                 _PROTECTED_PREFIX, len(_dashboard_token))
    else:
        log.info('Bearer auth disabled (SAMURAI_DASHBOARD_TOKEN not set)')

    # ── Rate limiting (write verbs only) ──────────────────────────────
    # Defence against runaway clients (buggy script, stuck retry loop) and
    # a soft DoS protection on the command pipeline. Reads (GET) are NOT
    # limited — the dashboard polls them aggressively by design.
    #
    # Implementation uses slowapi's MovingWindowRateLimiter directly rather
    # than the framework's decorator/exception path because we want a
    # global rule across all write endpoints, not per-route limits.
    #
    # Override via env vars:
    #   SAMURAI_RATELIMIT_WRITES — e.g. "30/minute" (default)
    #   SAMURAI_RATELIMIT=off    — disable entirely
    if _HAS_SLOWAPI and os.environ.get('SAMURAI_RATELIMIT', 'on').lower() != 'off':
        from limits import parse as _parse_limit
        from limits.storage import MemoryStorage
        from limits.strategies import MovingWindowRateLimiter

        write_limit_str = os.environ.get('SAMURAI_RATELIMIT_WRITES', '30/minute')
        try:
            _limit_item = _parse_limit(write_limit_str)
        except Exception as exc:
            log.warning('Invalid SAMURAI_RATELIMIT_WRITES=%r (%s) — using 30/minute',
                        write_limit_str, exc)
            _limit_item = _parse_limit('30/minute')
            write_limit_str = '30/minute'

        _rate_storage = MemoryStorage()
        _rate_strategy = MovingWindowRateLimiter(_rate_storage)

        @app.middleware('http')
        async def _rate_limit_writes(request: Request, call_next):
            if request.method in ('POST', 'PUT', 'PATCH', 'DELETE') and \
                    request.url.path.startswith('/api/'):
                key = get_remote_address(request) or 'anonymous'
                # `hit` returns True if the request fits inside the window,
                # False if the limit has been exceeded.
                if not _rate_strategy.hit(_limit_item, key):
                    return JSONResponse(
                        status_code=429,
                        content={'error': 'rate limit exceeded',
                                 'detail': f'limit={write_limit_str}'},
                        headers={'Retry-After': '5'},
                    )
            return await call_next(request)

        log.info('Rate limit active: %s on POST/PUT/PATCH/DELETE /api/*',
                 write_limit_str)
    elif not _HAS_SLOWAPI:
        log.warning('slowapi not installed — rate limiting disabled. '
                    'Run: pip install slowapi')

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

    # ── Idempotency-Key middleware (#59) ──────────────────────────────
    # Clients (frontend api.ts retry wrapper, mobile app) may retry POSTs
    # after a network blip when the original request actually reached the
    # server. Without idempotency the second request executes the command
    # twice (e.g. emergency stop fired thrice, calibration profile saved
    # twice). With an Idempotency-Key header, the cached response from the
    # first request is returned for any retry within a 1-minute window.
    #
    # In-memory cache — single dashboard process, no horizontal scaling
    # currently. Bound: 256 entries (LRU-evicted), 60s TTL.
    _IDEM_TTL_S = 60.0
    _IDEM_MAX_ENTRIES = 256
    _idem_cache: dict[str, tuple[float, int, bytes, str]] = {}
    _idem_lock = asyncio.Lock()

    def _idem_evict_expired(now: float) -> None:
        # Drop expired entries plus, if still over budget, the oldest.
        expired = [k for k, (ts, *_) in _idem_cache.items() if now - ts > _IDEM_TTL_S]
        for k in expired:
            _idem_cache.pop(k, None)
        if len(_idem_cache) > _IDEM_MAX_ENTRIES:
            # Sorted by insertion ts ascending → oldest first.
            for k, _ in sorted(_idem_cache.items(), key=lambda kv: kv[1][0])[
                    : len(_idem_cache) - _IDEM_MAX_ENTRIES]:
                _idem_cache.pop(k, None)

    @app.middleware('http')
    async def idempotency_key(request: Request, call_next):
        # Only mutating verbs care; reads are inherently idempotent.
        if request.method not in ('POST', 'PUT', 'PATCH', 'DELETE'):
            return await call_next(request)
        key = request.headers.get('idempotency-key') or \
              request.headers.get('Idempotency-Key')
        if not key:
            return await call_next(request)

        # Scope by method + path so an Idempotency-Key reused across distinct
        # endpoints doesn't accidentally short-circuit a different operation.
        cache_key = f'{request.method} {request.url.path} {key}'
        now = time.monotonic()

        async with _idem_lock:
            cached = _idem_cache.get(cache_key)
            if cached is not None:
                ts, status, body, ctype = cached
                if now - ts <= _IDEM_TTL_S:
                    return Response(
                        content=body,
                        status_code=status,
                        media_type=ctype,
                        headers={'Idempotency-Replay': 'true'},
                    )
                _idem_cache.pop(cache_key, None)

        response = await call_next(request)

        # Only cache successful responses — retrying a 500 should be allowed
        # to actually re-attempt the operation.
        if 200 <= response.status_code < 300:
            body_bytes = b''
            async for chunk in response.body_iterator:
                body_bytes += chunk
            ctype = response.headers.get('content-type', 'application/json')
            async with _idem_lock:
                _idem_evict_expired(now)
                _idem_cache[cache_key] = (now, response.status_code,
                                          body_bytes, ctype)
            return Response(content=body_bytes,
                            status_code=response.status_code,
                            media_type=ctype,
                            headers={k: v for k, v in response.headers.items()
                                     if k.lower() not in ('content-length',)})
        return response

    # ── Security headers ──────────────────────────────────────────────
    # Defence-in-depth для SPA: даже если рендерим untrusted текст
    # (имя зоны, имя пресета, log-сообщение — всё пишется операторами,
    # но прилетает через REST), хотим ограничить blast radius если
    # XSS прорвётся.
    #
    # Регистрируется ПОСЛЕДНИМ — Starlette `add_middleware` делает
    # `insert(0)`, поэтому последний зарегистрированный становится
    # самым внешним в стеке. Это критично: 401 от bearer_auth и
    # 429 от rate_limit поднимаются через security_headers и тоже
    # получают CSP/X-Frame-Options/etc.
    #
    # Override via env:
    #   SAMURAI_SECURITY_HEADERS=off — отключить middleware
    #   SAMURAI_CSP=<policy>        — заменить CSP целиком
    #   SAMURAI_HSTS=on             — добавлять Strict-Transport-Security
    #                                  (включай только когда фронтит HTTPS-прокси)
    if os.environ.get('SAMURAI_SECURITY_HEADERS', 'on').lower() != 'off':
        # Default CSP подобран под Vite-сборку фронта:
        # - 'self' для скриптов/стилей (хешированные assets под /assets/)
        # - 'unsafe-inline' для стилей: Vite иногда инжектит inline <style>
        # - data:/blob: для картинок (camera frame, map.png)
        # - ws:/wss:/http:/https: для connect-src (Socket.IO + WebSocket H264)
        # - frame-ancestors 'none' блокирует встраивание в <iframe>
        # 'wasm-unsafe-eval' нужен для useGLTF: Three.js Draco/Meshopt-декодер
        # компилируется как WebAssembly, и без этого CSP-токена браузер блокирует
        # WebAssembly.instantiate → useGLTF падает → WebGLRenderer Context Lost.
        # Google Fonts (Inter, JetBrains Mono) подключены в compute_node/static/
        # index.html, поэтому fonts.googleapis.com (stylesheet) и
        # fonts.gstatic.com (шрифты) добавлены в style-src и font-src.
        _DEFAULT_CSP = (
            "default-src 'self'; "
            "script-src 'self' 'wasm-unsafe-eval'; "
            "style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; "
            "img-src 'self' data: blob:; "
            "font-src 'self' data: https://fonts.gstatic.com; "
            # blob: нужен для GLTFLoader — Three.js извлекает текстуры из GLB
            # как Blob и fetch'ит их через blob:-URL.
            "connect-src 'self' ws: wss: http: https: blob:; "
            "object-src 'none'; "
            "base-uri 'self'; "
            "frame-ancestors 'none'"
        )
        _csp = os.environ.get('SAMURAI_CSP', _DEFAULT_CSP).strip()
        _hsts_on = os.environ.get('SAMURAI_HSTS', 'off').lower() == 'on'

        @app.middleware('http')
        async def security_headers(request: Request, call_next):
            response = await call_next(request)
            # Не перетираем заголовки если handler уже выставил свой CSP
            # (полезно для редких случаев — например, если openapi-ui
            # требует более слабый policy).
            response.headers.setdefault('X-Content-Type-Options', 'nosniff')
            response.headers.setdefault('X-Frame-Options', 'DENY')
            response.headers.setdefault('Referrer-Policy',
                                        'strict-origin-when-cross-origin')
            if _csp:
                response.headers.setdefault('Content-Security-Policy', _csp)
            if _hsts_on:
                response.headers.setdefault(
                    'Strict-Transport-Security',
                    'max-age=31536000; includeSubDomains',
                )
            return response

        log.info('Security headers active (CSP=%d chars, HSTS=%s)',
                 len(_csp), _hsts_on)

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
    app.include_router(control.path_planner_router, prefix='/api/v1/path_planner')
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
    app.include_router(system.shutdown_router, prefix='/api/v1/system/shutdown')
    # camera (HTTP) + samcan (proxy)
    app.include_router(camera.router, prefix='/api/v1/camera')
    app.include_router(samcan.router, prefix='/api/v1/samcan')
    # МПС — Модель Пространства Состояний (учебный модуль курсовой Козлова, feat/mps)
    app.include_router(mps.router, prefix='/api/v1/mps')
    # WebSocket /ws/mps/telemetry — без /api префикса, не трогается middleware-ом.
    app.include_router(mps.ws_router)
    # /ws/robot/live_state — постоянный канал состояния робота для DashboardPage.
    app.include_router(robot.ws_router)
    # Подключаем MQTT broadcaster → WS broker. mqtt_handlers зовёт _broadcast_mps()
    # из своего thread'а, broker делает call_soon_threadsafe в asyncio loop.
    if mqtt is not None:
        mqtt.set_mps_ws_broadcaster(mps.mps_broker.broadcast)
        mqtt.set_mps_live_state_broadcaster(mps.mps_live_state_broker.broadcast)

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

    for spa_path in ('/dashboard', '/admin', '/3d', '/mps'):
        app.add_api_route(spa_path, _serve_spa, methods=['GET'])

    # ── Static mounts ─────────────────────────────────────────────────
    if static_dir:
        assets_dir = os.path.join(static_dir, 'assets')
        if os.path.isdir(assets_dir):
            app.mount('/assets', StaticFiles(directory=assets_dir), name='assets')
        # GLB/GLTF и прочие 3D-public-ассеты лежат в compute_node/static/models/
        # (Vite копирует frontend/public/models/* в outDir при build). Без явного
        # mount запросы /models/* падали в SPA-fallback и отдавали index.html, что
        # ломало useGLTF (он ждёт binary, получает HTML → Context Lost).
        models_dir = os.path.join(static_dir, 'models')
        if os.path.isdir(models_dir):
            app.mount('/models', StaticFiles(directory=models_dir), name='models')
        app.mount('/static', StaticFiles(directory=static_dir), name='static')

    # ── /ws/robot/live_state aggregator ──────────────────────────────
    # Регистрируем ДО early-return для enable_socketio=False, чтобы
    # run_standalone и любые embedding-сценарии тоже получали поток.
    async def _robot_live_state_loop():
        """10 Hz aggregator → /ws/robot/live_state.

        Отдельный loop от _push_loop потому что:
          - всегда тикает (нет dirty-skip);
          - изоляция: если SocketIO-broadcast тормозит, WS-канал не страдает.
        """
        while True:
            await asyncio.sleep(0.1)               # 10 Hz
            await _run_robot_live_state_tick(state)

    @app.on_event('startup')
    async def _start_robot_live_state_loop():
        from .routers.robot import robot_live_state_broker
        robot_live_state_broker.attach_loop(asyncio.get_running_loop())
        asyncio.create_task(_robot_live_state_loop())

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
        last_payload = None
        idle_keepalive = 0     # tick counter; force a re-emit every ~5s even if idle
        IDLE_KEEPALIVE_TICKS = 50   # 5s @ 10 Hz — keeps reconnecting clients fresh
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            # Fast-path skip when nothing has changed since the last tick.
            # The dirty flag is set by handlers/routers via state.mark_dirty()
            # whenever they mutate state; if it's clear we can avoid the
            # ~100µs snapshot+JSON cost AND the per-client emit() fan-out.
            # Periodic re-emit (idle_keepalive) preserves the previous
            # behaviour of pushing periodic frames to clients that may
            # reconnect mid-idle.
            dirty = state.consume_dirty()
            if not dirty:
                idle_keepalive += 1
                if idle_keepalive < IDLE_KEEPALIVE_TICKS or last_payload is None:
                    continue
                idle_keepalive = 0   # fall through to re-emit cached payload
            else:
                idle_keepalive = 0

            try:
                if dirty or last_payload is None:
                    payload = state.legacy_socketio_state()
                    payload_json = json.dumps(payload, separators=(',', ':'), default=str)
                else:
                    payload, payload_json = last_payload, last_json
            except Exception as exc:
                log.exception('SocketIO push: state serialization failed: %s', exc)
                continue
            if payload_json == last_json and not dirty:
                continue
            last_json = payload_json
            last_payload = payload
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
