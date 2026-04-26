"""
Routers: detection (YOLO results) + balls + closest object.

Source: state.detection.ball_detection_raw (сырой dict от Pi/remote-GPU)
+ state.detection.{enabled, backend, fps, balls}.

Маппинг старых endpoints:
  GET  /api/detection             → /detection
  GET  /api/detection/closest     → /detection/closest
  GET  /api/balls                 → /balls           (отдельный sub-router)
  POST /api/detection/toggle      → /detection/toggle
  GET  /api/detection/status      → /detection/status

POST /detection/toggle публикует:
  - MQTT: detection/enable on|off (для standalone object_detector_node)
  - ROS2: /yolo/enable on|off (для laptop YOLO ноды) — если ROS2 доступен
"""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Query

from ..schemas.common import CommandAck
from ..schemas.detection import (
    BallInfo,
    BallsResponse,
    ClosestDetectionResponse,
    Detection,
    DetectionResponse,
    DetectionStatusResponse,
    DetectionToggleCommand,
)
from ._deps import MQTTDep, ROS2Dep, StateDep

router = APIRouter()


def _detection_from_dict(o: dict) -> Detection:
    """Парсинг одного объекта из raw payload Pi/remote-GPU.

    Поля могут варьироваться (cls/class, color/colour, bbox/x+y+w+h).
    """
    bbox = o.get('bbox') or [o.get('x', 0), o.get('y', 0),
                             o.get('x', 0) + o.get('w', 0),
                             o.get('y', 0) + o.get('h', 0)]
    if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        x = int(bbox[0])
        y = int(bbox[1])
        w = int(bbox[2] - bbox[0])
        h = int(bbox[3] - bbox[1])
    else:
        x, y, w, h = int(o.get('x', 0)), int(o.get('y', 0)), int(o.get('w', 0)), int(o.get('h', 0))
    return Detection(
        cls=o.get('class', o.get('cls', 'object')),
        colour=o.get('colour', o.get('color', 'unknown')),
        x=x, y=y, w=w, h=h,
        conf=float(o.get('conf', o.get('confidence', 0.0))),
        distance=float(o.get('distance', -1.0)),
        dist_method=o.get('dist_method', 'none'),
        world_x=o.get('world_x'),
        world_y=o.get('world_y'),
    )


def _objects_from_state(state) -> list[Detection]:
    with state.lock:
        raw = dict(state.detection.ball_detection_raw)
    objects = raw.get('objects', raw.get('balls', []))
    out: list[Detection] = []
    for o in objects:
        if isinstance(o, dict):
            try:
                out.append(_detection_from_dict(o))
            except Exception:
                continue
    return out


# ── /detection ────────────────────────────────────────────────────────
@router.get('', response_model=DetectionResponse, tags=['detection'])
async def get_detection(state: StateDep) -> DetectionResponse:
    """Все объекты последнего YOLO кадра."""
    objs = _objects_from_state(state)
    with state.lock:
        ts = state.detection.ball_detection_raw.get('ts', 0.0) if isinstance(
            state.detection.ball_detection_raw, dict) else 0.0
    return DetectionResponse(objects=objs, count=len(objs), ts=float(ts or 0.0))


@router.get('/closest', response_model=ClosestDetectionResponse, tags=['detection'])
async def get_closest(
    state: StateDep,
    color: Optional[str] = Query(default=None, description='Фильтр по цвету (red, blue, ...)'),
) -> ClosestDetectionResponse:
    """Ближайший объект, опционально с фильтром по цвету.

    Сортировка: по distance (если задана), иначе по площади bbox (больше=ближе).
    """
    objs = _objects_from_state(state)
    if color:
        cf = color.lower().strip()
        objs = [o for o in objs if o.colour.lower() == cf]
    if not objs:
        return ClosestDetectionResponse(detection=None)

    def _key(o: Detection) -> float:
        if o.distance > 0:
            return o.distance
        # Bigger bbox = closer; вернуть отрицательное чтобы сортировать "по убыванию площади"
        return -float(o.w * o.h)

    return ClosestDetectionResponse(detection=sorted(objs, key=_key)[0])


@router.get('/status', response_model=DetectionStatusResponse, tags=['detection'])
async def detection_status(state: StateDep) -> DetectionStatusResponse:
    with state.lock:
        return DetectionStatusResponse(
            enabled=state.detection.enabled,
            backend=state.detection.backend,
            fps=state.detection.fps,
        )


@router.post('/toggle', response_model=CommandAck, tags=['detection'])
async def toggle_detection(
    cmd: DetectionToggleCommand,
    state: StateDep,
    mqtt: MQTTDep,
    ros2: ROS2Dep,
) -> CommandAck:
    """Включить/выключить YOLO. Публикуем в MQTT и (если ROS2 доступен) в /yolo/enable."""
    with state.lock:
        state.detection.enabled = cmd.enabled
    mqtt.publish('detection/enable', 'on' if cmd.enabled else 'off', qos=1)
    if ros2 is not None:
        ros2.set_yolo_enabled(cmd.enabled)
    return CommandAck()


# ── /balls (отдельный sub-router) ──────────────────────────────────────
balls_router = APIRouter()


@balls_router.get('', response_model=BallsResponse, tags=['detection'])
async def get_balls(state: StateDep) -> BallsResponse:
    """Tracked balls (постоянный ID между кадрами).

    Симулятор/трекер заполняет state.detection.balls. На реальном роботе
    обычно пусто (трекинг не реализован) — возвращаем то, что есть.
    """
    with state.lock:
        balls = list(state.detection.balls)
    return BallsResponse(balls=[BallInfo(**b.model_dump()) for b in balls])
