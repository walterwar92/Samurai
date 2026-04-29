"""
Routers: SLAM map info/PNG, forbidden zones, save/load, planned path.

Маппинг старых endpoints:
  GET  /api/map/info       → /map/info
  GET  /api/map/list       → /map/list      (читает ~/maps/*.yaml)
  GET  /api/map/image      → /map/image     (PNG бинарный response)
  GET  /api/slam_map       → /slam_map      (отдельный sub-router)
  GET  /api/zones          → /zones
  POST /api/zones          → /zones
  DELETE /api/zones/{id}   → /zones/{id}
  POST /api/zones/clear    → /zones/clear
  POST /api/map/save       → /map/save      (нужен ROS2)
  POST /api/map/load       → /map/load      (нужен ROS2)
  GET  /api/planned_path   → /planned_path  (отдельный sub-router)
"""
from __future__ import annotations

import logging
import os
from typing import Optional

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException
from fastapi.responses import Response

from ..schemas.common import CommandAck
from ..schemas.maps import (
    ForbiddenZone,
    MapInfoResponse,
    MapListResponse,
    MapLoadCommand,
    MapSaveCommand,
    PlannedPath,
    PlannedPathResponse,
    SlamMapResponse,
    ZoneCreateCommand,
    ZoneCreatedResponse,
    ZonesResponse,
)
from ._deps import MQTTDep, ROS2Dep, StateDep

log = logging.getLogger(__name__)

# Лимит одновременных зон. Подкрепляется на стороне Pi (path planner).
ZONE_LIMIT = 50

router = APIRouter()


# ── /map/* ─────────────────────────────────────────────────────────────
@router.get('/info', response_model=MapInfoResponse, tags=['maps'])
async def get_map_info(state: StateDep) -> MapInfoResponse:
    with state.lock:
        info = state.map.info
        # Fallback: Pi-side SLAM info
        if info is None and state.map.slam is not None:
            info = state.map.slam.info
    if info is None:
        raise HTTPException(503, 'Map not available yet')
    return MapInfoResponse(**info.model_dump())


@router.get('/list', response_model=MapListResponse, tags=['maps'])
async def list_maps() -> MapListResponse:
    """Список сохранённых SLAM карт из ~/maps/*.yaml."""
    maps_dir = os.path.expanduser('~/maps')
    if not os.path.isdir(maps_dir):
        return MapListResponse(maps=[])
    items = sorted(
        f[:-len('.yaml')]
        for f in os.listdir(maps_dir)
        if f.endswith('.yaml')
    )
    return MapListResponse(maps=items)


def _render_slam_png(slam) -> Optional[bytes]:
    """Сгенерировать PNG из Pi-side SlamMapData (fallback когда ROS2 SLAM молчит)."""
    if slam is None or slam.info is None:
        return None
    info = slam.info
    w, h = info.width, info.height
    res = info.resolution
    ox, oy = info.origin_x, info.origin_y

    img = np.full((h, w, 3), 128, dtype=np.uint8)

    for obs in slam.obstacles:
        ci = int((obs[0] - ox) / res)
        cj = int((obs[1] - oy) / res)
        if 0 <= ci < w and 0 <= cj < h:
            img[cj, ci] = [30, 30, 30]

    for pt in slam.trail:
        ci = int((pt[0] - ox) / res)
        cj = int((pt[1] - oy) / res)
        if 0 <= ci < w and 0 <= cj < h:
            img[cj, ci] = [255, 180, 50]  # blue-ish (BGR)

    rx = slam.robot.get('x', 0.0) if slam.robot else 0.0
    ry = slam.robot.get('y', 0.0) if slam.robot else 0.0
    rci = int((rx - ox) / res)
    rcj = int((ry - oy) / res)
    for di in range(-2, 3):
        for dj in range(-2, 3):
            ni, nj = rci + di, rcj + dj
            if 0 <= ni < w and 0 <= nj < h:
                img[nj, ni] = [0, 0, 255]  # red dot (BGR)

    img = cv2.flip(img, 0)
    ok, png = cv2.imencode('.png', img)
    return png.tobytes() if ok else None


@router.get('/image', tags=['maps'])
async def get_map_image(state: StateDep) -> Response:
    """PNG карты. Приоритет: state.map.png (ROS2 SLAM Toolbox);
    fallback: рендер из Pi-side SLAM (slam_map_node)."""
    with state.lock:
        png = state.map.png
        slam = state.map.slam
    if not png:
        png = _render_slam_png(slam)
    if not png:
        raise HTTPException(503, 'Map not available yet')
    return Response(content=png, media_type='image/png')


@router.post('/save', response_model=CommandAck, tags=['maps'])
async def save_map(cmd: MapSaveCommand, ros2: ROS2Dep) -> CommandAck:
    """Сохранить текущую SLAM карту (ROS2 /map_manager/save). Требует ROS2 stack."""
    if ros2 is None:
        raise HTTPException(503, 'ROS2 not available')
    ros2.save_map(cmd.name)
    return CommandAck()


@router.post('/load', response_model=CommandAck, tags=['maps'])
async def load_map(cmd: MapLoadCommand, ros2: ROS2Dep) -> CommandAck:
    """Загрузить сохранённую карту в SLAM (ROS2 /map_manager/load)."""
    if ros2 is None:
        raise HTTPException(503, 'ROS2 not available')
    ros2.load_map(cmd.name)
    return CommandAck()


# ── /slam_map ──────────────────────────────────────────────────────────
slam_map_router = APIRouter()


@slam_map_router.get('', response_model=SlamMapResponse, tags=['maps'])
async def get_slam_map(state: StateDep) -> SlamMapResponse:
    """Pi-side ultrasonic SLAM data (obstacles, trail, robot, objects, info)."""
    with state.lock:
        slam = state.map.slam
    if slam is None:
        raise HTTPException(503, 'SLAM map not available')
    return SlamMapResponse(**slam.model_dump(by_alias=True))


# ── /zones ─────────────────────────────────────────────────────────────
zones_router = APIRouter()


@zones_router.get('', response_model=ZonesResponse, tags=['maps'])
async def get_zones(state: StateDep) -> ZonesResponse:
    with state.lock:
        zones = list(state.map.zones)
    return ZonesResponse(zones=zones)


@zones_router.post('', response_model=ZoneCreatedResponse, tags=['maps'])
async def create_zone(
    cmd: ZoneCreateCommand,
    state: StateDep,
    mqtt: MQTTDep,
) -> ZoneCreatedResponse:
    """Создать запретную зону (нормализуется чтобы x1<x2, y1<y2)."""
    with state.lock:
        if len(state.map.zones) >= ZONE_LIMIT:
            raise HTTPException(400, f'zone limit reached ({ZONE_LIMIT})')
    zone = state.add_zone(
        x1=min(cmd.x1, cmd.x2), y1=min(cmd.y1, cmd.y2),
        x2=max(cmd.x1, cmd.x2), y2=max(cmd.y1, cmd.y2),
    )
    # Push весь актуальный список на Pi (path planner перечитает)
    with state.lock:
        zones_payload = [z.model_dump() for z in state.map.zones]
    mqtt.publish('zones/update', zones_payload, qos=1)
    return ZoneCreatedResponse(zone=zone)


@zones_router.delete('/{zone_id}', response_model=CommandAck, tags=['maps'])
async def delete_zone(zone_id: int, state: StateDep, mqtt: MQTTDep) -> CommandAck:
    removed = state.remove_zone(zone_id)
    if not removed:
        raise HTTPException(404, f'Zone {zone_id} not found')
    with state.lock:
        zones_payload = [z.model_dump() for z in state.map.zones]
    mqtt.publish('zones/update', zones_payload, qos=1)
    return CommandAck()


@zones_router.post('/clear', response_model=CommandAck, tags=['maps'])
async def clear_zones(state: StateDep, mqtt: MQTTDep) -> CommandAck:
    state.clear_zones()
    mqtt.publish('zones/update', '[]', qos=1)
    return CommandAck()


# ── /planned_path ──────────────────────────────────────────────────────
planned_path_router = APIRouter()


@planned_path_router.get('', response_model=PlannedPathResponse, tags=['maps'])
async def get_planned_path(state: StateDep) -> PlannedPathResponse:
    """Текущий планируемый путь (от path_planner или path_recorder).

    На текущей реализации path_recorder_path = [[x, y], ...] из Pi.
    """
    with state.lock:
        wps = list(state.control.path_recorder_path) or []
    waypoints: list[list[float]] = []
    for w in wps:
        if isinstance(w, (list, tuple)) and len(w) >= 2:
            waypoints.append([float(w[0]), float(w[1])])
        elif isinstance(w, dict) and 'x' in w and 'y' in w:
            waypoints.append([float(w['x']), float(w['y'])])
    return PlannedPathResponse(
        **PlannedPath(waypoints=waypoints).model_dump()
    )
