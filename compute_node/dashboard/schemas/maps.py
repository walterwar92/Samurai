"""
SLAM map, forbidden zones, planned paths.
"""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Domain models ──────────────────────────────────────────────────────────
class MapInfo(BaseModel):
    """OccupancyGrid метаданные."""
    width: int
    height: int
    resolution: float = Field(description='Метров на ячейку (default 0.05)')
    origin_x: float = 0.0
    origin_y: float = 0.0
    size_m: float = 10.0


class ForbiddenZone(BaseModel):
    """Запретная зона (рисуется на dashboard, учитывается path planner'ом)."""
    id: int
    x1: float
    y1: float
    x2: float
    y2: float


class SlamObject(BaseModel):
    """Объект из реестра slam_map_node."""
    id: str
    cls: str = Field(default='unknown', alias='class')
    colour: str = 'unknown'
    x: float
    y: float
    conf: float = 0.0
    dist: float = -1.0
    count: int = 1
    ts: float = 0.0

    model_config = {'populate_by_name': True}


class SlamMapData(BaseModel):
    """Карта от slam_map_node — публикуется ~0.5 Hz через MQTT."""
    obstacles: list[list[float]] = Field(
        default_factory=list,
        description='[[wx, wy], ...] координаты occupied ячеек'
    )
    trail: list[list[float]] = Field(
        default_factory=list,
        description='[[x, y], ...] последние ~500 точек одометрии'
    )
    robot: dict = Field(default_factory=dict, description='{x, y, theta}')
    detected_objects: list[SlamObject] = Field(default_factory=list)
    info: Optional[MapInfo] = None
    stats: dict = Field(
        default_factory=dict,
        description='{occupied, free, unknown, detected_objects}'
    )
    ts: float = 0.0


class PlannedPath(BaseModel):
    """Точки планируемого пути (от path_planner или path_recorder)."""
    waypoints: list[list[float]] = Field(
        default_factory=list,
        description='[[x, y], ...] в мировых координатах'
    )
    goal: Optional[list[float]] = None
    success: bool = True


# ── Request models ─────────────────────────────────────────────────────────
class ZoneCreateCommand(BaseModel):
    """POST /api/zones."""
    x1: float
    y1: float
    x2: float
    y2: float


class MapSaveCommand(BaseModel):
    """POST /api/map/save."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$',
                      description='Безопасное имя без / и спецсимволов')


class MapLoadCommand(BaseModel):
    """POST /api/map/load."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')


# ── Response models ────────────────────────────────────────────────────────
class MapInfoResponse(OkResponse, MapInfo):
    pass


class MapListResponse(OkResponse):
    maps: list[str] = Field(default_factory=list)


class ZonesResponse(OkResponse):
    zones: list[ForbiddenZone] = Field(default_factory=list)


class SlamMapResponse(OkResponse, SlamMapData):
    pass


class PlannedPathResponse(OkResponse, PlannedPath):
    pass
