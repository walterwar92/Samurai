"""
YOLO detections, ball tracking, gestures.
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# Цвета HSV-классификации (см. config.yaml hsv_colours).
Colour = Literal['red', 'orange', 'yellow', 'green', 'blue', 'white', 'black', 'unknown']


class Detection(BaseModel):
    """Один объект из детектора (compute_node/detector.py)."""
    cls: str = Field(default='object', alias='class',
                     description='YOLO class name (sports ball, person, ...)')
    colour: Colour = 'unknown'
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0
    conf: float = 0.0
    distance: float = Field(default=-1.0, description='Метры. -1 = bbox слишком мал')
    dist_method: Literal['mono', 'ultra', 'blend', 'none'] = 'none'
    world_x: Optional[float] = Field(
        default=None,
        description='Мировая X координата (м), если поза робота известна'
    )
    world_y: Optional[float] = None

    model_config = {'populate_by_name': True}  # принимать оба: 'cls' и 'class'


class DetectionResult(BaseModel):
    """Сводка всех детекций кадра."""
    objects: list[Detection] = Field(default_factory=list)
    count: int = 0
    ts: float = 0.0


class BallInfo(BaseModel):
    """Tracked ball на арене (постоянный ID между кадрами)."""
    id: int
    colour: Colour = 'unknown'
    x: float
    y: float
    grabbed: bool = False


class GestureEvent(BaseModel):
    """Событие жеста от gesture_node (если запущен)."""
    name: str
    confidence: float = 0.0
    ts: float = 0.0


# ── Response models ────────────────────────────────────────────────────────
class DetectionResponse(OkResponse, DetectionResult):
    pass


class ClosestDetectionResponse(OkResponse):
    detection: Optional[Detection] = None


class BallsResponse(OkResponse):
    balls: list[BallInfo] = Field(default_factory=list)


# ── Control ───────────────────────────────────────────────────────────────
class DetectionToggleCommand(BaseModel):
    """POST /api/detection/toggle — включить/выключить YOLO на ноуте."""
    enabled: bool


class DetectionStatusResponse(OkResponse):
    enabled: bool
    backend: Optional[str] = Field(
        default=None,
        description='yolo | hsv (active backend в detector.py)'
    )
    fps: Optional[float] = None
