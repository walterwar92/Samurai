"""
Claw, head, arm, LED actuator schemas.
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Domain models ──────────────────────────────────────────────────────────
ClawAction = Literal['open', 'close']


class ClawState(BaseModel):
    """Состояние клешни."""
    open: bool = False
    angle: Optional[float] = Field(default=None, description='Текущий угол в градусах')


class HeadState(BaseModel):
    """Состояние head-серво (pan камеры)."""
    angle: float = 90.0
    frozen: bool = False
    locked: bool = True


class ArmState(BaseModel):
    """Состояние 4-DOF руки. j1..j4 = углы суставов в градусах."""
    j1: float = 0.0  # Основание (CH0)
    j2: float = 120.0  # Сустав 1 (CH1)
    j3: float = 0.0  # Сустав 2 (CH2)
    j4: float = 0.0  # Клешня (CH3)
    frozen: list[bool] = Field(default_factory=lambda: [False] * 4)
    locked: bool = True


class ActuatorsBundle(BaseModel):
    """Все актуаторы одним объектом для GET /api/actuators."""
    claw: ClawState = Field(default_factory=ClawState)
    head: Optional[HeadState] = None
    arm: Optional[ArmState] = None


class LedState(BaseModel):
    """Состояние WS2812B."""
    animation: str = 'off'  # off, solid, blink, pulse, rainbow, police, ...
    color: Optional[str] = Field(
        default=None,
        description='Hex цвет (#RRGGBB) или имя (red, blue, ...)'
    )
    brightness: int = Field(default=76, ge=0, le=255)
    speed: float = 1.0


# ── Request models ─────────────────────────────────────────────────────────
class ClawCommand(BaseModel):
    """POST /api/actuators/claw."""
    state: Optional[ClawAction] = None  # 'open' | 'close'
    angle: Optional[float] = Field(default=None, description='Прямой угол (если state не задан)')


class HeadCommand(BaseModel):
    """POST /api/actuators/head."""
    angle: Optional[float] = None
    center: bool = False
    locked: Optional[bool] = None


class ArmJointCommand(BaseModel):
    """POST /api/actuators/arm — установка одного или нескольких суставов."""
    j1: Optional[float] = None
    j2: Optional[float] = None
    j3: Optional[float] = None
    j4: Optional[float] = None
    home: bool = False
    freeze: Optional[bool] = None
    unfreeze: Optional[bool] = None
    preset: Optional[str] = Field(default=None, description='Имя пресета (см. servo_presets.json)')


class LedCommand(BaseModel):
    """POST /api/led/command."""
    animation: str
    color: Optional[str] = None
    brightness: Optional[int] = Field(default=None, ge=0, le=255)
    speed: Optional[float] = None


# ── Response models ────────────────────────────────────────────────────────
class ActuatorsResponse(OkResponse, ActuatorsBundle):
    pass


class ClawResponse(OkResponse, ClawState):
    pass


class HeadResponse(OkResponse, HeadState):
    pass


class ArmResponse(OkResponse, ArmState):
    pass


# ── Presets ────────────────────────────────────────────────────────────────
class PresetInfo(BaseModel):
    """Один пресет позы (head или arm)."""
    name: str
    angles: list[float] = Field(default_factory=list)


class PresetListResponse(OkResponse):
    presets: list[PresetInfo] = Field(default_factory=list)


class PresetSaveCommand(BaseModel):
    """Сохранить текущую позу как пресет."""
    name: str
    angles: Optional[list[float]] = Field(
        default=None,
        description='Если задан — сохранить эти углы вместо текущих'
    )
