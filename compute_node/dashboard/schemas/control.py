"""
FSM commands, patrol, follow_me, path_recorder, calibration, precision drive,
mission, explorer schemas.
"""
from __future__ import annotations

from typing import Any, Literal, Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Shared ────────────────────────────────────────────────────────────────
class StatusResponse(OkResponse):
    """Универсальная обёртка для GET .../status — отдаёт сырой dict состояния.

    Используется когда формат статуса от Pi-нод (calibration, mission,
    explorer, path_recorder, ...) часто меняется и строгая типизация
    мешает развитию.
    """
    status: dict[str, Any] = Field(default_factory=dict)


class ToggleCommand(BaseModel):
    """Универсальный POST .../toggle body."""
    enabled: bool


# ── FSM ────────────────────────────────────────────────────────────────────
FsmStateName = Literal[
    'IDLE', 'SEARCHING', 'TARGETING', 'APPROACHING', 'GRABBING',
    'CALLING', 'RETURNING', 'PATROLLING', 'FOLLOWING', 'PATH_REPLAY',
]


class FsmCommand(BaseModel):
    """POST /api/fsm/command — текстовая команда (как голосовая)."""
    text: str = Field(max_length=200)


class FsmTransitionCommand(BaseModel):
    """POST /api/fsm/transition — принудительный переход (admin)."""
    state: FsmStateName


class FsmStateResponse(OkResponse):
    state: str = 'IDLE'
    target_colour: str = ''
    target_action: str = ''


# ── Patrol ────────────────────────────────────────────────────────────────
class PatrolWaypoint(BaseModel):
    x: float
    y: float
    theta: Optional[float] = None


class PatrolWaypointsCommand(BaseModel):
    """POST /api/patrol/waypoints."""
    waypoints: list[PatrolWaypoint]


PatrolAction = Literal['start', 'stop', 'pause', 'resume']


class PatrolCommand(BaseModel):
    """POST /api/patrol/command."""
    command: PatrolAction


# ── Follow-me ─────────────────────────────────────────────────────────────
class FollowMeCommand(BaseModel):
    """POST /api/follow_me."""
    command: Literal['start', 'stop']
    target_distance: Optional[float] = Field(default=None, description='Метры')


# ── Path recorder ─────────────────────────────────────────────────────────
PathRecorderAction = Literal['record', 'stop', 'replay', 'pause', 'resume']


class PathRecorderCommand(BaseModel):
    """POST /api/path_recorder/command."""
    command: PathRecorderAction
    name: Optional[str] = Field(
        default=None,
        max_length=64,
        pattern=r'^[a-zA-Z0-9_\-]*$',
        description='Имя пути для load/save (только safe chars)'
    )


class PathListResponse(OkResponse):
    paths: list[str] = Field(default_factory=list)


class PathRecorderPathResponse(OkResponse):
    """Текущие записанные waypoints (для проигрывания / отрисовки)."""
    path: list[list[float]] = Field(default_factory=list)
    waypoints: int = 0


# ── Precision drive ───────────────────────────────────────────────────────
PrecisionScenario = Literal['cross', 'square', 'line', 'zigzag', 'goto']


class PrecisionDriveCommand(BaseModel):
    """POST /api/precision_drive/command."""
    scenario: PrecisionScenario
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_theta: Optional[float] = None
    distance_cm: Optional[float] = None


# ── Calibration ───────────────────────────────────────────────────────────
class CalibrationProfile(BaseModel):
    """Профиль одометрии для конкретной поверхности."""
    name: str
    scale_fwd: float = 1.0
    scale_bwd: float = 1.0
    motor_trim: float = 0.0


class CalibrationSetCommand(BaseModel):
    """POST /api/calibration/set."""
    scale_fwd: Optional[float] = None
    scale_bwd: Optional[float] = None
    motor_trim: Optional[float] = None


class CalibrationProfileSaveCommand(BaseModel):
    """POST /api/calibration/profile/save."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')
    description: str = Field(default='', max_length=256)


class CalibrationProfileLoadCommand(BaseModel):
    """POST /api/calibration/profile/load."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')


class CalibrationProfileDeleteCommand(BaseModel):
    """POST /api/calibration/profile/delete."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')


class CalibrationCommand(BaseModel):
    """POST /api/calibration/command — admin string command (start/stop/reset)."""
    command: Literal['start', 'stop', 'reset', 'pause', 'resume']


class CalibrationProfileListResponse(OkResponse):
    profiles: list[CalibrationProfile] = Field(default_factory=list)
    active: Optional[str] = None


class CalibrationCoefficientsResponse(OkResponse):
    """Активные коэффициенты — формат свободный (зависит от профиля)."""
    coefficients: dict[str, Any] = Field(default_factory=dict)


# ── Path planner (#3, 2026-04) ────────────────────────────────────────────
class PathPlannerGoalCommand(BaseModel):
    """POST /api/path_planner/goto — цель планирования (мировые м)."""
    x: float
    y: float


class PathPlannerPathResponse(OkResponse):
    """Текущий запланированный путь."""
    waypoints: list[list[float]] = Field(default_factory=list)
    goal: Optional[list[float]] = None


class PathPlannerStatusResponse(OkResponse):
    """Статус последнего запроса к планировщику."""
    state: str = Field(default='idle', description='idle|success|failed|error')
    message: Optional[str] = None
    planning_ms: Optional[float] = None


# ── Mission ───────────────────────────────────────────────────────────────
class MissionCommand(BaseModel):
    """POST /api/mission/command — высокоуровневые сценарии."""
    name: str
    args: dict = Field(default_factory=dict)


class MissionListResponse(OkResponse):
    missions: list[str] = Field(default_factory=list)


# ── Explorer ──────────────────────────────────────────────────────────────
ExplorerStrategy = Literal['frontier', 'spiral', 'zigzag']


class ExplorerCommand(BaseModel):
    """POST /api/explorer/command."""
    command: Literal['start', 'stop']
    strategy: Optional[ExplorerStrategy] = 'frontier'


# ── TTS ───────────────────────────────────────────────────────────────────
class TTSToggleCommand(BaseModel):
    enabled: bool


class TTSSpeakCommand(BaseModel):
    text: str = Field(max_length=500)


# ── Multi-robot ───────────────────────────────────────────────────────────
class MultiRobotCallCommand(BaseModel):
    """POST /api/multi_robot/call — вызвать второго робота."""
    target_id: str
    colour: Optional[str] = None
    action: Literal['grab', 'burn'] = 'grab'


class RobotInfo(BaseModel):
    """Информация о другом роботе в сети."""
    id: str
    online: bool
    last_seen_s: Optional[float] = None
    state: Optional[str] = None


class MultiRobotListResponse(OkResponse):
    robots: list[RobotInfo] = Field(default_factory=list)
