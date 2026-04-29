"""
Robot pose, velocity, status.
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Domain models ──────────────────────────────────────────────────────────
class RobotPose(BaseModel):
    """2D позиция робота в мировых координатах (метры, радианы)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = Field(default=0.0, description='Курс в радианах')


class VelocityDetail(BaseModel):
    """Линейная X/Y и угловая Z составляющие скорости."""
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class RobotVelocity(BaseModel):
    """Скорость робота: оценённая (из одометрии) и заданная (последний cmd_vel)."""
    estimated: VelocityDetail = Field(default_factory=VelocityDetail)
    commanded: VelocityDetail = Field(default_factory=VelocityDetail)


class OdometrySources(BaseModel):
    """Параллельные оценки позиции от разных источников одометрии (метры, рад/с).

    Публикуется motor_node-ом в каждом odom-тике как diagnostic, чтобы
    дашборд мог рендерить wheel/imu/complementary/ekf одновременно и
    пользователь мог сравнить точность каждого режима на живом роботе.
    """
    x_wheel: float = 0.0
    y_wheel: float = 0.0
    x_imu: float = 0.0
    y_imu: float = 0.0
    vx_imu: float = 0.0
    vy_imu: float = 0.0
    stationary_imu: bool = True
    # Какой источник используется как primary x/y в этом тике.
    source: Literal['wheel', 'imu', 'complementary', 'ekf'] = 'wheel'


class RobotStatus(BaseModel):
    """Статус FSM робота — что робот делает прямо сейчас."""
    state: str = Field(default='IDLE', description='Имя FSM-состояния')
    target_colour: str = ''
    target_action: str = ''


# ── Request models ─────────────────────────────────────────────────────────
class VelocityCommand(BaseModel):
    """POST /api/robot/velocity body."""
    linear: float = Field(default=0.0, description='Линейная скорость м/с (X в локальной СК)')
    angular: float = Field(default=0.0, description='Угловая скорость рад/с')


class ResetPositionCommand(BaseModel):
    """POST /api/robot/reset_position body — опционально с явными координатами."""
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None


# ── Response models ────────────────────────────────────────────────────────
class PoseResponse(OkResponse, RobotPose):
    """Composite OK + RobotPose поля для /api/robot/pose."""
    pass


class VelocityResponse(OkResponse):
    """GET /api/robot/velocity → estimated + commanded блоки."""
    estimated: VelocityDetail = Field(default_factory=VelocityDetail)
    commanded: VelocityDetail = Field(default_factory=VelocityDetail)


# ── Speed profile ──────────────────────────────────────────────────────────
SpeedProfile = Literal['slow', 'normal', 'fast']


class SpeedProfileCommand(BaseModel):
    profile: SpeedProfile


class SpeedProfileResponse(OkResponse):
    profile: SpeedProfile = 'normal'
