"""
Pydantic-контракт для /ws/robot/live_state.

См. docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
"""
from __future__ import annotations

from typing import Literal, Optional
from pydantic import BaseModel, Field


class RobotLiveStatePose(BaseModel):
    """Поза робота в world frame."""
    x: float = Field(description='m, world frame')
    y: float = Field(description='m, world frame')
    yaw_rad: float = Field(description='heading в радианах (state.robot.pose.yaw)')
    yaw_deg: float = Field(description='heading в градусах (для удобства UI)')


class RobotLiveStateVel(BaseModel):
    """Текущая оценённая скорость."""
    linear: float = Field(description='м/с — VelocityDetail.linear_x')
    angular: float = Field(description='рад/с — VelocityDetail.angular_z')


class RobotLiveStateImu(BaseModel):
    """IMU snapshot: ориентация + сырые гироскоп/акселерометр + EKF bias."""
    ypr_deg: list[float] = Field(
        description='[yaw, pitch, roll] в °. EKF-результат если has_ekf, '
                    'иначе raw-fallback из акселя.',
    )
    gyro: list[float] = Field(description='[x, y, z] рад/с')
    accel: list[float] = Field(description='[x, y, z] м/с²')
    ekf_bias_deg: Optional[list[float]] = Field(
        default=None,
        description='[x, y, z] °/с; None если EKF выключен',
    )
    has_ekf: bool


class RobotLiveStatePoint(BaseModel):
    """Полный snapshot состояния робота для UI-панели."""
    ts: float = Field(description='Pi-clock unix-секунды')
    pose: RobotLiveStatePose
    vel: RobotLiveStateVel
    imu: RobotLiveStateImu
    stationary: bool = Field(description='ZUPT-флаг (state.robot.stationary)')
    schema_version: Literal['1.0'] = '1.0'
