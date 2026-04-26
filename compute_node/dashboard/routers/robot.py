"""
Routers: pose / velocity / stop / reset / speed_profile.

Маппинг старых endpoints (compute_node/dashboard_node.py) → новые routes:

  GET  /api/robot/pose         → GET  router /pose
  GET  /api/robot/velocity     → GET  router /velocity
  POST /api/robot/velocity     → POST router /velocity
  POST /api/robot/stop         → POST router /stop
  POST /api/emergency_stop     → POST router /emergency_stop  (alias)
  POST /api/robot/reset_position → POST router /reset_position
  GET  /api/speed_profile      → GET  router /speed_profile
  POST /api/speed_profile      → POST router /speed_profile

В app.py этот router подключается под prefix='/api/v1/robot' (для
endpoints с router-path начинающимся на /). /emergency_stop и
/speed_profile получат отдельные префиксы — см. C11 factory.
"""
from __future__ import annotations

import time

from fastapi import APIRouter

from ..schemas.common import CommandAck
from ..schemas.robot import (
    PoseResponse,
    SpeedProfileCommand,
    SpeedProfileResponse,
    VelocityCommand,
    VelocityResponse,
)
from ._deps import MQTTDep, StateDep

router = APIRouter()


# ── GET pose ────────────────────────────────────────────────────────────
@router.get('/pose', response_model=PoseResponse, tags=['robot'])
async def get_pose(state: StateDep) -> PoseResponse:
    """Текущая поза робота (мировые координаты, м/радианы)."""
    with state.lock:
        p = state.robot.pose
    return PoseResponse(x=p.x, y=p.y, yaw=p.yaw)


# ── GET / POST velocity ─────────────────────────────────────────────────
@router.get('/velocity', response_model=VelocityResponse, tags=['robot'])
async def get_velocity(state: StateDep) -> VelocityResponse:
    """Скорость робота: estimated (одометрия) и commanded (последний cmd_vel)."""
    with state.lock:
        return VelocityResponse(
            estimated=state.robot.velocity_estimated,
            commanded=state.robot.velocity_commanded,
        )


@router.post('/velocity', response_model=CommandAck, tags=['robot'])
async def set_velocity(
    cmd: VelocityCommand,
    state: StateDep,
    mqtt: MQTTDep,
) -> CommandAck:
    """Послать cmd_vel/manual напрямую на Pi (приоритет выше autonomous)."""
    linear_x = round(cmd.linear, 3)
    angular_z = round(cmd.angular, 3)
    mqtt.publish('cmd_vel/manual',
                 {'linear_x': linear_x, 'angular_z': angular_z},
                 qos=0)
    from ..schemas.robot import VelocityDetail
    with state.lock:
        state.robot.velocity_commanded = VelocityDetail(
            linear_x=linear_x,
            linear_y=0.0,
            angular_z=angular_z,
        )
    return CommandAck()


# ── POST stop / emergency_stop ──────────────────────────────────────────
@router.post('/stop', response_model=CommandAck, tags=['robot'])
async def stop(state: StateDep, mqtt: MQTTDep) -> CommandAck:
    """Немедленный стоп — нулевая скорость напрямую на Pi."""
    mqtt.publish('cmd_vel/manual', {'linear_x': 0.0, 'angular_z': 0.0}, qos=0)
    from ..schemas.robot import VelocityDetail
    with state.lock:
        state.robot.velocity_commanded = VelocityDetail()
    state.append_event_log({
        'ts': time.time(), 'source': 'dashboard', 'level': 'INFO',
        'text': 'emergency stop',
    })
    return CommandAck()


# ── POST reset_position ────────────────────────────────────────────────
@router.post('/reset_position', response_model=CommandAck, tags=['robot'])
async def reset_position(state: StateDep, mqtt: MQTTDep) -> CommandAck:
    """Сбросить одометрию: текущая поза становится (0,0,0) — новый home."""
    mqtt.publish('reset_position', 'reset', qos=1)
    from ..schemas.robot import RobotPose
    with state.lock:
        state.robot.pose = RobotPose()
        state.robot.stationary = True
    return CommandAck()


# ── GET / POST speed_profile (отдельный путь — без /robot/) ──────────
# Подключается с prefix='/api/v1/speed_profile' в app.py factory.
speed_router = APIRouter()


@speed_router.get('', response_model=SpeedProfileResponse, tags=['robot'])
async def get_speed_profile(state: StateDep) -> SpeedProfileResponse:
    with state.lock:
        return SpeedProfileResponse(profile=state.robot.speed_profile)


@speed_router.post('', response_model=CommandAck, tags=['robot'])
async def set_speed_profile(
    cmd: SpeedProfileCommand,
    mqtt: MQTTDep,
) -> CommandAck:
    """Переключить профиль скорости (slow|normal|fast)."""
    mqtt.publish('speed_profile', cmd.profile, qos=1)
    return CommandAck()


# ── /emergency_stop (для backward-compat — отдельный путь без /robot/) ──
emergency_router = APIRouter()


@emergency_router.post('', response_model=CommandAck, tags=['robot'])
async def emergency_stop(state: StateDep, mqtt: MQTTDep) -> CommandAck:
    """Alias для POST /robot/stop, путь /api/emergency_stop."""
    return await stop(state, mqtt)
