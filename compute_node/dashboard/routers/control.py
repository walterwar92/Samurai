"""
Routers: высокоуровневое управление — patrol, follow_me, path_recorder,
precision drive, calibration, mission, explorer, TTS, toggles.

Маппинг старых endpoints (см. compute_node/dashboard_node.py L1569-1828):

  POST /api/patrol/command           → /patrol/command
  POST /api/patrol/waypoints         → /patrol/waypoints
  POST /api/follow_me                → /follow_me

  POST /api/path_recorder/command    → /path_recorder/command
  GET  /api/path_recorder/status     → /path_recorder/status
  GET  /api/path_recorder/path       → /path_recorder/path
  GET  /api/path_recorder/list       → /path_recorder/list  (~/paths/*.json)

  POST /api/precision_drive/command  → /precision_drive/command
  GET  /api/precision_drive/status   → /precision_drive/status

  POST /api/calibration/command            → /calibration/command
  POST /api/calibration/set                → /calibration/set
  POST /api/calibration/profile/load       → /calibration/profile/load
  POST /api/calibration/profile/save       → /calibration/profile/save
  POST /api/calibration/profile/delete     → /calibration/profile/delete
  GET  /api/calibration/profile/list       → /calibration/profile/list
  GET  /api/calibration/coefficients       → /calibration/coefficients

  POST /api/mission/command          → /mission/command
  GET  /api/mission/list             → /mission/list  (~/missions/*.json)

  POST /api/explorer/command         → /explorer/command

  POST /api/tts/toggle               → /tts/toggle
  POST /api/tts/speak                → /tts/speak

  POST /api/obstacle_avoidance/toggle → /obstacle_avoidance/toggle
  POST /api/collision_guard/toggle    → /collision_guard/toggle

В app.py каждая под-фича получит свой prefix (/patrol, /follow_me, ...).
"""
from __future__ import annotations

import os

from fastapi import APIRouter

from ..schemas.common import CommandAck
from ..schemas.control import (
    CalibrationCoefficientsResponse,
    CalibrationCommand,
    CalibrationProfile,
    CalibrationProfileDeleteCommand,
    CalibrationProfileListResponse,
    CalibrationProfileLoadCommand,
    CalibrationProfileSaveCommand,
    CalibrationSetCommand,
    ExplorerCommand,
    FollowMeCommand,
    MissionCommand,
    MissionListResponse,
    PathListResponse,
    PathPlannerGoalCommand,
    PathPlannerPathResponse,
    PathPlannerStatusResponse,
    PathRecorderCommand,
    PathRecorderPathResponse,
    PatrolCommand,
    PatrolWaypointsCommand,
    PrecisionDriveCommand,
    StatusResponse,
    ToggleCommand,
    TTSSpeakCommand,
    TTSToggleCommand,
)
from ._deps import MQTTDep, StateDep

# ── Patrol ──────────────────────────────────────────────────────────────
patrol_router = APIRouter()


@patrol_router.post('/command', response_model=CommandAck, tags=['control'])
async def patrol_command(cmd: PatrolCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('patrol/command', cmd.command, qos=1)
    return CommandAck()


@patrol_router.post('/waypoints', response_model=CommandAck, tags=['control'])
async def patrol_waypoints(
    cmd: PatrolWaypointsCommand, mqtt: MQTTDep
) -> CommandAck:
    """Установить список waypoints для patrol-маршрута."""
    payload = [{'x': w.x, 'y': w.y, **({'theta': w.theta} if w.theta is not None else {})}
               for w in cmd.waypoints]
    mqtt.publish('patrol/waypoints', payload, qos=1)
    return CommandAck()


# ── Follow-me ───────────────────────────────────────────────────────────
follow_me_router = APIRouter()


@follow_me_router.post('', response_model=CommandAck, tags=['control'])
async def follow_me(cmd: FollowMeCommand, mqtt: MQTTDep) -> CommandAck:
    payload = {'command': cmd.command}
    if cmd.target_distance is not None:
        payload['target_distance'] = cmd.target_distance
    mqtt.publish('follow_me/command', payload, qos=1)
    return CommandAck()


# ── Path recorder ───────────────────────────────────────────────────────
path_recorder_router = APIRouter()


@path_recorder_router.post('/command', response_model=CommandAck, tags=['control'])
async def path_recorder_command(
    cmd: PathRecorderCommand, mqtt: MQTTDep
) -> CommandAck:
    payload = {'command': cmd.command}
    if cmd.name:
        payload['name'] = cmd.name
    mqtt.publish('path_recorder/command', payload, qos=1)
    return CommandAck()


@path_recorder_router.get('/status', response_model=StatusResponse, tags=['control'])
async def path_recorder_status(state: StateDep) -> StatusResponse:
    with state.lock:
        return StatusResponse(status=dict(state.control.path_recorder_status))


@path_recorder_router.get('/path', response_model=PathRecorderPathResponse, tags=['control'])
async def path_recorder_path(state: StateDep) -> PathRecorderPathResponse:
    with state.lock:
        raw = list(state.control.path_recorder_path)
    points: list[list[float]] = []
    for p in raw:
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            points.append([float(p[0]), float(p[1])])
        elif isinstance(p, dict) and 'x' in p and 'y' in p:
            points.append([float(p['x']), float(p['y'])])
    return PathRecorderPathResponse(path=points, waypoints=len(points))


@path_recorder_router.get('/list', response_model=PathListResponse, tags=['control'])
async def path_recorder_list() -> PathListResponse:
    paths_dir = os.path.expanduser('~/paths')
    if not os.path.isdir(paths_dir):
        return PathListResponse(paths=[])
    items = sorted(
        f[:-len('.json')]
        for f in os.listdir(paths_dir)
        if f.endswith('.json')
    )
    return PathListResponse(paths=items)


# ── Precision drive ─────────────────────────────────────────────────────
precision_router = APIRouter()


@precision_router.post('/command', response_model=CommandAck, tags=['control'])
async def precision_drive_command(
    cmd: PrecisionDriveCommand, mqtt: MQTTDep
) -> CommandAck:
    """Сценарии точного драйвинга (cross/square/line/zigzag/goto)."""
    payload = cmd.model_dump(exclude_none=True)
    mqtt.publish('precision_drive/command', payload, qos=1)
    return CommandAck()


@precision_router.get('/status', response_model=StatusResponse, tags=['control'])
async def precision_drive_status(state: StateDep) -> StatusResponse:
    with state.lock:
        return StatusResponse(status=dict(state.control.precision_drive_status))


# ── Calibration ─────────────────────────────────────────────────────────
calibration_router = APIRouter()


@calibration_router.post('/command', response_model=CommandAck, tags=['control'])
async def calibration_command(
    cmd: CalibrationCommand, mqtt: MQTTDep
) -> CommandAck:
    """Admin: start/stop/reset калибровки."""
    mqtt.publish('calibration/command', cmd.command, qos=1)
    return CommandAck()


@calibration_router.post('/set', response_model=CommandAck, tags=['control'])
async def calibration_set(
    cmd: CalibrationSetCommand, mqtt: MQTTDep
) -> CommandAck:
    """Установить scale_fwd/bwd/motor_trim напрямую."""
    payload = cmd.model_dump(exclude_none=True)
    mqtt.publish('calibration/set', payload, qos=1)
    return CommandAck()


@calibration_router.post('/profile/load', response_model=CommandAck, tags=['control'])
async def calibration_profile_load(
    cmd: CalibrationProfileLoadCommand, mqtt: MQTTDep
) -> CommandAck:
    mqtt.publish('calibration/profile/load', {'name': cmd.name}, qos=1)
    return CommandAck()


@calibration_router.post('/profile/save', response_model=CommandAck, tags=['control'])
async def calibration_profile_save(
    cmd: CalibrationProfileSaveCommand, mqtt: MQTTDep
) -> CommandAck:
    mqtt.publish('calibration/profile/save', {'name': cmd.name}, qos=1)
    return CommandAck()


@calibration_router.post('/profile/delete', response_model=CommandAck, tags=['control'])
async def calibration_profile_delete(
    cmd: CalibrationProfileDeleteCommand, mqtt: MQTTDep
) -> CommandAck:
    mqtt.publish('calibration/profile/delete', {'name': cmd.name}, qos=1)
    return CommandAck()


@calibration_router.get(
    '/profile/list', response_model=CalibrationProfileListResponse, tags=['control'])
async def calibration_profile_list(
    state: StateDep, mqtt: MQTTDep
) -> CalibrationProfileListResponse:
    """Запросить актуальный список (Pi пушит через calibration/profile/all)
    и одновременно вернуть текущий кэш из state."""
    mqtt.publish('calibration/profile/list', '{}', qos=1)
    with state.lock:
        raw = list(state.control.calibration_profiles)
        active = (state.control.calibration_coeffs or {}).get('profile')
    profiles: list[CalibrationProfile] = []
    for p in raw:
        if isinstance(p, dict) and 'name' in p:
            profiles.append(CalibrationProfile(
                name=p['name'],
                scale_fwd=p.get('scale_fwd', 1.0),
                scale_bwd=p.get('scale_bwd', 1.0),
                motor_trim=p.get('motor_trim', 0.0),
            ))
        elif isinstance(p, str):
            profiles.append(CalibrationProfile(name=p))
    return CalibrationProfileListResponse(profiles=profiles, active=active)


@calibration_router.get(
    '/coefficients', response_model=CalibrationCoefficientsResponse, tags=['control'])
async def calibration_coefficients(state: StateDep) -> CalibrationCoefficientsResponse:
    with state.lock:
        active = (state.control.calibration_coeffs or {}).get('profile')
    # В старом dashboard_node возвращался _calibration_active целиком — но он
    # содержит только имя профиля. Coeffs приходят с того же топика, храним
    # их в calibration_status как fallback.
    with state.lock:
        coeffs = dict(state.control.calibration_status.get('coefficients', {}))
        if not coeffs:
            coeffs = {'name': active} if active else {}
    return CalibrationCoefficientsResponse(coefficients=coeffs)


# ── Path planner (#3, 2026-04) ──────────────────────────────────────────
path_planner_router = APIRouter()


@path_planner_router.post('/goto', response_model=CommandAck, tags=['control'])
async def path_planner_goto(
    cmd: PathPlannerGoalCommand, mqtt: MQTTDep
) -> CommandAck:
    """Запросить планирование A* до точки (x, y) в мировых координатах.

    Path planner живёт на ноутбуке (compute_node/path_planner) и публикует
    результат в samurai/{robot_id}/path_planner/path. Этот endpoint
    отправляет goal — нода-планировщик асинхронно посчитает путь.
    """
    mqtt.publish('path_planner/goal', {'x': cmd.x, 'y': cmd.y}, qos=1)
    return CommandAck()


@path_planner_router.get('/path', response_model=PathPlannerPathResponse,
                         tags=['control'])
async def path_planner_path(state: StateDep) -> PathPlannerPathResponse:
    with state.lock:
        return PathPlannerPathResponse(
            waypoints=list(state.control.path_planner_path),
            goal=state.control.path_planner_goal,
        )


@path_planner_router.get('/status', response_model=PathPlannerStatusResponse,
                         tags=['control'])
async def path_planner_status(state: StateDep) -> PathPlannerStatusResponse:
    with state.lock:
        st = dict(state.control.path_planner_status)
    return PathPlannerStatusResponse(
        state=st.get('state', 'idle'),
        message=st.get('message'),
        planning_ms=st.get('planning_ms'),
    )


# ── Mission ─────────────────────────────────────────────────────────────
mission_router = APIRouter()


@mission_router.post('/command', response_model=CommandAck, tags=['control'])
async def mission_command(cmd: MissionCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('mission/command', cmd.model_dump(), qos=1)
    return CommandAck()


@mission_router.get('/list', response_model=MissionListResponse, tags=['control'])
async def mission_list() -> MissionListResponse:
    missions_dir = os.path.expanduser('~/missions')
    if not os.path.isdir(missions_dir):
        return MissionListResponse(missions=[])
    items = sorted(
        f[:-len('.json')]
        for f in os.listdir(missions_dir)
        if f.endswith('.json')
    )
    return MissionListResponse(missions=items)


# ── Explorer ────────────────────────────────────────────────────────────
explorer_router = APIRouter()


@explorer_router.post('/command', response_model=CommandAck, tags=['control'])
async def explorer_command(cmd: ExplorerCommand, mqtt: MQTTDep) -> CommandAck:
    payload = {'command': cmd.command}
    if cmd.strategy:
        payload['strategy'] = cmd.strategy
    mqtt.publish('explorer/command', payload, qos=1)
    return CommandAck()


# ── TTS ─────────────────────────────────────────────────────────────────
tts_router = APIRouter()


@tts_router.post('/toggle', response_model=CommandAck, tags=['control'])
async def tts_toggle(
    cmd: TTSToggleCommand, state: StateDep, mqtt: MQTTDep
) -> CommandAck:
    with state.lock:
        state.system.tts_enabled = cmd.enabled
    mqtt.publish('tts/enable', 'on' if cmd.enabled else 'off', qos=1)
    return CommandAck()


@tts_router.post('/speak', response_model=CommandAck, tags=['control'])
async def tts_speak(cmd: TTSSpeakCommand, mqtt: MQTTDep) -> CommandAck:
    text = cmd.text.strip()
    if text:
        mqtt.publish('tts/command', text, qos=1)
    return CommandAck()


# ── Toggles (obstacle avoidance + collision guard) ──────────────────────
obstacle_router = APIRouter()


@obstacle_router.post('/toggle', response_model=CommandAck, tags=['control'])
async def obstacle_avoidance_toggle(
    cmd: ToggleCommand, state: StateDep, mqtt: MQTTDep
) -> CommandAck:
    """Включить/выключить obstacle avoidance во время path replay."""
    with state.lock:
        state.control.obstacle_avoidance_enabled = cmd.enabled
    mqtt.publish('obstacle_avoidance/enable',
                 'on' if cmd.enabled else 'off', qos=1)
    return CommandAck()


collision_guard_router = APIRouter()


@collision_guard_router.post('/toggle', response_model=CommandAck, tags=['control'])
async def collision_guard_toggle(
    cmd: ToggleCommand, state: StateDep, mqtt: MQTTDep
) -> CommandAck:
    """Включить/выключить collision guard для manual control."""
    with state.lock:
        state.control.collision_guard_enabled = cmd.enabled
    mqtt.publish('collision_guard/enable',
                 'on' if cmd.enabled else 'off', qos=1)
    return CommandAck()
