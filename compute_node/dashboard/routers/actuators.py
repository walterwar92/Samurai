"""
Routers: claw / head / arm / led actuators.

Маппинг старых endpoints → новые:

  GET  /api/actuators                  → /actuators        (router)
  GET  /api/actuators/claw             → /actuators/claw
  POST /api/actuators/claw             → /actuators/claw   (ClawCommand)
  GET  /api/actuators/head             → /actuators/head
  POST /api/actuators/head             → /actuators/head   (HeadCommand)
  GET  /api/actuators/head/presets     → /actuators/head/presets
  POST /api/actuators/head/preset/save → /actuators/head/preset/save  (PresetSaveCommand)
  POST /api/actuators/head/preset/load → /actuators/head/preset/load  (PresetLoadCommand)
  DELETE /api/actuators/head/preset/{name}
  GET  /api/actuators/arm              → /actuators/arm
  POST /api/actuators/arm              → /actuators/arm    (ArmJointCommand)
  GET  /api/actuators/arm/presets      → /actuators/arm/presets
  POST /api/actuators/arm/preset/save  → /actuators/arm/preset/save
  POST /api/actuators/arm/preset/load  → /actuators/arm/preset/load
  DELETE /api/actuators/arm/preset/{name}
  POST /api/led/command                → /led/command       (отдельный led_router)

Старая семантика командных полей (Pi-side head_node/arm_node):
  head/command:
    {"angle": N}                  | {"command": "center"}
    {"command": "lock"|"unlock"}  | {"command": "freeze"|"unfreeze"}
    {"command": "save_preset"|"load_preset"|"delete_preset", "name": "..."}
  arm/command (1-indexed!):
    {"joint": 1..4, "angle": 0..180} | {"joints": [..,..,..,..]}
    {"command": "home"|"unlock"|"freeze"|"unfreeze", "joint": opt}
    {"command": "save_preset"|"load_preset"|"delete_preset", "name": "..."}
"""
from __future__ import annotations

from fastapi import APIRouter, HTTPException

from ..schemas.actuators import (
    ActuatorsResponse,
    ArmJointCommand,
    ArmResponse,
    ClawCommand,
    ClawResponse,
    HeadCommand,
    HeadResponse,
    LedCommand,
    PresetInfo,
    PresetListResponse,
    PresetLoadCommand,
    PresetSaveCommand,
)
from ..schemas.common import CommandAck
from ._deps import MQTTDep, StateDep

# Главный actuators router (под /api/v1/actuators)
router = APIRouter()


# ── /actuators (bundle) ────────────────────────────────────────────────
@router.get('', response_model=ActuatorsResponse, tags=['actuators'])
async def get_actuators(state: StateDep) -> ActuatorsResponse:
    with state.lock:
        return ActuatorsResponse(
            claw=state.actuators.claw,
            head=state.actuators.head,
            arm=state.actuators.arm,
        )


# ── /actuators/claw ────────────────────────────────────────────────────
@router.get('/claw', response_model=ClawResponse, tags=['actuators'])
async def get_claw(state: StateDep) -> ClawResponse:
    with state.lock:
        c = state.actuators.claw
    return ClawResponse(open=c.open, angle=c.angle)


@router.post('/claw', response_model=CommandAck, tags=['actuators'])
async def set_claw(cmd: ClawCommand, mqtt: MQTTDep) -> CommandAck:
    """Клешня = arm joint 4 (1-indexed). open=0°, close=180°."""
    if cmd.angle is not None:
        angle = max(0.0, min(180.0, float(cmd.angle)))
    elif cmd.state == 'open':
        angle = 0.0
    elif cmd.state == 'close':
        angle = 180.0
    else:
        raise HTTPException(400, 'state ("open"/"close") or angle required')
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    return CommandAck()


# ── /actuators/head ────────────────────────────────────────────────────
@router.get('/head', response_model=HeadResponse, tags=['actuators'])
async def get_head(state: StateDep) -> HeadResponse:
    with state.lock:
        h = state.actuators.head
    return HeadResponse(angle=h.angle, frozen=h.frozen, locked=h.locked)


@router.post('/head', response_model=CommandAck, tags=['actuators'])
async def set_head(cmd: HeadCommand, mqtt: MQTTDep) -> CommandAck:
    """Один POST → одна команда. Если задано несколько полей — порядок:
    angle → center → locked → frozen (последняя команда побеждает)."""
    if cmd.angle is not None:
        a = max(0.0, min(180.0, float(cmd.angle)))
        mqtt.publish('head/command', {'angle': a}, qos=1)
    if cmd.center:
        mqtt.publish('head/command', {'command': 'center'}, qos=1)
    if cmd.locked is not None:
        mqtt.publish('head/command',
                     {'command': 'lock' if cmd.locked else 'unlock'}, qos=1)
    if cmd.frozen is not None:
        mqtt.publish('head/command',
                     {'command': 'freeze' if cmd.frozen else 'unfreeze'}, qos=1)
    return CommandAck()


@router.get('/head/presets', response_model=PresetListResponse, tags=['actuators'])
async def list_head_presets(state: StateDep, mqtt: MQTTDep) -> PresetListResponse:
    """Cached список presets из state. Параллельно публикуем list_presets чтобы
    head_node освежил кэш через head/presets retained."""
    mqtt.publish('head/command', {'command': 'list_presets'}, qos=1)
    with state.lock:
        raw = dict(state.actuators.head_presets)
    presets = [PresetInfo(name=n, angles=([a] if isinstance(a, (int, float)) else list(a or [])))
               for n, a in raw.items()]
    return PresetListResponse(presets=presets)


@router.post('/head/preset/save', response_model=CommandAck, tags=['actuators'])
async def save_head_preset(cmd: PresetSaveCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('head/command',
                 {'command': 'save_preset', 'name': cmd.name}, qos=1)
    return CommandAck()


@router.post('/head/preset/load', response_model=CommandAck, tags=['actuators'])
async def load_head_preset(cmd: PresetLoadCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('head/command',
                 {'command': 'load_preset', 'name': cmd.name}, qos=1)
    return CommandAck()


@router.delete('/head/preset/{name}', response_model=CommandAck, tags=['actuators'])
async def delete_head_preset(name: str, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('head/command',
                 {'command': 'delete_preset', 'name': name}, qos=1)
    return CommandAck()


# ── /actuators/arm ─────────────────────────────────────────────────────
@router.get('/arm', response_model=ArmResponse, tags=['actuators'])
async def get_arm(state: StateDep) -> ArmResponse:
    with state.lock:
        a = state.actuators.arm
    return ArmResponse(j1=a.j1, j2=a.j2, j3=a.j3, j4=a.j4,
                       frozen=a.frozen, locked=a.locked)


@router.post('/arm', response_model=CommandAck, tags=['actuators'])
async def set_arm(cmd: ArmJointCommand, mqtt: MQTTDep) -> CommandAck:
    """Поддерживаемые комбинации (порядок проверки):
      home=true                 → {"command": "home"}
      preset='X'                → {"command": "load_preset", "name": "X"}
      freeze=true/false         → {"command": "freeze"|"unfreeze", joint?: N}
      joints=[..,..,..,..]      → {"joints": [...]}
      jN установлен             → {"joint": N, "angle": jN}
    """
    if cmd.home:
        mqtt.publish('arm/command', {'command': 'home'}, qos=1)
        return CommandAck()
    if cmd.preset:
        mqtt.publish('arm/command',
                     {'command': 'load_preset', 'name': cmd.preset}, qos=1)
        return CommandAck()
    if cmd.freeze is not None:
        payload = {'command': 'freeze' if cmd.freeze else 'unfreeze'}
        if cmd.joint_index is not None:
            payload['joint'] = cmd.joint_index
        mqtt.publish('arm/command', payload, qos=1)
        return CommandAck()
    if cmd.joints is not None:
        if len(cmd.joints) < 4:
            raise HTTPException(400, 'joints must have 4 angles')
        clamped = [max(0.0, min(180.0, float(a))) for a in cmd.joints[:4]]
        mqtt.publish('arm/command', {'joints': clamped}, qos=1)
        return CommandAck()
    # Single-joint mode (любая комбинация j1..j4)
    sent_any = False
    for idx, value in enumerate([cmd.j1, cmd.j2, cmd.j3, cmd.j4], start=1):
        if value is None:
            continue
        a = max(0.0, min(180.0, float(value)))
        mqtt.publish('arm/command', {'joint': idx, 'angle': a}, qos=1)
        sent_any = True
    if not sent_any:
        raise HTTPException(
            400, 'specify one of: jN, joints, home, preset, freeze')
    return CommandAck()


@router.get('/arm/presets', response_model=PresetListResponse, tags=['actuators'])
async def list_arm_presets(state: StateDep, mqtt: MQTTDep) -> PresetListResponse:
    mqtt.publish('arm/command', {'command': 'list_presets'}, qos=1)
    with state.lock:
        raw = dict(state.actuators.arm_presets)
    presets = [PresetInfo(name=n, angles=list(a) if isinstance(a, (list, tuple)) else [a])
               for n, a in raw.items()]
    return PresetListResponse(presets=presets)


@router.post('/arm/preset/save', response_model=CommandAck, tags=['actuators'])
async def save_arm_preset(cmd: PresetSaveCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('arm/command',
                 {'command': 'save_preset', 'name': cmd.name}, qos=1)
    return CommandAck()


@router.post('/arm/preset/load', response_model=CommandAck, tags=['actuators'])
async def load_arm_preset(cmd: PresetLoadCommand, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('arm/command',
                 {'command': 'load_preset', 'name': cmd.name}, qos=1)
    return CommandAck()


@router.delete('/arm/preset/{name}', response_model=CommandAck, tags=['actuators'])
async def delete_arm_preset(name: str, mqtt: MQTTDep) -> CommandAck:
    mqtt.publish('arm/command',
                 {'command': 'delete_preset', 'name': name}, qos=1)
    return CommandAck()


# ── /led/command (отдельный sub-router) ─────────────────────────────────
led_router = APIRouter()


@led_router.post('/command', response_model=CommandAck, tags=['actuators'])
async def led_command(cmd: LedCommand, mqtt: MQTTDep) -> CommandAck:
    """LED панель WS2812B. Pi-side led_node ждёт {mode, color?, brightness?}.

    Маппим animation→mode для backward-compat с led_node.
    """
    payload: dict = {'mode': cmd.animation}
    if cmd.color:
        payload['color'] = cmd.color
    if cmd.brightness is not None:
        payload['brightness'] = cmd.brightness
    if cmd.speed is not None:
        payload['speed'] = cmd.speed
    mqtt.publish('led/command', payload, qos=1)
    return CommandAck()
