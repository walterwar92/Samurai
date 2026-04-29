"""
Routers: FSM state + voice/text commands + admin force-transitions.

Маппинг старых endpoints:
  GET  /api/fsm              → /fsm
  POST /api/fsm/command      → /fsm/command       (FsmCommand: text)
  POST /api/fsm/transition   → /fsm/transition    (FsmTransitionCommand: state)

POST /fsm/command публикует voice_command на Pi (как будто пользователь
сказал голосом). FSM на Pi сам распарсит и решит как реагировать.

POST /fsm/transition — admin override; публикует fsm/transition + дублирует
голосом ("переход STATE") для логирования в voice_log.
"""
from __future__ import annotations

import time

from fastapi import APIRouter

from ..schemas.common import CommandAck
from ..schemas.control import FsmCommand, FsmStateResponse, FsmTransitionCommand
from ._deps import MQTTDep, StateDep

router = APIRouter()


@router.get('', response_model=FsmStateResponse, tags=['fsm'])
async def get_fsm(state: StateDep) -> FsmStateResponse:
    """Текущее состояние FSM на Pi (state, target_colour, target_action)."""
    with state.lock:
        f = state.robot.fsm
    return FsmStateResponse(
        state=f.state,
        target_colour=f.target_colour,
        target_action=f.target_action,
    )


@router.post('/command', response_model=CommandAck, tags=['fsm'])
async def send_fsm_command(
    cmd: FsmCommand,
    state: StateDep,
    mqtt: MQTTDep,
) -> CommandAck:
    """Послать текстовую команду на Pi (через voice_command топик).

    Pi-side voice_listener распарсит как обычное голосовое выражение.
    """
    text = cmd.text.strip()
    mqtt.publish('voice_command', text, qos=1)
    state.append_event_log({
        'ts': time.time(),
        'source': 'dashboard',
        'level': 'INFO',
        'text': f'api_command: {text}',
    })
    return CommandAck()


@router.post('/transition', response_model=CommandAck, tags=['fsm'])
async def force_fsm_transition(
    cmd: FsmTransitionCommand,
    state: StateDep,
    mqtt: MQTTDep,
) -> CommandAck:
    """Admin override: принудительно переключить FSM в указанное состояние."""
    target = cmd.state  # Pydantic Literal уже отвалидировал
    mqtt.publish('fsm/transition', target, qos=1)
    # Дублируем как голосовая команда — для unified логирования и UI feedback.
    mqtt.publish('voice_command', f'переход {target}', qos=1)
    state.append_event_log({
        'ts': time.time(),
        'source': 'dashboard',
        'level': 'WARN',
        'text': f'force fsm transition → {target}',
    })
    return CommandAck()
