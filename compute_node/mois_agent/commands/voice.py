"""Голос и FSM: voice text → fsm intent, принудительные переходы, TTS."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, bad_params, from_http, opt_param


def handle_voice(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"text": "поедь вперёд"} — отправить как голосовую команду в FSM."""
    text = opt_param(params, "text")
    if not text or not isinstance(text, str):
        return bad_params("text: обязательная строка")
    return from_http(
        ctx.client.post("/api/v1/fsm/command", json_body={"text": text})
    )


def handle_fsm_transition(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"state": "idle"|"hunt"|...} — принудительный переход (admin)."""
    state = opt_param(params, "state")
    if not state or not isinstance(state, str):
        return bad_params("state: обязательная строка (имя FSM-состояния)")
    return from_http(
        ctx.client.post("/api/v1/fsm/transition", json_body={"state": state})
    )


def handle_tts_speak(params: Mapping[str, Any], ctx) -> HandlerResult:
    text = opt_param(params, "text")
    if not text or not isinstance(text, str):
        return bad_params("text: обязательная строка")
    return from_http(
        ctx.client.post("/api/v1/tts/speak", json_body={"text": text})
    )


def handle_tts_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return bad_params("enabled: bool (true/false)")
    return from_http(
        ctx.client.post("/api/v1/tts/toggle", json_body={"enabled": bool(enabled)})
    )


COMMANDS = {
    "voice": {
        "description": "Голосовая команда (текст) → FSM",
        "params_schema": {
            "text": {
                "type": "string",
                "maxLength": 200,
                "description": "Текст команды (как от Vosk)",
            },
        },
        "handler": handle_voice,
    },
    "fsm_transition": {
        "description": "Принудительный переход FSM (admin)",
        "params_schema": {
            "state": {
                "type": "string",
                "maxLength": 32,
                "description": "Имя состояния (e.g. idle, hunt)",
            },
        },
        "handler": handle_fsm_transition,
    },
    "tts_speak": {
        "description": "Озвучить текст через TTS",
        "params_schema": {
            "text": {
                "type": "string",
                "maxLength": 500,
                "description": "Текст для синтеза речи",
            },
        },
        "handler": handle_tts_speak,
    },
    "tts_toggle": {
        "description": "Включить/выключить TTS",
        "params_schema": {
            "enabled": {
                "type": "boolean",
                "description": "Включить TTS",
            },
        },
        "handler": handle_tts_toggle,
    },
}
