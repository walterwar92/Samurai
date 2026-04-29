"""Голос и FSM: voice text → fsm intent, принудительные переходы, TTS."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, err, from_http, opt_param


def handle_voice(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"text": "поедь вперёд"} — отправить как голосовую команду в FSM."""
    text = opt_param(params, "text")
    if not text or not isinstance(text, str):
        return err("text: обязательная строка")
    return from_http(
        ctx.client.post("/api/v1/fsm/command", json_body={"text": text})
    )


def handle_fsm_transition(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"state": "idle"|"hunt"|...} — принудительный переход (admin)."""
    state = opt_param(params, "state")
    if not state or not isinstance(state, str):
        return err("state: обязательная строка (имя FSM-состояния)")
    return from_http(
        ctx.client.post("/api/v1/fsm/transition", json_body={"state": state})
    )


def handle_tts_speak(params: Mapping[str, Any], ctx) -> HandlerResult:
    text = opt_param(params, "text")
    if not text or not isinstance(text, str):
        return err("text: обязательная строка")
    return from_http(
        ctx.client.post("/api/v1/tts/speak", json_body={"text": text})
    )


def handle_tts_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return err("enabled: bool (true/false)")
    return from_http(
        ctx.client.post("/api/v1/tts/toggle", json_body={"enabled": bool(enabled)})
    )


COMMANDS = {
    "voice": {
        "description": "Голосовая команда (текст) → FSM",
        "params_schema": {
            "type": "object",
            "properties": {"text": {"type": "string"}},
            "required": ["text"],
        },
        "handler": handle_voice,
    },
    "fsm_transition": {
        "description": "Принудительный переход FSM (admin)",
        "params_schema": {
            "type": "object",
            "properties": {"state": {"type": "string"}},
            "required": ["state"],
        },
        "handler": handle_fsm_transition,
    },
    "tts_speak": {
        "description": "Озвучить текст через TTS",
        "params_schema": {
            "type": "object",
            "properties": {"text": {"type": "string"}},
            "required": ["text"],
        },
        "handler": handle_tts_speak,
    },
    "tts_toggle": {
        "description": "Включить/выключить TTS",
        "params_schema": {
            "type": "object",
            "properties": {"enabled": {"type": "boolean"}},
            "required": ["enabled"],
        },
        "handler": handle_tts_toggle,
    },
}
