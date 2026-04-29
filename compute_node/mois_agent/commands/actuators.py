"""Актуаторы: claw, head, arm, LED."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, err, from_http, opt_param


def handle_claw(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"state": "open"|"close"}."""
    state = opt_param(params, "state")
    if state not in {"open", "close"}:
        return err("state: open|close")
    return from_http(
        ctx.client.post("/api/v1/actuators/claw", json_body={"state": state})
    )


def handle_head(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"angle": int} или {"preset": "center"|...}."""
    body: dict
    if "angle" in params:
        try:
            body = {"angle": int(params["angle"])}
        except (TypeError, ValueError):
            return err("angle должен быть целым числом")
    elif "preset" in params:
        body = {"preset": str(params["preset"])}
    else:
        return err("укажи angle (int) или preset (str)")
    return from_http(ctx.client.post("/api/v1/actuators/head", json_body=body))


def handle_arm(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"j1":..,"j2":..,"j3":..,"j4":..} или {"preset": "home"|...}."""
    if "preset" in params:
        body = {"preset": str(params["preset"])}
    else:
        body = {}
        for k in ("j1", "j2", "j3", "j4"):
            if k in params:
                try:
                    body[k] = int(params[k])
                except (TypeError, ValueError):
                    return err(f"{k} должен быть целым числом")
        if not body:
            return err("укажи preset или хотя бы один j1..j4")
    return from_http(ctx.client.post("/api/v1/actuators/arm", json_body=body))


def handle_arm_presets(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/actuators/arm/presets"))


def handle_head_presets(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/actuators/head/presets"))


def handle_led(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"mode": "off|solid|blink|...", ...} — пробрасываем как есть."""
    if not isinstance(params, dict):
        return err("LED: ожидается объект параметров")
    return from_http(ctx.client.post("/api/v1/led/command", json_body=dict(params)))


COMMANDS = {
    "claw": {
        "description": "Клешня: open или close",
        "params_schema": {
            "type": "object",
            "properties": {"state": {"enum": ["open", "close"]}},
            "required": ["state"],
        },
        "handler": handle_claw,
    },
    "head": {
        "description": "Голова (pan): angle (deg) или preset name",
        "params_schema": {
            "type": "object",
            "properties": {
                "angle": {"type": "integer", "minimum": -90, "maximum": 90},
                "preset": {"type": "string"},
            },
        },
        "handler": handle_head,
    },
    "arm": {
        "description": "Рука 4-DOF: j1..j4 (deg) или preset (e.g. home)",
        "params_schema": {
            "type": "object",
            "properties": {
                "j1": {"type": "integer"},
                "j2": {"type": "integer"},
                "j3": {"type": "integer"},
                "j4": {"type": "integer"},
                "preset": {"type": "string"},
            },
        },
        "handler": handle_arm,
    },
    "arm_presets": {
        "description": "Список пресетов руки",
        "params_schema": None,
        "handler": handle_arm_presets,
    },
    "head_presets": {
        "description": "Список пресетов головы",
        "params_schema": None,
        "handler": handle_head_presets,
    },
    "led": {
        "description": "LED-команда (произвольный объект параметров)",
        "params_schema": {"type": "object"},
        "handler": handle_led,
    },
}
