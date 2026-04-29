"""Актуаторы: claw, head, arm, LED.

`head` и `arm` принимают либо угол(ы), либо preset (mutually exclusive),
поэтому params_schema=None — UI рисовать форму не должен. Доступно через
прямой API-вызов. Для UI-friendly альтернативы см. head_angle / arm_preset.
"""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, bad_params, from_http, opt_param


def handle_claw(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"state": "open"|"close"}."""
    state = opt_param(params, "state")
    if state not in {"open", "close"}:
        return bad_params("state: open|close")
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
            return bad_params("angle должен быть целым числом")
    elif "preset" in params:
        body = {"preset": str(params["preset"])}
    else:
        return bad_params("укажи angle (int) или preset (str)")
    return from_http(ctx.client.post("/api/v1/actuators/head", json_body=body))


def handle_head_angle(params: Mapping[str, Any], ctx) -> HandlerResult:
    """UI-friendly: только угол."""
    angle = opt_param(params, "angle")
    if angle is None:
        return bad_params("angle: обязательный")
    try:
        angle_int = int(angle)
    except (TypeError, ValueError):
        return bad_params("angle должен быть целым числом")
    return from_http(
        ctx.client.post("/api/v1/actuators/head", json_body={"angle": angle_int})
    )


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
                    return bad_params(f"{k} должен быть целым числом")
        if not body:
            return bad_params("укажи preset или хотя бы один j1..j4")
    return from_http(ctx.client.post("/api/v1/actuators/arm", json_body=body))


def handle_arm_preset(params: Mapping[str, Any], ctx) -> HandlerResult:
    """UI-friendly: только имя пресета руки."""
    preset = opt_param(params, "preset")
    if not preset:
        return bad_params("preset: обязательный")
    return from_http(
        ctx.client.post(
            "/api/v1/actuators/arm", json_body={"preset": str(preset)}
        )
    )


def handle_arm_presets(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/actuators/arm/presets"))


def handle_head_presets(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/actuators/head/presets"))


def handle_led(params: Mapping[str, Any], ctx) -> HandlerResult:
    """LED-команда (пробрасываем как есть)."""
    if not isinstance(params, dict):
        return bad_params("LED: ожидается объект параметров")
    return from_http(ctx.client.post("/api/v1/led/command", json_body=dict(params)))


def handle_led_mode(params: Mapping[str, Any], ctx) -> HandlerResult:
    """UI-friendly: один параметр — режим."""
    mode = opt_param(params, "mode")
    if not mode:
        return bad_params("mode: обязательный")
    return from_http(
        ctx.client.post("/api/v1/led/command", json_body={"mode": str(mode)})
    )


COMMANDS = {
    "claw": {
        "description": "Клешня: открыть или закрыть",
        "params_schema": {
            "state": {
                "type": "string",
                "enum": ["open", "close"],
                "description": "Состояние клешни",
            },
        },
        "handler": handle_claw,
    },
    "head": {
        "description": "Голова (pan): задать angle ИЛИ preset (через API; для UI см. head_angle / head_preset)",
        "params_schema": None,
        "handler": handle_head,
    },
    "head_angle": {
        "description": "Голова (pan): угол в градусах",
        "params_schema": {
            "angle": {
                "type": "integer",
                "minimum": -90,
                "maximum": 90,
                "default": 0,
                "description": "Угол поворота головы, °",
            },
        },
        "handler": handle_head_angle,
    },
    "arm": {
        "description": "Рука 4-DOF: задать j1..j4 ИЛИ preset (через API; для UI см. arm_preset)",
        "params_schema": None,
        "handler": handle_arm,
    },
    "arm_preset": {
        "description": "Рука 4-DOF: применить пресет (имя из arm_presets)",
        "params_schema": {
            "preset": {
                "type": "string",
                "description": "Имя пресета (e.g. home)",
            },
        },
        "handler": handle_arm_preset,
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
        "description": "LED-команда (произвольный объект параметров; для UI см. led_mode)",
        "params_schema": None,
        "handler": handle_led,
    },
    "led_mode": {
        "description": "LED: режим",
        "params_schema": {
            "mode": {
                "type": "string",
                "enum": ["off", "solid", "blink", "rainbow"],
                "description": "Режим подсветки",
            },
        },
        "handler": handle_led_mode,
    },
}
