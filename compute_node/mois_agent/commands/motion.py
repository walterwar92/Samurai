"""Движение робота: drive/stop/e-stop/reset/speed-profile."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, bad_params, from_http, opt_param


def handle_drive(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"linear": float, "angular": float} — задать скорость."""
    try:
        linear = float(opt_param(params, "linear", 0.0))
        angular = float(opt_param(params, "angular", 0.0))
    except (TypeError, ValueError) as exc:
        return bad_params(f"linear/angular должны быть числами: {exc}")
    return from_http(
        ctx.client.post(
            "/api/v1/robot/velocity",
            json_body={"linear": linear, "angular": angular},
        )
    )


def handle_stop(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.post("/api/v1/robot/stop"))


def handle_emergency_stop(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.post("/api/v1/emergency_stop"))


def handle_reset_position(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.post("/api/v1/robot/reset_position"))


def handle_set_speed_profile(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"profile": "slow"|"normal"|"fast"}."""
    profile = opt_param(params, "profile")
    if profile not in {"slow", "normal", "fast"}:
        return bad_params("profile: slow|normal|fast")
    return from_http(
        ctx.client.post("/api/v1/speed_profile", json_body={"profile": profile})
    )


def handle_get_speed_profile(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/speed_profile"))


COMMANDS = {
    "drive": {
        "description": "Задать скорость робота",
        "params_schema": {
            "linear": {
                "type": "number",
                "minimum": -0.5,
                "maximum": 0.5,
                "default": 0.0,
                "description": "Линейная скорость, м/с",
            },
            "angular": {
                "type": "number",
                "minimum": -2.0,
                "maximum": 2.0,
                "default": 0.0,
                "description": "Угловая скорость, рад/с",
            },
        },
        "handler": handle_drive,
    },
    "stop": {
        "description": "Плавно остановить движение",
        "params_schema": None,
        "handler": handle_stop,
    },
    "emergency_stop": {
        "description": "Аварийный стоп (E-stop)",
        "params_schema": None,
        "handler": handle_emergency_stop,
    },
    "reset_position": {
        "description": "Сбросить одометрию робота в (0, 0, 0)",
        "params_schema": None,
        "handler": handle_reset_position,
    },
    "set_speed_profile": {
        "description": "Профиль скорости",
        "params_schema": {
            "profile": {
                "type": "string",
                "enum": ["slow", "normal", "fast"],
                "default": "normal",
                "description": "Профиль скорости",
            },
        },
        "handler": handle_set_speed_profile,
    },
    "get_speed_profile": {
        "description": "Получить активный профиль скорости",
        "params_schema": None,
        "handler": handle_get_speed_profile,
    },
}
