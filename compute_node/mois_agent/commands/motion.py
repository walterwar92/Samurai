"""Движение робота: drive/stop/e-stop/reset/speed-profile."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, err, from_http, opt_param


def handle_drive(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"linear": float, "angular": float} — задать скорость."""
    try:
        linear = float(opt_param(params, "linear", 0.0))
        angular = float(opt_param(params, "angular", 0.0))
    except (TypeError, ValueError) as exc:
        return err(f"linear/angular должны быть числами: {exc}")
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
        return err("profile: slow|normal|fast")
    return from_http(
        ctx.client.post("/api/v1/speed_profile", json_body={"profile": profile})
    )


def handle_get_speed_profile(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/speed_profile"))


COMMANDS = {
    "drive": {
        "description": "Задать скорость робота: linear (м/с), angular (рад/с)",
        "params_schema": {
            "type": "object",
            "properties": {
                "linear": {"type": "number", "default": 0.0},
                "angular": {"type": "number", "default": 0.0},
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
        "description": "Профиль скорости: slow|normal|fast",
        "params_schema": {
            "type": "object",
            "properties": {
                "profile": {"enum": ["slow", "normal", "fast"]},
            },
            "required": ["profile"],
        },
        "handler": handle_set_speed_profile,
    },
    "get_speed_profile": {
        "description": "Получить активный профиль скорости",
        "params_schema": None,
        "handler": handle_get_speed_profile,
    },
}
