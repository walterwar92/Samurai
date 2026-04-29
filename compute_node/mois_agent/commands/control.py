"""Высокоуровневое управление: patrol, follow_me, path_recorder, detection toggle."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, bad_params, from_http, opt_param


def handle_patrol(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"command": "start"|"stop"}."""
    cmd = opt_param(params, "command")
    if cmd not in {"start", "stop"}:
        return bad_params("command: start|stop")
    return from_http(
        ctx.client.post("/api/v1/patrol/command", json_body={"command": cmd})
    )


def handle_patrol_waypoints(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"waypoints": [[x,y], ...]} — задать маршрут патруля."""
    waypoints = opt_param(params, "waypoints")
    if not isinstance(waypoints, list):
        return bad_params("waypoints: массив [[x,y], ...]")
    return from_http(
        ctx.client.post(
            "/api/v1/patrol/waypoints", json_body={"waypoints": waypoints}
        )
    )


def handle_follow_me(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"enabled": bool}."""
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return bad_params("enabled: bool")
    return from_http(
        ctx.client.post("/api/v1/follow_me", json_body={"enabled": bool(enabled)})
    )


def handle_path_recorder(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"command": "start"|"stop"|"play", "name": optional}."""
    cmd = opt_param(params, "command")
    if cmd not in {"start", "stop", "play"}:
        return bad_params("command: start|stop|play")
    body = {"command": cmd}
    name = opt_param(params, "name")
    if name:
        body["name"] = str(name)
    return from_http(
        ctx.client.post("/api/v1/path_recorder/command", json_body=body)
    )


def handle_path_recorder_status(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/path_recorder/status"))


def handle_path_recorder_list(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/path_recorder/list"))


def handle_detection_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"enabled": bool}."""
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return bad_params("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/detection/toggle", json_body={"enabled": bool(enabled)}
        )
    )


def handle_obstacle_avoidance_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return bad_params("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/obstacle_avoidance/toggle",
            json_body={"enabled": bool(enabled)},
        )
    )


def handle_collision_guard_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return bad_params("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/collision_guard/toggle",
            json_body={"enabled": bool(enabled)},
        )
    )


COMMANDS = {
    "patrol": {
        "description": "Патруль: start|stop по последним waypoints",
        "params_schema": {
            "command": {
                "type": "string",
                "enum": ["start", "stop"],
                "description": "Команда патруля",
            },
        },
        "handler": handle_patrol,
    },
    "patrol_waypoints": {
        "description": "Задать список точек патрулирования (массив [[x,y]...] — через API)",
        "params_schema": None,
        "handler": handle_patrol_waypoints,
    },
    "follow_me": {
        "description": "Follow-me режим on/off",
        "params_schema": {
            "enabled": {
                "type": "boolean",
                "description": "Включить follow-me",
            },
        },
        "handler": handle_follow_me,
    },
    "path_recorder": {
        "description": "Запись/воспроизведение маршрута",
        "params_schema": {
            "command": {
                "type": "string",
                "enum": ["start", "stop", "play"],
                "description": "Действие: start|stop|play",
            },
            "name": {
                "type": "string",
                "maxLength": 64,
                "description": "Имя маршрута (для start/play)",
            },
        },
        "handler": handle_path_recorder,
    },
    "path_recorder_status": {
        "description": "Статус записи маршрута",
        "params_schema": None,
        "handler": handle_path_recorder_status,
    },
    "path_recorder_list": {
        "description": "Список сохранённых маршрутов",
        "params_schema": None,
        "handler": handle_path_recorder_list,
    },
    "detection_toggle": {
        "description": "Вкл/выкл YOLO-детектор",
        "params_schema": {
            "enabled": {
                "type": "boolean",
                "description": "Включить детектор",
            },
        },
        "handler": handle_detection_toggle,
    },
    "obstacle_avoidance_toggle": {
        "description": "Вкл/выкл объезд препятствий",
        "params_schema": {
            "enabled": {
                "type": "boolean",
                "description": "Включить объезд",
            },
        },
        "handler": handle_obstacle_avoidance_toggle,
    },
    "collision_guard_toggle": {
        "description": "Вкл/выкл защиту от столкновений (US)",
        "params_schema": {
            "enabled": {
                "type": "boolean",
                "description": "Включить защиту",
            },
        },
        "handler": handle_collision_guard_toggle,
    },
}
