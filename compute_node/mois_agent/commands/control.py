"""Высокоуровневое управление: patrol, follow_me, path_recorder, detection toggle."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, err, from_http, opt_param


def handle_patrol(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"command": "start"|"stop"}."""
    cmd = opt_param(params, "command")
    if cmd not in {"start", "stop"}:
        return err("command: start|stop")
    return from_http(
        ctx.client.post("/api/v1/patrol/command", json_body={"command": cmd})
    )


def handle_patrol_waypoints(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"waypoints": [[x,y], ...]} — задать маршрут патруля."""
    waypoints = opt_param(params, "waypoints")
    if not isinstance(waypoints, list):
        return err("waypoints: массив [[x,y], ...]")
    return from_http(
        ctx.client.post(
            "/api/v1/patrol/waypoints", json_body={"waypoints": waypoints}
        )
    )


def handle_follow_me(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"enabled": bool}."""
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return err("enabled: bool")
    return from_http(
        ctx.client.post("/api/v1/follow_me", json_body={"enabled": bool(enabled)})
    )


def handle_path_recorder(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"command": "start"|"stop"|"play", "name": optional}."""
    cmd = opt_param(params, "command")
    if cmd not in {"start", "stop", "play"}:
        return err("command: start|stop|play")
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
        return err("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/detection/toggle", json_body={"enabled": bool(enabled)}
        )
    )


def handle_obstacle_avoidance_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return err("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/obstacle_avoidance/toggle",
            json_body={"enabled": bool(enabled)},
        )
    )


def handle_collision_guard_toggle(params: Mapping[str, Any], ctx) -> HandlerResult:
    enabled = opt_param(params, "enabled")
    if enabled is None:
        return err("enabled: bool")
    return from_http(
        ctx.client.post(
            "/api/v1/collision_guard/toggle",
            json_body={"enabled": bool(enabled)},
        )
    )


COMMANDS = {
    "patrol": {
        "description": "Патруль: start/stop по списку waypoints",
        "params_schema": {
            "type": "object",
            "properties": {"command": {"enum": ["start", "stop"]}},
            "required": ["command"],
        },
        "handler": handle_patrol,
    },
    "patrol_waypoints": {
        "description": "Задать waypoints патруля",
        "params_schema": {
            "type": "object",
            "properties": {
                "waypoints": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "number"}},
                }
            },
            "required": ["waypoints"],
        },
        "handler": handle_patrol_waypoints,
    },
    "follow_me": {
        "description": "Follow-me режим on/off",
        "params_schema": {
            "type": "object",
            "properties": {"enabled": {"type": "boolean"}},
            "required": ["enabled"],
        },
        "handler": handle_follow_me,
    },
    "path_recorder": {
        "description": "Запись/воспроизведение маршрута: start|stop|play",
        "params_schema": {
            "type": "object",
            "properties": {
                "command": {"enum": ["start", "stop", "play"]},
                "name": {"type": "string"},
            },
            "required": ["command"],
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
            "type": "object",
            "properties": {"enabled": {"type": "boolean"}},
            "required": ["enabled"],
        },
        "handler": handle_detection_toggle,
    },
    "obstacle_avoidance_toggle": {
        "description": "Вкл/выкл объезд препятствий",
        "params_schema": {
            "type": "object",
            "properties": {"enabled": {"type": "boolean"}},
            "required": ["enabled"],
        },
        "handler": handle_obstacle_avoidance_toggle,
    },
    "collision_guard_toggle": {
        "description": "Вкл/выкл защиту от столкновений (US)",
        "params_schema": {
            "type": "object",
            "properties": {"enabled": {"type": "boolean"}},
            "required": ["enabled"],
        },
        "handler": handle_collision_guard_toggle,
    },
}
