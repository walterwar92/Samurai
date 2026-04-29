"""Read-only команды состояния: pose, sensors, battery, fsm, detections."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, from_http, opt_param


def handle_get_status(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/status"))


def handle_get_pose(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/robot/pose"))


def handle_get_velocity(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/robot/velocity"))


def handle_get_battery(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/battery"))


def handle_get_temperature(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/temperature"))


def handle_get_sensors(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/sensors"))


def handle_get_ultrasonic(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/sensors/ultrasonic"))


def handle_get_imu(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/sensors/imu"))


def handle_get_fsm(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/fsm"))


def handle_get_detections(params: Mapping[str, Any], ctx) -> HandlerResult:
    color = opt_param(params, "color")
    if color:
        return from_http(
            ctx.client.get("/api/v1/detection/closest", params={"color": color})
        )
    return from_http(ctx.client.get("/api/v1/detection"))


def handle_get_actuators(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/actuators"))


def handle_get_mqtt(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/mqtt/status"))


def handle_get_log(params: Mapping[str, Any], ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/log"))


COMMANDS = {
    "get_status": {
        "description": "Полный snapshot робота (pose, velocity, sensors, fsm, ...)",
        "params_schema": None,
        "handler": handle_get_status,
    },
    "get_pose": {
        "description": "Текущая поза {x, y, yaw}",
        "params_schema": None,
        "handler": handle_get_pose,
    },
    "get_velocity": {
        "description": "Текущая скорость {linear, angular}",
        "params_schema": None,
        "handler": handle_get_velocity,
    },
    "get_battery": {
        "description": "Уровень батареи (%)",
        "params_schema": None,
        "handler": handle_get_battery,
    },
    "get_temperature": {
        "description": "Температура CPU (°C)",
        "params_schema": None,
        "handler": handle_get_temperature,
    },
    "get_sensors": {
        "description": "Все сенсоры (US + IMU)",
        "params_schema": None,
        "handler": handle_get_sensors,
    },
    "get_ultrasonic": {
        "description": "Дистанция от УЗ-сенсора (м)",
        "params_schema": None,
        "handler": handle_get_ultrasonic,
    },
    "get_imu": {
        "description": "IMU: accel/gyro/yaw",
        "params_schema": None,
        "handler": handle_get_imu,
    },
    "get_fsm": {
        "description": "Текущее состояние FSM (idle/hunt/...)",
        "params_schema": None,
        "handler": handle_get_fsm,
    },
    "get_detections": {
        "description": "YOLO-детекции; параметр color → ближайший по цвету",
        "params_schema": {
            "type": "object",
            "properties": {
                "color": {
                    "type": "string",
                    "enum": ["red", "orange", "yellow", "green", "blue", "white", "black"],
                },
            },
        },
        "handler": handle_get_detections,
    },
    "get_actuators": {
        "description": "Состояние claw/head/arm",
        "params_schema": None,
        "handler": handle_get_actuators,
    },
    "get_mqtt": {
        "description": "Статус MQTT-соединения dashboard ↔ Pi",
        "params_schema": None,
        "handler": handle_get_mqtt,
    },
    "get_log": {
        "description": "Последние события и голосовые команды",
        "params_schema": None,
        "handler": handle_get_log,
    },
}
