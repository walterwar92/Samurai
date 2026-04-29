"""Сбор телеметрии для periodic POST ?action=telemetry.

Дёргаем dashboard /api/v1/status (он уже агрегирует pose, fsm, sensors,
battery, mqtt connected). Если запрос упал — отправляем минимальный
объект {"online": false, "error": "..."} чтобы сайт видел что-то.
"""
from __future__ import annotations

import logging
import time
from typing import Any

LOG = logging.getLogger("mois.telemetry")


def _safe_get(obj: Any, *path: str, default: Any = None) -> Any:
    cur = obj
    for k in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
        if cur is None:
            return default
    return cur


def collect_telemetry(ctx) -> dict:
    """Вернёт компактный dict с ключевыми метриками робота."""
    started = time.time()
    r = ctx.client.get("/api/v1/status")
    if not r.ok:
        return {
            "online": False,
            "error": r.error or f"HTTP {r.status}",
            "ts": int(started),
        }

    body = r.body if isinstance(r.body, dict) else {}
    out = {
        "online": True,
        "ts": int(started),
        "robot_id": ctx.config.robot_id,
        "battery_pct": _safe_get(body, "battery", "percent"),
        "temperature_c": _safe_get(body, "temperature", "celsius"),
        "pose": _safe_get(body, "pose"),
        "velocity": _safe_get(body, "velocity"),
        "fsm_state": _safe_get(body, "fsm", "state"),
        "speed_profile": _safe_get(body, "speed_profile", "active"),
        "mqtt_connected": _safe_get(body, "mqtt", "connected"),
        "ultrasonic_m": _safe_get(body, "sensors", "ultrasonic", "distance_m"),
        "imu_yaw_deg": _safe_get(body, "sensors", "imu", "yaw_deg"),
        "detection_enabled": _safe_get(body, "detection", "enabled"),
        "detections_count": _safe_get(body, "detection", "count"),
    }
    return {k: v for k, v in out.items() if v is not None or k in ("online", "ts")}
