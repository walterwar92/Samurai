"""Базовые команды: ping, info, dashboard health-check."""
from __future__ import annotations

import platform
import socket
import time
from typing import Any, Mapping

from ._utils import HandlerResult, err, ok


def handle_ping(params: Mapping[str, Any], ctx) -> HandlerResult:
    return ok({"pong": True, "ts": int(time.time())})


def handle_info(params: Mapping[str, Any], ctx) -> HandlerResult:
    info = {
        "agent": ctx.config.agent_name,
        "robot_id": ctx.config.robot_id,
        "host": socket.gethostname(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "dashboard_url": ctx.config.dashboard_url,
        "poll_interval": ctx.config.poll_interval,
        "telemetry_interval": ctx.config.telemetry_interval,
    }
    return ok(info)


def handle_dashboard_health(params: Mapping[str, Any], ctx) -> HandlerResult:
    """Проверить что dashboard жив (GET /api/v1/status)."""
    r = ctx.client.get("/api/v1/status")
    if r.ok:
        return ok({"dashboard": "ok", "status": r.body})
    return err(f"dashboard {ctx.config.dashboard_url} недоступен: {r.error}")


COMMANDS = {
    "ping": {
        "description": "Проверка связи (агент жив)",
        "params_schema": None,
        "handler": handle_ping,
    },
    "info": {
        "description": "Информация об агенте и хосте",
        "params_schema": None,
        "handler": handle_info,
    },
    "dashboard_health": {
        "description": "Жив ли локальный dashboard FastAPI :5000",
        "params_schema": None,
        "handler": handle_dashboard_health,
    },
}
