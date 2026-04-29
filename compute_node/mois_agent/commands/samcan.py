"""Samcan (второй робот, Arduino Uno).

Dashboard проксирует /api/v1/samcan/* → http://127.0.0.1:5005/*. Если
samcan_url не задан в конфиге — используем dashboard как прокси.
Если задан — стучим напрямую (для случая когда dashboard выключен).
"""
from __future__ import annotations

from typing import Any, Mapping

from ..client import DashboardClient
from ._utils import HandlerResult, bad_params, from_http, opt_param

# Список команд Arduino — для UI-плашек
SAMCAN_BASIC_CMDS = [
    "F", "B", "L", "R", "S",  # движение
    "O", "X", "G",            # клешня: Open/X-close/Grab
    "P",                      # park
    "D", "K", "C", "Z", "H",  # detach/kick/calibrate/zero/help
]

SAMCAN_SCENARIOS = [
    "fwd_stop", "fwd_back", "square", "wiggle", "open_close", "grab_demo",
]

SAMCAN_PRESETS = ["park", "forward", "grab"]


def _samcan(ctx) -> tuple[DashboardClient, str]:
    """Вернуть (client, url_prefix) для samcan-вызовов."""
    if ctx.samcan_client is not None:
        return ctx.samcan_client, ""  # прямой клиент → пути без /api/v1/samcan
    return ctx.client, "/api/v1/samcan"


def handle_samcan_cmd(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"cmd": "F"|"B"|..., "arg": optional int}."""
    cmd = opt_param(params, "cmd")
    if not cmd or not isinstance(cmd, str):
        return bad_params("cmd: обязательная строка (F/B/L/R/S/O/X/G/...)")
    body: dict = {"cmd": cmd}
    if "arg" in params and params["arg"] not in ("", None):
        body["arg"] = params["arg"]
    client, prefix = _samcan(ctx)
    return from_http(client.post(f"{prefix}/api/samcan/cmd", json_body=body))


def handle_samcan_scenario(params: Mapping[str, Any], ctx) -> HandlerResult:
    scenario = opt_param(params, "scenario")
    if not scenario:
        return bad_params("scenario: обязательно")
    client, prefix = _samcan(ctx)
    return from_http(
        client.post(
            f"{prefix}/api/samcan/scenario", json_body={"scenario": str(scenario)}
        )
    )


def handle_samcan_state(params, ctx) -> HandlerResult:
    client, prefix = _samcan(ctx)
    return from_http(client.get(f"{prefix}/api/samcan/state"))


def handle_samcan_log(params: Mapping[str, Any], ctx) -> HandlerResult:
    lines = opt_param(params, "lines", 50)
    try:
        lines_int = int(lines)
    except (TypeError, ValueError):
        return bad_params("lines должен быть числом")
    client, prefix = _samcan(ctx)
    return from_http(
        client.get(f"{prefix}/api/samcan/log", params={"lines": lines_int})
    )


def handle_samcan_scenarios(params, ctx) -> HandlerResult:
    client, prefix = _samcan(ctx)
    return from_http(client.get(f"{prefix}/api/samcan/scenarios"))


def handle_samcan_diag(params, ctx) -> HandlerResult:
    client, prefix = _samcan(ctx)
    return from_http(client.get(f"{prefix}/api/samcan/diag"))


def handle_samcan_preset_apply(params: Mapping[str, Any], ctx) -> HandlerResult:
    preset = opt_param(params, "preset")
    if not preset:
        return bad_params("preset: park|forward|grab")
    client, prefix = _samcan(ctx)
    return from_http(
        client.post(
            f"{prefix}/api/samcan/preset/apply", json_body={"preset": str(preset)}
        )
    )


COMMANDS = {
    "samcan_cmd": {
        "description": "Samcan: одиночная Serial-команда (F/B/L/R/S/O/X/G/...)",
        "params_schema": {
            "cmd": {
                "type": "string",
                "enum": SAMCAN_BASIC_CMDS,
                "description": "Код команды Arduino",
            },
        },
        "handler": handle_samcan_cmd,
    },
    "samcan_scenario": {
        "description": "Samcan: запустить сценарий",
        "params_schema": {
            "scenario": {
                "type": "string",
                "enum": SAMCAN_SCENARIOS,
                "description": "Имя сценария",
            },
        },
        "handler": handle_samcan_scenario,
    },
    "samcan_state": {
        "description": "Samcan: телеметрия + статус соединения",
        "params_schema": None,
        "handler": handle_samcan_state,
    },
    "samcan_log": {
        "description": "Samcan: последние строки Serial-лога",
        "params_schema": {
            "lines": {
                "type": "integer",
                "minimum": 1,
                "maximum": 500,
                "default": 50,
                "description": "Сколько последних строк",
            },
        },
        "handler": handle_samcan_log,
    },
    "samcan_scenarios": {
        "description": "Список доступных Samcan сценариев",
        "params_schema": None,
        "handler": handle_samcan_scenarios,
    },
    "samcan_diag": {
        "description": "Samcan: диагностика портов и счётчиков",
        "params_schema": None,
        "handler": handle_samcan_diag,
    },
    "samcan_preset_apply": {
        "description": "Применить пресет Samcan",
        "params_schema": {
            "preset": {
                "type": "string",
                "enum": SAMCAN_PRESETS,
                "description": "Имя пресета",
            },
        },
        "handler": handle_samcan_preset_apply,
    },
}
