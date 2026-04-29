"""Карты SLAM: list/save/load + zones."""
from __future__ import annotations

from typing import Any, Mapping

from ._utils import HandlerResult, bad_params, from_http, opt_param


def handle_map_list(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/map/list"))


def handle_map_info(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/map/info"))


def handle_map_save(params: Mapping[str, Any], ctx) -> HandlerResult:
    """{"name": "..."}."""
    name = opt_param(params, "name")
    if not name:
        return bad_params("name: имя карты обязательно")
    return from_http(
        ctx.client.post("/api/v1/map/save", json_body={"name": str(name)})
    )


def handle_map_load(params: Mapping[str, Any], ctx) -> HandlerResult:
    name = opt_param(params, "name")
    if not name:
        return bad_params("name: имя карты обязательно")
    return from_http(
        ctx.client.post("/api/v1/map/load", json_body={"name": str(name)})
    )


def handle_zones_list(params, ctx) -> HandlerResult:
    return from_http(ctx.client.get("/api/v1/zones"))


def handle_zones_clear(params, ctx) -> HandlerResult:
    return from_http(ctx.client.post("/api/v1/zones/clear"))


COMMANDS = {
    "map_list": {
        "description": "Список сохранённых SLAM-карт",
        "params_schema": None,
        "handler": handle_map_list,
    },
    "map_info": {
        "description": "Метаданные текущей карты",
        "params_schema": None,
        "handler": handle_map_info,
    },
    "map_save": {
        "description": "Сохранить текущую SLAM-карту",
        "params_schema": {
            "name": {
                "type": "string",
                "maxLength": 64,
                "description": "Имя карты",
            },
        },
        "handler": handle_map_save,
    },
    "map_load": {
        "description": "Загрузить ранее сохранённую карту",
        "params_schema": {
            "name": {
                "type": "string",
                "maxLength": 64,
                "description": "Имя карты",
            },
        },
        "handler": handle_map_load,
    },
    "zones_list": {
        "description": "Geo-fence: список запретных зон",
        "params_schema": None,
        "handler": handle_zones_list,
    },
    "zones_clear": {
        "description": "Geo-fence: очистить все зоны",
        "params_schema": None,
        "handler": handle_zones_clear,
    },
}
