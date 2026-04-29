"""Реестр команд MOIS-агента.

Каждая категория экспортирует свой словарь COMMANDS — здесь склеиваем.
Если команды конфликтуют по имени — позже-зарегистрированная перезатрёт
раннюю; в проде такого быть не должно, поэтому warn-логируем дубль.
"""
from __future__ import annotations

import logging

from . import (
    actuators,
    basic,
    control,
    maps,
    motion,
    samcan,
    state,
    voice,
)

LOG = logging.getLogger("mois.commands")


def _merge(*registries: dict) -> dict:
    out: dict = {}
    for reg in registries:
        for name, spec in reg.items():
            if name in out:
                LOG.warning("Конфликт имени команды: %s (перезаписываем)", name)
            out[name] = spec
    return out


COMMANDS = _merge(
    basic.COMMANDS,
    motion.COMMANDS,
    state.COMMANDS,
    actuators.COMMANDS,
    voice.COMMANDS,
    control.COMMANDS,
    maps.COMMANDS,
    samcan.COMMANDS,
)


def list_capabilities() -> list[dict]:
    """Сериализация всех команд для POST ?action=capabilities."""
    out = []
    for name, spec in COMMANDS.items():
        entry: dict = {"name": name}
        if spec.get("description"):
            entry["description"] = spec["description"]
        if spec.get("params_schema"):
            entry["params_schema"] = spec["params_schema"]
        out.append(entry)
    return out
