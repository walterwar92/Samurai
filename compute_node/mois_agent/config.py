"""Конфиг MOIS-агента.

Источники (приоритет ↓):
  1. CLI-флаги (--api-url, --api-token, --dashboard, --robot-id, ...)
  2. Переменные окружения (MOIS_API_URL, MOIS_API_TOKEN, ...)
  3. Файл config.json рядом с пакетом
  4. Дефолты (только для интервалов и dashboard URL)

api_url и api_token обязательны — без них падаем с понятной ошибкой.
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("mois.config")

DEFAULT_DASHBOARD = "http://127.0.0.1:5000"
DEFAULT_POLL = 10
DEFAULT_TELEMETRY = 30
DEFAULT_REQUEST_TIMEOUT = 10.0


@dataclass(frozen=True)
class AgentConfig:
    api_url: str
    api_token: str
    dashboard_url: str = DEFAULT_DASHBOARD
    robot_id: str = "robot1"
    poll_interval: int = DEFAULT_POLL
    telemetry_interval: int = DEFAULT_TELEMETRY
    request_timeout: float = DEFAULT_REQUEST_TIMEOUT
    samcan_url: Optional[str] = None  # None → samcan через dashboard /api/v1/samcan/*
    agent_name: str = "samurai-mois"


def _load_file(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            LOG.warning("config.json не объект, игнорирую")
            return {}
        return data
    except (OSError, json.JSONDecodeError) as exc:
        LOG.warning("config.json не прочитан: %s", exc)
        return {}


def load_config(
    config_path: Optional[Path] = None,
    cli_overrides: Optional[dict] = None,
) -> AgentConfig:
    """Слить файл, ENV и CLI в AgentConfig.

    Бросает RuntimeError если api_url или api_token пусты.
    """
    if config_path is None:
        config_path = Path(__file__).parent / "config.json"

    file_cfg = _load_file(config_path)
    env_cfg = {
        "api_url": os.environ.get("MOIS_API_URL"),
        "api_token": os.environ.get("MOIS_API_TOKEN"),
        "dashboard_url": os.environ.get("MOIS_DASHBOARD_URL"),
        "robot_id": os.environ.get("MOIS_ROBOT_ID"),
        "poll_interval": os.environ.get("MOIS_POLL_INTERVAL"),
        "telemetry_interval": os.environ.get("MOIS_TELEMETRY_INTERVAL"),
        "samcan_url": os.environ.get("MOIS_SAMCAN_URL"),
        "agent_name": os.environ.get("MOIS_AGENT_NAME"),
    }
    cli = cli_overrides or {}

    def pick(*keys: str) -> Optional[str]:
        for src in (cli, env_cfg, file_cfg):
            for k in keys:
                v = src.get(k)
                if v not in (None, ""):
                    return v
        return None

    api_url = pick("api_url")
    api_token = pick("api_token")
    if not api_url or not api_token:
        raise RuntimeError(
            "MOIS-агент: нужны api_url и api_token. Положи их в "
            f"{config_path} или передай через --api-url / --api-token "
            "(или MOIS_API_URL / MOIS_API_TOKEN env)."
        )

    def pick_int(default: int, *keys: str) -> int:
        v = pick(*keys)
        if v is None:
            return default
        try:
            return int(v)
        except (TypeError, ValueError):
            LOG.warning("Не int для %s: %r — беру дефолт %d", keys, v, default)
            return default

    return AgentConfig(
        api_url=str(api_url),
        api_token=str(api_token),
        dashboard_url=str(pick("dashboard_url") or DEFAULT_DASHBOARD).rstrip("/"),
        robot_id=str(pick("robot_id") or "robot1"),
        poll_interval=max(1, pick_int(DEFAULT_POLL, "poll_interval")),
        telemetry_interval=max(5, pick_int(DEFAULT_TELEMETRY, "telemetry_interval")),
        request_timeout=float(pick("request_timeout") or DEFAULT_REQUEST_TIMEOUT),
        samcan_url=(str(pick("samcan_url")).rstrip("/") if pick("samcan_url") else None),
        agent_name=str(pick("agent_name") or "samurai-mois"),
    )
