"""Хелперы для handlers команд.

`HandlerResult` — это единый формат ответа агента: (exit_code, stdout, stderr).
Любой handler `(params: dict, ctx) -> HandlerResult` либо bytes-like dict.

`ctx` — простой namespace с: client (DashboardClient), config (AgentConfig),
samcan_client (DashboardClient или None), state (общий dict для кеша
телеметрии).
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Mapping, Optional

from ..client import HTTPResult


@dataclass
class HandlerResult:
    exit_code: int = 0
    stdout: str = ""
    stderr: str = ""

    def to_dict(self) -> dict:
        return {
            "exit_code": int(self.exit_code),
            "stdout": str(self.stdout),
            "stderr": str(self.stderr),
        }


def ok(payload: Any = "OK") -> HandlerResult:
    if isinstance(payload, str):
        return HandlerResult(0, payload, "")
    return HandlerResult(0, json.dumps(payload, ensure_ascii=False), "")


def err(message: str, exit_code: int = 1) -> HandlerResult:
    return HandlerResult(int(exit_code), "", str(message))


def from_http(result: HTTPResult, *, success_payload: Any = None) -> HandlerResult:
    """Преобразовать HTTP-ответ в HandlerResult.

    success_payload (если задан) идёт в stdout вместо тела.
    """
    if result.ok:
        if success_payload is not None:
            return ok(success_payload)
        return ok(result.body if result.body is not None else "OK")
    body_str = ""
    if result.body is not None:
        body_str = (
            result.body
            if isinstance(result.body, str)
            else json.dumps(result.body, ensure_ascii=False)
        )
    msg = result.error or "request failed"
    if body_str:
        msg = f"{msg}: {body_str}"
    return err(msg, exit_code=result.status or 1)


def need_param(params: Mapping[str, Any], key: str) -> Any:
    """Вернёт значение или бросит KeyError с понятным текстом."""
    if key not in params:
        raise KeyError(f"missing required param: {key}")
    return params[key]


def opt_param(
    params: Mapping[str, Any], key: str, default: Any = None
) -> Optional[Any]:
    return params.get(key, default)
