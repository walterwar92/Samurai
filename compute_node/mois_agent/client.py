"""HTTP-клиент к dashboard FastAPI (:5000) — обёртка над httpx.

Все handlers команд используют `DashboardClient` для вызова локальных
REST endpoints. Если dashboard выключен — возвращаем структурированную
ошибку (а не падаем), чтобы агент мог сообщить о ней на сайт.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import httpx

LOG = logging.getLogger("mois.client")


@dataclass
class HTTPResult:
    """Результат REST-вызова, удобно мапится в exit_code/stdout/stderr."""

    ok: bool
    status: int
    body: Any
    error: Optional[str] = None

    @property
    def exit_code(self) -> int:
        return 0 if self.ok else 1


class DashboardClient:
    """Тонкая обёртка над httpx.Client для нашего dashboard.

    Базовый url задаётся при создании; дальше используем относительные пути.
    Все методы возвращают HTTPResult — никогда не бросают.
    """

    def __init__(self, base_url: str, timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self._client = httpx.Client(
            base_url=self.base_url,
            timeout=timeout,
            headers={"User-Agent": "samurai-mois-agent/1.0"},
        )

    def close(self) -> None:
        try:
            self._client.close()
        except Exception:  # noqa: BLE001
            pass

    def __enter__(self) -> "DashboardClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def request(
        self,
        method: str,
        path: str,
        *,
        json_body: Optional[Mapping[str, Any]] = None,
        params: Optional[Mapping[str, Any]] = None,
    ) -> HTTPResult:
        try:
            resp = self._client.request(method, path, json=json_body, params=params)
        except httpx.RequestError as exc:
            msg = f"{type(exc).__name__}: {exc}"
            LOG.warning("dashboard %s %s — %s", method, path, msg)
            return HTTPResult(False, 0, None, error=msg)

        body: Any
        ctype = resp.headers.get("content-type", "")
        if "application/json" in ctype:
            try:
                body = resp.json()
            except ValueError:
                body = resp.text
        else:
            body = resp.text

        ok = 200 <= resp.status_code < 300
        err = None if ok else f"HTTP {resp.status_code}"
        return HTTPResult(ok, resp.status_code, body, error=err)

    def get(self, path: str, **kwargs) -> HTTPResult:
        return self.request("GET", path, **kwargs)

    def post(self, path: str, **kwargs) -> HTTPResult:
        return self.request("POST", path, **kwargs)

    def put(self, path: str, **kwargs) -> HTTPResult:
        return self.request("PUT", path, **kwargs)

    def delete(self, path: str, **kwargs) -> HTTPResult:
        return self.request("DELETE", path, **kwargs)
