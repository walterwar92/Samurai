"""MOIS-агент: основной цикл poll/execute/telemetry.

Совместим с MOIS Edge Function (Supabase) API:
  - POST  api_url?action=capabilities  {"commands":[{name,description,params_schema}]}
  - GET   api_url?action=poll          → {"poll_interval":int,"commands":[{id,command,params}]}
  - POST  api_url?action=cmd-result    {"command_id","exit_code","stdout","stderr","exec_time_ms"}
  - POST  api_url?action=telemetry     {"custom":{...}}

Авторизация — Bearer api_token в Authorization-заголовке.
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from dataclasses import dataclass
from typing import Any, Optional

import httpx

from .client import DashboardClient
from .commands import COMMANDS, list_capabilities
from .config import AgentConfig, load_config
from .telemetry import collect_telemetry

LOG = logging.getLogger("mois.agent")


@dataclass
class HandlerContext:
    """Передаётся в каждый handler. Хранит клиентов и конфиг."""

    config: AgentConfig
    client: DashboardClient
    samcan_client: Optional[DashboardClient] = None
    state: Optional[dict] = None


class MOISAgent:
    def __init__(self, config: AgentConfig) -> None:
        self.config = config
        self._stop = False
        self._dashboard = DashboardClient(
            config.dashboard_url, timeout=config.request_timeout
        )
        self._samcan: Optional[DashboardClient] = (
            DashboardClient(config.samcan_url, timeout=config.request_timeout)
            if config.samcan_url
            else None
        )
        self._ctx = HandlerContext(
            config=config,
            client=self._dashboard,
            samcan_client=self._samcan,
            state={},
        )
        self._http = httpx.Client(
            timeout=config.request_timeout,
            headers={
                "Authorization": f"Bearer {config.api_token}",
                "Content-Type": "application/json",
                "User-Agent": f"{config.agent_name}/1.0",
            },
        )
        self._poll_interval = config.poll_interval

    def stop(self) -> None:
        self._stop = True

    def close(self) -> None:
        try:
            self._http.close()
        except Exception:  # noqa: BLE001
            pass
        self._dashboard.close()
        if self._samcan:
            self._samcan.close()

    # ── REST к Edge Function ────────────────────────────────────────────
    def _post(self, action: str, body: dict) -> Optional[dict]:
        url = f"{self.config.api_url}?action={action}"
        try:
            r = self._http.post(url, json=body)
        except httpx.RequestError as exc:
            LOG.warning("POST %s — %s", action, exc)
            return None
        if r.status_code >= 400:
            LOG.warning("POST %s → HTTP %d: %s", action, r.status_code, r.text[:200])
            return None
        try:
            return r.json()
        except ValueError:
            return None

    def _get(self, action: str) -> Optional[dict]:
        url = f"{self.config.api_url}?action={action}"
        try:
            r = self._http.get(url)
        except httpx.RequestError as exc:
            LOG.warning("GET %s — %s", action, exc)
            return None
        if r.status_code >= 400:
            LOG.warning("GET %s → HTTP %d: %s", action, r.status_code, r.text[:200])
            return None
        try:
            return r.json()
        except ValueError:
            return None

    # ── Этапы ───────────────────────────────────────────────────────────
    def send_capabilities(self) -> None:
        caps = list_capabilities()
        result = self._post("capabilities", {"commands": caps})
        if result is None:
            LOG.warning("[CAPS] не отправились — повторим при следующем poll")
        else:
            LOG.info("[CAPS] зарегистрировано %d команд", len(caps))

    def poll_and_execute(self) -> None:
        data = self._get("poll")
        if data is None:
            return

        # Адаптивный poll-интервал, если сервер прислал
        new = data.get("poll_interval")
        if isinstance(new, (int, float)):
            clamped = max(1, min(300, int(new)))
            if clamped != self._poll_interval:
                LOG.info(
                    "[POLL] интервал обновлён: %ds → %ds",
                    self._poll_interval,
                    clamped,
                )
                self._poll_interval = clamped

        commands = data.get("commands") or []
        for cmd_obj in commands:
            self._execute_one(cmd_obj)

    def _execute_one(self, cmd_obj: dict) -> None:
        cmd_name = str(cmd_obj.get("command", ""))
        cmd_id = str(cmd_obj.get("id", ""))
        params = cmd_obj.get("params") or {}
        if not isinstance(params, dict):
            params = {}

        LOG.info("[CMD] %s (id=%s, params=%s)", cmd_name, cmd_id, params)

        spec = COMMANDS.get(cmd_name)
        t0 = time.time()
        if spec is None:
            result = {
                "exit_code": 1,
                "stdout": "",
                "stderr": f"Unknown command: {cmd_name}",
            }
        else:
            try:
                handler = spec["handler"]
                hr = handler(params, self._ctx)
                result = (
                    hr.to_dict()
                    if hasattr(hr, "to_dict")
                    else {
                        "exit_code": int(hr.get("exit_code", 0)),
                        "stdout": str(hr.get("stdout", "")),
                        "stderr": str(hr.get("stderr", "")),
                    }
                )
            except Exception as exc:  # noqa: BLE001
                LOG.exception("[CMD] handler %s упал", cmd_name)
                result = {
                    "exit_code": 1,
                    "stdout": "",
                    "stderr": f"handler crashed: {type(exc).__name__}: {exc}",
                }

        exec_ms = int((time.time() - t0) * 1000)
        body = {
            "command_id": cmd_id,
            "exit_code": result.get("exit_code", 0),
            "stdout": result.get("stdout", ""),
            "stderr": result.get("stderr", ""),
            "exec_time_ms": exec_ms,
        }
        self._post("cmd-result", body)
        LOG.info(
            "[CMD] %s → exit=%s exec=%dms",
            cmd_name,
            body["exit_code"],
            exec_ms,
        )

    def send_telemetry(self) -> None:
        data = collect_telemetry(self._ctx)
        self._post("telemetry", {"custom": data})
        LOG.debug("[TEL] sent keys=%s", list(data.keys()))

    # ── Основной цикл ───────────────────────────────────────────────────
    def run(self) -> None:
        LOG.info(
            "MOIS-агент стартует: api=%s, dashboard=%s, robot=%s, poll=%ds, tel=%ds",
            self.config.api_url,
            self.config.dashboard_url,
            self.config.robot_id,
            self._poll_interval,
            self.config.telemetry_interval,
        )
        self.send_capabilities()

        last_poll = 0.0
        last_telemetry = 0.0
        while not self._stop:
            now = time.time()
            if now - last_poll >= self._poll_interval:
                self.poll_and_execute()
                last_poll = now
            if now - last_telemetry >= self.config.telemetry_interval:
                self.send_telemetry()
                last_telemetry = now
            time.sleep(1)
        LOG.info("MOIS-агент остановлен")


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m compute_node.mois_agent",
        description="MOIS HTTP-агент Samurai (мост сайт ↔ робот через dashboard :5000)",
    )
    p.add_argument("--api-url", help="URL Supabase Edge Function (robot-gateway)")
    p.add_argument("--api-token", help="Bearer-токен агента (rt_...)")
    p.add_argument(
        "--dashboard",
        dest="dashboard_url",
        help="URL локального dashboard (default: http://127.0.0.1:5000)",
    )
    p.add_argument("--samcan", dest="samcan_url", help="URL Samcan bridge (optional)")
    p.add_argument("--robot-id", dest="robot_id", help="MQTT robot_id")
    p.add_argument("--poll", dest="poll_interval", type=int, help="Интервал poll, сек")
    p.add_argument(
        "--telemetry",
        dest="telemetry_interval",
        type=int,
        help="Интервал телеметрии, сек",
    )
    p.add_argument(
        "--config",
        dest="config_path",
        help="Путь к config.json (default: рядом с пакетом)",
    )
    p.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    p.add_argument(
        "--list-commands",
        action="store_true",
        help="Распечатать список команд и выйти",
    )
    return p


def main(argv: Optional[list[str]] = None) -> int:
    # Windows-консоль по умолчанию cp1251 — а у нас в описаниях кириллица
    # и стрелки. Переключаем stdout/stderr на UTF-8 ДО argparse.print_help.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass

    args = _build_arg_parser().parse_args(argv)

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    if args.list_commands:
        for name, spec in sorted(COMMANDS.items()):
            print(f"  {name:32} {spec.get('description', '')}")
        return 0

    cli_overrides: dict[str, Any] = {}
    for k in (
        "api_url",
        "api_token",
        "dashboard_url",
        "samcan_url",
        "robot_id",
        "poll_interval",
        "telemetry_interval",
    ):
        v = getattr(args, k, None)
        if v not in (None, ""):
            cli_overrides[k] = v

    from pathlib import Path

    config_path = Path(args.config_path) if args.config_path else None

    try:
        config = load_config(config_path=config_path, cli_overrides=cli_overrides)
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 2

    agent = MOISAgent(config)

    def _on_signal(signum, _frame):
        LOG.info("Получен сигнал %d — останавливаюсь", signum)
        agent.stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        agent.run()
    finally:
        agent.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
