"""
Vpered Serial Bridge — мост между frontend и Arduino Uno (USB Serial).

Запуск:
    python compute_node/vpered_bridge.py --port COM3
    python compute_node/vpered_bridge.py --port /dev/ttyUSB0
    python compute_node/vpered_bridge.py --auto         # авто-поиск порта

Сервер слушает на :5005. Frontend (vite на :5173) проксирует
/api/vpered/* сюда (см. vite.config.ts).

REST:
    POST /api/vpered/cmd        {"cmd": "F"} | {"cmd": "M", "arg": 45}
    POST /api/vpered/scenario   {"name": "fwd_back" | "square"}
    GET  /api/vpered/state      → последняя телеметрия + статус соединения
    GET  /api/vpered/log        → последние N строк из Serial

Команды Arduino: F B L R S O X G P M<deg> N<deg> D K C T Z H
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import re
import sys
import time
from typing import Any

import serial
import serial.tools.list_ports
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("vpered-bridge")

# ────────────────────────────────────────────────────────────
# State (process-global, single Arduino)
# ────────────────────────────────────────────────────────────

class BridgeState:
    def __init__(self) -> None:
        self.connected: bool = False
        self.port: str | None = None
        self.last_telemetry: dict[str, Any] = {}
        self.last_seen_ts: float = 0.0
        self.log_buffer: list[str] = []          # последние строки из Serial
        self.LOG_MAX = 200
        self.serial: serial.Serial | None = None

    def add_log(self, line: str) -> None:
        self.log_buffer.append(line)
        if len(self.log_buffer) > self.LOG_MAX:
            self.log_buffer = self.log_buffer[-self.LOG_MAX:]

    def parse_telemetry(self, line: str) -> bool:
        """Парсит строку вида 'T,th=2.30,om=0.5,d=120.4,L=180,R=180,m=FWD,...'"""
        if not line.startswith("T,"):
            return False
        d: dict[str, Any] = {}
        for part in line[2:].split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k, v = k.strip(), v.strip()
            try:
                d[k] = float(v) if "." in v else int(v)
            except ValueError:
                d[k] = v
        self.last_telemetry = d
        self.last_seen_ts = time.time()
        return True


state = BridgeState()


# ────────────────────────────────────────────────────────────
# Serial reader thread (asyncio task)
# ────────────────────────────────────────────────────────────

async def serial_reader_task(port: str, baud: int = 9600) -> None:
    """Открывает Serial, читает строки, парсит телеметрию.
    При обрыве — пытается переподключиться раз в 2 сек."""
    while True:
        try:
            log.info("Открываю %s @ %d", port, baud)
            ser = serial.Serial(port, baud, timeout=0.1)
            state.serial = ser
            state.connected = True
            state.port = port
            log.info("Подключено")
            await asyncio.sleep(0.5)
            try:
                ser.write(b"H\n")  # запрос help — заодно проверка связи
            except Exception:
                pass

            while True:
                try:
                    raw = ser.readline()
                except Exception as e:
                    log.warning("read error: %s", e)
                    break
                if not raw:
                    await asyncio.sleep(0.005)
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    continue
                if not line:
                    continue
                state.add_log(line)
                if not state.parse_telemetry(line):
                    log.debug("RX: %s", line)
        except serial.SerialException as e:
            log.warning("Serial error: %s", e)
        except Exception as e:
            log.exception("reader exception: %s", e)
        finally:
            state.connected = False
            state.serial = None
            try:
                ser.close()  # type: ignore[name-defined]
            except Exception:
                pass
        log.info("Переподключение через 2 сек...")
        await asyncio.sleep(2.0)


def write_serial(text: str) -> None:
    if not state.connected or not state.serial:
        raise HTTPException(503, "Robot not connected")
    if not text.endswith("\n"):
        text += "\n"
    try:
        state.serial.write(text.encode("ascii"))
    except Exception as e:
        log.error("write error: %s", e)
        raise HTTPException(500, f"write failed: {e}")


# ────────────────────────────────────────────────────────────
# FastAPI
# ────────────────────────────────────────────────────────────

app = FastAPI(title="Vpered Bridge")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class CmdReq(BaseModel):
    cmd: str          # 'F','B','L','R','S','O','X','G','P','M','N','D','K','C','T','Z','H'
    arg: int | None = None


class ScenarioReq(BaseModel):
    name: str         # 'fwd_back', 'square', 'wiggle', 'open_close', 'grab_demo'


VALID_CMDS = set("FBLRSOXGPDKCTZHMN")


@app.post("/api/vpered/cmd")
async def post_cmd(req: CmdReq) -> dict:
    cmd = req.cmd.strip().upper()
    if not cmd or cmd[0] not in VALID_CMDS:
        raise HTTPException(400, f"unknown cmd: {cmd!r}")
    payload = cmd[0]
    if cmd[0] in ("M", "N"):
        if req.arg is None:
            raise HTTPException(400, f"{cmd[0]} requires arg (0-180)")
        payload = f"{cmd[0]}{int(req.arg)}"
    write_serial(payload)
    return {"ok": True, "sent": payload}


# Сценарии — последовательность команд с задержками. Реализованы как
# фоновая корутина чтобы не блокировать HTTP.
SCENARIOS: dict[str, list[tuple[str, float]]] = {
    "fwd_stop":   [("F", 2.5), ("S", 0.0)],
    "fwd_back":   [("F", 2.0), ("S", 0.5), ("L", 1.4), ("L", 0.1), ("F", 2.0), ("S", 0.0)],
    "square":     [("F", 1.5), ("S", 0.3), ("R", 0.7), ("S", 0.2)] * 4 + [("S", 0.0)],
    "wiggle":     [("L", 0.4), ("R", 0.8), ("L", 0.4), ("S", 0.0)],
    "open_close": [("O", 0.6), ("X", 0.6), ("O", 0.6), ("X", 0.0)],
    "grab_demo":  [("G", 0.0)],
}


async def run_scenario(steps: list[tuple[str, float]]) -> None:
    for cmd, delay in steps:
        try:
            write_serial(cmd)
        except HTTPException:
            return
        if delay > 0:
            await asyncio.sleep(delay)


@app.post("/api/vpered/scenario")
async def post_scenario(req: ScenarioReq) -> dict:
    name = req.name.strip().lower()
    steps = SCENARIOS.get(name)
    if steps is None:
        raise HTTPException(404, f"scenario '{name}' not found. Available: {list(SCENARIOS)}")
    asyncio.create_task(run_scenario(steps))
    return {"ok": True, "scenario": name, "steps": len(steps)}


@app.get("/api/vpered/state")
async def get_state() -> dict:
    fresh = state.connected and (time.time() - state.last_seen_ts < 2.0)
    return {
        "connected": state.connected,
        "port": state.port,
        "telemetry_fresh": fresh,
        "telemetry": state.last_telemetry,
        "scenarios": list(SCENARIOS),
    }


@app.get("/api/vpered/log")
async def get_log(lines: int = 50) -> dict:
    return {"lines": state.log_buffer[-lines:]}


@app.get("/api/vpered/scenarios")
async def get_scenarios() -> dict:
    return {"scenarios": list(SCENARIOS)}


# ────────────────────────────────────────────────────────────
# Port discovery
# ────────────────────────────────────────────────────────────

def find_arduino_port() -> str | None:
    """Ищет первый порт с описанием похожим на Arduino."""
    candidates = list(serial.tools.list_ports.comports())
    for p in candidates:
        desc = (p.description or "").lower()
        if any(s in desc for s in ("arduino", "ch340", "ch341", "usb-serial", "usb serial")):
            return p.device
    if candidates:
        return candidates[0].device
    return None


# ────────────────────────────────────────────────────────────
# Lifespan: start serial reader on boot
# ────────────────────────────────────────────────────────────

@app.on_event("startup")
async def on_startup() -> None:
    if app.state.serial_port:
        asyncio.create_task(serial_reader_task(app.state.serial_port, app.state.baud))
    else:
        log.warning("no serial port — starting in disconnected mode")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM-port (e.g. COM3, /dev/ttyUSB0)")
    ap.add_argument("--auto", action="store_true", help="auto-detect first Arduino-like port")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=5005)
    args = ap.parse_args()

    port = args.port
    if not port and args.auto:
        port = find_arduino_port()
        if port:
            log.info("auto-detected port: %s", port)
    if not port:
        ports = list(serial.tools.list_ports.comports())
        log.warning("no --port; available: %s", [p.device for p in ports])

    app.state.serial_port = port
    app.state.baud = args.baud

    uvicorn.run(app, host=args.host, port=args.http_port, log_level="info")


if __name__ == "__main__":
    main()
