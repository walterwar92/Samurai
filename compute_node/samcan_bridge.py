"""
Samcan Serial Bridge — мост между frontend и Arduino Uno (USB Serial).

Запуск:
    python compute_node/samcan_bridge.py --port COM3
    python compute_node/samcan_bridge.py --port /dev/ttyUSB0
    python compute_node/samcan_bridge.py --auto         # авто-поиск порта

Сервер слушает на :5005. Frontend (vite на :5173) проксирует
/api/samcan/* сюда (см. vite.config.ts).

REST:
    POST /api/samcan/cmd        {"cmd": "F"} | {"cmd": "M", "arg": 45}
    POST /api/samcan/scenario   {"name": "fwd_back" | "square"}
    GET  /api/samcan/state      → последняя телеметрия + статус соединения
    GET  /api/samcan/log        → последние N строк из Serial

Команды Arduino: F B L R S O X G P M<deg> N<deg> D K C T Z H
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Any

import serial
import serial.tools.list_ports
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("samcan-bridge")

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
        # Диагностика
        self.last_error: str = ""
        self.last_attempt_ts: float = 0.0
        self.attempted_port: str | None = None
        self.rx_count: int = 0                   # сколько строк пришло
        # Бракованные пакеты: счётчики и образец последней плохой строки.
        # Видны через /api/samcan/state, помогают диагностировать кабель/EMI.
        self.malformed_count: int = 0
        self.last_malformed: str = ""

    def add_log(self, line: str) -> None:
        self.log_buffer.append(line)
        if len(self.log_buffer) > self.LOG_MAX:
            self.log_buffer = self.log_buffer[-self.LOG_MAX:]

    # ── Telemetry schema ─────────────────────────────────────────
    # Maps each expected field to its coercion function. Unknown keys are
    # ignored. Coercion failures (corrupted bytes from EMI, truncated
    # packets) drop the bad field rather than overwriting last_telemetry
    # with garbage; downstream code keeps the last-known-good value.
    TELEMETRY_FIELDS: dict[str, Any] = {
        'th': float, 'om': float, 'd': float,    # heading / omega / distance
        'L': int, 'R': int,                      # left/right PWM
        'm': str,                                # mode (FWD/STOP/...)
        'kp': float, 'ki': float, 'kd': float,   # PID gains
        'b': int, 'a': int, 'c': int,            # base/arm/claw servo angles
        'h': int,                                # head angle
        'v': float,                              # battery voltage
        'rng': float,                            # ultrasonic
    }
    MAX_LINE_LEN = 512

    def parse_telemetry(self, line: str) -> bool:
        """Парсит строку 'T,th=2.30,om=0.5,d=120.4,L=180,R=180,m=FWD'.

        Безопасен к обрезанным/повреждённым пакетам: bad fields молча
        отбрасываются, last_telemetry мержится — последнее валидное значение
        для каждого поля сохраняется. Слишком длинные строки (corrupted
        UART) отвергаются целиком.
        """
        if not line.startswith("T,"):
            return False
        if len(line) > self.MAX_LINE_LEN:
            self._record_malformed(line, reason='line too long')
            return False

        parsed: dict[str, Any] = {}
        any_bad = False
        for part in line[2:].split(","):
            if "=" not in part:
                # Empty trailing parts after a stray comma are normal — skip
                # silently; only bracket genuinely malformed structure later.
                if part.strip():
                    any_bad = True
                continue
            k, _, v = part.partition("=")
            k = k.strip()
            v = v.strip()
            if not k or not v:
                any_bad = True
                continue
            coerce = self.TELEMETRY_FIELDS.get(k)
            if coerce is None:
                # Unknown but well-formed field — accept as-is for forward
                # compatibility with newer firmware revisions.
                parsed[k] = v
                continue
            try:
                parsed[k] = coerce(v)
            except (ValueError, TypeError):
                any_bad = True   # field corrupted — skip, keep prior value

        if any_bad and not parsed:
            # Whole packet was unintelligible — don't touch state.
            self._record_malformed(line, reason='no usable fields')
            return False

        # Merge instead of replace so a partial packet doesn't blank prior fields.
        if self.last_telemetry:
            merged = dict(self.last_telemetry)
            merged.update(parsed)
            self.last_telemetry = merged
        else:
            self.last_telemetry = parsed
        self.last_seen_ts = time.time()
        return True

    def _record_malformed(self, line: str, reason: str) -> None:
        self.malformed_count += 1
        self.last_malformed = f'{reason}: {line[:120]}'
        # Log first 5 occurrences then 1-in-100 to avoid log flood while
        # still surfacing intermittent bus issues in journalctl.
        if self.malformed_count <= 5 or self.malformed_count % 100 == 0:
            log.warning('Bad telemetry (#%d, %s): %r',
                        self.malformed_count, reason, line[:120])


state = BridgeState()


# ────────────────────────────────────────────────────────────
# Persistent presets (JSON file next to config.yaml)
# ────────────────────────────────────────────────────────────

PRESETS_FILE = Path(__file__).resolve().parent.parent / "samcan_presets.json"

DEFAULT_PRESETS: dict[str, Any] = {
    # углы BASE/ARM для поз
    "park":    {"base": 90, "arm": 90, "claw": 135},
    "forward": {"base": 90, "arm": 40},
    # углы клешни для действий
    "claw_open":   70,
    "claw_closed": 150,
    # задержки последовательности grab, мс
    "settle_ms":   400,
    "hold_ms":     300,
}


def load_presets() -> dict[str, Any]:
    if PRESETS_FILE.exists():
        try:
            data = json.loads(PRESETS_FILE.read_text(encoding="utf-8"))
            # merge с defaults чтобы новые ключи появлялись если расширим
            merged = DEFAULT_PRESETS.copy()
            merged.update(data)
            # но вложенные объекты тоже мержим
            for k, dv in DEFAULT_PRESETS.items():
                if isinstance(dv, dict) and isinstance(merged.get(k), dict):
                    m = dv.copy()
                    m.update(merged[k])
                    merged[k] = m
            return merged
        except Exception as e:
            log.warning("presets load error: %s — using defaults", e)
    return DEFAULT_PRESETS.copy()


def save_presets(presets: dict[str, Any]) -> None:
    try:
        PRESETS_FILE.write_text(json.dumps(presets, indent=2, ensure_ascii=False), encoding="utf-8")
        log.info("presets saved → %s", PRESETS_FILE)
    except Exception as e:
        log.error("presets save error: %s", e)
        raise HTTPException(500, f"cannot write {PRESETS_FILE.name}: {e}")


presets: dict[str, Any] = load_presets()


# ────────────────────────────────────────────────────────────
# Serial reader thread (asyncio task)
# ────────────────────────────────────────────────────────────

async def serial_reader_task(port_hint: str | None, baud: int = 9600) -> None:
    """Открывает Serial, читает строки, парсит телеметрию.
    При обрыве или если порт не указан — пытается переподключиться раз в 2 сек.
    Если port_hint пустой — каждую итерацию пересканирует find_arduino_port()."""
    while True:
        # Определяем порт на этой итерации
        current_port = port_hint or find_arduino_port()
        state.attempted_port = current_port
        state.last_attempt_ts = time.time()

        if not current_port:
            ports_list = [p.device for p in serial.tools.list_ports.comports()]
            state.last_error = (
                "Не найден ни один COM-порт" if not ports_list
                else f"Arduino-порт не определён автоматически. Доступны: {', '.join(ports_list)}. "
                      f"Запусти с --samcan-port <порт>"
            )
            log.warning(state.last_error)
            await asyncio.sleep(2.0)
            continue

        ser: serial.Serial | None = None
        try:
            log.info("Открываю %s @ %d", current_port, baud)
            ser = serial.Serial(current_port, baud, timeout=0.1)
            state.serial = ser
            state.connected = True
            state.port = current_port
            state.last_error = ""
            state.rx_count = 0
            log.info("Подключено к %s", current_port)
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
                    state.last_error = f"read error: {e}"
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
                state.rx_count += 1
                if not state.parse_telemetry(line):
                    log.debug("RX: %s", line)
        except serial.SerialException as e:
            log.warning("Serial error on %s: %s", current_port, e)
            state.last_error = f"{current_port}: {e}"
        except PermissionError as e:
            log.warning("Permission denied %s: %s", current_port, e)
            state.last_error = f"{current_port}: permission denied (порт занят другим приложением?)"
        except Exception as e:
            log.exception("reader exception: %s", e)
            state.last_error = f"{type(e).__name__}: {e}"
        finally:
            state.connected = False
            state.serial = None
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        log.info("Переподключение через 2 сек... (last error: %s)", state.last_error or "—")
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

app = FastAPI(title="Samcan Bridge")

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


# Y добавлен для подбора DIR-байта обратного хода
VALID_CMDS = set("FBLRSOXGPDKCTZHMNY")
# Команды, которым нужен числовой аргумент (встраивается в payload)
CMDS_WITH_ARG = {"M", "N", "Y"}


@app.post("/api/samcan/cmd")
async def post_cmd(req: CmdReq) -> dict:
    cmd = req.cmd.strip().upper()
    if not cmd or cmd[0] not in VALID_CMDS:
        raise HTTPException(400, f"unknown cmd: {cmd!r}")
    payload = cmd[0]
    if cmd[0] in CMDS_WITH_ARG:
        if req.arg is None:
            raise HTTPException(400, f"{cmd[0]} requires numeric arg")
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


@app.post("/api/samcan/scenario")
async def post_scenario(req: ScenarioReq) -> dict:
    name = req.name.strip().lower()
    steps = SCENARIOS.get(name)
    if steps is None:
        raise HTTPException(404, f"scenario '{name}' not found. Available: {list(SCENARIOS)}")
    asyncio.create_task(run_scenario(steps))
    return {"ok": True, "scenario": name, "steps": len(steps)}


@app.get("/api/samcan/state")
async def get_state() -> dict:
    fresh = state.connected and (time.time() - state.last_seen_ts < 2.0)
    return {
        "connected": state.connected,
        "port": state.port,
        "telemetry_fresh": fresh,
        "telemetry": state.last_telemetry,
        "malformed_count": state.malformed_count,
        "last_malformed": state.last_malformed,
        "scenarios": list(SCENARIOS),
        # Диагностика — чтобы UI мог показать причину «нет связи»
        "last_error": state.last_error,
        "attempted_port": state.attempted_port,
        "rx_count": state.rx_count,
        "age_sec": round(time.time() - state.last_seen_ts, 2) if state.last_seen_ts else None,
    }


@app.get("/api/samcan/diag")
async def get_diag() -> dict:
    """Полная диагностика — доступные порты, последняя ошибка, счётчики."""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            "device": p.device,
            "description": p.description,
            "hwid": p.hwid,
            "manufacturer": p.manufacturer,
            "vid": p.vid,
            "pid": p.pid,
        })
    return {
        "connected": state.connected,
        "port": state.port,
        "attempted_port": state.attempted_port,
        "last_error": state.last_error,
        "last_attempt_ts": state.last_attempt_ts,
        "last_seen_ts": state.last_seen_ts,
        "rx_count": state.rx_count,
        "available_ports": ports,
    }


@app.get("/api/samcan/log")
async def get_log(lines: int = 50) -> dict:
    return {"lines": state.log_buffer[-lines:]}


@app.get("/api/samcan/scenarios")
async def get_scenarios() -> dict:
    return {"scenarios": list(SCENARIOS)}


# ────────────────────────────────────────────────────────────
# Presets API
# ────────────────────────────────────────────────────────────

class PresetSaveReq(BaseModel):
    name: str                    # park | forward | claw_open | claw_closed
    base: int | None = None
    arm: int | None = None
    claw: int | None = None


class PresetsPutReq(BaseModel):
    presets: dict[str, Any]


@app.get("/api/samcan/presets")
async def get_presets() -> dict:
    return {"presets": presets}


@app.put("/api/samcan/presets")
async def put_presets(req: PresetsPutReq) -> dict:
    presets.clear()
    presets.update(req.presets)
    save_presets(presets)
    return {"ok": True, "presets": presets}


@app.post("/api/samcan/preset/save")
async def save_preset_action(req: PresetSaveReq) -> dict:
    """Сохранить конкретный пресет. Принимает частичное обновление:
    - park / forward: base / arm / claw (что передано)
    - claw_open / claw_closed: только claw, сохраняется в корне
    """
    name = req.name.strip().lower()
    if name in ("park", "forward"):
        pose = dict(presets.get(name, {}))
        if req.base is not None: pose["base"] = int(req.base)
        if req.arm  is not None: pose["arm"]  = int(req.arm)
        if req.claw is not None: pose["claw"] = int(req.claw)
        presets[name] = pose
    elif name == "claw_open":
        if req.claw is None:
            raise HTTPException(400, "claw required for claw_open")
        presets["claw_open"] = int(req.claw)
    elif name == "claw_closed":
        if req.claw is None:
            raise HTTPException(400, "claw required for claw_closed")
        presets["claw_closed"] = int(req.claw)
    else:
        raise HTTPException(400, f"unknown preset: {name}")
    save_presets(presets)
    return {"ok": True, "name": name, "presets": presets}


async def apply_pose(base: int | None, arm: int | None, claw: int | None, settle_ms: int = 0) -> None:
    """Применить пресет-позу: отправить команды BASE/ARM/CLAW с настройкой."""
    if base is not None:
        write_serial(f"N{int(base)}")
        if settle_ms:
            await asyncio.sleep(settle_ms / 1000)
    if arm is not None:
        write_serial(f"M{int(arm)}")
        if settle_ms:
            await asyncio.sleep(settle_ms / 1000)
    if claw is not None:
        write_serial(f"M{int(claw)}")  # CLAW тоже через сервокомманду — у нас нет отдельного опкода


@app.post("/api/samcan/preset/apply")
async def apply_preset(req: ScenarioReq) -> dict:
    """Применить сохранённый пресет. Поддерживает: park, forward, grab."""
    name = req.name.strip().lower()

    if name == "park":
        p = presets.get("park", DEFAULT_PRESETS["park"])
        asyncio.create_task(_apply_park(p))
        return {"ok": True, "applied": "park", "pose": p}

    if name == "forward":
        p = presets.get("forward", DEFAULT_PRESETS["forward"])
        asyncio.create_task(_apply_forward(p))
        return {"ok": True, "applied": "forward", "pose": p}

    if name == "grab":
        asyncio.create_task(_apply_grab())
        return {"ok": True, "applied": "grab"}

    raise HTTPException(404, f"preset '{name}' not applicable")


async def _apply_park(p: dict) -> None:
    settle = int(presets.get("settle_ms", 400))
    # claw → arm → base (парк сверху — сначала подтягиваем руку)
    if "claw" in p: write_serial(f"O"); await asyncio.sleep(0.05)  # force servo attach
    if "claw" in p:
        # точечный угол — через отдельную команду servo claw нельзя, используем O/X если совпадает
        # в остальных случаях — ручное O -> M не подходит для claw. Пропускаем.
        pass
    if "arm" in p:  write_serial(f"M{int(p['arm'])}");  await asyncio.sleep(settle/1000)
    if "base" in p: write_serial(f"N{int(p['base'])}"); await asyncio.sleep(settle/1000)


async def _apply_forward(p: dict) -> None:
    settle = int(presets.get("settle_ms", 400))
    if "base" in p: write_serial(f"N{int(p['base'])}"); await asyncio.sleep(settle/1000)
    if "arm" in p:  write_serial(f"M{int(p['arm'])}");  await asyncio.sleep(settle/1000)


async def _apply_grab() -> None:
    """Custom grab с учётом пользовательских пресетов forward и claw_open/closed.
    Последовательность: open claw → move to forward → settle → close claw → lift arm.
    """
    settle = int(presets.get("settle_ms", 400))
    hold = int(presets.get("hold_ms", 300))
    fwd = presets.get("forward", DEFAULT_PRESETS["forward"])
    park = presets.get("park", DEFAULT_PRESETS["park"])

    # 1. открыть клешню — используем Arduino команду 'O' (она зажмёт CLAW_OPEN
    # из flash-пресета, который совпадает по дефолту, но фактически это
    # fallback; альтернативно можно закомитить M<angle> но в прошивке M
    # пишет в ARM. Оставляем 'O' для совместимости.)
    write_serial("O")
    await asyncio.sleep(settle/1000)

    # 2. рука вперёд
    if "base" in fwd: write_serial(f"N{int(fwd['base'])}"); await asyncio.sleep(settle/1000)
    if "arm" in fwd:  write_serial(f"M{int(fwd['arm'])}");  await asyncio.sleep(settle/1000)

    # 3. пауза
    await asyncio.sleep(hold/1000)

    # 4. закрыть клешню
    write_serial("X")
    await asyncio.sleep(settle/1000)

    # 5. вернуть руку в парк
    if "arm" in park: write_serial(f"M{int(park['arm'])}"); await asyncio.sleep(settle/1000)


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
    # Всегда стартуем reader. Если порт не задан — каждую итерацию
    # reader сам пересканирует через find_arduino_port().
    asyncio.create_task(serial_reader_task(app.state.serial_port, app.state.baud))


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
