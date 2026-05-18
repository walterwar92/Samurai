"""
Routers: status snapshot, log, hardware presets, multi-robot, mqtt status.

Маппинг старых endpoints (compute_node/dashboard_node.py):

  GET  /api/status            → /status
  GET  /api/log               → /log?limit=50
  GET  /api/mqtt/status       → /mqtt/status
  GET  /api/multi_robot/list  → /multi_robot/list
  POST /api/multi_robot/call  → /multi_robot/call

  GET    /api/hardware/presets             → /hardware/presets
  GET    /api/hardware/presets/{name}      → /hardware/presets/{name}
  POST   /api/hardware/presets             → /hardware/presets
  DELETE /api/hardware/presets/{name}      → /hardware/presets/{name}
  POST   /api/hardware/apply               → /hardware/apply
  GET    /api/hardware/active              → /hardware/active
"""
from __future__ import annotations

import asyncio
import logging
import os
import signal
import sys
import time

from fastapi import APIRouter, BackgroundTasks, HTTPException, Query

from ..schemas.common import CommandAck
from ..schemas.control import MultiRobotCallCommand
from ..schemas.system import (
    HardwareActiveResponse,
    HardwareApplyCommand,
    HardwarePresetListResponse,
    HardwarePresetResponse,
    HardwarePresetSaveCommand,
    HardwarePresetSummary,
    LogEntry,
    LogResponse,
    MqttStatusResponse,
    MultiRobotInfo,
    MultiRobotListResponseSystem,
    StatusSnapshotResponse,
)
from ._deps import MQTTDep, StateDep

log = logging.getLogger(__name__)


# Hardware presets module — лежит в корне Samurai/, а compute_node/ это пакет.
# Путь добавляем sys.path при первом import (idempotent через try/except).
def _import_hw_presets():
    try:
        import hardware_presets  # noqa: F401
        return hardware_presets
    except ImportError:
        root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        if root not in sys.path:
            sys.path.insert(0, root)
        import hardware_presets
        return hardware_presets


# ── /status, /log, /mqtt/status ────────────────────────────────────────
status_router = APIRouter()


@status_router.get('', response_model=StatusSnapshotResponse, tags=['system'])
async def get_status(state: StateDep) -> StatusSnapshotResponse:
    """Атомарный snapshot всего state — для polling-fallback на фронте."""
    snap = state.snapshot()
    snap.pop('ok', None)  # OkResponse выставит сам
    return StatusSnapshotResponse(**snap)


log_router = APIRouter()


@log_router.get('', response_model=LogResponse, tags=['system'])
async def get_log(
    state: StateDep,
    limit: int = Query(default=50, ge=1, le=200),
) -> LogResponse:
    """Последние event/voice записи (max 200)."""
    with state.lock:
        items = list(state.system.event_log)[-limit:]
    entries = [LogEntry(**(e if isinstance(e, dict) else {'text': str(e)})) for e in items]
    return LogResponse(log=entries, count=len(entries))


mqtt_status_router = APIRouter()


@mqtt_status_router.get('', response_model=MqttStatusResponse, tags=['system'])
async def get_mqtt_status(state: StateDep, mqtt: MQTTDep) -> MqttStatusResponse:
    return MqttStatusResponse(
        connected=mqtt.connected,
        broker=mqtt._broker,
        port=mqtt._port,
        robot_id=mqtt._robot_id,
    )


# ── Multi-robot ────────────────────────────────────────────────────────
multi_robot_router = APIRouter()


@multi_robot_router.get(
    '/list', response_model=MultiRobotListResponseSystem, tags=['system'])
async def multi_robot_list(state: StateDep, mqtt: MQTTDep) -> MultiRobotListResponseSystem:
    """Stub: возвращает только текущего робота. Multi-prefix MQTT TODO."""
    with state.lock:
        status = state.robot.fsm
        bat = state.sensors.battery
    return MultiRobotListResponseSystem(robots=[
        MultiRobotInfo(
            id=mqtt._robot_id,
            connected=mqtt.connected,
            state=status.state,
            battery=bat.percent,
            last_seen_ms=int(time.time() * 1000),
        )
    ])


@multi_robot_router.post('/call', response_model=CommandAck, tags=['system'])
async def multi_robot_call(
    cmd: MultiRobotCallCommand, mqtt: MQTTDep
) -> CommandAck:
    """Послать другому роботу call (для координации по mesh)."""
    payload = {'target_id': cmd.target_id, 'action': cmd.action}
    if cmd.colour:
        payload['colour'] = cmd.colour
    mqtt.publish('call_robot', payload, qos=1)
    return CommandAck()


# ── Hardware presets ───────────────────────────────────────────────────
hardware_router = APIRouter()


@hardware_router.get(
    '/presets', response_model=HardwarePresetListResponse, tags=['system'])
async def hardware_presets_list() -> HardwarePresetListResponse:
    hw = _import_hw_presets()
    raw = hw.list_presets()
    summaries: list[HardwarePresetSummary] = []
    for p in raw:
        if isinstance(p, dict):
            summaries.append(HardwarePresetSummary(
                name=p.get('name', ''),
                file=p.get('file'),
                created_at=p.get('created_at'),
            ))
        elif isinstance(p, str):
            summaries.append(HardwarePresetSummary(name=p))
    return HardwarePresetListResponse(presets=summaries, active=hw.get_active())


@hardware_router.get(
    '/presets/{name:path}',
    response_model=HardwarePresetResponse,
    tags=['system'],
)
async def hardware_preset_get(name: str) -> HardwarePresetResponse:
    hw = _import_hw_presets()
    preset = hw.get_preset(name)
    if preset is None:
        raise HTTPException(404, f'Preset not found: {name}')
    return HardwarePresetResponse(preset=preset)


@hardware_router.post(
    '/presets', response_model=HardwarePresetResponse, tags=['system'])
async def hardware_preset_save(
    cmd: HardwarePresetSaveCommand,
) -> HardwarePresetResponse:
    hw = _import_hw_presets()
    payload = {'name': cmd.name, **cmd.config}
    fname = hw.save_preset(payload)
    log.info('Hardware preset saved: %s → %s', cmd.name, fname)
    return HardwarePresetResponse(preset={'name': cmd.name, 'file': fname})


@hardware_router.delete(
    '/presets/{name:path}', response_model=CommandAck, tags=['system'])
async def hardware_preset_delete(name: str) -> CommandAck:
    hw = _import_hw_presets()
    if not hw.delete_preset(name):
        raise HTTPException(404, f'Preset not found: {name}')
    return CommandAck()


@hardware_router.post('/apply', response_model=CommandAck, tags=['system'])
async def hardware_preset_apply(
    cmd: HardwareApplyCommand, mqtt: MQTTDep
) -> CommandAck:
    """Активировать пресет: запись active + push в MQTT для пере-конфигурации Pi."""
    hw = _import_hw_presets()
    preset = hw.get_preset(cmd.name)
    if preset is None:
        raise HTTPException(404, f'Preset not found: {cmd.name}')
    hw.set_active(cmd.name)
    mqtt.publish('hardware/config', preset, qos=1)
    return CommandAck()


@hardware_router.get(
    '/active', response_model=HardwareActiveResponse, tags=['system'])
async def hardware_active() -> HardwareActiveResponse:
    hw = _import_hw_presets()
    return HardwareActiveResponse(active=hw.get_active())


# ── System shutdown ────────────────────────────────────────────────────
shutdown_router = APIRouter()


def _request_compute_shutdown_via_file() -> bool:
    """Попросить bash-launcher на хосте сделать `docker stop samurai_compute`.

    Внутри Docker `compute_node/dashboard` крутится как ребёнок `ros2 launch`
    (PID 1). Сигналы к PID 1 ядром игнорируются по умолчанию, плюс у нод в
    launch файле может быть `respawn=True` — поэтому SIGTERM в FastAPI или
    PID 1 не гарантирует завершение контейнера.

    Решение: создать маркер-файл в проектной директории (она mount'ится с
    хоста как `-v $SAMURAI_ROOT:/root/Samurai`). Параллельный watcher в
    `scripts/cmds/compute.sh` обнаружит файл и сделает `docker stop`, что
    корректно остановит контейнер и активирует trap `cleanup_all`.

    Возвращает True если файл создан (Docker mount существует), иначе False
    (значит мы вне Docker — fallback на SIGTERM себе).
    """
    mount_root = '/root/Samurai'
    if not os.path.isdir(mount_root):
        return False
    flag_path = os.path.join(mount_root, '.shutdown_request')
    try:
        with open(flag_path, 'w', encoding='utf-8') as f:
            f.write('dashboard\n')
        return True
    except OSError as e:
        log.warning('Не удалось создать %s: %s', flag_path, e)
        return False


@shutdown_router.post('', response_model=CommandAck, tags=['system'])
async def system_shutdown(
    background_tasks: BackgroundTasks,
    mqtt: MQTTDep,
) -> CommandAck:
    """Полное выключение робота и дашборда.

    Pi-side: SystemNode ловит samurai/{robot_id}/system/shutdown и шлёт
    SIGTERM родительскому процессу (robot_launcher).

    Compute-side:
    - В Docker: создаём `.shutdown_request` в shared volume; bash-watcher
      на хосте делает `docker stop samurai_compute`; trap cleanup_all
      освобождает lock и тушит samcan bridge.
    - Вне Docker (dev): SIGTERM uvicorn'у — он сам корректно выйдет.
    """
    mqtt.publish('system/shutdown', {'source': 'dashboard'}, qos=1)

    used_file = _request_compute_shutdown_via_file()

    async def _shutdown_self() -> None:
        await asyncio.sleep(0.5)
        if used_file:
            # bash-watcher теперь сделает docker stop — мы можем продолжать
            # отвечать на запросы пока контейнер не остановится. SIGTERM
            # себе не шлём, иначе FastAPI умрёт и ros2 launch с respawn
            # его перезапустит до того как docker stop отработает.
            return
        os.kill(os.getpid(), signal.SIGTERM)

    background_tasks.add_task(_shutdown_self)
    log.warning(
        'System shutdown requested from dashboard (file-trigger=%s)',
        used_file,
    )
    return CommandAck()
