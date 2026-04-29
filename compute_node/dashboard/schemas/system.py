"""
System schemas: snapshot, log, hardware presets, mqtt status.
"""
from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Snapshot / log ─────────────────────────────────────────────────────
class StatusSnapshotResponse(OkResponse):
    """GET /api/status — атомарный snapshot всего state.

    Структура динамическая (см. DashboardState.snapshot()), фронт берёт
    нужные поля. Не типизируем строго — снапшот часто меняется.
    """
    sim_time: float = 0.0
    pose: dict = Field(default_factory=dict)
    velocity: dict = Field(default_factory=dict)
    robot_status: dict = Field(default_factory=dict)
    speed_profile: str = 'normal'
    sensors: dict = Field(default_factory=dict)
    battery: dict = Field(default_factory=dict)
    temperature: dict = Field(default_factory=dict)
    watchdog: dict = Field(default_factory=dict)
    actuators: dict = Field(default_factory=dict)
    detection: dict = Field(default_factory=dict)
    map: dict = Field(default_factory=dict)
    control: dict = Field(default_factory=dict)
    camera: dict = Field(default_factory=dict)
    system: dict = Field(default_factory=dict)


class LogEntry(BaseModel):
    """Одна запись event/voice лога."""
    ts: Optional[float] = None
    time: Optional[str] = Field(default=None, description='HH:MM:SS')
    type: Optional[str] = Field(default=None, description='voice|api_command|system')
    source: Optional[str] = None
    level: Optional[str] = None
    text: str = ''


class LogResponse(OkResponse):
    log: list[LogEntry] = Field(default_factory=list)
    count: int = 0


# ── MQTT status ────────────────────────────────────────────────────────
class MqttStatusResponse(OkResponse):
    connected: bool = False
    broker: str = ''
    port: int = 1883
    robot_id: str = 'robot1'


# ── Hardware presets ───────────────────────────────────────────────────
class HardwarePresetSummary(BaseModel):
    """Краткая инфа о пресете для list-endpoint."""
    name: str
    file: Optional[str] = None
    created_at: Optional[str] = None


class HardwarePresetListResponse(OkResponse):
    presets: list[HardwarePresetSummary] = Field(default_factory=list)
    active: Optional[str] = None


class HardwarePresetResponse(OkResponse):
    preset: dict[str, Any] = Field(default_factory=dict)


class HardwarePresetSaveCommand(BaseModel):
    """POST /api/hardware/presets — сохранить пресет (имя обязательно)."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')
    config: dict[str, Any] = Field(
        default_factory=dict,
        description='Произвольная конфигурация для сохранения'
    )


class HardwareApplyCommand(BaseModel):
    """POST /api/hardware/apply — применить пресет."""
    name: str = Field(min_length=1, max_length=64,
                      pattern=r'^[a-zA-Z0-9_\-]+$')


class HardwareActiveResponse(OkResponse):
    active: Optional[str] = None


# ── Multi-robot ────────────────────────────────────────────────────────
class MultiRobotInfo(BaseModel):
    """Состояние одного робота в кластере."""
    id: str
    connected: bool = False
    state: str = 'unknown'
    battery: int = -1
    last_seen_ms: int = 0


class MultiRobotListResponseSystem(OkResponse):
    """Альтернатива control.MultiRobotListResponse — формат старого dashboard_node."""
    robots: list[MultiRobotInfo] = Field(default_factory=list)
