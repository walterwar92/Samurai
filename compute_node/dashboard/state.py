"""
DashboardState — централизованное состояние dashboard-сервера.

Все данные, которые приходят из MQTT/ROS2 и читаются REST/WebSocket
endpoint'ами, агрегируются здесь. Доступ через threading.RLock (re-entrant —
позволяет делать вложенные `with state.lock:` без deadlock).

В C5-C10 routers будут читать этот state. В C4 handlers будут его обновлять.
В C13 старый dashboard_node.py будет удалён, текущая копия его state мигрирует
сюда без потерь.

Использование:
    state = DashboardState()

    # Update (handler):
    with state.lock:
        state.pose = RobotPose(x=1.0, y=0.5, yaw=0.0)
        state.event_log.append(LogEntry(...))

    # Read (router):
    with state.lock:
        snapshot = state.snapshot()
    return snapshot
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional

from .schemas.actuators import ArmState, ClawState, HeadState, LedState
from .schemas.detection import BallInfo, Detection, DetectionResult
from .schemas.maps import ForbiddenZone, MapInfo, SlamMapData
from .schemas.robot import RobotPose, RobotStatus, VelocityDetail
from .schemas.sensors import (
    BatteryStatus,
    ImuData,
    TemperatureData,
    UltrasonicData,
    WatchdogStatus,
)


# ── Внутренние блоки (для группировки большого state) ─────────────────────
@dataclass
class _RobotBlock:
    pose: RobotPose = field(default_factory=RobotPose)
    velocity_estimated: VelocityDetail = field(default_factory=VelocityDetail)
    velocity_commanded: VelocityDetail = field(default_factory=VelocityDetail)
    fsm: RobotStatus = field(default_factory=RobotStatus)
    speed_profile: str = 'normal'  # slow|normal|fast


@dataclass
class _SensorsBlock:
    ultrasonic: UltrasonicData = field(default_factory=UltrasonicData)
    imu: ImuData = field(default_factory=ImuData)
    battery: BatteryStatus = field(default_factory=BatteryStatus)
    temperature: TemperatureData = field(default_factory=TemperatureData)
    watchdog: WatchdogStatus = field(default_factory=WatchdogStatus)


@dataclass
class _ActuatorsBlock:
    claw: ClawState = field(default_factory=ClawState)
    head: HeadState = field(default_factory=HeadState)
    arm: ArmState = field(default_factory=ArmState)
    led: LedState = field(default_factory=LedState)
    head_presets: dict = field(default_factory=dict)
    arm_presets: dict = field(default_factory=dict)


@dataclass
class _DetectionBlock:
    """YOLO детекции от compute_node/detector.py."""
    result: DetectionResult = field(default_factory=DetectionResult)
    closest: Optional[Detection] = None
    balls: list[BallInfo] = field(default_factory=list)
    enabled: bool = True
    backend: Optional[str] = None  # 'yolo' | 'hsv'
    fps: float = 0.0
    annotated_jpeg: Optional[bytes] = None  # последний аннотированный кадр
    yolo_status: dict = field(default_factory=dict)  # online/source/ts


@dataclass
class _MapBlock:
    info: Optional[MapInfo] = None
    png: Optional[bytes] = None  # сгенерированный PNG карты
    slam: Optional[SlamMapData] = None
    zones: list[ForbiddenZone] = field(default_factory=list)
    zone_counter: int = 0
    saved_maps: list[str] = field(default_factory=list)


@dataclass
class _ControlBlock:
    """Status высокоуровневых поведений (patrol/follow_me/explorer/...)."""
    patrol_status: dict = field(default_factory=dict)
    patrol_waypoints: list = field(default_factory=list)
    follow_me_status: dict = field(default_factory=dict)
    path_recorder_status: dict = field(default_factory=dict)
    path_recorder_path: list = field(default_factory=list)
    path_recorder_list: list[str] = field(default_factory=list)
    precision_drive_status: dict = field(default_factory=dict)
    precision_drive_result: dict = field(default_factory=dict)
    calibration_status: dict = field(default_factory=dict)
    calibration_result: dict = field(default_factory=dict)
    calibration_active_profile: Optional[str] = None
    calibration_profiles: list = field(default_factory=list)
    explorer_status: dict = field(default_factory=dict)
    mission_status: dict = field(default_factory=dict)
    mission_list: list[str] = field(default_factory=list)
    obstacle_avoidance_enabled: bool = True
    collision_guard_enabled: bool = True


@dataclass
class _CameraBlock:
    """Discovery + status H.264 потока (с #9, 2026-04)."""
    h264_endpoint: Optional[dict] = None
    yolo_remote_online: bool = False


@dataclass
class _SystemBlock:
    voice_log: deque = field(default_factory=lambda: deque(maxlen=20))
    event_log: deque = field(default_factory=lambda: deque(maxlen=100))
    tts_enabled: bool = True
    multi_robots: list = field(default_factory=list)
    mqtt_connected: bool = False
    start_time: float = field(default_factory=time.time)


# ── Главный класс ──────────────────────────────────────────────────────────
class DashboardState:
    """
    Агрегированное состояние dashboard. Все мутации/чтения — через self.lock.

    Поля сгруппированы в логические блоки чтобы было меньше "плоского беспорядка"
    (старый dashboard_node имел ~45 self._* полей вперемежку).

    snapshot() возвращает атомарный dict — основа для GET /api/status и для
    state push через WebSocket /ws/state каждые 100мс (см. dashboard.state_push_hz).
    """

    def __init__(self):
        # RLock — re-entrant: один поток может вложенно брать lock без deadlock.
        # Старый dashboard_node использовал Lock, что иногда приводило к
        # потенциальным deadlock'ам при цепочках обновлений.
        self.lock: threading.RLock = threading.RLock()

        self.robot = _RobotBlock()
        self.sensors = _SensorsBlock()
        self.actuators = _ActuatorsBlock()
        self.detection = _DetectionBlock()
        self.map = _MapBlock()
        self.control = _ControlBlock()
        self.camera = _CameraBlock()
        self.system = _SystemBlock()

    # ── Atomic snapshot ────────────────────────────────────────────────
    def snapshot(self) -> dict[str, Any]:
        """
        Атомарно собирает полное состояние в dict — для /api/status, /ws/state,
        SocketIO state_update event.

        Формат совместим с тем что возвращал старый DashboardNode._snapshot()
        (для backward-compat фронта во время миграции).
        """
        with self.lock:
            r = self.robot
            s = self.sensors
            a = self.actuators
            d = self.detection
            m = self.map
            c = self.control
            cam = self.camera
            sys_ = self.system

            return {
                'ok': True,
                'sim_time': time.time() - sys_.start_time,
                'pose': r.pose.model_dump(),
                'velocity': {
                    'estimated': r.velocity_estimated.model_dump(),
                    'commanded': r.velocity_commanded.model_dump(),
                },
                'robot_status': r.fsm.model_dump(),
                'speed_profile': r.speed_profile,
                'sensors': {
                    'ultrasonic': s.ultrasonic.model_dump(),
                    'imu': s.imu.model_dump(),
                },
                'battery': s.battery.model_dump(),
                'temperature': s.temperature.model_dump(),
                'watchdog': s.watchdog.model_dump(),
                'actuators': {
                    'claw': a.claw.model_dump(),
                    'head': a.head.model_dump(),
                    'arm': a.arm.model_dump(),
                    'led': a.led.model_dump(),
                },
                'detection': {
                    'result': d.result.model_dump(by_alias=True),
                    'closest': d.closest.model_dump(by_alias=True) if d.closest else None,
                    'balls': [b.model_dump() for b in d.balls],
                    'enabled': d.enabled,
                    'backend': d.backend,
                    'fps': d.fps,
                    'yolo_status': d.yolo_status,
                },
                'map': {
                    'info': m.info.model_dump() if m.info else None,
                    'zones': [z.model_dump() for z in m.zones],
                    'slam': m.slam.model_dump() if m.slam else None,
                    'saved_maps': m.saved_maps,
                },
                'control': {
                    'patrol': c.patrol_status,
                    'follow_me': c.follow_me_status,
                    'path_recorder': {
                        'status': c.path_recorder_status,
                        'path': c.path_recorder_path,
                        'list': c.path_recorder_list,
                    },
                    'precision_drive': {
                        'status': c.precision_drive_status,
                        'result': c.precision_drive_result,
                    },
                    'calibration': {
                        'status': c.calibration_status,
                        'result': c.calibration_result,
                        'active': c.calibration_active_profile,
                        'profiles': c.calibration_profiles,
                    },
                    'explorer': c.explorer_status,
                    'mission': {
                        'status': c.mission_status,
                        'list': c.mission_list,
                    },
                    'obstacle_avoidance': c.obstacle_avoidance_enabled,
                    'collision_guard': c.collision_guard_enabled,
                },
                'camera': {
                    'h264_endpoint': cam.h264_endpoint,
                    'yolo_remote_online': cam.yolo_remote_online,
                },
                'system': {
                    'voice_log': list(sys_.voice_log),
                    'event_log': list(sys_.event_log),
                    'tts_enabled': sys_.tts_enabled,
                    'multi_robots': sys_.multi_robots,
                    'mqtt_connected': sys_.mqtt_connected,
                },
            }

    # ── Convenience helpers ────────────────────────────────────────────
    def append_event_log(self, entry: dict):
        """Добавить запись в event_log с автоматическим locking."""
        with self.lock:
            self.system.event_log.append(entry)

    def append_voice_log(self, entry: dict):
        with self.lock:
            self.system.voice_log.append(entry)

    def add_zone(self, x1: float, y1: float, x2: float, y2: float) -> ForbiddenZone:
        """Атомарно создать зону с auto-incremented ID."""
        with self.lock:
            self.map.zone_counter += 1
            z = ForbiddenZone(
                id=self.map.zone_counter,
                x1=x1, y1=y1, x2=x2, y2=y2,
            )
            self.map.zones.append(z)
            return z

    def remove_zone(self, zone_id: int) -> bool:
        """Удалить зону по ID. True если удалена."""
        with self.lock:
            before = len(self.map.zones)
            self.map.zones = [z for z in self.map.zones if z.id != zone_id]
            return len(self.map.zones) < before

    def clear_zones(self):
        with self.lock:
            self.map.zones = []
            self.map.zone_counter = 0
