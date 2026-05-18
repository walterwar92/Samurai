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
from .schemas.mps import MpsMatrices, MpsScenarioResult, MpsTelemetryPoint
from .schemas.robot import OdometrySources, RobotPose, RobotStatus, VelocityDetail
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
    stationary: bool = True
    # MQTT odom — primary source from Pi. ROS2 /odometry/filtered только если
    # MQTT odom stale (>2s). Timestamp нужен для приоритета.
    mqtt_odom_ts: float = 0.0
    # Параллельные источники одометрии (этап 1A: diagnostic publish).
    # Заполняется в _h_odom — UI рендерит все треки одновременно.
    odom_sources: OdometrySources = field(default_factory=OdometrySources)


@dataclass
class _SensorsBlock:
    ultrasonic: UltrasonicData = field(default_factory=UltrasonicData)
    imu: ImuData = field(default_factory=ImuData)
    battery: BatteryStatus = field(default_factory=BatteryStatus)
    temperature: TemperatureData = field(default_factory=TemperatureData)
    watchdog: WatchdogStatus = field(default_factory=WatchdogStatus)
    # EKF bias (gx, gy, gz) — для admin/IMU debug панели
    imu_ekf_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Laser scan overlay для отрисовки на карте
    scan_points: list = field(default_factory=list)


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
    # Сырой ball_detection payload от Pi/remote-GPU (для legacy /api/detection,
    # который возвращает {detection: <raw dict>}). Структура зависит от
    # источника: {'objects': [...]} или {'balls': [...]}.
    ball_detection_raw: dict = field(default_factory=dict)


@dataclass
class _MapBlock:
    info: Optional[MapInfo] = None
    png: Optional[bytes] = None  # сгенерированный PNG карты
    slam: Optional[SlamMapData] = None
    zones: list[ForbiddenZone] = field(default_factory=list)
    zone_counter: int = 0
    saved_maps: list[str] = field(default_factory=list)
    # Когда True — карту даёт ROS2 SLAM Toolbox через /map. Когда False —
    # рендерим PNG из Pi-side ultrasonic SLAM (slam_map_node).
    ros2_map_active: bool = False
    # Счётчик обновлений Pi-side SLAM PNG (для cache busting).
    slam_map_version: int = 0


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
    calibration_coeffs: Optional[dict] = None  # {profile, scale_fwd, scale_bwd, motor_trim}
    calibration_profiles: list = field(default_factory=list)
    explorer_status: dict = field(default_factory=dict)
    mission_status: dict = field(default_factory=dict)
    mission_list: list[str] = field(default_factory=list)
    obstacle_avoidance_enabled: bool = True
    collision_guard_enabled: bool = True
    # Path planner (#3, 2026-04) — A* нода на ноутбуке.
    # path_planner_path: список waypoints [[x,y], ...] последнего планирования.
    # path_planner_status: dict {state, message?, planning_ms?}
    # path_planner_goal: текущая цель [x, y] или None если не задана.
    path_planner_path: list = field(default_factory=list)
    path_planner_status: dict = field(default_factory=dict)
    path_planner_goal: Optional[list] = None


@dataclass
class _MpsBlock:
    """МПС — Модель Пространства Состояний (course module, feat/mps).

    `applied` — матрицы, активные сейчас на роботе (synced via MQTT
    `mps/matrices/applied`). `draft` — изменённые в UI, не отправленные.
    `history` — последние 20 завершённых прогонов (FIFO). `active_run` —
    прогон в статусе running (только один на source 'robot' за раз).
    `last_telemetry` — кольцевой буфер 1000 точек (≈20 c при 50 Hz) для
    подключения посреди стрима (WS replay) и для диагностики длинных
    прогонов. Старые 200 точек = 4 c обрезали начало 10-секундного
    timeout-прогона (видели в diagnostics 2026-05-15: run #2 telemetry
    начиналась с t=6.04 c из 10.02 c — TURN-фаза целиком потеряна).
    """
    applied: Optional[MpsMatrices] = None
    draft: Optional[MpsMatrices] = None
    history: deque = field(default_factory=lambda: deque(maxlen=20))
    active_run: Optional[MpsScenarioResult] = None
    last_telemetry: deque = field(default_factory=lambda: deque(maxlen=1000))


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

        # Dirty flag (#21): producers (MQTT/ROS2 handlers, REST POSTs) call
        # mark_dirty() after mutating state. The /ws/state push loop checks
        # consume_dirty() each tick and skips the whole snapshot+serialize+
        # broadcast pipeline when nothing has changed. With 4+ connected
        # clients this is the difference between every-tick fan-out and
        # idle CPU. Initial value True so the first tick always emits an
        # initial state snapshot to fresh subscribers.
        self._dirty: bool = True

        self.robot = _RobotBlock()
        self.sensors = _SensorsBlock()
        self.actuators = _ActuatorsBlock()
        self.detection = _DetectionBlock()
        self.map = _MapBlock()
        self.control = _ControlBlock()
        self.camera = _CameraBlock()
        self.system = _SystemBlock()
        self.mps = _MpsBlock()

    # ── Dirty-flag helpers (#21) ───────────────────────────────────────
    def mark_dirty(self) -> None:
        """Producer signal that state has changed since the last consume."""
        self._dirty = True

    def consume_dirty(self) -> bool:
        """Atomically read-and-clear the dirty flag. True iff state changed."""
        # Bool read+write under the GIL is effectively atomic, but the lock
        # gives a clean memory-ordering boundary against concurrent writers
        # so a flag set DURING this consume isn't lost.
        with self.lock:
            was = self._dirty
            self._dirty = False
            return was

    # ── Atomic snapshot ────────────────────────────────────────────────
    def snapshot(self) -> dict[str, Any]:
        """
        Атомарно собирает полное состояние в dict — для /api/status, /ws/state,
        SocketIO state_update event.

        Формат совместим с тем что возвращал старый DashboardNode._snapshot()
        (для backward-compat фронта во время миграции).

        Lock pattern (#25 review, 2026-04-26):
        Pydantic models are replaced wholesale by handlers (`r.pose = new_pose`)
        rather than mutated in place, so capturing references inside the lock and
        calling model_dump() outside is safe — handlers may swap the slot but
        won't mutate the object we already hold. Mutable containers (zones,
        balls, voice_log, event_log) are shallow-copied inside the lock to make
        their iteration outside the lock equally safe.

        Result: lock is held for ~30 attribute reads + 6 list copies (a few µs)
        instead of ~30 model_dump() calls (300 µs–1 ms). Writers no longer wait
        for serialisation to finish on every dashboard tick.
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

            # Capture model references and shallow-copy mutable containers.
            # Everything below this point can run without the lock.
            pose = r.pose
            v_est = r.velocity_estimated
            v_cmd = r.velocity_commanded
            fsm = r.fsm
            speed_profile = r.speed_profile

            ultrasonic = s.ultrasonic
            imu = s.imu
            battery = s.battery
            temperature = s.temperature
            watchdog = s.watchdog

            claw = a.claw
            head = a.head
            arm = a.arm
            led = a.led

            det_result = d.result
            det_closest = d.closest
            balls = list(d.balls)             # shallow copy — list might be appended to
            det_enabled = d.enabled
            det_backend = d.backend
            det_fps = d.fps
            yolo_status = dict(d.yolo_status)

            map_info = m.info
            zones = list(m.zones)
            slam = m.slam
            saved_maps = list(m.saved_maps)

            patrol_status = dict(c.patrol_status)
            follow_me_status = dict(c.follow_me_status)
            path_status = dict(c.path_recorder_status)
            path_path = list(c.path_recorder_path)
            path_list = list(c.path_recorder_list)
            pdrive_status = dict(c.precision_drive_status)
            pdrive_result = dict(c.precision_drive_result)
            cal_status = dict(c.calibration_status)
            cal_result = dict(c.calibration_result)
            cal_active = (c.calibration_coeffs or {}).get('profile')
            cal_profiles = list(c.calibration_profiles)
            explorer_status = dict(c.explorer_status)
            mission_status = dict(c.mission_status)
            mission_list = list(c.mission_list)
            obstacle_avoidance = c.obstacle_avoidance_enabled
            collision_guard = c.collision_guard_enabled

            cam_h264 = cam.h264_endpoint
            yolo_online = cam.yolo_remote_online

            voice_log = list(sys_.voice_log)
            event_log = list(sys_.event_log)
            tts_enabled = sys_.tts_enabled
            multi_robots = list(sys_.multi_robots)
            mqtt_connected = sys_.mqtt_connected
            start_time = sys_.start_time

        # ── Lock released — model_dump() now runs concurrently with writers ──
        return {
            'ok': True,
            'sim_time': time.time() - start_time,
            'pose': pose.model_dump(),
            'velocity': {
                'estimated': v_est.model_dump(),
                'commanded': v_cmd.model_dump(),
            },
            'robot_status': fsm.model_dump(),
            'speed_profile': speed_profile,
            'sensors': {
                'ultrasonic': ultrasonic.model_dump(),
                'imu': imu.model_dump(),
            },
            'battery': battery.model_dump(),
            'temperature': temperature.model_dump(),
            'watchdog': watchdog.model_dump(),
            'actuators': {
                'claw': claw.model_dump(),
                'head': head.model_dump(),
                'arm': arm.model_dump(),
                'led': led.model_dump(),
            },
            'detection': {
                'result': det_result.model_dump(by_alias=True),
                'closest': det_closest.model_dump(by_alias=True) if det_closest else None,
                'balls': [b.model_dump() for b in balls],
                'enabled': det_enabled,
                'backend': det_backend,
                'fps': det_fps,
                'yolo_status': yolo_status,
            },
            'map': {
                'info': map_info.model_dump() if map_info else None,
                'zones': [z.model_dump() for z in zones],
                'slam': slam.model_dump() if slam else None,
                'saved_maps': saved_maps,
            },
            'control': {
                'patrol': patrol_status,
                'follow_me': follow_me_status,
                'path_recorder': {
                    'status': path_status,
                    'path': path_path,
                    'list': path_list,
                },
                'precision_drive': {
                    'status': pdrive_status,
                    'result': pdrive_result,
                },
                'calibration': {
                    'status': cal_status,
                    'result': cal_result,
                    'active': cal_active,
                    'profiles': cal_profiles,
                },
                'explorer': explorer_status,
                'mission': {
                    'status': mission_status,
                    'list': mission_list,
                },
                'obstacle_avoidance': obstacle_avoidance,
                'collision_guard': collision_guard,
            },
            'camera': {
                'h264_endpoint': cam_h264,
                'yolo_remote_online': yolo_online,
            },
            'system': {
                'voice_log': voice_log,
                'event_log': event_log,
                'tts_enabled': tts_enabled,
                'multi_robots': multi_robots,
                'mqtt_connected': mqtt_connected,
            },
        }

    # ── Convenience helpers ────────────────────────────────────────────
    def append_event_log(self, entry: dict):
        """Добавить запись в event_log с автоматическим locking."""
        with self.lock:
            self.system.event_log.append(entry)
        self.mark_dirty()

    def append_voice_log(self, entry: dict):
        with self.lock:
            self.system.voice_log.append(entry)
        self.mark_dirty()

    def add_zone(self, x1: float, y1: float, x2: float, y2: float) -> ForbiddenZone:
        """Атомарно создать зону с auto-incremented ID."""
        with self.lock:
            self.map.zone_counter += 1
            z = ForbiddenZone(
                id=self.map.zone_counter,
                x1=x1, y1=y1, x2=x2, y2=y2,
            )
            self.map.zones.append(z)
        self.mark_dirty()
        return z

    def remove_zone(self, zone_id: int) -> bool:
        """Удалить зону по ID. True если удалена."""
        with self.lock:
            before = len(self.map.zones)
            self.map.zones = [z for z in self.map.zones if z.id != zone_id]
            removed = len(self.map.zones) < before
        if removed:
            self.mark_dirty()
        return removed

    def clear_zones(self):
        with self.lock:
            had_any = len(self.map.zones) > 0
            self.map.zones = []
            self.map.zone_counter = 0
        if had_any:
            self.mark_dirty()

    # ── Legacy SocketIO push (для совместимости со старым фронтом) ───
    def legacy_socketio_state(self) -> dict[str, Any]:
        """Воспроизводит формат старого DashboardNode.get_state().

        Используется до тех пор пока фронт не переедет на REST polling +
        WebSocket /ws/state (запланировано в #6 Zustand). После C13/C12
        этот метод можно будет удалить.
        """
        import math
        with self.lock:
            r, s, a, d, m, c = (
                self.robot, self.sensors, self.actuators,
                self.detection, self.map, self.control,
            )
            cam, sys_ = self.camera, self.system

            yaw_rad = r.pose.yaw
            yaw_deg = round(math.degrees(yaw_rad), 1)

            bd = d.ball_detection_raw if isinstance(d.ball_detection_raw, dict) else {}
            det_objects = bd.get('objects', bd.get('balls', []))

            i = s.imu
            ekf = i.ekf
            return {
                'status': r.fsm.model_dump(),
                'detection': bd,
                'all_detections': det_objects,
                'range_m': s.ultrasonic.range_m,
                'imu': {
                    'yaw': i.yaw, 'pitch': i.pitch, 'roll': i.roll,
                    'gyro': i.gyro.model_dump(),
                    'accel': i.accel.model_dump(),
                },
                'imu_ypr': [i.yaw, i.pitch, i.roll],
                'imu_accel_x': i.accel.x,
                'imu_gyro_z': i.gyro.z,
                'imu_accel': [i.accel.x, i.accel.y, i.accel.z],
                'imu_gyro': [i.gyro.x, i.gyro.y, i.gyro.z],
                'imu_ypr_raw': [i.yaw, i.pitch, i.roll],
                'imu_ypr_ekf': [ekf.yaw, ekf.pitch, ekf.roll] if ekf else None,
                'imu_ekf_bias': list(s.imu_ekf_bias) if ekf else None,
                'imu_has_ekf': ekf is not None,
                'pose': {**r.pose.model_dump(), 'yaw_deg': yaw_deg},
                'odom_sources': r.odom_sources.model_dump(),
                'stationary': r.stationary,
                'velocity': {
                    **r.velocity_estimated.model_dump(),
                    'linear': r.velocity_estimated.linear_x,
                    'angular': r.velocity_estimated.angular_z,
                    'speed': abs(r.velocity_estimated.linear_x),
                },
                'cmd_velocity': r.velocity_commanded.model_dump(),
                'map_info': m.info.model_dump() if m.info else {},
                'scan_points': list(s.scan_points),
                'voice_log': list(sys_.voice_log),
                'event_log': list(sys_.event_log)[-30:],
                'battery': s.battery.model_dump(),
                'battery_voltage': s.battery.voltage,
                'battery_percent': s.battery.percent,
                'cpu_temp': s.temperature.value,
                'temperature': s.temperature.model_dump(),
                'watchdog': s.watchdog.model_dump(),
                'speed_profile': r.speed_profile,
                'actuators': {
                    'claw': 'open' if a.claw.open else 'closed',
                    'claw_open': a.claw.open,
                },
                'head': a.head.model_dump(),
                'arm': a.arm.model_dump(),
                'arm_presets': list(a.arm_presets),
                'head_presets': list(a.head_presets),
                'slam_map': m.slam.model_dump(by_alias=True) if m.slam else None,
                'path_recorder': dict(c.path_recorder_status) if c.path_recorder_status else None,
                'recorded_path': list(c.path_recorder_path) if c.path_recorder_path else None,
                'detection_enabled': d.enabled,
                'obstacle_avoidance_enabled': c.obstacle_avoidance_enabled,
                'collision_guard_enabled': c.collision_guard_enabled,
                'calibration': dict(c.calibration_status) if c.calibration_status else None,
                'calibration_result': dict(c.calibration_result) if c.calibration_result else None,
                'calibration_coeffs': dict(c.calibration_coeffs) if c.calibration_coeffs else None,
                'calibration_profiles': list(c.calibration_profiles) if c.calibration_profiles else None,
                'explorer': dict(c.explorer_status) if c.explorer_status else None,
                'mission': dict(c.mission_status) if c.mission_status else None,
                'tts_enabled': sys_.tts_enabled,
                'precision_drive': dict(c.precision_drive_status) if c.precision_drive_status else None,
                'precision_drive_result': dict(c.precision_drive_result) if c.precision_drive_result else None,
                'led': a.led.model_dump(),
                'sim_time': round(time.time() - sys_.start_time, 2),
                'zones': [z.model_dump() for z in m.zones],
                'planned_path': list(c.path_recorder_path) if c.path_recorder_path else [],
                'balls': [b.model_dump() for b in d.balls],
                'camera': {
                    'h264_endpoint': cam.h264_endpoint,
                    'yolo_remote_online': cam.yolo_remote_online,
                },
            }
