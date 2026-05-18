"""
MQTTHandlers — paho-mqtt клиент с подписками на Pi-топики.

Все callbacks обновляют DashboardState (через state.lock). Команды
публикуются методом publish() с префиксом samurai/{robot_id}/.

Ранее этот код жил в DashboardNode._mqtt_* методах внутри
compute_node/dashboard_node.py (~400 строк). Извлечён в C4 (#7) для
разбивки монолита.

Использование:
    handlers = MQTTHandlers('192.168.1.10', 1883, 'robot1', state,
                            username='admin', password='secret')
    handlers.start()  # async connect + loop_start
    ...
    handlers.publish('cmd_vel/manual', {'linear_x': 0.2, 'angular_z': 0.0})
    ...
    handlers.stop()
"""
from __future__ import annotations

import json
import logging
import math
import time
from typing import Callable, Optional

import paho.mqtt.client as mqtt_client

from .schemas.actuators import ArmState, ClawState, HeadState, LedState
from .schemas.maps import MapInfo, SlamMapData, SlamObject
from .schemas.robot import OdometrySources, RobotPose, RobotStatus, VelocityDetail
from .schemas.sensors import (
    BatteryStatus,
    ImuData,
    ImuYpr,
    TemperatureData,
    UltrasonicData,
    Vec3,
    WatchdogStatus,
)
from .state import DashboardState

log = logging.getLogger(__name__)


# Sensor topics для подписки (после connect). Список ровно тот, что был
# в DashboardNode._mqtt_on_connect — сохраняем 1:1 для backward-compat.
# 'camera' (JPEG) удалён 2026-04 в #9 — H.264 идёт TCP→/ws/h264.
_SUBSCRIBE_TOPICS = [
    'camera/endpoint', 'range', 'imu', 'battery', 'temperature',
    'status', 'odom', 'claw/state', 'head/state', 'arm/state',
    'watchdog', 'voice_command', 'speed_profile/active',
    'slam_map', 'path_recorder/status', 'path_recorder/path',
    'calibration/status', 'calibration/result',
    'calibration/active', 'calibration/profile/all',
    'calibration/profile/saved', 'calibration/profile/error',
    'arm/presets', 'head/presets',
    'explorer/status', 'mission/status',
    'precision_drive/status', 'precision_drive/result',
    'collision_guard/state',
    # Path planner на ноутбуке (#3, 2026-04)
    'path_planner/path', 'path_planner/status',
    # Remote GPU YOLO детекции (через MQTT)
    'ball_detection', 'detections', 'yolo/annotated', 'yolo/status',
    # Centralized log events с Pi-нод
    'log/events',
    # LED панель
    'led/state',
    # МПС — Модель Пространства Состояний (учебный модуль, feat/mps).
    # Telemetry — 50 Гц QoS 0; остальные QoS 1.
    'mps/matrices/applied',
    'mps/scenario/finished',
    'mps/telemetry',
    'mps/error',
    'mps/live_state',
]


class MQTTHandlers:
    """paho-mqtt клиент + диспатч топиков → DashboardState мутаций.

    Один экземпляр на dashboard. Безопасно вызывать publish() из любого
    потока (paho сам синхронизирует send queue).
    """

    def __init__(
        self,
        broker: str,
        port: int,
        robot_id: str,
        state: DashboardState,
        username: Optional[str] = None,
        password: Optional[str] = None,
        client_id: str = 'samurai_dashboard',
    ):
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._prefix = f'samurai/{robot_id}'
        self._state = state
        # mps/live_state — последний фрейм для send-on-connect.
        self._last_live_state: Optional[dict] = None

        self._client = mqtt_client.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)
        if username:
            self._client.username_pw_set(username, password or '')
        self._auth_str = f' user={username}' if username else ' anonymous'

    # ── Lifecycle ──────────────────────────────────────────────────
    def start(self) -> None:
        """Async connect + start network thread. Не блокирует."""
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        log.info(
            'MQTT client connecting → %s:%d%s',
            self._broker, self._port, self._auth_str
        )

    def stop(self) -> None:
        """Disconnect + остановить сетевой поток."""
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        with self._state.lock:
            return self._state.system.mqtt_connected

    # ── Publish (command path) ─────────────────────────────────────
    def publish(self, subtopic: str, payload, qos: int = 0) -> bool:
        """Опубликовать в samurai/{robot_id}/{subtopic}.

        payload: str | bytes | dict | list. dict/list сериализуется в JSON.
        Возвращает False если broker не подключён (warning в лог).
        """
        if not self.connected:
            log.warning('MQTT not connected — cannot publish %s', subtopic)
            return False
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        self._client.publish(f'{self._prefix}/{subtopic}', payload, qos=qos)
        return True

    # ── Connection callbacks ───────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed rc=%s', rc)
            return
        with self._state.lock:
            self._state.system.mqtt_connected = True
        for t in _SUBSCRIBE_TOPICS:
            client.subscribe(f'{self._prefix}/{t}', qos=0)
        log.info(
            'MQTT connected to %s — subscribed to %d topics',
            self._broker, len(_SUBSCRIBE_TOPICS)
        )

    def _on_disconnect(self, client, userdata, rc):
        with self._state.lock:
            self._state.system.mqtt_connected = False
        if rc != 0:
            log.warning('MQTT disconnected rc=%s, reconnecting...', rc)

    # ── Message dispatch ───────────────────────────────────────────
    def _on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self._prefix) + 1:]
        handler = self._dispatch.get(suffix)
        if handler is None:
            return
        try:
            handler(self, msg.payload)
        except Exception as exc:
            log.error('MQTT msg error [%s]: %s', suffix, exc)
            return
        # Single mark_dirty() after every successful MQTT-driven mutation.
        # The push loop reads-and-clears this flag to decide whether the
        # next tick needs a fresh snapshot+broadcast (#21).
        self._state.mark_dirty()

    # ── Per-topic handlers ─────────────────────────────────────────
    def _h_camera_endpoint(self, payload: bytes):
        if not payload:
            with self._state.lock:
                self._state.camera.h264_endpoint = None
            log.warning('Camera endpoint cleared (Pi camera offline)')
            return
        try:
            data = json.loads(payload)
        except Exception as e:
            log.error('Bad camera/endpoint JSON: %s', e)
            return
        with self._state.lock:
            self._state.camera.h264_endpoint = data
        log.info(
            'Camera endpoint: %s:%s (%s %sx%s @%sfps)',
            data.get('host'), data.get('port'),
            data.get('codec'), data.get('width'), data.get('height'),
            data.get('fps')
        )

    def _h_range(self, payload: bytes):
        try:
            d = json.loads(payload)
            r = float(d.get('range', d) if isinstance(d, dict) else d)
        except Exception:
            try:
                r = float(payload)
            except Exception:
                return
        with self._state.lock:
            self._state.sensors.ultrasonic = UltrasonicData(
                range_m=round(r, 3),
                age_s=0.0,
            )

    def _h_imu(self, payload: bytes):
        d = json.loads(payload)
        gx = d.get('gx', 0.0)
        gy = d.get('gy', 0.0)
        gz = d.get('gz', 0.0)
        ax = d.get('ax', 0.0)
        ay = d.get('ay', 0.0)
        az = d.get('az', 1.0)
        # Raw pitch/roll из акселя (yaw недоступен без магнитометра)
        pitch_raw = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        roll_raw = math.degrees(math.atan2(-ax, az))
        ekf = d.get('ekf')
        ekf_obj: Optional[ImuYpr] = None
        ekf_bias = None
        if ekf:
            ekf_obj = ImuYpr(
                yaw=round(ekf.get('yaw', 0.0), 1),
                pitch=round(ekf.get('pitch', 0.0), 1),
                roll=round(ekf.get('roll', 0.0), 1),
            )
            ekf_bias = [
                ekf.get('bias_gx', 0.0),
                ekf.get('bias_gy', 0.0),
                ekf.get('bias_gz', 0.0),
            ]
        with self._state.lock:
            # Главные yaw/pitch/roll: EKF если есть, иначе raw
            yaw = ekf_obj.yaw if ekf_obj else 0.0
            pitch = ekf_obj.pitch if ekf_obj else round(pitch_raw, 1)
            roll = ekf_obj.roll if ekf_obj else round(roll_raw, 1)
            self._state.sensors.imu = ImuData(
                yaw=yaw, pitch=pitch, roll=roll,
                gyro=Vec3(x=round(gx, 4), y=round(gy, 4), z=round(gz, 4)),
                accel=Vec3(x=round(ax, 4), y=round(ay, 4), z=round(az, 4)),
                ekf=ekf_obj,
            )
            if ekf_bias is not None:
                self._state.sensors.imu_ekf_bias = ekf_bias

    def _h_battery(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.sensors.battery = BatteryStatus(
                voltage=d.get('voltage', d.get('v', -1.0)),
                percent=d.get('percent', d.get('percentage', -1)),
                status=d.get('status'),
            )

    def _h_temperature(self, payload: bytes):
        # temperature_node.py публикует голый float (`self.publish('temperature', 42.5)`),
        # старые publishers могли слать dict {value, unit}. json.loads успешно парсит
        # `b'42.5'` как float — без isinstance-ветки d.get() упадёт AttributeError.
        try:
            d = json.loads(payload)
        except Exception:
            try:
                d = {'value': float(payload), 'unit': 'C'}
            except Exception:
                return
        if isinstance(d, (int, float)):
            d = {'value': float(d), 'unit': 'C'}
        elif not isinstance(d, dict):
            return
        with self._state.lock:
            self._state.sensors.temperature = TemperatureData(
                value=d.get('value', -1.0),
                unit=d.get('unit', 'C'),
            )

    def _h_status(self, payload: bytes):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            d = json.loads(text)
        except Exception:
            return
        with self._state.lock:
            self._state.robot.fsm = RobotStatus(
                state=d.get('state', d.get('fsm_state', 'IDLE')),
                target_colour=d.get('target_colour', d.get('target', '')),
                target_action=d.get('target_action', d.get('action', '')),
            )

    def _h_odom(self, payload: bytes):
        d = json.loads(payload)
        x = d.get('x', 0.0)
        y = d.get('y', 0.0)
        theta = d.get('theta', 0.0)
        vx = d.get('vx', 0.0)
        vz = d.get('vz', 0.0)
        stationary = d.get('stationary', False)
        # x/y из payload — в сантиметрах, конвертируем в метры для UI.
        # Дополнительные поля (x_wheel, x_imu, ...) — diagnostic для
        # сравнения wheel/IMU/complementary/EKF на дашборде (этап 1A).
        x_wheel_cm = d.get('x_wheel', d.get('x', 0.0))
        y_wheel_cm = d.get('y_wheel', d.get('y', 0.0))
        x_imu_cm = d.get('x_imu', 0.0)
        y_imu_cm = d.get('y_imu', 0.0)
        with self._state.lock:
            self._state.robot.pose = RobotPose(
                x=round(x, 3), y=round(y, 3), yaw=round(theta, 3))
            self._state.robot.velocity_estimated = VelocityDetail(
                linear_x=round(vx, 3),
                linear_y=0.0,
                angular_z=round(vz, 3),
            )
            self._state.robot.stationary = bool(stationary)
            self._state.robot.mqtt_odom_ts = time.time()
            self._state.robot.odom_sources = OdometrySources(
                x_wheel=round(x_wheel_cm / 100.0, 4),
                y_wheel=round(y_wheel_cm / 100.0, 4),
                x_imu=round(x_imu_cm / 100.0, 4),
                y_imu=round(y_imu_cm / 100.0, 4),
                vx_imu=round(d.get('vx_imu', 0.0), 3),
                vy_imu=round(d.get('vy_imu', 0.0), 3),
                stationary_imu=bool(d.get('stationary_imu', True)),
                source=str(d.get('source', 'wheel')),
            )

    def _h_claw_state(self, payload: bytes):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            d = json.loads(text) if text.startswith('{') else {'angle': int(text)}
        except Exception:
            return
        angle = d.get('angle', 90)
        with self._state.lock:
            self._state.actuators.claw = ClawState(
                open=angle < 60, angle=float(angle))

    def _h_head_state(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.actuators.head = HeadState(
                angle=d.get('angle', 90.0),
                frozen=d.get('frozen', False),
                locked=d.get('locked', True),
            )

    def _h_arm_state(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.actuators.arm = ArmState(
                j1=d.get('j1', 0.0),
                j2=d.get('j2', 120.0),
                j3=d.get('j3', 0.0),
                j4=d.get('j4', 0.0),
                frozen=d.get('frozen', [False] * 4),
                locked=d.get('locked', True),
            )
            # j4 — клешня; обновляем claw open флаг
            j4 = d.get('j4', 90)
            self._state.actuators.claw = ClawState(
                open=j4 < 60, angle=float(j4))

    def _h_watchdog(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.sensors.watchdog = WatchdogStatus(
                online=d.get('online', False),
                last_seen_s=d.get('last_seen_s'),
                critical_topics=d.get('critical_topics', {}),
            )

    def _h_voice_command(self, payload: bytes):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        except Exception:
            return
        entry = {'text': text, 'time': time.strftime('%H:%M:%S'), 'type': 'voice'}
        self._state.append_voice_log(entry)
        self._state.append_event_log(entry)

    def _h_speed_profile(self, payload: bytes):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.robot.speed_profile = text.strip()

    def _h_slam_map(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            try:
                slam = SlamMapData(
                    obstacles=d.get('obstacles', []),
                    trail=d.get('trail', []),
                    robot=d.get('robot', {}),
                    detected_objects=[
                        SlamObject(
                            id=str(o.get('id', '')),
                            cls=o.get('class', o.get('cls', 'unknown')),
                            colour=o.get('colour', 'unknown'),
                            x=o.get('x', 0.0),
                            y=o.get('y', 0.0),
                            conf=o.get('conf', 0.0),
                            dist=o.get('dist', -1.0),
                            count=o.get('count', 1),
                            ts=o.get('ts', 0.0),
                        )
                        for o in d.get('detected_objects', [])
                    ],
                    info=MapInfo(
                        width=d['info'].get('width', 200),
                        height=d['info'].get('height', 200),
                        resolution=d['info'].get('resolution', 0.05),
                        origin_x=d['info'].get('origin_x', -5.0),
                        origin_y=d['info'].get('origin_y', -5.0),
                    ) if 'info' in d else None,
                    stats=d.get('stats', {}),
                    ts=d.get('ts', 0.0),
                )
            except Exception as exc:
                log.error('SlamMapData parse error: %s', exc)
                return
            self._state.map.slam = slam
            # Если ROS2 SLAM Toolbox молчит — Pi-side SLAM становится
            # источником карты для /api/map/info (PNG генерится по запросу).
            if not self._state.map.ros2_map_active and slam.info is not None:
                self._state.map.info = slam.info
                self._state.map.slam_map_version += 1

    def _h_path_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.path_recorder_status = d

    def _h_path_data(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.path_recorder_path = d if isinstance(d, list) else []

    def _h_calibration_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.calibration_status = d

    def _h_calibration_result(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.calibration_result = d

    def _h_calibration_active(self, payload: bytes):
        """motor_node публикует {profile, scale_fwd, scale_bwd, motor_trim}.
        Сохраняем весь dict — фронт ждёт полную структуру.
        Неполный/некорректный payload игнорируем (не затираем хороший state).
        """
        try:
            d = json.loads(payload)
        except Exception:
            return
        if not isinstance(d, dict):
            return
        required = ('profile', 'scale_fwd', 'scale_bwd', 'motor_trim')
        if not all(k in d for k in required):
            return
        try:
            new_coeffs = {
                'profile': str(d['profile']),
                'scale_fwd': float(d['scale_fwd']),
                'scale_bwd': float(d['scale_bwd']),
                'motor_trim': float(d['motor_trim']),
            }
        except (TypeError, ValueError):
            return
        with self._state.lock:
            self._state.control.calibration_coeffs = new_coeffs
            self._state.control.calibration_active_profile = new_coeffs['profile']

    def _h_calibration_profile_all(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            if isinstance(d, list):
                self._state.control.calibration_profiles = d
            elif isinstance(d, dict):
                self._state.control.calibration_profiles = d.get('profiles', [])

    def _h_arm_presets(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.actuators.arm_presets = d if isinstance(d, dict) else {}

    def _h_head_presets(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.actuators.head_presets = d if isinstance(d, dict) else {}

    def _h_explorer_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.explorer_status = d

    def _h_mission_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.mission_status = d

    def _h_precision_drive_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.precision_drive_status = d

    def _h_precision_drive_result(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.precision_drive_result = d

    def _h_path_planner_path(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.path_planner_path = d.get('waypoints', [])
            self._state.control.path_planner_goal = d.get('goal')

    def _h_path_planner_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.control.path_planner_status = d

    def _h_collision_guard_state(self, payload: bytes):
        try:
            val = payload.decode('utf-8', errors='ignore').strip().lower()
        except Exception:
            return
        with self._state.lock:
            self._state.control.collision_guard_enabled = (val == 'on')

    def _h_ball_detection(self, payload: bytes):
        try:
            data = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.detection.ball_detection_raw = data

    def _h_yolo_annotated(self, payload: bytes):
        with self._state.lock:
            self._state.detection.annotated_jpeg = bytes(payload)

    def _h_yolo_status(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        online = d.get('online', False)
        with self._state.lock:
            self._state.detection.yolo_status = d
            self._state.camera.yolo_remote_online = online
        log.info('Remote GPU YOLO: %s', 'ONLINE' if online else 'OFFLINE')

    def _h_log_event(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        entry = {
            'ts': d.get('ts', time.time()),
            'source': d.get('node', '?'),
            'level': d.get('level', 'INFO'),
            'text': d.get('msg', ''),
        }
        self._state.append_event_log(entry)

    def _h_led_state(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            self._state.actuators.led = LedState(
                animation=d.get('animation', d.get('mode', 'off')),
                color=d.get('color'),
                brightness=d.get('brightness', 76),
                speed=d.get('speed', 1.0),
            )

    def _h_noop(self, payload: bytes):
        """Подписка для трейса; payload игнорируется."""
        pass

    # ── МПС — Модель Пространства Состояний (feat/mps) ────────────
    # Hook для broadcast в WebSocket /ws/mps/telemetry. Регистрируется
    # из app.py при создании WebSocket-эндпойнта; None по умолчанию —
    # тогда фрейм просто оседает в state.last_telemetry.
    _mps_ws_broadcaster: Optional[Callable] = None
    # Hook для broadcast в WebSocket /ws/mps/live_state.
    _mps_live_state_broadcaster: Optional[Callable] = None

    def set_mps_ws_broadcaster(self, broadcaster: Optional[Callable]) -> None:
        """Plumbing: app.py регистрирует функцию `broadcast(frame: dict)`."""
        self._mps_ws_broadcaster = broadcaster

    def set_mps_live_state_broadcaster(self, broadcaster: Optional[Callable]) -> None:
        """Plumbing: app.py регистрирует функцию `broadcast(frame: dict)`."""
        self._mps_live_state_broadcaster = broadcaster

    def _broadcast_mps(self, frame: dict) -> None:
        cb = self._mps_ws_broadcaster
        if cb is None:
            return
        try:
            cb(frame)
        except Exception as exc:
            log.warning('mps WS broadcast failed: %s', exc)

    def _h_mps_matrices_applied(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        from .schemas.mps import MpsMatrices
        try:
            m = MpsMatrices.model_validate(d.get('matrices', {}))
        except Exception as exc:
            log.warning('mps/matrices/applied: bad payload: %s', exc)
            return
        with self._state.lock:
            self._state.mps.applied = m
            self._state.mps.draft = None

    def _h_mps_scenario_finished(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        from .schemas.mps import MpsScenarioResult
        # robot publishes без telemetry; добиваем буфером из state.
        with self._state.lock:
            telemetry = list(self._state.mps.last_telemetry)
            applied = self._state.mps.applied
        d.setdefault('telemetry', telemetry)
        d.setdefault('matrices_snapshot', applied.model_dump() if applied else None)
        try:
            result = MpsScenarioResult.model_validate(d)
        except Exception as exc:
            log.warning('mps/scenario/finished: bad payload: %s', exc)
            return
        with self._state.lock:
            # Idempotent: если /scenario/abort уже положил запись с этим
            # run_id в history (status='aborted'), заменяем её — иначе при
            # позднем mps/scenario/finished от Pi будет дубликат.
            history = self._state.mps.history
            for i, past in enumerate(history):
                if past.run_id == result.run_id:
                    history[i] = result
                    break
            else:
                history.appendleft(result)
            if (self._state.mps.active_run is not None and
                    self._state.mps.active_run.run_id == result.run_id):
                self._state.mps.active_run = None
            self._state.mps.last_telemetry.clear()
        self._broadcast_mps({
            'type': 'finished',
            'run_id': result.run_id,
            'result': result.model_dump(mode='json'),
        })

    def _h_mps_telemetry(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        from .schemas.mps import MpsTelemetryPoint
        run_id = str(d.get('run_id', ''))
        try:
            point = MpsTelemetryPoint.model_validate(d.get('point', {}))
        except Exception:
            return
        with self._state.lock:
            self._state.mps.last_telemetry.append(point)
            # Update active_run.telemetry on the fly (in case fronт делает poll).
            active = self._state.mps.active_run
            if active is not None and active.run_id == run_id:
                # Pydantic immutable list — пересоберём через model_copy.
                # Но active_run.telemetry — это list, его можно append'ать.
                active.telemetry.append(point)
        self._broadcast_mps({
            'type': 'telemetry',
            'run_id': run_id,
            'point': point.model_dump(mode='json'),
        })

    def _h_mps_error(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        log.warning('mps error from Pi: %s', d.get('message', d))
        # Прокинем во фронт через WS если есть подписчики.
        self._broadcast_mps({
            'type': 'error',
            'run_id': d.get('run_id'),
            'error_type': d.get('error_type', 'other'),
            'message': str(d.get('message', '')),
        })

    def _h_mps_live_state(self, payload: bytes):
        """Обновляет _last_live_state и шлёт фрейм в /ws/mps/live_state.

        Контракт: docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.1.
        Невалидные payload'ы (некорректные длины x/u, отсутствующие ключи)
        молча игнорируются — это live-канал, дроп лучше падения.
        """
        try:
            d = json.loads(payload)
        except Exception:
            return
        if not isinstance(d, dict):
            return
        try:
            x = d['x']
            u = d['u']
        except KeyError:
            return
        if not (isinstance(x, list) and len(x) == 5
                and isinstance(u, list) and len(u) == 2):
            return
        with self._state.lock:
            self._last_live_state = d
        cb = self._mps_live_state_broadcaster
        if cb is not None:
            try:
                cb({'type': 'live_state', 'point': d})
            except Exception as exc:
                log.warning('mps live_state WS broadcast failed: %s', exc)

    # ── Топик → handler dispatch ────────────────────────────────────
    # Заполняется ниже после определения класса (Python требует сначала
    # завершить class body чтобы методы стали bound).
    _dispatch: dict[str, Callable] = {}


# Сборка диспатчера — после определения класса, чтобы избежать forward refs.
MQTTHandlers._dispatch = {
    'camera/endpoint': MQTTHandlers._h_camera_endpoint,
    'range': MQTTHandlers._h_range,
    'imu': MQTTHandlers._h_imu,
    'battery': MQTTHandlers._h_battery,
    'temperature': MQTTHandlers._h_temperature,
    'status': MQTTHandlers._h_status,
    'odom': MQTTHandlers._h_odom,
    'claw/state': MQTTHandlers._h_claw_state,
    'head/state': MQTTHandlers._h_head_state,
    'arm/state': MQTTHandlers._h_arm_state,
    'watchdog': MQTTHandlers._h_watchdog,
    'voice_command': MQTTHandlers._h_voice_command,
    'speed_profile/active': MQTTHandlers._h_speed_profile,
    'slam_map': MQTTHandlers._h_slam_map,
    'path_recorder/status': MQTTHandlers._h_path_status,
    'path_recorder/path': MQTTHandlers._h_path_data,
    'calibration/status': MQTTHandlers._h_calibration_status,
    'calibration/result': MQTTHandlers._h_calibration_result,
    'calibration/active': MQTTHandlers._h_calibration_active,
    'calibration/profile/all': MQTTHandlers._h_calibration_profile_all,
    'calibration/profile/saved': MQTTHandlers._h_noop,
    'calibration/profile/error': MQTTHandlers._h_noop,
    'arm/presets': MQTTHandlers._h_arm_presets,
    'head/presets': MQTTHandlers._h_head_presets,
    'explorer/status': MQTTHandlers._h_explorer_status,
    'mission/status': MQTTHandlers._h_mission_status,
    'precision_drive/status': MQTTHandlers._h_precision_drive_status,
    'precision_drive/result': MQTTHandlers._h_precision_drive_result,
    'path_planner/path': MQTTHandlers._h_path_planner_path,
    'path_planner/status': MQTTHandlers._h_path_planner_status,
    'collision_guard/state': MQTTHandlers._h_collision_guard_state,
    'ball_detection': MQTTHandlers._h_ball_detection,
    'detections': MQTTHandlers._h_noop,
    'yolo/annotated': MQTTHandlers._h_yolo_annotated,
    'yolo/status': MQTTHandlers._h_yolo_status,
    'log/events': MQTTHandlers._h_log_event,
    'led/state': MQTTHandlers._h_led_state,
    # МПС — Модель Пространства Состояний (учебный модуль, feat/mps)
    'mps/matrices/applied': MQTTHandlers._h_mps_matrices_applied,
    'mps/scenario/finished': MQTTHandlers._h_mps_scenario_finished,
    'mps/telemetry': MQTTHandlers._h_mps_telemetry,
    'mps/error': MQTTHandlers._h_mps_error,
    'mps/live_state': MQTTHandlers._h_mps_live_state,
}
