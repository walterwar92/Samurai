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
from .schemas.robot import RobotPose, RobotStatus, VelocityDetail
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
    # Remote GPU YOLO детекции (через MQTT)
    'ball_detection', 'detections', 'yolo/annotated', 'yolo/status',
    # Centralized log events с Pi-нод
    'log/events',
    # LED панель
    'led/state',
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
        try:
            d = json.loads(payload)
        except Exception:
            try:
                d = {'value': float(payload), 'unit': 'C'}
            except Exception:
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
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            if isinstance(d, dict):
                self._state.control.calibration_active_profile = d.get('name')
            else:
                self._state.control.calibration_active_profile = str(d)

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
    'collision_guard/state': MQTTHandlers._h_collision_guard_state,
    'ball_detection': MQTTHandlers._h_ball_detection,
    'detections': MQTTHandlers._h_noop,
    'yolo/annotated': MQTTHandlers._h_yolo_annotated,
    'yolo/status': MQTTHandlers._h_yolo_status,
    'log/events': MQTTHandlers._h_log_event,
    'led/state': MQTTHandlers._h_led_state,
}
