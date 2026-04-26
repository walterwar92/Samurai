"""
FrameSource — источник кадров для детектора.

Три варианта:
  - H264TCPFrameSource: MQTT discovery + TCP H.264 stream (default с 2026-04)
  - MQTTFrameSource: legacy paho.mqtt подписка на samurai/{id}/camera (JPEG)
                     [DEPRECATED, оставлено как stub для tests]
  - ROS2FrameSource: rclpy подписка на /camera/image_raw/compressed + /odom + /range

Все источники поддерживают on_frame(callback) с FrameContext (BGR + ленивый HSV
+ актуальная поза робота + последнее показание ультразвука).
"""
from __future__ import annotations

import json
import logging
import math
import os
import sys
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Optional

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

from .base import FrameContext, RobotPose

log = logging.getLogger(__name__)

# Optional MQTT credentials resolver (общий с другими compute_node)
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
    from config_loader import get_mqtt_credentials as _resolve_mqtt_creds
except ImportError:
    def _resolve_mqtt_creds():
        u = os.environ.get('SAMURAI_MQTT_USER', '').strip()
        p = os.environ.get('SAMURAI_MQTT_PASS', '')
        return (u, p) if u and p else (None, None)


FrameCallback = Callable[[FrameContext], None]


class FrameSource(ABC):
    """ABC: подписывается на источник кадров и вызывает callback(FrameContext)."""

    @abstractmethod
    def start(self): ...
    @abstractmethod
    def stop(self): ...
    @abstractmethod
    def on_frame(self, callback: FrameCallback): ...

    @property
    def name(self) -> str:
        return self.__class__.__name__


# ─────────────────────────────────────────────────────────────────────────────
# MQTT
# ─────────────────────────────────────────────────────────────────────────────
class MQTTFrameSource(FrameSource):
    """
    Подписка на samurai/{robot_id}/camera (бинарный JPEG) + odom + range.

    Обновляет внутренний FrameContext (pose, ultrasonic) на лету,
    при поступлении кадра вызывает callback(ctx).
    """

    def __init__(self,
                 broker: str,
                 port: int = 1883,
                 robot_id: str = 'robot1',
                 client_id: str = 'samurai_detector',
                 mqtt_user: Optional[str] = None,
                 mqtt_pwd: Optional[str] = None,
                 will_topic: Optional[str] = None,
                 will_payload: Optional[bytes] = None):
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) необходим для MQTTFrameSource')
        try:
            import paho.mqtt.client as mqtt  # type: ignore
        except ImportError as e:
            raise RuntimeError('paho-mqtt не установлен') from e

        self._prefix = f'samurai/{robot_id}'
        self._broker = broker
        self._port = port
        self._callback: Optional[FrameCallback] = None

        self._pose = RobotPose()
        self._range_m = 2.0
        self._range_ts = 0.0

        self._client = mqtt.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)

        if will_topic and will_payload is not None:
            self._client.will_set(will_topic, will_payload, qos=1, retain=True)

        # Auth
        if mqtt_user is None and mqtt_pwd is None:
            mqtt_user, mqtt_pwd = _resolve_mqtt_creds()
        if mqtt_user is not None:
            self._client.username_pw_set(mqtt_user, mqtt_pwd)
            log.info('MQTT auth: user=%s', mqtt_user)

    def on_frame(self, callback: FrameCallback):
        self._callback = callback

    def start(self):
        log.info('MQTTFrameSource connecting → %s:%d', self._broker, self._port)
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()

    def stop(self):
        self._client.loop_stop()
        self._client.disconnect()

    @property
    def client(self):
        """Доступ к paho-клиенту (для публикаций из того же соединения)."""
        return self._client

    @property
    def topic_prefix(self) -> str:
        return self._prefix

    # ── MQTT callbacks ─────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed (rc=%d)', rc)
            return
        log.info('MQTT connected — subscribing to camera/range/odom')
        client.subscribe(f'{self._prefix}/camera', qos=0)
        client.subscribe(f'{self._prefix}/range',  qos=0)
        client.subscribe(f'{self._prefix}/odom',   qos=0)

    def _on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self._prefix) + 1:]
        if suffix == 'camera':
            # DEPRECATED: camera publishes H.264 over TCP since 2026-04.
            # Этот handler оставлен для обратной совместимости с simulator.py
            # пока simulator не мигрирует на H.264.
            self._handle_camera(msg.payload)
        elif suffix == 'range':
            self._handle_range(msg.payload)
        elif suffix == 'odom':
            self._handle_odom(msg.payload)

    def _handle_camera(self, payload: bytes):
        """[DEPRECATED] JPEG payload from MQTT — for simulator compat only."""
        if self._callback is None:
            return
        np_arr = np.frombuffer(payload, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        ctx = FrameContext(
            bgr=frame,
            pose=RobotPose(self._pose.x, self._pose.y, self._pose.theta, self._pose.valid),
            ultrasonic_m=self._range_m,
            ultrasonic_age_s=time.monotonic() - self._range_ts if self._range_ts else 999.0,
            timestamp=time.time(),
        )
        try:
            self._callback(ctx)
        except Exception as e:
            log.error('Frame callback error: %s', e)

    def _handle_range(self, payload: bytes):
        try:
            data = json.loads(payload)
        except Exception:
            try:
                data = float(payload.decode('utf-8', 'ignore'))
                self._range_m = data
                self._range_ts = time.monotonic()
                return
            except Exception:
                return
        if isinstance(data, dict):
            try:
                r = float(data.get('range', 2.0))
                if 0.02 <= r <= 2.0:
                    self._range_m = r
                    self._range_ts = time.monotonic()
            except (TypeError, ValueError):
                pass

    def _handle_odom(self, payload: bytes):
        try:
            data = json.loads(payload)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        try:
            self._pose = RobotPose(
                x=float(data.get('x', 0.0)) / 100.0,    # cm → m
                y=float(data.get('y', 0.0)) / 100.0,
                theta=float(data.get('theta', 0.0)),
                valid=True,
            )
        except (TypeError, ValueError):
            pass


# ─────────────────────────────────────────────────────────────────────────────
# ROS2
# ─────────────────────────────────────────────────────────────────────────────
class ROS2FrameSource(FrameSource):
    """
    Подписка на /camera/image_raw/compressed + /odom + /range через rclpy.

    Запускает rclpy.spin в фоне через SingleThreadedExecutor (не блокирует main).
    """

    def __init__(self, node_name: str = 'samurai_detector_ros2'):
        try:
            import rclpy  # type: ignore  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                'rclpy недоступен. ROS2FrameSource работает только в ROS2-окружении'
            ) from e
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) необходим')

        self._node_name = node_name
        self._callback: Optional[FrameCallback] = None
        self._pose = RobotPose()
        self._range_m = 2.0
        self._range_ts = 0.0
        self._spin_thread: Optional[threading.Thread] = None
        self._executor = None
        self._node = None

    def on_frame(self, callback: FrameCallback):
        self._callback = callback

    def start(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import CompressedImage, Range
        from nav_msgs.msg import Odometry

        rclpy.init(args=None)
        self._node = Node(self._node_name)

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self._node.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_image, cam_qos)
        self._node.create_subscription(Range, '/range', self._on_range, 10)
        self._node.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name='ros2_spin')
        self._spin_thread.start()
        log.info('ROS2FrameSource started (node=%s)', self._node_name)

    def stop(self):
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

    @property
    def node(self):
        """Доступ к rclpy.Node (для создания публикаторов в том же контексте)."""
        return self._node

    # ── Subscriptions ──────────────────────────────────────────
    def _on_image(self, msg):
        if self._callback is None:
            return
        np_arr = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        ctx = FrameContext(
            bgr=frame,
            pose=RobotPose(self._pose.x, self._pose.y, self._pose.theta, self._pose.valid),
            ultrasonic_m=self._range_m,
            ultrasonic_age_s=time.monotonic() - self._range_ts if self._range_ts else 999.0,
            timestamp=time.time(),
        )
        try:
            self._callback(ctx)
        except Exception as e:
            log.error('Frame callback error: %s', e)

    def _on_range(self, msg):
        try:
            r = float(msg.range)
            if 0.02 <= r <= 2.0:
                self._range_m = r
                self._range_ts = time.monotonic()
        except Exception:
            pass

    def _on_odom(self, msg):
        try:
            p = msg.pose.pose
            qz = p.orientation.z; qw = p.orientation.w
            theta = 2.0 * math.atan2(qz, qw)
            self._pose = RobotPose(
                x=p.position.x, y=p.position.y, theta=theta, valid=True)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# H.264 over TCP (с 2026-04 — primary source)
# ─────────────────────────────────────────────────────────────────────────────
class H264TCPFrameSource(FrameSource):
    """
    Получает H.264 поток от camera_node через TCP, декодирует через PyAV.

    Pipeline:
      MQTT discovery → samurai/{id}/camera/endpoint → {host, port}
        → TCP connect → raw H.264 NAL units (Annex B)
        → PyAV CodecContext.parse + decode → av.VideoFrame
        → frame.to_ndarray(format='bgr24') → np.ndarray BGR
        → FrameContext → callback

    Также подписывается на range/odom через тот же MQTT-клиент чтобы
    обновлять pose и ultrasonic_age в FrameContext (как MQTTFrameSource).

    Reconnect logic: при потере TCP соединения — exponential backoff,
    при изменении discovery (Pi сменил IP) — переподключиться.
    """

    READ_BUFFER_SIZE = 64 * 1024
    RECONNECT_INITIAL = 0.5
    RECONNECT_MAX = 5.0

    def __init__(self,
                 broker: str,
                 port: int = 1883,
                 robot_id: str = 'robot1',
                 client_id: str = 'samurai_h264_source',
                 mqtt_user: Optional[str] = None,
                 mqtt_pwd: Optional[str] = None):
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) необходим для H264TCPFrameSource')
        try:
            import av  # type: ignore  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                'PyAV (av) не установлен. pip install av — нужен для H.264 декодирования'
            ) from e
        try:
            import paho.mqtt.client as mqtt  # type: ignore
        except ImportError as e:
            raise RuntimeError('paho-mqtt не установлен') from e

        self._prefix = f'samurai/{robot_id}'
        self._broker = broker
        self._port = port
        self._callback: Optional[FrameCallback] = None

        # Robot state (обновляется через MQTT odom/range)
        self._pose = RobotPose()
        self._range_m = 2.0
        self._range_ts = 0.0

        # Camera endpoint (обновляется через MQTT discovery)
        self._endpoint: Optional[dict] = None
        self._endpoint_lock = threading.Lock()
        self._endpoint_changed = threading.Event()

        # TCP / decoder state
        self._sock: Optional[object] = None
        self._tcp_thread: Optional[threading.Thread] = None
        self._running = False

        # MQTT
        self._client = mqtt.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)

        if mqtt_user is None and mqtt_pwd is None:
            mqtt_user, mqtt_pwd = _resolve_mqtt_creds()
        if mqtt_user is not None:
            self._client.username_pw_set(mqtt_user, mqtt_pwd)
            log.info('MQTT auth: user=%s', mqtt_user)

    def on_frame(self, callback: FrameCallback):
        self._callback = callback

    def start(self):
        log.info('H264TCPFrameSource: connecting to MQTT %s:%d for discovery',
                 self._broker, self._port)
        self._running = True
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        # TCP connect происходит в отдельном потоке — он ждёт discovery
        # сообщения и потом коннектится
        self._tcp_thread = threading.Thread(
            target=self._tcp_loop, name='h264_tcp', daemon=True)
        self._tcp_thread.start()

    def stop(self):
        self._running = False
        self._endpoint_changed.set()  # разблокировать tcp_loop если ждёт
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._client.loop_stop()
        self._client.disconnect()

    @property
    def client(self):
        """Доступ к paho-клиенту (для MQTTPublisher на том же соединении)."""
        return self._client

    @property
    def topic_prefix(self) -> str:
        return self._prefix

    # ── MQTT ───────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed (rc=%d)', rc)
            return
        log.info('MQTT connected — subscribing to camera/endpoint, range, odom')
        client.subscribe(f'{self._prefix}/camera/endpoint', qos=1)
        client.subscribe(f'{self._prefix}/range', qos=0)
        client.subscribe(f'{self._prefix}/odom', qos=0)

    def _on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self._prefix) + 1:]
        if suffix == 'camera/endpoint':
            self._handle_endpoint(msg.payload)
        elif suffix == 'range':
            self._handle_range(msg.payload)
        elif suffix == 'odom':
            self._handle_odom(msg.payload)

    def _handle_endpoint(self, payload: bytes):
        if not payload:
            # Pi выключил камеру — empty retain message
            with self._endpoint_lock:
                self._endpoint = None
            self._endpoint_changed.set()
            log.warning('Camera endpoint cleared — Pi camera offline')
            return
        try:
            data = json.loads(payload)
        except Exception as e:
            log.error('Bad endpoint JSON: %s', e)
            return

        with self._endpoint_lock:
            old = self._endpoint
            self._endpoint = data
            if old is None or (
                old.get('host') != data.get('host')
                or old.get('port') != data.get('port')
            ):
                log.info('Camera endpoint: %s:%d (codec=%s, %dx%d)',
                         data.get('host'), data.get('port'),
                         data.get('codec'),
                         data.get('width'), data.get('height'))
                self._endpoint_changed.set()

    def _handle_range(self, payload: bytes):
        try:
            data = json.loads(payload)
            if isinstance(data, dict):
                r = float(data.get('range', 2.0))
                if 0.02 <= r <= 2.0:
                    self._range_m = r
                    self._range_ts = time.monotonic()
        except Exception:
            pass

    def _handle_odom(self, payload: bytes):
        try:
            data = json.loads(payload)
            if isinstance(data, dict):
                self._pose = RobotPose(
                    x=float(data.get('x', 0.0)) / 100.0,
                    y=float(data.get('y', 0.0)) / 100.0,
                    theta=float(data.get('theta', 0.0)),
                    valid=True,
                )
        except Exception:
            pass

    # ── TCP / decoder loop ─────────────────────────────────────
    def _tcp_loop(self):
        """Background thread: connect → read → decode → callback. Reconnect on errors."""
        import av  # type: ignore

        backoff = self.RECONNECT_INITIAL
        while self._running:
            # Wait for discovery
            with self._endpoint_lock:
                ep = self._endpoint
            if ep is None:
                self._endpoint_changed.wait(timeout=1.0)
                self._endpoint_changed.clear()
                continue

            host = ep.get('host')
            port = int(ep.get('port', 8554))
            if not host:
                time.sleep(0.5)
                continue

            # TCP connect
            import socket as sk
            sock = sk.socket(sk.AF_INET, sk.SOCK_STREAM)
            sock.settimeout(5.0)
            try:
                log.info('Connecting to H.264 stream %s:%d', host, port)
                sock.connect((host, port))
                sock.settimeout(2.0)
                sock.setsockopt(sk.IPPROTO_TCP, sk.TCP_NODELAY, 1)
                self._sock = sock
                backoff = self.RECONNECT_INITIAL  # reset на successful connect
                log.info('H.264 TCP connected')
            except OSError as e:
                log.warning('H.264 TCP connect failed: %s — retry in %.1fs', e, backoff)
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(backoff)
                backoff = min(backoff * 2, self.RECONNECT_MAX)
                continue

            # Decode loop
            codec = av.CodecContext.create('h264', 'r')
            try:
                while self._running:
                    try:
                        chunk = sock.recv(self.READ_BUFFER_SIZE)
                    except sk.timeout:
                        continue
                    except OSError as e:
                        log.warning('H.264 socket read error: %s', e)
                        break
                    if not chunk:
                        log.warning('H.264 stream EOF — Pi disconnected')
                        break

                    # Check if endpoint changed (Pi сменил IP) — выходим
                    if self._endpoint_changed.is_set():
                        self._endpoint_changed.clear()
                        log.info('Endpoint changed — reconnecting')
                        break

                    try:
                        packets = codec.parse(chunk)
                        for packet in packets:
                            for frame in codec.decode(packet):
                                self._dispatch_frame(frame)
                    except av.AVError as e:
                        log.warning('H.264 decode error: %s', e)
                        # Не разрываем connection — может починиться следующим IDR
                        continue
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._sock = None

            # Reconnect after disconnect
            if self._running:
                time.sleep(backoff)
                backoff = min(backoff * 2, self.RECONNECT_MAX)

    def _dispatch_frame(self, av_frame):
        if self._callback is None:
            return
        try:
            bgr = av_frame.to_ndarray(format='bgr24')
        except Exception as e:
            log.error('Frame conversion error: %s', e)
            return
        ctx = FrameContext(
            bgr=bgr,
            pose=RobotPose(self._pose.x, self._pose.y, self._pose.theta, self._pose.valid),
            ultrasonic_m=self._range_m,
            ultrasonic_age_s=time.monotonic() - self._range_ts if self._range_ts else 999.0,
            timestamp=time.time(),
        )
        try:
            self._callback(ctx)
        except Exception as e:
            log.error('Frame callback error: %s', e)
