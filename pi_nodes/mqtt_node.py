"""
MqttNode — base class for all Pi-side MQTT nodes.

Replaces rclpy.Node. Provides:
  - Auto-connection to Mosquitto broker with reconnect
  - publish() / subscribe() with JSON or raw bytes
  - create_timer(period_sec, callback) — threaded periodic timers
  - Structured logging with node name prefix
  - Graceful shutdown via SIGINT/SIGTERM
  - Config access via config_loader.cfg()
"""

import json
import logging
import os
import random
import signal
import socket
import sys
import threading
import time
from typing import Callable, Any, Optional

import paho.mqtt.client as mqtt

# Load default robot_id from config.yaml (single source of truth)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
try:
    from config_loader import cfg as _cfg
    _DEFAULT_ROBOT_ID = _cfg('mqtt.robot_id', 'robot1')
except ImportError:
    _DEFAULT_ROBOT_ID = 'robot1'


def get_local_ip(timeout: float = 2.0) -> str:
    """Auto-detect local network IP (non-routed UDP trick, no packets sent)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


class MqttNode:
    """Base class for all Pi-side MQTT nodes.

    Usage::

        class MyNode(MqttNode):
            def __init__(self, **kw):
                super().__init__('my_node', **kw)
                self.subscribe('cmd_vel', self._on_cmd)
                self.create_timer(0.05, self._tick)

            def _on_cmd(self, topic, data):
                ...

            def _tick(self):
                self.publish('odom', {'x': 0, 'y': 0})

        if __name__ == '__main__':
            node = MyNode(broker='127.0.0.1')
            node.start()
            node.spin()
    """

    def __init__(self, name: str, *,
                 broker: str = '127.0.0.1',
                 port: int = 1883,
                 robot_id: str = _DEFAULT_ROBOT_ID):
        self.name = name
        self._robot_id = robot_id
        self._broker = broker
        self._port = port
        self._running = False
        self._timers: list[threading.Thread] = []
        self._lock = threading.Lock()

        # Logging
        self._log = logging.getLogger(name)
        if not self._log.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(
                logging.Formatter(f'[%(levelname)s] [{name}] %(message)s'))
            self._log.addHandler(handler)
            self._log.setLevel(logging.INFO)

        # MQTT client with keepalive and LWT (Last Will & Testament)
        self._client = mqtt.Client(
            client_id=f'samurai_{robot_id}_{name}',
            protocol=mqtt.MQTTv311,
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        # Fast reconnect: 0.5s min, 5s max (was 1-30s)
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)
        # LWT: if broker doesn't hear from us, publish offline status
        # QoS 0 — avoids blocking; retain ensures last state is kept
        self._client.will_set(
            f'samurai/{robot_id}/{name}/online', 'false',
            qos=0, retain=True)
        self._subscriptions: dict[str, Any] = {}
        self._mqtt_connected = False

    # ── Connection ─────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = True
        self._log.info('MQTT connected (rc=%d)', rc)
        # Publish online status (QoS 0 — non-blocking)
        client.publish(
            f'samurai/{self._robot_id}/{self.name}/online', 'true',
            qos=0, retain=True)
        # Re-subscribe to all topics on reconnect
        for topic_full in self._subscriptions:
            qos = self._subscriptions[topic_full].get('qos', 0)
            client.subscribe(topic_full, qos)

    def _on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self._log.warning('MQTT disconnected unexpectedly (rc=%d) — reconnecting...', rc)

    # ── Topic helpers ──────────────────────────────────────────
    def topic(self, suffix: str) -> str:
        """Build full MQTT topic: samurai/{robot_id}/{suffix}."""
        return f'samurai/{self._robot_id}/{suffix}'

    # ── Publish ────────────────────────────────────────────────
    def publish(self, suffix: str, payload,
                qos: int = 0, retain: bool = False):
        """Publish to samurai/{robot_id}/{suffix}.

        payload: dict → JSON, str → UTF-8, bytes → raw binary.
        """
        full_topic = self.topic(suffix)
        self.publish_raw(full_topic, payload, qos=qos, retain=retain)

    def publish_raw(self, full_topic: str, payload,
                    qos: int = 0, retain: bool = False):
        """Publish to an arbitrary MQTT topic (no robot_id prefix)."""
        if isinstance(payload, dict):
            data = json.dumps(payload, separators=(',', ':'))
        elif isinstance(payload, (bytes, bytearray)):
            data = payload
        elif isinstance(payload, bool):
            data = json.dumps(payload)
        elif isinstance(payload, (int, float)):
            data = str(payload)
        else:
            data = str(payload)
        self._client.publish(full_topic, data, qos=qos, retain=retain)

    # ── Subscribe ──────────────────────────────────────────────
    def subscribe(self, suffix: str, callback: Callable,
                  qos: int = 0, parse_json: bool = True):
        """Subscribe to samurai/{robot_id}/{suffix}.

        callback(topic: str, payload: dict | str | bytes)
        If parse_json=True, attempts JSON decode; falls back to str.
        If parse_json=False, passes raw bytes.
        """
        full_topic = self.topic(suffix)
        self.subscribe_raw(full_topic, callback,
                           qos=qos, parse_json=parse_json)

    def subscribe_raw(self, full_topic: str, callback: Callable,
                      qos: int = 0, parse_json: bool = True):
        """Subscribe to an arbitrary MQTT topic (no robot_id prefix)."""

        def _wrapper(_client, _userdata, mqtt_msg):
            raw = mqtt_msg.payload
            if parse_json:
                try:
                    data = json.loads(raw)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    data = raw.decode('utf-8', errors='replace')
            else:
                data = raw
            try:
                callback(mqtt_msg.topic, data)
            except Exception as exc:
                self._log.error('Callback error on %s: %s', mqtt_msg.topic, exc)

        self._subscriptions[full_topic] = {'wrapper': _wrapper, 'qos': qos}
        self._client.message_callback_add(full_topic, _wrapper)
        if self._client.is_connected():
            self._client.subscribe(full_topic, qos)

    # ── Timers ─────────────────────────────────────────────────
    def create_timer(self, period_sec: float,
                     callback: Callable[[], None],
                     name: str = '') -> threading.Thread:
        """Start a periodic timer in a daemon thread."""
        timer_name = name or f'{self.name}_timer_{len(self._timers)}'

        def _loop():
            next_time = time.monotonic() + period_sec
            while self._running:
                now = time.monotonic()
                if now >= next_time:
                    try:
                        callback()
                    except Exception as exc:
                        self._log.error('Timer %s error: %s', timer_name, exc)
                    next_time += period_sec
                    if next_time < now:
                        next_time = now + period_sec
                else:
                    # Sleep until next tick (was 5ms busy-loop → GIL contention)
                    time.sleep(max(next_time - now, 0.001))

        t = threading.Thread(target=_loop, name=timer_name, daemon=True)
        self._timers.append(t)
        return t

    # ── Lifecycle ──────────────────────────────────────────────
    def start(self):
        """Connect to broker and start all timers. Call once."""
        self._running = True
        # keepalive=15s — broker detects dead client faster (default was 60s)
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        # Heartbeat every 5s — keeps MQTT connection alive over WiFi
        # Jitter 0-2s per node to avoid thundering herd on broker
        heartbeat_period = 5.0 + random.uniform(0.0, 2.0)
        self.create_timer(heartbeat_period, self._heartbeat, name=f'{self.name}_heartbeat')
        for t in self._timers:
            t.start()
        self._log.info('%s started (broker=%s:%d, id=%s)',
                       self.name, self._broker, self._port, self._robot_id)

    def _heartbeat(self):
        """Periodic heartbeat to keep MQTT alive and detect dead connections."""
        if self._mqtt_connected:
            self._client.publish(
                f'samurai/{self._robot_id}/{self.name}/heartbeat',
                str(int(time.time())), qos=0)

    def spin(self):
        """Block until SIGINT/SIGTERM. Call after start()."""
        stop_event = threading.Event()

        def _sig_handler(signum, frame):
            stop_event.set()

        signal.signal(signal.SIGINT, _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)
        stop_event.wait()
        self.shutdown()

    def shutdown(self):
        """Graceful shutdown: stop timers, disconnect MQTT."""
        self._running = False
        self._log.info('%s shutting down...', self.name)
        self.on_shutdown()
        # Publish offline before disconnect (QoS 0 — non-blocking)
        if self._mqtt_connected:
            self._client.publish(
                f'samurai/{self._robot_id}/{self.name}/online', 'false',
                qos=0, retain=True)
        for t in self._timers:
            t.join(timeout=2.0)
        self._client.loop_stop()
        self._client.disconnect()

    def on_shutdown(self):
        """Override in subclasses for cleanup (stop motors, etc.)."""
        pass

    # ── Logging shortcuts ──────────────────────────────────────
    def log_info(self, msg: str, *args):
        self._log.info(msg, *args)

    def log_warn(self, msg: str, *args):
        self._log.warning(msg, *args)
        self._publish_log_event('WARN', msg % args if args else msg)

    def log_error(self, msg: str, *args):
        self._log.error(msg, *args)
        self._publish_log_event('ERROR', msg % args if args else msg)

    def _publish_log_event(self, level: str, text: str):
        """Publish error/warning to centralized log topic for dashboard aggregation."""
        if self._mqtt_connected:
            try:
                self._client.publish(
                    f'samurai/{self._robot_id}/log/events',
                    json.dumps({
                        'node': self.name,
                        'level': level,
                        'msg': text[:200],
                        'ts': time.time(),
                    }, separators=(',', ':')),
                    qos=0)
            except Exception:
                pass  # never let logging crash the node

    # ── Clock ──────────────────────────────────────────────────
    @staticmethod
    def now_sec() -> float:
        """Monotonic time (for dt, odometry, etc.)."""
        return time.monotonic()

    @staticmethod
    def timestamp() -> float:
        """Unix timestamp (for message headers)."""
        return time.time()
