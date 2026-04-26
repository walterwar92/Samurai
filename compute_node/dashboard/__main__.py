"""
Entry point: python -m compute_node.dashboard

Заменяет compute_node/dashboard_node.py (1840 строк). Старый файл
остаётся в репо до C13 для backward-compat — там тот же командный API.

Поток исполнения:
  1. ENV → broker, port, robot_id, MQTT credentials.
  2. DashboardState — централизованное состояние.
  3. MQTTHandlers — paho-mqtt клиент, подписки на samurai/{robot_id}/...
  4. (опц.) rclpy.init + Node + ROS2Subscribers — для SLAM/EKF/YOLO топиков.
     Если rclpy недоступен (на dev-машине без ROS2) — пропускается.
  5. create_app(state, mqtt, ros2) → FastAPI + Socket.IO + статика.
  6. uvicorn.run.

ENV vars:
  MQTT_BROKER  — Pi IP (если не задан → standalone mode без MQTT)
  MQTT_PORT    — default 1883
  ROBOT_ID     — default 'robot1'
  PORT         — порт dashboard (default 5000)
  SAMURAI_MQTT_USER / _PASS — опц., если на брокере включена auth
"""
from __future__ import annotations

import logging
import os
import socket
import sys
import threading

# ── ENV / config ───────────────────────────────────────────────────────
MQTT_BROKER = os.environ.get('MQTT_BROKER', '').strip()
MQTT_PORT = int(os.environ.get('MQTT_PORT', '1883'))
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')
DASHBOARD_PORT = int(os.environ.get('PORT', '5000'))


def _resolve_mqtt_creds() -> tuple[str | None, str | None]:
    """Optional MQTT auth — пробуем config_loader, fallback на ENV."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
        from config_loader import get_mqtt_credentials
        return get_mqtt_credentials()
    except ImportError:
        u = os.environ.get('SAMURAI_MQTT_USER', '').strip()
        p = os.environ.get('SAMURAI_MQTT_PASS', '')
        return (u, p) if u and p else (None, None)


def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
    log = logging.getLogger('dashboard')

    # 1. State + handlers
    from .state import DashboardState
    state = DashboardState()

    mqtt = None
    if MQTT_BROKER:
        from .mqtt_handlers import MQTTHandlers
        user, pwd = _resolve_mqtt_creds()
        mqtt = MQTTHandlers(
            broker=MQTT_BROKER, port=MQTT_PORT,
            robot_id=ROBOT_ID, state=state,
            username=user, password=pwd,
        )
        mqtt.start()
    else:
        log.warning('MQTT_BROKER not set — running without MQTT (sensor data unavailable)')

    # 2. ROS2 (опц.)
    ros2 = None
    rclpy_node = None
    spin_thread = None
    try:
        import rclpy
        from rclpy.node import Node
        from .ros2_subscribers import ROS2Subscribers

        rclpy.init()
        rclpy_node = Node('dashboard_node')
        ros2 = ROS2Subscribers(rclpy_node, state)
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(rclpy_node,), daemon=True)
        spin_thread.start()
        log.info('ROS2 subscribers attached (rclpy spinning in background)')
    except Exception as exc:
        log.info('ROS2 unavailable (%s) — SLAM/EKF/YOLO subscriptions skipped', exc)

    # 3. App + uvicorn
    from .app import create_app
    import uvicorn

    app = create_app(state, mqtt=mqtt, ros2=ros2)
    ip = _local_ip()
    log.info('Dashboard ready — http://%s:%d  /docs at /openapi.json',
             ip, DASHBOARD_PORT)

    try:
        uvicorn.run(app, host='0.0.0.0', port=DASHBOARD_PORT,
                    log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        if mqtt is not None:
            mqtt.stop()
        if rclpy_node is not None:
            try:
                rclpy_node.destroy_node()
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
