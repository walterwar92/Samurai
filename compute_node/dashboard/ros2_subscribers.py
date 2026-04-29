"""
ROS2Subscribers — подписки rclpy на ROS2-топики (SLAM, EKF, YOLO).

Источник данных, дополнительный к MQTT (см. mqtt_handlers.py). MQTT —
primary path Pi→ноут (всегда работает); ROS2 — secondary, для данных
которые публикует только laptop-side стек:

  /yolo/annotated/compressed → CompressedImage  → state.detection.annotated_jpeg
  /map                       → OccupancyGrid    → state.map.png + map.info + ros2_map_active
  /odometry/filtered         → Odometry         → state.robot.pose/velocity (only if MQTT odom stale >2s)
  /scan                      → LaserScan        → state.sensors.scan_points
  /ball_detection            → String (JSON)    → state.detection.ball_detection_raw

Publishers (для команд через ROS2 bridge → Pi):

  /map_manager/save  (String)  — save_map(name)
  /map_manager/load  (String)  — load_map(name)
  /yolo/enable       (String)  — set_yolo_enabled(on|off)

Извлечено из DashboardNode в C4 (#7).
"""
from __future__ import annotations

import logging
import math
import time

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

from .schemas.maps import MapInfo
from .schemas.robot import RobotPose, VelocityDetail
from .state import DashboardState

log = logging.getLogger(__name__)


class ROS2Subscribers:
    """Все ROS2 subscriptions/publishers для dashboard.

    Принимает существующий rclpy.node.Node (создаётся в main()) и
    DashboardState. Все callbacks обновляют state.lock.
    """

    def __init__(self, node: Node, state: DashboardState):
        self._node = node
        self._state = state

        # ── QoS profiles ──────────────────────────────────────────
        # SLAM map: TRANSIENT_LOCAL — последняя map хранится у издателя,
        # late-joiners получают сразу.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscriptions ─────────────────────────────────────────
        node.create_subscription(
            CompressedImage,
            '/yolo/annotated/compressed',
            self._yolo_image_cb,
            5,
        )
        node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_cb,
            map_qos,
        )
        node.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            10,
        )
        node.create_subscription(
            LaserScan,
            '/scan',
            self._scan_cb,
            10,
        )
        node.create_subscription(
            String,
            '/ball_detection',
            self._detection_cb,
            10,
        )

        # ── Publishers (commands) ─────────────────────────────────
        self._pub_map_save = node.create_publisher(
            String, '/map_manager/save', 10)
        self._pub_map_load = node.create_publisher(
            String, '/map_manager/load', 10)
        self._pub_yolo_enable = node.create_publisher(
            String, '/yolo/enable', 10)

    # ── Command publishers ─────────────────────────────────────────
    def save_map(self, name: str) -> None:
        msg = String()
        msg.data = name
        self._pub_map_save.publish(msg)

    def load_map(self, name: str) -> None:
        msg = String()
        msg.data = name
        self._pub_map_load.publish(msg)

    def set_yolo_enabled(self, enabled: bool) -> None:
        msg = String()
        msg.data = 'on' if enabled else 'off'
        self._pub_yolo_enable.publish(msg)

    # ── Callbacks ──────────────────────────────────────────────────
    def _yolo_image_cb(self, msg: CompressedImage) -> None:
        """Аннотированный YOLO кадр (ROS2 detector) — кэш в state."""
        with self._state.lock:
            self._state.detection.annotated_jpeg = bytes(msg.data)

    def _detection_cb(self, msg: String) -> None:
        """YOLO ball_detection из ROS2 (laptop-side детектор)."""
        try:
            import json
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._state.lock:
            self._state.detection.ball_detection_raw = data

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """SLAM Toolbox /map → render PNG + map_info."""
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 128, dtype=np.uint8)
        img[data == 0] = [240, 240, 240]
        img[data > 50] = [30, 30, 30]
        img = cv2.flip(img, 0)
        ok, png = cv2.imencode('.png', img)
        if not ok:
            return
        with self._state.lock:
            self._state.map.ros2_map_active = True
            self._state.map.png = png.tobytes()
            self._state.map.info = MapInfo(
                width=w,
                height=h,
                resolution=msg.info.resolution,
                origin_x=msg.info.origin.position.x,
                origin_y=msg.info.origin.position.y,
            )

    def _odom_cb(self, msg: Odometry) -> None:
        """Filtered odometry от EKF.

        Используем ТОЛЬКО когда MQTT odom stale (>2s) — иначе возникает
        oscillation на дашборде из-за смешивания см (MQTT) и метров (ROS2).
        """
        with self._state.lock:
            if time.time() - self._state.robot.mqtt_odom_ts < 2.0:
                return  # MQTT odom is primary
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._state.lock:
            # ROS2 одометрия в метрах — конвертируем в см чтобы совпадало
            # с MQTT odom от Pi.
            self._state.robot.pose = RobotPose(
                x=round(pos.x * 100.0, 3),
                y=round(pos.y * 100.0, 3),
                yaw=round(yaw, 3),
            )
            self._state.robot.velocity_estimated = VelocityDetail(
                linear_x=round(msg.twist.twist.linear.x, 3),
                linear_y=round(msg.twist.twist.linear.y, 3),
                angular_z=round(msg.twist.twist.angular.z, 3),
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        """Лазерный скан → точки в локальной СК для overlay'а на карте."""
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                points.append([
                    round(r * math.cos(angle), 3),
                    round(r * math.sin(angle), 3),
                ])
            angle += msg.angle_increment
        with self._state.lock:
            self._state.sensors.scan_points = points
