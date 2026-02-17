#!/usr/bin/env python3
"""
dashboard_node — Web dashboard for Samurai robot monitoring.

Runs on the compute laptop alongside YOLO/SLAM/Nav2.
Provides a real-time web interface at http://localhost:5000

Subscribes:
    /yolo/annotated     (sensor_msgs/Image)      — annotated camera feed
    /robot_status       (std_msgs/String)         — FSM state JSON
    /ball_detection     (std_msgs/String)         — ball detection JSON
    /range              (sensor_msgs/Range)       — ultrasonic distance
    /imu/data           (sensor_msgs/Imu)         — IMU orientation
    /map                (nav_msgs/OccupancyGrid)  — SLAM map
    /odometry/filtered  (nav_msgs/Odometry)       — robot pose
    /voice_command      (std_msgs/String)         — voice commands
    /scan               (sensor_msgs/LaserScan)   — pseudo-laser scan
"""

import json
import math
import os
import socket
import threading
import time
from collections import deque


def _get_local_ip() -> str:
    """Auto-detect local network IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, Range, Imu, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

from flask import Flask, Response, render_template, send_from_directory
from flask_socketio import SocketIO
from flask_cors import CORS


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 5000)
        self._port = self.get_parameter('port').value

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # --- Shared state ---
        self._latest_frame = None          # JPEG bytes
        self._robot_status = {}            # FSM state dict
        self._ball_detection = {}          # latest detection dict
        self._range_m = -1.0              # ultrasonic range
        self._imu_ypr = [0.0, 0.0, 0.0]  # yaw, pitch, roll (degrees)
        self._map_png = None               # PNG bytes of SLAM map
        self._map_info = {}                # map metadata (resolution, origin)
        self._robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._scan_points = []             # laser scan as list of (x,y) in map frame
        self._voice_log = deque(maxlen=20)

        # --- QoS for map (transient local) ---
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Subscriptions ---
        self.create_subscription(Image, '/yolo/annotated', self._image_cb, 5)
        self.create_subscription(String, '/robot_status', self._status_cb, 10)
        self.create_subscription(String, '/ball_detection', self._detection_cb, 10)
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(String, '/voice_command', self._voice_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.get_logger().info(f'Dashboard node started — http://{_get_local_ip()}:{self._port}')

    # ── ROS2 Callbacks ───────────────────────────────────────────

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with self._lock:
            self._latest_frame = jpeg.tobytes()

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._robot_status = data
        except json.JSONDecodeError:
            pass

    def _detection_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._ball_detection = data
        except json.JSONDecodeError:
            pass

    def _range_cb(self, msg: Range):
        with self._lock:
            self._range_m = round(msg.range, 3)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        # Quaternion to Euler (yaw, pitch, roll)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        with self._lock:
            self._imu_ypr = [round(yaw, 1), round(pitch, 1), round(roll, 1)]

    def _map_cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # Convert: -1 (unknown) → grey, 0 (free) → white, 100 (occupied) → black
        img = np.full((h, w, 3), 128, dtype=np.uint8)  # grey default
        img[data == 0] = [240, 240, 240]    # free → light
        img[data > 50] = [30, 30, 30]       # occupied → dark

        img = cv2.flip(img, 0)  # flip Y axis for image coordinates
        _, png = cv2.imencode('.png', img)

        with self._lock:
            self._map_png = png.tobytes()
            self._map_info = {
                'width': w,
                'height': h,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
            }

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            self._robot_pose = {
                'x': round(pos.x, 3),
                'y': round(pos.y, 3),
                'yaw': round(yaw, 3),
            }

    def _voice_cb(self, msg: String):
        entry = {
            'text': msg.data,
            'time': time.strftime('%H:%M:%S'),
        }
        with self._lock:
            self._voice_log.append(entry)

    def _scan_cb(self, msg: LaserScan):
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([round(x, 3), round(y, 3)])
            angle += msg.angle_increment
        with self._lock:
            self._scan_points = points

    # ── Data Access (for Flask) ──────────────────────────────────

    def get_frame(self):
        with self._lock:
            return self._latest_frame

    def get_state(self):
        with self._lock:
            return {
                'status': self._robot_status,
                'detection': self._ball_detection,
                'range_m': self._range_m,
                'imu_ypr': self._imu_ypr,
                'pose': self._robot_pose,
                'map_info': self._map_info,
                'scan_points': self._scan_points,
                'voice_log': list(self._voice_log),
            }

    def get_map_png(self):
        with self._lock:
            return self._map_png


# ═════════════════════════════════════════════════════════════════
# Flask Application
# ═════════════════════════════════════════════════════════════════

def create_app(ros_node: DashboardNode):
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    app = Flask(__name__, template_folder=template_dir,
                static_folder=static_dir, static_url_path='')
    CORS(app)
    socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

    @app.route('/')
    @app.route('/dashboard')
    @app.route('/admin')
    def serve_spa():
        return send_from_directory(static_dir, 'index.html')

    @app.route('/video_feed')
    def video_feed():
        def generate():
            while True:
                frame = ros_node.get_frame()
                if frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n'
                           + frame + b'\r\n')
                else:
                    time.sleep(0.1)
                time.sleep(0.033)  # ~30 fps cap
        return Response(generate(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/map.png')
    def map_image():
        png = ros_node.get_map_png()
        if png is None:
            # Return 1x1 transparent pixel
            return Response(b'\x89PNG\r\n\x1a\n', mimetype='image/png')
        return Response(png, mimetype='image/png')

    def emit_state():
        """Background thread: push state to all clients via SocketIO."""
        while True:
            state = ros_node.get_state()
            socketio.emit('state_update', state)
            socketio.sleep(0.25)  # 4 Hz updates

    socketio.start_background_task(emit_state)

    return app, socketio


# ═════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    # Spin ROS2 in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run Flask in the main thread
    app, socketio = create_app(node)
    port = node._port
    try:
        socketio.run(app, host='0.0.0.0', port=port, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
