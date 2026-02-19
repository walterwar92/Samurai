#!/usr/bin/env python3
"""
dashboard_node — Web dashboard for Samurai robot monitoring.

Runs on the compute laptop alongside YOLO/SLAM/Nav2.
Provides a real-time web interface at http://localhost:5000

Subscribes:
    /yolo/annotated       (Image)         — annotated camera feed
    /robot_status         (String)        — FSM state JSON
    /ball_detection       (String)        — ball detection JSON
    /range                (Range)         — ultrasonic distance
    /imu/data             (Imu)           — IMU orientation
    /map                  (OccupancyGrid) — SLAM map
    /odometry/filtered    (Odometry)      — robot pose
    /voice_command        (String)        — voice commands
    /scan                 (LaserScan)     — pseudo-laser scan
    /battery              (Float32)       — battery voltage
    /battery_percent      (Float32)       — battery percentage
    /cpu_temperature      (Float32)       — CPU temperature
    /watchdog             (String)        — node health JSON
    /patrol/status        (String)        — patrol status JSON
    /path_recorder/status (String)        — path recorder status JSON
    /follow_me/status     (String)        — follow-me status JSON
    /qr_detection         (String)        — QR detection JSON
    /gesture/command      (String)        — gesture command
    /speed_profile/active (String)        — current speed profile
"""

import json
import math
import os
import socket
import threading
import time
from collections import deque


def _get_local_ip() -> str:
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
from sensor_msgs.msg import CompressedImage, Range, Imu, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Float32

from flask import Flask, Response, request, jsonify, send_from_directory
from flask_socketio import SocketIO
from flask_cors import CORS


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 5000)
        self._port = self.get_parameter('port').value

        self._lock = threading.Lock()

        # --- Shared state ---
        self._latest_frame = None
        self._robot_status = {}
        self._ball_detection = {}
        self._range_m = -1.0
        self._imu_ypr = [0.0, 0.0, 0.0]
        self._map_png = None
        self._map_info = {}
        self._robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._scan_points = []
        self._voice_log = deque(maxlen=20)

        # New state fields
        self._battery_voltage = 0.0
        self._battery_percent = 0.0
        self._cpu_temp = 0.0
        self._watchdog_status = {}
        self._patrol_status = {}
        self._path_recorder_status = {}
        self._follow_me_status = {}
        self._qr_detection = {}
        self._gesture_command = ''
        self._speed_profile = 'normal'

        # --- QoS for map (transient local) ---
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Original subscriptions ---
        self.create_subscription(CompressedImage, '/yolo/annotated/compressed', self._image_cb, 5)
        self.create_subscription(String, '/robot_status', self._status_cb, 10)
        self.create_subscription(String, '/ball_detection', self._detection_cb, 10)
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(String, '/voice_command', self._voice_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        # --- New subscriptions ---
        self.create_subscription(Float32, '/battery', self._battery_cb, 10)
        self.create_subscription(Float32, '/battery_percent', self._battery_pct_cb, 10)
        self.create_subscription(Float32, '/cpu_temperature', self._cpu_temp_cb, 10)
        self.create_subscription(String, '/watchdog', self._watchdog_cb, 10)
        self.create_subscription(String, '/patrol/status', self._patrol_status_cb, 10)
        self.create_subscription(String, '/path_recorder/status', self._path_status_cb, 10)
        self.create_subscription(String, '/follow_me/status', self._follow_status_cb, 10)
        self.create_subscription(String, '/qr_detection', self._qr_cb, 10)
        self.create_subscription(String, '/gesture/command', self._gesture_cb, 10)
        self.create_subscription(String, '/speed_profile/active', self._speed_profile_cb, 10)

        # --- Publishers for commands from dashboard ---
        self._speed_profile_pub = self.create_publisher(String, '/speed_profile', 10)
        self._patrol_wp_pub = self.create_publisher(String, '/patrol/waypoints', 10)
        self._patrol_cmd_pub = self.create_publisher(String, '/patrol/command', 10)
        self._follow_cmd_pub = self.create_publisher(String, '/follow_me/command', 10)
        self._path_cmd_pub = self.create_publisher(String, '/path_recorder/command', 10)
        self._map_save_pub = self.create_publisher(String, '/map_manager/save', 10)
        self._map_load_pub = self.create_publisher(String, '/map_manager/load', 10)
        self._map_list_pub = self.create_publisher(String, '/map_manager/list', 10)
        self._voice_cmd_pub = self.create_publisher(String, '/voice_command', 10)

        # Listen for map list responses
        self._map_list_data = []
        self.create_subscription(String, '/map_manager/map_list', self._map_list_cb, 10)
        self._path_list_data = []
        self.create_subscription(String, '/path_recorder/path_list', self._path_list_cb, 10)

        self.get_logger().info(f'Dashboard node started — http://{_get_local_ip()}:{self._port}')

    # ── Original ROS2 Callbacks ────────────────────────────────────

    def _image_cb(self, msg: CompressedImage):
        # Кадр уже JPEG от yolo_detector — передаём напрямую без decode+encode
        with self._lock:
            self._latest_frame = bytes(msg.data)

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

        img = np.full((h, w, 3), 128, dtype=np.uint8)
        img[data == 0] = [240, 240, 240]
        img[data > 50] = [30, 30, 30]

        img = cv2.flip(img, 0)
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

    # ── New ROS2 Callbacks ─────────────────────────────────────────

    def _battery_cb(self, msg: Float32):
        with self._lock:
            self._battery_voltage = round(msg.data, 2)

    def _battery_pct_cb(self, msg: Float32):
        with self._lock:
            self._battery_percent = round(msg.data, 1)

    def _cpu_temp_cb(self, msg: Float32):
        with self._lock:
            self._cpu_temp = round(msg.data, 1)

    def _watchdog_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._watchdog_status = data
        except json.JSONDecodeError:
            pass

    def _patrol_status_cb(self, msg: String):
        try:
            with self._lock:
                self._patrol_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _path_status_cb(self, msg: String):
        try:
            with self._lock:
                self._path_recorder_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _follow_status_cb(self, msg: String):
        try:
            with self._lock:
                self._follow_me_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _qr_cb(self, msg: String):
        try:
            with self._lock:
                self._qr_detection = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _gesture_cb(self, msg: String):
        with self._lock:
            self._gesture_command = msg.data

    def _speed_profile_cb(self, msg: String):
        with self._lock:
            self._speed_profile = msg.data

    def _map_list_cb(self, msg: String):
        try:
            self._map_list_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _path_list_cb(self, msg: String):
        try:
            self._path_list_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

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
                'battery_voltage': self._battery_voltage,
                'battery_percent': self._battery_percent,
                'cpu_temp': self._cpu_temp,
                'watchdog': self._watchdog_status,
                'patrol': self._patrol_status,
                'path_recorder': self._path_recorder_status,
                'follow_me': self._follow_me_status,
                'qr_detection': self._qr_detection,
                'gesture': self._gesture_command,
                'speed_profile': self._speed_profile,
            }

    def get_map_png(self):
        with self._lock:
            return self._map_png

    def pub_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)


# ═════════════════════════════════════════════════════════════════
# Flask Application
# ═════════════════════════════════════════════════════════════════

def _find_compute_node_dir() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        candidate = os.path.join(d, 'compute_node')
        if os.path.isdir(candidate) and os.path.isdir(os.path.join(candidate, 'static')):
            return candidate
        d = os.path.dirname(d)
    return os.path.dirname(os.path.abspath(__file__))


def create_app(ros_node: DashboardNode):
    base_dir = _find_compute_node_dir()
    template_dir = os.path.join(base_dir, 'templates')
    static_dir = os.path.join(base_dir, 'static')
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
                time.sleep(0.033)
        return Response(generate(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/map.png')
    def map_image():
        png = ros_node.get_map_png()
        if png is None:
            return Response(b'\x89PNG\r\n\x1a\n', mimetype='image/png')
        return Response(png, mimetype='image/png')

    # ── New API routes ─────────────────────────────────────────

    @app.route('/api/speed_profile', methods=['POST'])
    def set_speed_profile():
        data = request.get_json(silent=True) or {}
        profile = data.get('profile', 'normal')
        ros_node.pub_string(ros_node._speed_profile_pub, profile)
        return jsonify({'ok': True, 'profile': profile})

    @app.route('/api/patrol/waypoints', methods=['POST'])
    def set_patrol_waypoints():
        data = request.get_json(silent=True) or {}
        waypoints = data.get('waypoints', [])
        ros_node.pub_string(ros_node._patrol_wp_pub, json.dumps(waypoints))
        return jsonify({'ok': True, 'count': len(waypoints)})

    @app.route('/api/patrol/command', methods=['POST'])
    def patrol_command():
        data = request.get_json(silent=True) or {}
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._patrol_cmd_pub, cmd)
        return jsonify({'ok': True, 'command': cmd})

    @app.route('/api/map/save', methods=['POST'])
    def save_map():
        data = request.get_json(silent=True) or {}
        name = data.get('name', 'default')
        ros_node.pub_string(ros_node._map_save_pub, name)
        return jsonify({'ok': True, 'name': name})

    @app.route('/api/map/load', methods=['POST'])
    def load_map():
        data = request.get_json(silent=True) or {}
        name = data.get('name', '')
        ros_node.pub_string(ros_node._map_load_pub, name)
        return jsonify({'ok': True, 'name': name})

    @app.route('/api/map/list', methods=['GET'])
    def list_maps():
        ros_node.pub_string(ros_node._map_list_pub, 'list')
        time.sleep(0.3)
        return jsonify({'maps': ros_node._map_list_data})

    @app.route('/api/follow_me', methods=['POST'])
    def follow_me_cmd():
        data = request.get_json(silent=True) or {}
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._follow_cmd_pub, cmd)
        return jsonify({'ok': True, 'command': cmd})

    @app.route('/api/path_recorder/command', methods=['POST'])
    def path_recorder_cmd():
        data = request.get_json(silent=True) or {}
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._path_cmd_pub, cmd)
        return jsonify({'ok': True, 'command': cmd})

    @app.route('/api/path_recorder/list', methods=['GET'])
    def list_paths():
        ros_node.pub_string(ros_node._path_cmd_pub, 'list')
        time.sleep(0.3)
        return jsonify({'paths': ros_node._path_list_data})

    # ── SocketIO events ────────────────────────────────────────

    @socketio.on('send_command')
    def handle_command(data):
        text = data.get('text', '')
        if text:
            ros_node.pub_string(ros_node._voice_cmd_pub, text)

    @socketio.on('set_speed_profile')
    def handle_speed_profile(data):
        profile = data.get('profile', 'normal')
        ros_node.pub_string(ros_node._speed_profile_pub, profile)

    @socketio.on('patrol_command')
    def handle_patrol_cmd(data):
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._patrol_cmd_pub, cmd)

    @socketio.on('patrol_waypoints')
    def handle_patrol_wp(data):
        waypoints = data.get('waypoints', [])
        ros_node.pub_string(ros_node._patrol_wp_pub, json.dumps(waypoints))

    @socketio.on('follow_me_command')
    def handle_follow_cmd(data):
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._follow_cmd_pub, cmd)

    @socketio.on('path_recorder_command')
    def handle_path_cmd(data):
        cmd = data.get('command', 'stop')
        ros_node.pub_string(ros_node._path_cmd_pub, cmd)

    @socketio.on('map_save')
    def handle_map_save(data):
        name = data.get('name', 'default')
        ros_node.pub_string(ros_node._map_save_pub, name)

    @socketio.on('map_load')
    def handle_map_load(data):
        name = data.get('name', '')
        ros_node.pub_string(ros_node._map_load_pub, name)

    def emit_state():
        while True:
            state = ros_node.get_state()
            socketio.emit('state_update', state)
            socketio.sleep(0.25)

    socketio.start_background_task(emit_state)

    return app, socketio


# ═════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

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
