#!/usr/bin/env python3
"""
dashboard_node — Web dashboard + REST API for Samurai robot.

Runs on the compute laptop alongside YOLO/SLAM/Nav2.
Provides a real-time web interface at http://localhost:5000

Data sources:
    PRIMARY   — Direct MQTT from Pi (camera, range, imu, battery, status,
                odom, claw/state, watchdog, temperature). Works even if
                ROS2 bridge is down. Broker IP from env MQTT_BROKER.
    SECONDARY — ROS2 topics from mqtt_bridge_compute (SLAM map,
                filtered odometry, YOLO annotated image).

Commands go directly to Pi via MQTT (no ROS2 bridge needed).

REST API reference:
    GET  /api/status                  — full robot snapshot
    GET  /api/robot/pose              — x, y, yaw
    GET  /api/robot/velocity          — linear/angular speed
    GET  /api/sensors                 — all sensors
    GET  /api/sensors/ultrasonic      — ultrasonic range only
    GET  /api/sensors/imu             — YPR + gyro + accel
    GET  /api/detection               — YOLO detections
    GET  /api/detection/closest       — closest object (optional ?color=)
    GET  /api/fsm                     — current FSM state
    GET  /api/actuators               — claw state
    GET  /api/battery                 — battery info
    GET  /api/speed_profile           — current speed profile
    GET  /api/camera/frame            — JPEG binary
    GET  /api/camera/frame.json       — base64 JPEG
    GET  /api/map/image               — PNG binary
    GET  /api/map/info                — map metadata
    GET  /api/map/list                — list saved maps
    GET  /api/log                     — voice/event log
    POST /api/robot/velocity          — {linear_x, angular_z}
    POST /api/fsm/command             — send voice command
    POST /api/actuators/claw          — open / close
    GET  /api/actuators/head          — head pan/tilt state
    POST /api/actuators/head          — {pan, tilt} or {command: "center"}
    GET  /api/actuators/arm           — arm joint angles
    POST /api/actuators/arm           — {joint, angle} or {joints} or {command: "home"}
    POST /api/speed_profile           — slow / normal / fast
    POST /api/patrol/command          — start / stop patrol
    POST /api/patrol/waypoints        — set waypoints
    POST /api/follow_me               — start / stop follow-me
    POST /api/path_recorder/command   — start / stop / play path
    GET  /api/path_recorder/list      — list saved paths
    POST /api/map/save                — save SLAM map
    POST /api/map/load                — load SLAM map

MQTT env vars (passed from start_laptop_robot.sh):
    MQTT_BROKER  — Pi IP address (required)
    MQTT_PORT    — broker port (default: 1883)
    ROBOT_ID     — robot identifier (default: robot1)
"""

import asyncio
import base64
import json
import math
import os
import socket
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt_client


def _get_local_ip() -> str:
    """Auto-detect local network IP address.

    Uses a non-routed UDP connect trick (no packets sent).
    2-second timeout prevents hanging when network is unavailable.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError as exc:
        import logging
        logging.getLogger(__name__).warning(
            'IP auto-detection failed (%s) — falling back to 127.0.0.1', exc)
        return '127.0.0.1'


import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String

import socketio
from fastapi import FastAPI, Request
from fastapi.responses import Response, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn


# =============================================================
# ROS2 Node
# =============================================================

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 5000)
        self._port = self.get_parameter('port').value

        self._lock = threading.Lock()

        # ── MQTT config from env (set by start_laptop_robot.sh) ─
        self._mqtt_broker = os.environ.get('MQTT_BROKER', '')
        self._mqtt_port   = int(os.environ.get('MQTT_PORT', '1883'))
        self._robot_id    = os.environ.get('ROBOT_ID', 'robot1')
        self._prefix      = f'samurai/{self._robot_id}'
        self._mqtt_connected = False

        # ── Shared state ────────────────────────────────────
        self._latest_frame   = None   # raw JPEG bytes (from camera MQTT)
        self._yolo_frame     = None   # annotated JPEG (from ROS2 YOLO)
        self._robot_status   = {}
        self._ball_detection = {}
        self._range_m        = -1.0
        self._imu_ypr        = [0.0, 0.0, 0.0]
        self._imu_gyro       = [0.0, 0.0, 0.0]
        self._imu_accel      = [0.0, 0.0, 0.0]
        self._imu_ypr_raw    = [0.0, 0.0, 0.0]
        self._imu_ypr_ekf    = [0.0, 0.0, 0.0]
        self._imu_ekf_bias   = [0.0, 0.0, 0.0]
        self._imu_has_ekf    = False
        self._map_png        = None
        self._map_info       = {}
        self._ros2_map_active = False   # True once ROS2 SLAM Toolbox publishes /map
        self._slam_map_version = 0     # counter for Pi-side SLAM PNG refresh
        self._robot_pose     = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._robot_velocity = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self._robot_stationary = True
        self._cmd_velocity   = {'linear_x': 0.0, 'angular_z': 0.0}
        self._scan_points    = []
        self._voice_log      = deque(maxlen=20)
        self._event_log      = deque(maxlen=100)
        self._battery        = {'voltage': -1.0, 'percent': -1, 'status': 'unknown'}
        self._temperature    = {'value': -1.0, 'unit': 'C'}
        self._watchdog       = {}
        self._speed_profile  = 'normal'
        self._claw_open      = False
        self._led_state      = {'mode': 'off', 'color': 'off'}
        self._head_state     = {'angle': 0.0}
        self._arm_state      = {'j1': 0.0, 'j2': 120.0, 'j3': 0.0, 'j4': 0.0}
        self._slam_map       = {}     # Pi-side ultrasonic SLAM map
        self._path_recorder  = {}     # path recorder status
        self._recorded_path  = []     # recorded path waypoints
        self._detection_enabled = True  # YOLO detection toggle
        self._obstacle_avoidance_enabled = False  # obstacle avoidance during replay
        self._collision_guard_enabled = False      # collision guard for manual control
        self._calibration_status = {}   # calibration node status
        self._calibration_result = {}   # calibration result
        self._explorer_status = {}      # explorer node status
        self._mission_status = {}       # mission node status
        self._precision_drive_status = {}  # precision drive status
        self._precision_drive_result = {}  # precision drive result
        self._tts_enabled = True        # TTS toggle
        self._zones = []                   # forbidden zones [{id, x1, y1, x2, y2}]
        self._zone_counter = 0             # auto-increment zone ID
        self._start_time = time.time()     # for sim_time compatibility

        # ── MQTT client (primary sensor source) ──────────────
        if self._mqtt_broker:
            self._mqtt = mqtt_client.Client(client_id='samurai_dashboard')
            self._mqtt.on_connect    = self._mqtt_on_connect
            self._mqtt.on_disconnect = self._mqtt_on_disconnect
            self._mqtt.on_message    = self._mqtt_on_message
            self._mqtt.reconnect_delay_set(min_delay=0.5, max_delay=5)
            self._mqtt.connect_async(self._mqtt_broker, self._mqtt_port,
                                     keepalive=15)
            self._mqtt.loop_start()
            self.get_logger().info(
                f'MQTT client connecting → {self._mqtt_broker}:{self._mqtt_port}')
        else:
            self._mqtt = None
            self.get_logger().warn(
                'MQTT_BROKER env not set — sensor data from ROS2 only')

        # ── ROS2: SLAM data (map, filtered odometry, YOLO image) ─
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Annotated YOLO image (ROS2 only — processed on laptop)
        self.create_subscription(
            CompressedImage, '/yolo/annotated/compressed', self._yolo_image_cb, 5)
        # SLAM map
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        # Filtered odometry from EKF (SLAM-fused pose)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)
        # Laser scan (for map overlay)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        # YOLO detections (ROS2 — for ball detection)
        self.create_subscription(
            String, '/ball_detection', self._detection_cb, 10)

        # ── ROS2 Publishers (→ mqtt_bridge_compute → Pi) ─────
        # Also kept for Nav2 integration (Nav2 publishes /cmd_vel)
        self._pub_map_save = self.create_publisher(String, '/map_manager/save', 10)
        self._pub_map_load = self.create_publisher(String, '/map_manager/load', 10)
        self._pub_yolo_enable = self.create_publisher(String, '/yolo/enable', 10)

        ip = _get_local_ip()
        self.get_logger().info(f'Dashboard node started — http://{ip}:{self._port}')
        self.get_logger().info(f'REST API           — http://{ip}:{self._port}/api/status')

    # ── MQTT Client Callbacks ──────────────────────────────────

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f'MQTT connect failed rc={rc}')
            return
        self._mqtt_connected = True
        p = self._prefix
        # Subscribe to all Pi sensor topics
        for topic in ['camera', 'range', 'imu', 'battery', 'temperature',
                      'status', 'odom', 'claw/state', 'head/state', 'arm/state',
                      'watchdog', 'voice_command', 'speed_profile/active',
                      'slam_map', 'path_recorder/status', 'path_recorder/path',
                      'calibration/status', 'calibration/result',
                      'explorer/status', 'mission/status',
                      'precision_drive/status', 'precision_drive/result',
                      'collision_guard/state',
                      # Remote GPU YOLO detections (via MQTT)
                      'ball_detection', 'detections', 'yolo/annotated',
                      'yolo/status',
                      # Centralized error log from all Pi nodes
                      'log/events',
                      # LED panel state
                      'led/state']:
            client.subscribe(f'{p}/{topic}', qos=0)
        self.get_logger().info(f'MQTT connected to {self._mqtt_broker} — subscribed to Pi topics')

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected (rc={rc}), reconnecting...')

    def _mqtt_on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self._prefix) + 1:]
        try:
            if suffix == 'camera':
                self._mqtt_camera(msg.payload)
            elif suffix == 'range':
                self._mqtt_range(msg.payload)
            elif suffix == 'imu':
                self._mqtt_imu(msg.payload)
            elif suffix == 'battery':
                self._mqtt_battery(msg.payload)
            elif suffix == 'temperature':
                self._mqtt_temperature(msg.payload)
            elif suffix == 'status':
                self._mqtt_status(msg.payload)
            elif suffix == 'odom':
                self._mqtt_odom(msg.payload)
            elif suffix == 'claw/state':
                self._mqtt_claw_state(msg.payload)
            elif suffix == 'head/state':
                self._mqtt_head_state(msg.payload)
            elif suffix == 'arm/state':
                self._mqtt_arm_state(msg.payload)
            elif suffix == 'watchdog':
                self._mqtt_watchdog(msg.payload)
            elif suffix == 'voice_command':
                self._mqtt_voice(msg.payload)
            elif suffix == 'speed_profile/active':
                self._mqtt_speed_profile(msg.payload)
            elif suffix == 'slam_map':
                self._mqtt_slam_map(msg.payload)
            elif suffix == 'path_recorder/status':
                self._mqtt_path_status(msg.payload)
            elif suffix == 'path_recorder/path':
                self._mqtt_path_data(msg.payload)
            elif suffix == 'calibration/status':
                self._mqtt_json_field(msg.payload, '_calibration_status')
            elif suffix == 'calibration/result':
                self._mqtt_json_field(msg.payload, '_calibration_result')
            elif suffix == 'explorer/status':
                self._mqtt_json_field(msg.payload, '_explorer_status')
            elif suffix == 'mission/status':
                self._mqtt_json_field(msg.payload, '_mission_status')
            elif suffix == 'precision_drive/status':
                self._mqtt_json_field(msg.payload, '_precision_drive_status')
            elif suffix == 'precision_drive/result':
                self._mqtt_json_field(msg.payload, '_precision_drive_result')
            elif suffix == 'collision_guard/state':
                val = msg.payload.decode('utf-8', errors='ignore').strip().lower()
                with self._lock:
                    self._collision_guard_enabled = (val == 'on')
            # Remote GPU YOLO detections (via MQTT)
            elif suffix == 'ball_detection':
                self._mqtt_ball_detection(msg.payload)
            elif suffix == 'detections':
                pass  # summary — not needed, ball_detection is primary
            elif suffix == 'yolo/annotated':
                self._mqtt_yolo_annotated(msg.payload)
            elif suffix == 'yolo/status':
                self._mqtt_yolo_status(msg.payload)
            elif suffix == 'log/events':
                self._mqtt_log_event(msg.payload)
            elif suffix == 'led/state':
                try:
                    d = json.loads(msg.payload)
                    with self._lock:
                        self._led_state = d
                except Exception:
                    pass
        except Exception as exc:
            self.get_logger().error(f'MQTT msg error [{suffix}]: {exc}')

    def _mqtt_camera(self, payload):
        flip = int(os.environ.get('CAMERA_FLIP', '-1'))  # -1=rotate180°, 0=vertical, 1=horizontal, 99=none
        if flip != 99:
            np_arr = np.frombuffer(bytes(payload), np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                frame = cv2.flip(frame, flip)
                _, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                payload = enc.tobytes()
        with self._lock:
            self._latest_frame = bytes(payload)

    def _mqtt_range(self, payload):
        try:
            d = json.loads(payload)
            r = float(d.get('range', d) if isinstance(d, dict) else d)
        except Exception:
            try:
                r = float(payload)
            except Exception:
                return
        with self._lock:
            self._range_m = round(r, 3)

    def _mqtt_imu(self, payload):
        d = json.loads(payload)
        gx = d.get('gx', 0.0)
        gy = d.get('gy', 0.0)
        gz = d.get('gz', 0.0)
        ax = d.get('ax', 0.0)
        ay = d.get('ay', 0.0)
        az = d.get('az', 1.0)
        # Raw pitch/roll from accel (no yaw)
        pitch = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
        roll  = math.degrees(math.atan2(-ax, az))
        # EKF filtered (if present in message from Pi)
        ekf = d.get('ekf')
        with self._lock:
            self._imu_gyro  = [round(gx, 4), round(gy, 4), round(gz, 4)]
            self._imu_accel = [round(ax, 4), round(ay, 4), round(az, 4)]
            self._imu_ypr_raw = [0.0, round(pitch, 1), round(roll, 1)]
            if ekf:
                self._imu_ypr_ekf = [
                    round(ekf.get('yaw', 0.0), 1),
                    round(ekf.get('pitch', 0.0), 1),
                    round(ekf.get('roll', 0.0), 1),
                ]
                self._imu_ekf_bias = [
                    ekf.get('bias_gx', 0.0),
                    ekf.get('bias_gy', 0.0),
                    ekf.get('bias_gz', 0.0),
                ]
                self._imu_has_ekf = True
            # Default: use EKF if available, raw otherwise
            self._imu_ypr = list(self._imu_ypr_ekf) if self._imu_has_ekf else list(self._imu_ypr_raw)

    def _mqtt_battery(self, payload):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._lock:
            self._battery = d

    def _mqtt_temperature(self, payload):
        try:
            d = json.loads(payload)
        except Exception:
            try:
                d = {'value': float(payload), 'unit': 'C'}
            except Exception:
                return
        with self._lock:
            self._temperature = d

    def _mqtt_status(self, payload):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            d = json.loads(text)
        except Exception:
            return
        with self._lock:
            self._robot_status = d

    def _mqtt_odom(self, payload):
        d = json.loads(payload)
        x     = d.get('x', 0.0)
        y     = d.get('y', 0.0)
        theta = d.get('theta', 0.0)
        vx    = d.get('vx', 0.0)
        vz    = d.get('vz', 0.0)
        stationary = d.get('stationary', False)
        with self._lock:
            self._robot_pose     = {'x': round(x, 3), 'y': round(y, 3),
                                    'yaw': round(theta, 3)}
            speed = d.get('speed', abs(vx))
            self._robot_velocity = {'linear_x': round(vx, 3),
                                    'linear_y': 0.0,
                                    'angular_z': round(vz, 3),
                                    'speed': round(speed, 3)}
            self._robot_stationary = stationary

    def _mqtt_claw_state(self, payload):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            d = json.loads(text) if text.startswith('{') else {'angle': int(text)}
        except Exception:
            return
        angle = d.get('angle', 90)
        with self._lock:
            self._claw_open = (angle < 60)

    def _mqtt_head_state(self, payload):
        try:
            d = json.loads(payload)
            with self._lock:
                self._head_state = d
        except Exception:
            pass

    def _mqtt_arm_state(self, payload):
        try:
            d = json.loads(payload)
            with self._lock:
                self._arm_state = d
                # j4 is the claw — open when angle < 60°
                j4 = d.get('j4', 90)
                self._claw_open = (j4 < 60)
        except Exception:
            pass

    def _mqtt_watchdog(self, payload):
        try:
            d = json.loads(payload)
            with self._lock:
                self._watchdog = d
        except Exception:
            pass

    def _mqtt_voice(self, payload):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            entry = {'text': text, 'time': time.strftime('%H:%M:%S'), 'type': 'voice'}
            with self._lock:
                self._voice_log.append(entry)
                self._event_log.append(entry)
        except Exception:
            pass

    def _mqtt_speed_profile(self, payload):
        try:
            text = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
            with self._lock:
                self._speed_profile = text.strip()
        except Exception:
            pass

    def _mqtt_slam_map(self, payload):
        try:
            d = json.loads(payload)
            need_png = False
            with self._lock:
                self._slam_map = d
                # When ROS2 SLAM Toolbox is not running, use Pi-side SLAM
                # to populate map_info and generate map PNG
                if not self._ros2_map_active and 'info' in d:
                    need_png = True
                    self._slam_map_version += 1
                    si = d['info']
                    self._map_info = {
                        'width':        si.get('width', 200),
                        'height':       si.get('height', 200),
                        'resolution':   si.get('resolution', 0.05),
                        'origin_x':     si.get('origin_x', -5.0),
                        'origin_y':     si.get('origin_y', -5.0),
                        'update_count': self._slam_map_version,
                    }
            if need_png:
                png = self.get_slam_map_png()
                if png:
                    with self._lock:
                        self._map_png = png
        except Exception:
            pass

    def _mqtt_path_status(self, payload):
        try:
            d = json.loads(payload)
            with self._lock:
                self._path_recorder = d
        except Exception:
            pass

    def _mqtt_json_field(self, payload, field_name):
        try:
            d = json.loads(payload)
            with self._lock:
                setattr(self, field_name, d)
        except Exception:
            pass

    def _mqtt_path_data(self, payload):
        try:
            d = json.loads(payload)
            with self._lock:
                self._recorded_path = d
        except Exception:
            pass

    # ── Remote GPU YOLO handlers (via MQTT) ─────────────────────

    def _mqtt_ball_detection(self, payload):
        """Ball detection from remote GPU laptop (via MQTT directly)."""
        try:
            data = json.loads(payload)
            with self._lock:
                self._ball_detection = data
        except Exception:
            pass

    def _mqtt_yolo_annotated(self, payload):
        """Annotated JPEG frame from remote GPU laptop (via MQTT)."""
        with self._lock:
            self._yolo_frame = bytes(payload)
            self._latest_frame = self._yolo_frame

    def _mqtt_yolo_status(self, payload):
        """GPU YOLO detector online/offline status."""
        try:
            d = json.loads(payload)
            online = d.get('online', False)
            self.get_logger().info(
                f'Remote GPU YOLO: {"ONLINE" if online else "OFFLINE"}')
        except Exception:
            pass

    def _mqtt_log_event(self, payload):
        """Centralized log event from Pi nodes — store in event_log."""
        try:
            d = json.loads(payload)
            entry = {
                'ts': d.get('ts', time.time()),
                'source': d.get('node', '?'),
                'level': d.get('level', 'INFO'),
                'text': d.get('msg', ''),
            }
            with self._lock:
                self._event_log.append(entry)
        except Exception:
            pass

    # ── MQTT publish helpers (commands → Pi directly) ──────────

    def _mqtt_pub(self, subtopic: str, payload, qos=0):
        """Publish directly to Pi via MQTT."""
        if self._mqtt and self._mqtt_connected:
            if isinstance(payload, (dict, list)):
                payload = json.dumps(payload)
            self._mqtt.publish(f'{self._prefix}/{subtopic}', payload, qos=qos)
            return True
        self.get_logger().warn(f'MQTT not connected — cannot publish {subtopic}')
        return False

    # ── ROS2 Callbacks (SLAM data only) ───────────────────────────

    def _yolo_image_cb(self, msg: CompressedImage):
        """Annotated YOLO frame — overlay on raw camera frame if available."""
        with self._lock:
            self._yolo_frame = bytes(msg.data)
            # Use annotated frame as display frame when available
            self._latest_frame = self._yolo_frame

    def _detection_cb(self, msg: String):
        """YOLO ball detection from ROS2 (processed on laptop)."""
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._ball_detection = data
        except json.JSONDecodeError:
            pass

    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 128, dtype=np.uint8)
        img[data == 0]   = [240, 240, 240]
        img[data > 50]   = [30,  30,  30]
        img = cv2.flip(img, 0)
        _, png = cv2.imencode('.png', img)
        with self._lock:
            self._ros2_map_active = True
            self._map_png  = png.tobytes()
            self._map_info = {
                'width': w, 'height': h,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
            }

    def _odom_cb(self, msg: Odometry):
        """Filtered odometry from EKF (SLAM-fused) — overrides raw MQTT odom."""
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._robot_pose = {'x': round(pos.x, 3), 'y': round(pos.y, 3),
                                'yaw': round(yaw, 3)}
            self._robot_velocity = {
                'linear_x':  round(msg.twist.twist.linear.x,  3),
                'linear_y':  round(msg.twist.twist.linear.y,  3),
                'angular_z': round(msg.twist.twist.angular.z, 3),
            }

    def _scan_cb(self, msg: LaserScan):
        points, angle = [], msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                points.append([round(r * math.cos(angle), 3), round(r * math.sin(angle), 3)])
            angle += msg.angle_increment
        with self._lock:
            self._scan_points = points

    # ── Command helpers (publish directly to Pi via MQTT) ──────────

    def send_command(self, command: str):
        """Send voice/FSM command to Pi."""
        self._mqtt_pub('voice_command', command, qos=1)
        entry = {'text': command, 'time': time.strftime('%H:%M:%S'), 'type': 'api_command'}
        with self._lock:
            self._event_log.append(entry)

    def send_velocity(self, linear_x: float, angular_z: float):
        """Send cmd_vel/manual directly to Pi (manual priority — overrides autonomous)."""
        self._mqtt_pub('cmd_vel/manual',
                       {'linear_x': round(linear_x, 3), 'angular_z': round(angular_z, 3)},
                       qos=0)
        with self._lock:
            self._cmd_velocity = {'linear_x': round(linear_x, 3),
                                  'angular_z': round(angular_z, 3)}

    def set_claw(self, state: str):
        # Claw is arm joint 4 (index 4, 1-based). open=0°, close=180°
        angle = 0 if state == 'open' else 180
        self._mqtt_pub('arm/command',
                       json.dumps({'joint': 4, 'angle': angle}), qos=1)
        with self._lock:
            self._claw_open = (state == 'open')

    def set_head(self, payload: dict):
        self._mqtt_pub('head/command', json.dumps(payload), qos=1)

    def set_arm(self, payload: dict):
        self._mqtt_pub('arm/command', json.dumps(payload), qos=1)

    def set_speed_profile(self, profile: str):
        self._mqtt_pub('speed_profile', profile, qos=1)

    def set_patrol_command(self, command: str):
        self._mqtt_pub('patrol/command', command, qos=1)

    def set_patrol_waypoints(self, waypoints: list):
        self._mqtt_pub('patrol/waypoints', json.dumps(waypoints), qos=1)

    def set_follow_me(self, command: str):
        self._mqtt_pub('follow_me/command', command, qos=1)

    def set_path_recorder(self, payload: dict):
        self._mqtt_pub('path_recorder/command', json.dumps(payload), qos=1)

    def reset_position(self):
        """Reset robot odometry to (0,0,0) — current pose becomes home."""
        self._mqtt_pub('reset_position', 'reset', qos=1)
        with self._lock:
            self._robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            self._robot_stationary = True

    def set_detection_enabled(self, enabled: bool):
        """Toggle YOLO detection on/off."""
        with self._lock:
            self._detection_enabled = enabled
        # Publish to ROS2 YOLO node
        msg = String()
        msg.data = 'on' if enabled else 'off'
        self._pub_yolo_enable.publish(msg)
        # Also publish to MQTT for standalone object_detector_node
        self._mqtt_pub('detection/enable', 'on' if enabled else 'off', qos=1)

    def set_obstacle_avoidance(self, enabled: bool):
        """Toggle obstacle avoidance during path replay."""
        with self._lock:
            self._obstacle_avoidance_enabled = enabled
        self._mqtt_pub('obstacle_avoidance/enable', 'on' if enabled else 'off', qos=1)

    def set_collision_guard(self, enabled: bool):
        """Toggle collision guard for manual control."""
        with self._lock:
            self._collision_guard_enabled = enabled
        self._mqtt_pub('collision_guard/enable', 'on' if enabled else 'off', qos=1)

    def save_map(self, name: str):
        self._pub_str(self._pub_map_save, name)

    def load_map(self, name: str):
        self._pub_str(self._pub_map_load, name)

    def _pub_str(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    # ── Data access (for Flask) ────────────────────────────────────

    def get_frame(self):
        with self._lock:
            return self._latest_frame

    def get_map_png(self):
        with self._lock:
            return self._map_png

    def get_slam_map_png(self):
        """Generate PNG from Pi-side SLAM map data (fallback when ROS2 SLAM unavailable)."""
        with self._lock:
            slam = self._slam_map
        if not slam:
            return None
        try:
            info = slam.get('info', {})
            w = info.get('width', 200)
            h = info.get('height', 200)
            res = info.get('resolution', 0.05)
            ox = info.get('origin_x', -5.0)
            oy = info.get('origin_y', -5.0)

            # Create image: gray=unknown, white=free, black=occupied
            img = np.full((h, w, 3), 128, dtype=np.uint8)

            # Mark obstacles
            for obs in slam.get('obstacles', []):
                ci = int((obs[0] - ox) / res)
                cj = int((obs[1] - oy) / res)
                if 0 <= ci < w and 0 <= cj < h:
                    img[cj, ci] = [30, 30, 30]

            # Mark trail (blue line)
            for pt in slam.get('trail', []):
                ci = int((pt[0] - ox) / res)
                cj = int((pt[1] - oy) / res)
                if 0 <= ci < w and 0 <= cj < h:
                    img[cj, ci] = [255, 180, 50]  # blue-ish in BGR

            # Mark robot position (red dot)
            robot = slam.get('robot', {})
            rx = robot.get('x', 0.0)
            ry = robot.get('y', 0.0)
            rci = int((rx - ox) / res)
            rcj = int((ry - oy) / res)
            for di in range(-2, 3):
                for dj in range(-2, 3):
                    ni, nj = rci + di, rcj + dj
                    if 0 <= ni < w and 0 <= nj < h:
                        img[nj, ni] = [0, 0, 255]  # red in BGR

            img = cv2.flip(img, 0)  # flip Y for image coordinates
            _, png = cv2.imencode('.png', img)
            return png.tobytes()
        except Exception:
            return None

    def get_state(self):
        """Full state dict for SocketIO push."""
        with self._lock:
            yaw_rad = self._robot_pose.get('yaw', 0.0)
            yaw_deg = round(math.degrees(yaw_rad), 1)
            # Build detection objects list from ball_detection
            det = self._ball_detection
            det_objects = det.get('objects', det.get('balls', []))
            return {
                'status':       self._robot_status,
                'detection':    det,
                'all_detections': det_objects,
                'range_m':      self._range_m,
                'imu': {
                    'yaw': self._imu_ypr[0], 'pitch': self._imu_ypr[1], 'roll': self._imu_ypr[2],
                    'gyro':  {'x': self._imu_gyro[0],  'y': self._imu_gyro[1],  'z': self._imu_gyro[2]},
                    'accel': {'x': self._imu_accel[0], 'y': self._imu_accel[1], 'z': self._imu_accel[2]},
                },
                'imu_ypr':     list(self._imu_ypr),
                'imu_accel_x': self._imu_accel[0],
                'imu_gyro_z':  self._imu_gyro[2],
                'imu_accel':   list(self._imu_accel),
                'imu_gyro':    list(self._imu_gyro),
                'imu_ypr_raw':   list(self._imu_ypr_raw),
                'imu_ypr_ekf':   list(self._imu_ypr_ekf) if self._imu_has_ekf else None,
                'imu_ekf_bias':  list(self._imu_ekf_bias) if self._imu_has_ekf else None,
                'imu_has_ekf':   self._imu_has_ekf,
                'pose':         {**self._robot_pose, 'yaw_deg': yaw_deg},
                'stationary':   self._robot_stationary,
                'velocity':     {
                    **self._robot_velocity,
                    'linear':  self._robot_velocity.get('linear_x', 0.0),
                    'angular': self._robot_velocity.get('angular_z', 0.0),
                    'speed':   self._robot_velocity.get('speed', 0.0),
                },
                'cmd_velocity': self._cmd_velocity,
                'map_info':     self._map_info,
                'scan_points':  self._scan_points,
                'voice_log':    list(self._voice_log),
                'event_log':    list(self._event_log)[-30:],
                'battery':         self._battery,
                'battery_voltage': self._battery.get('voltage', -1.0),
                'battery_percent': self._battery.get('percent', self._battery.get('percentage', -1)),
                'cpu_temp':        self._temperature.get('value', -1.0) if isinstance(self._temperature, dict) else float(self._temperature),
                'temperature':     self._temperature,
                'watchdog':        self._watchdog,
                'speed_profile': self._speed_profile,
                'actuators': {
                    'claw':  'open'  if self._claw_open else 'closed',
                    'claw_open': self._claw_open,
                },
                'head': dict(self._head_state),
                'arm':  dict(self._arm_state),
                'slam_map':       dict(self._slam_map) if self._slam_map else None,
                'path_recorder':  dict(self._path_recorder) if self._path_recorder else None,
                'recorded_path':  list(self._recorded_path) if self._recorded_path else None,
                'detection_enabled': self._detection_enabled,
                'obstacle_avoidance_enabled': self._obstacle_avoidance_enabled,
                'collision_guard_enabled': self._collision_guard_enabled,
                'calibration':        dict(self._calibration_status) if self._calibration_status else None,
                'calibration_result': dict(self._calibration_result) if self._calibration_result else None,
                'explorer':           dict(self._explorer_status) if self._explorer_status else None,
                'mission':            dict(self._mission_status) if self._mission_status else None,
                'tts_enabled':        self._tts_enabled,
                'precision_drive':        dict(self._precision_drive_status) if self._precision_drive_status else None,
                'precision_drive_result': dict(self._precision_drive_result) if self._precision_drive_result else None,
                'led':                dict(self._led_state),
                # Fields expected by admin.html
                'sim_time':      round(time.time() - self._start_time, 2),
                'zones':         list(self._zones),
                'planned_path':  list(self._recorded_path) if self._recorded_path else [],
                'balls':         [],  # populated by simulator only
            }

    def _snapshot(self):
        """Compact snapshot for /api/status."""
        with self._lock:
            return {
                'robot_status':  self._robot_status,
                'pose':          dict(self._robot_pose),
                'velocity':      dict(self._robot_velocity),
                'cmd_velocity':  dict(self._cmd_velocity),
                'sensors': {
                    'ultrasonic': {'range_m': self._range_m},
                    'imu': {
                        'yaw': self._imu_ypr[0], 'pitch': self._imu_ypr[1], 'roll': self._imu_ypr[2],
                        'gyro':  {'x': self._imu_gyro[0],  'y': self._imu_gyro[1],  'z': self._imu_gyro[2]},
                        'accel': {'x': self._imu_accel[0], 'y': self._imu_accel[1], 'z': self._imu_accel[2]},
                    },
                },
                'detection':     dict(self._ball_detection),
                'actuators': {
                    'claw':  'open' if self._claw_open else 'closed',
                },
                'head': dict(self._head_state),
                'arm':  dict(self._arm_state),
                'battery':       dict(self._battery),
                'temperature':   dict(self._temperature),
                'watchdog':      dict(self._watchdog),
                'speed_profile': self._speed_profile,
                'map_info':      dict(self._map_info),
                'collision_guard_enabled': self._collision_guard_enabled,
                'event_log':     list(self._event_log)[-20:],
            }


# =============================================================
# FastAPI Application
# =============================================================

def _find_static_dir() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        candidate = os.path.join(d, 'compute_node')
        if os.path.isdir(candidate) and os.path.isdir(os.path.join(candidate, 'static')):
            return candidate
        d = os.path.dirname(d)
    return os.path.dirname(os.path.abspath(__file__))


def _ok(**kwargs):
    return {'ok': True, **kwargs}


def _err(message: str):
    return JSONResponse({'ok': False, 'error': message}, status_code=400)


def create_app(ros_node: DashboardNode):
    base_dir   = _find_static_dir()
    static_dir = os.path.join(base_dir, 'static')

    # ── Socket.IO server ──────────────────────────────────────────
    sio = socketio.AsyncServer(
        async_mode='asgi',
        cors_allowed_origins='*',
        logger=False,
        engineio_logger=False,
        ping_interval=10,   # server pings browser every 10s (default 25)
        ping_timeout=5,     # 5s to respond before disconnect (default 20)
    )

    @sio.event
    async def connect(sid, environ):
        pass

    @sio.event
    async def disconnect(sid):
        pass

    @sio.event
    async def send_command(sid, data):
        text = data.get('text', '') if isinstance(data, dict) else str(data)
        ros_node.send_command(text)

    @sio.event
    async def reset_sim(sid, data):
        pass  # no-op in robot mode

    # ── FastAPI app ───────────────────────────────────────────────
    app = FastAPI(title='Samurai Dashboard')
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_methods=['*'],
        allow_headers=['*'],
    )

    # ── Background: push state via Socket.IO (4 Hz) ───────────────
    async def _sio_push_loop():
        _last_json = ''
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            state = ros_node.get_state()
            state_json = json.dumps(state, separators=(',', ':'))
            if state_json == _last_json:
                continue
            _last_json = state_json
            await sio.emit('state_update', state)

    @app.on_event('startup')
    async def startup():
        asyncio.create_task(_sio_push_loop())

    # ── Page routes (React SPA — all pages served from index.html) ─
    templates_dir = os.path.join(base_dir, 'templates')
    index_html = os.path.join(static_dir, 'index.html')

    def _serve_spa():
        """Serve React SPA index.html, fallback to old HTML templates."""
        if os.path.isfile(index_html):
            return FileResponse(index_html)
        return JSONResponse({'error': 'Frontend not built — run npm build in compute_node/frontend/'}, 404)

    @app.get('/')
    async def serve_root():
        return _serve_spa()

    @app.get('/dashboard')
    async def serve_dashboard():
        return _serve_spa()

    @app.get('/admin')
    async def serve_admin():
        return _serve_spa()

    @app.get('/3d')
    async def serve_3d():
        return _serve_spa()

    # ── MJPEG video stream ────────────────────────────────────────
    async def _mjpeg_generator():
        while True:
            frame = ros_node.get_frame()
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            await asyncio.sleep(0.033)  # ~30fps cap

    @app.get('/video_feed')
    async def video_feed():
        from starlette.responses import StreamingResponse
        return StreamingResponse(_mjpeg_generator(),
                                 media_type='multipart/x-mixed-replace; boundary=frame')

    # ── WebSocket binary camera stream (low-latency) ───────────
    from fastapi import WebSocket as FastAPIWebSocket, WebSocketDisconnect

    @app.websocket('/ws/camera')
    async def ws_camera(ws: FastAPIWebSocket):
        await ws.accept()
        try:
            while True:
                frame = ros_node.get_frame()
                if frame:
                    await ws.send_bytes(frame)
                await asyncio.sleep(0.033)  # ~30fps cap
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.get('/map.png')
    async def map_image_legacy():
        png = ros_node.get_map_png()
        return Response(content=png or b'\x89PNG\r\n\x1a\n', media_type='image/png')

    # ==========================================================
    # REST API — GET
    # ==========================================================

    @app.get('/api/')
    async def api_index():
        return _ok(endpoints={
            'GET': ['/api/status', '/api/robot/pose', '/api/robot/velocity',
                    '/api/sensors', '/api/sensors/ultrasonic', '/api/sensors/imu',
                    '/api/detection', '/api/detection/closest',
                    '/api/fsm', '/api/actuators', '/api/battery', '/api/speed_profile',
                    '/api/camera/frame', '/api/camera/frame.json',
                    '/api/map/image', '/api/map/info', '/api/map/list',
                    '/api/slam_map', '/api/path_recorder/status',
                    '/api/path_recorder/path', '/api/path_recorder/list',
                    '/api/log'],
            'POST': ['/api/emergency_stop', '/api/robot/reset_position',
                     '/api/fsm/command',
                     '/api/actuators/claw',
                     '/api/speed_profile', '/api/patrol/command', '/api/patrol/waypoints',
                     '/api/follow_me', '/api/path_recorder/command',
                     '/api/map/save', '/api/map/load'],
        })

    @app.get('/api/status')
    async def api_status():
        return ros_node._snapshot()

    @app.get('/api/robot/pose')
    async def api_pose():
        with ros_node._lock:
            return _ok(**ros_node._robot_pose)

    @app.get('/api/robot/velocity')
    async def api_velocity():
        with ros_node._lock:
            return _ok(estimated=dict(ros_node._robot_velocity),
                       commanded=dict(ros_node._cmd_velocity))

    @app.post('/api/robot/velocity')
    async def api_set_velocity(req: Request):
        body = await req.json()
        linear_x  = float(body.get('linear_x',  body.get('linear',  0.0)))
        angular_z = float(body.get('angular_z', body.get('angular', 0.0)))
        ros_node.send_velocity(linear_x, angular_z)
        return _ok(linear_x=linear_x, angular_z=angular_z)

    @app.get('/api/mqtt/status')
    async def api_mqtt_status():
        return _ok(
            connected=ros_node._mqtt_connected,
            broker=ros_node._mqtt_broker,
            port=ros_node._mqtt_port,
            robot_id=ros_node._robot_id,
        )

    @app.get('/api/sensors')
    async def api_sensors():
        with ros_node._lock:
            return _ok(
                ultrasonic={'range_m': ros_node._range_m},
                imu={'yaw': ros_node._imu_ypr[0], 'pitch': ros_node._imu_ypr[1], 'roll': ros_node._imu_ypr[2],
                     'gyro':  {'x': ros_node._imu_gyro[0],  'y': ros_node._imu_gyro[1],  'z': ros_node._imu_gyro[2]},
                     'accel': {'x': ros_node._imu_accel[0], 'y': ros_node._imu_accel[1], 'z': ros_node._imu_accel[2]}},
            )

    @app.get('/api/sensors/ultrasonic')
    async def api_ultrasonic():
        with ros_node._lock:
            return _ok(range_m=ros_node._range_m)

    @app.get('/api/sensors/imu')
    async def api_imu():
        with ros_node._lock:
            return _ok(yaw=ros_node._imu_ypr[0], pitch=ros_node._imu_ypr[1], roll=ros_node._imu_ypr[2],
                       gyro ={'x': ros_node._imu_gyro[0],  'y': ros_node._imu_gyro[1],  'z': ros_node._imu_gyro[2]},
                       accel={'x': ros_node._imu_accel[0], 'y': ros_node._imu_accel[1], 'z': ros_node._imu_accel[2]})

    @app.get('/api/detection')
    async def api_detection():
        with ros_node._lock:
            return _ok(detection=ros_node._ball_detection)

    @app.get('/api/detection/closest')
    async def api_detection_closest(color: str = ''):
        color_filter = color.lower().strip()
        with ros_node._lock:
            det = ros_node._ball_detection
        objects = det.get('objects', det.get('balls', []))
        if color_filter:
            objects = [o for o in objects
                       if str(o.get('color', o.get('class', ''))).lower() == color_filter]
        if not objects:
            return _ok(found=False, object=None)
        def _sort_key(o):
            if 'distance' in o:
                return o['distance']
            b = o.get('bbox', [0, 0, 0, 0])
            return -((b[2] - b[0]) * (b[3] - b[1])) if len(b) >= 4 else 0
        return _ok(found=True, object=sorted(objects, key=_sort_key)[0])

    @app.get('/api/fsm')
    async def api_fsm():
        with ros_node._lock:
            status = ros_node._robot_status
        state = status.get('state', status.get('fsm_state', 'unknown'))
        return _ok(state=state, details=status)

    @app.get('/api/actuators')
    async def api_actuators():
        with ros_node._lock:
            return _ok(claw='open' if ros_node._claw_open else 'closed')

    @app.get('/api/battery')
    async def api_battery():
        with ros_node._lock:
            return {'ok': True, **ros_node._battery}

    @app.get('/api/speed_profile')
    async def api_speed_profile_get():
        with ros_node._lock:
            return _ok(profile=ros_node._speed_profile)

    @app.get('/api/camera/frame')
    async def api_camera_frame():
        frame = ros_node.get_frame()
        if not frame:
            return JSONResponse({'ok': False, 'error': 'No frame available yet'}, status_code=503)
        return Response(content=frame, media_type='image/jpeg')

    @app.get('/api/camera/frame.json')
    async def api_camera_frame_json():
        frame = ros_node.get_frame()
        if not frame:
            return JSONResponse({'ok': False, 'error': 'No frame available yet'}, status_code=503)
        return _ok(jpeg_b64=base64.b64encode(frame).decode())

    @app.get('/api/map/image')
    async def api_map_image():
        png = ros_node.get_map_png()
        if not png:
            # Fallback: generate PNG from Pi-side SLAM map
            png = ros_node.get_slam_map_png()
        if not png:
            return JSONResponse({'ok': False, 'error': 'Map not available yet'}, status_code=503)
        return Response(content=png, media_type='image/png')

    @app.get('/api/map/info')
    async def api_map_info():
        with ros_node._lock:
            info = dict(ros_node._map_info)
        if not info:
            # Fallback: Pi-side SLAM map info
            with ros_node._lock:
                slam = ros_node._slam_map
            if slam and 'info' in slam:
                return _ok(**slam['info'])
            return JSONResponse({'ok': False, 'error': 'Map not available yet'}, status_code=503)
        return _ok(**info)

    @app.get('/api/map/list')
    async def api_map_list():
        maps_dir = os.path.expanduser('~/maps')
        if not os.path.isdir(maps_dir):
            return _ok(maps=[])
        maps = sorted(f.replace('.yaml', '') for f in os.listdir(maps_dir) if f.endswith('.yaml'))
        return _ok(maps=maps)

    @app.get('/api/path_recorder/list')
    async def api_path_list():
        paths_dir = os.path.expanduser('~/paths')
        if not os.path.isdir(paths_dir):
            return _ok(paths=[])
        paths = sorted(f.replace('.json', '') for f in os.listdir(paths_dir) if f.endswith('.json'))
        return _ok(paths=paths)

    @app.get('/api/log')
    async def api_log(limit: int = 50):
        limit = min(limit, 100)
        with ros_node._lock:
            log = list(ros_node._event_log)[-limit:]
        return _ok(log=log, count=len(log))

    # ==========================================================
    # REST API — POST
    # ==========================================================

    @app.post('/api/robot/reset_position')
    async def api_reset_position():
        """Reset odometry to (0,0,0) — current pose becomes new home."""
        ros_node.reset_position()
        return _ok(action='reset_position', pose={'x': 0.0, 'y': 0.0, 'yaw': 0.0})

    @app.get('/api/slam_map')
    async def api_slam_map():
        """Get Pi-side ultrasonic SLAM map data."""
        with ros_node._lock:
            slam = dict(ros_node._slam_map) if ros_node._slam_map else {}
        if not slam:
            return JSONResponse({'ok': False, 'error': 'SLAM map not available'}, status_code=503)
        return _ok(**slam)

    @app.get('/api/path_recorder/status')
    async def api_path_recorder_status():
        with ros_node._lock:
            status = dict(ros_node._path_recorder) if ros_node._path_recorder else {}
        return _ok(**status)

    @app.get('/api/path_recorder/path')
    async def api_path_recorder_path():
        with ros_node._lock:
            path = list(ros_node._recorded_path) if ros_node._recorded_path else []
        return _ok(path=path, waypoints=len(path))

    @app.post('/api/emergency_stop')
    @app.post('/api/robot/stop')
    async def api_emergency_stop():
        """Immediate stop — sends zero velocity directly to Pi."""
        ros_node.send_velocity(0.0, 0.0)
        return _ok(action='stop')

    @app.post('/api/fsm/command')
    async def api_fsm_command(req: Request):
        body = await req.json()
        command = body.get('command', body.get('text', '')).strip()
        if not command:
            return _err('"command" field is required')
        # Reject unreasonably long strings (max 200 chars)
        if len(command) > 200:
            return _err('command too long (max 200 characters)')
        ros_node.send_command(command)
        return _ok(sent=command)

    @app.post('/api/actuators/claw')
    async def api_claw(req: Request):
        body = await req.json()
        # Accept both { open: bool } (frontend) and { state: "open"/"close" } (legacy)
        if 'open' in body:
            cmd = 'open' if body['open'] else 'close'
        else:
            state = body.get('state', '').lower().strip()
            if state not in ('open', 'close', 'closed'):
                return _err('body must contain "open" (bool) or "state" ("open"/"close")')
            cmd = 'open' if state == 'open' else 'close'
        ros_node.set_claw(cmd)
        return _ok(claw='open' if cmd == 'open' else 'closed')

    @app.post('/api/led/command')
    async def api_led_command(req: Request):
        body = await req.json()
        mode = body.get('mode', 'off')
        color = body.get('color', '')
        payload = {'mode': mode}
        if color:
            payload['color'] = color
        ros_node._mqtt_pub('led/command', payload, qos=1)
        return _ok(led=mode)

    @app.post('/api/actuators/head')
    async def api_head(req: Request):
        body = await req.json()
        # Accept: {"angle": 0-180} or {"command": "center"}
        if body.get('command') == 'center':
            ros_node._mqtt_pub('head/command', 'center', qos=1)
            return _ok(head='center')
        if 'angle' in body:
            angle = max(0, min(180, float(body['angle'])))
            ros_node.set_head({'angle': angle})
            return _ok(angle=angle)
        return _err('body must contain "angle" or "command":"center"')

    @app.post('/api/actuators/arm')
    async def api_arm(req: Request):
        body = await req.json()
        # Accept: {"joint": 1, "angle": 90} or {"joints": [90,90,90,90]} or {"command":"home"}
        if body.get('command') == 'home':
            ros_node._mqtt_pub('arm/command', 'home', qos=1)
            return _ok(arm='home')
        if 'joint' in body and 'angle' in body:
            joint = int(body['joint'])
            angle = max(0, min(180, float(body['angle'])))
            ros_node.set_arm({'joint': joint, 'angle': angle})
            return _ok(joint=joint, angle=angle)
        if 'joints' in body:
            joints = [max(0, min(180, float(a))) for a in body['joints'][:4]]
            ros_node.set_arm({'joints': joints})
            return _ok(joints=joints)
        return _err('body must contain "joint"+"angle", "joints", or "command":"home"')

    @app.get('/api/actuators/head')
    async def api_head_get():
        with ros_node._lock:
            return _ok(**ros_node._head_state)

    @app.get('/api/actuators/arm')
    async def api_arm_get():
        with ros_node._lock:
            return _ok(**ros_node._arm_state)

    @app.post('/api/speed_profile')
    async def api_speed_profile_set(req: Request):
        body    = await req.json()
        profile = body.get('profile', '').lower().strip()
        if profile not in ('slow', 'normal', 'fast'):
            return _err('profile must be "slow", "normal", or "fast"')
        ros_node.set_speed_profile(profile)
        return _ok(profile=profile)

    @app.post('/api/patrol/command')
    async def api_patrol_command(req: Request):
        body    = await req.json()
        command = body.get('command', '').lower().strip()
        if command not in ('start', 'stop'):
            return _err('command must be "start" or "stop"')
        ros_node.set_patrol_command(command)
        return _ok(patrol=command)

    @app.post('/api/patrol/waypoints')
    async def api_patrol_waypoints(req: Request):
        body      = await req.json()
        waypoints = body.get('waypoints', [])
        if not isinstance(waypoints, list) or not waypoints:
            return _err('waypoints must be a non-empty list of [x, y] pairs')
        ros_node.set_patrol_waypoints(waypoints)
        return _ok(waypoints_set=len(waypoints))

    @app.post('/api/follow_me')
    async def api_follow_me(req: Request):
        body    = await req.json()
        command = body.get('command', '').lower().strip()
        if command not in ('start', 'stop'):
            return _err('command must be "start" or "stop"')
        ros_node.set_follow_me(command)
        return _ok(follow_me=command)

    @app.post('/api/path_recorder/command')
    async def api_path_recorder(req: Request):
        body    = await req.json()
        command = body.get('command', '').lower().strip()
        if command not in ('start', 'stop', 'play', 'record', 'replay', 'clear',
                           'save', 'load'):
            return _err('command must be "start"/"record", "stop", "play"/"replay"')
        ros_node.set_path_recorder(body)
        return _ok(path_recorder=command)

    @app.post('/api/map/save')
    async def api_map_save(req: Request):
        body = await req.json()
        name = body.get('name', 'map').strip()
        if not name:
            return _err('"name" is required')
        ros_node.save_map(name)
        return _ok(saved=name)

    @app.post('/api/map/load')
    async def api_map_load(req: Request):
        body = await req.json()
        name = body.get('name', '').strip()
        if not name:
            return _err('"name" is required')
        ros_node.load_map(name)
        return _ok(loaded=name)

    @app.post('/api/detection/toggle')
    async def api_detection_toggle(req: Request):
        body = await req.json()
        enabled = body.get('enabled', True)
        ros_node.set_detection_enabled(bool(enabled))
        return _ok(detection_enabled=bool(enabled))

    @app.get('/api/detection/status')
    async def api_detection_status():
        with ros_node._lock:
            return _ok(detection_enabled=ros_node._detection_enabled)

    @app.post('/api/obstacle_avoidance/toggle')
    async def api_obstacle_avoidance_toggle(req: Request):
        body = await req.json()
        enabled = body.get('enabled', True)
        ros_node.set_obstacle_avoidance(bool(enabled))
        return _ok(obstacle_avoidance_enabled=bool(enabled))

    @app.post('/api/collision_guard/toggle')
    async def api_collision_guard_toggle(req: Request):
        body = await req.json()
        enabled = body.get('enabled', True)
        ros_node.set_collision_guard(bool(enabled))
        return _ok(collision_guard_enabled=bool(enabled))

    # ── Temperature ────────────────────────────────────────────────
    @app.get('/api/temperature')
    async def api_temperature():
        with ros_node._lock:
            temp = ros_node._temperature
        if isinstance(temp, dict):
            return _ok(**temp)
        return _ok(value=float(temp), unit='C')

    # ── FSM transition (force state change) ──────────────────────
    @app.post('/api/fsm/transition')
    async def api_fsm_transition(req: Request):
        body = await req.json()
        state = body.get('state', '').upper().strip()
        valid = ('IDLE', 'SEARCHING', 'TARGETING', 'APPROACHING',
                 'GRABBING', 'CALLING', 'RETURNING')
        if state not in valid:
            return _err(f'state must be one of {valid}')
        ros_node._mqtt_pub('fsm/transition', state, qos=1)
        ros_node.send_command(f'переход {state}')
        return _ok(transition=state)

    # ── Zones (forbidden map zones) ──────────────────────────────
    @app.get('/api/zones')
    async def api_zones_get():
        with ros_node._lock:
            return _ok(zones=list(ros_node._zones))

    @app.post('/api/zones')
    async def api_zones_create(req: Request):
        body = await req.json()
        x1, y1 = float(body.get('x1', 0)), float(body.get('y1', 0))
        x2, y2 = float(body.get('x2', 0)), float(body.get('y2', 0))
        with ros_node._lock:
            if len(ros_node._zones) >= 50:
                return JSONResponse({'error': 'zone limit reached (50)'},
                                    status_code=400)
            ros_node._zone_counter += 1
            zone = {
                'id': ros_node._zone_counter,
                'x1': min(x1, x2), 'y1': min(y1, y2),
                'x2': max(x1, x2), 'y2': max(y1, y2),
            }
            ros_node._zones.append(zone)
        ros_node._mqtt_pub('zones/update', json.dumps(ros_node._zones), qos=1)
        return _ok(zone=zone)

    @app.delete('/api/zones/{zone_id}')
    async def api_zones_delete(zone_id: int):
        with ros_node._lock:
            ros_node._zones = [z for z in ros_node._zones if z['id'] != zone_id]
        ros_node._mqtt_pub('zones/update', json.dumps(ros_node._zones), qos=1)
        return _ok(deleted=zone_id)

    @app.post('/api/zones/clear')
    async def api_zones_clear():
        with ros_node._lock:
            ros_node._zones = []
            ros_node._zone_counter = 0
        ros_node._mqtt_pub('zones/update', '[]', qos=1)
        return _ok(cleared=True)

    # ── Calibration ────────────────────────────────────────────────
    @app.post('/api/calibration/command')
    async def api_calibration_cmd(req: Request):
        body = await req.json()
        command = body.get('command', '').strip().lower()
        ros_node._mqtt_pub('calibration/command', command, qos=1)
        return _ok(calibration=command)

    # ── Explorer ───────────────────────────────────────────────────
    @app.post('/api/explorer/command')
    async def api_explorer_cmd(req: Request):
        body = await req.json()
        command = body.get('command', '').strip().lower()
        ros_node._mqtt_pub('explorer/command', command, qos=1)
        return _ok(explorer=command)

    # ── Mission ────────────────────────────────────────────────────
    @app.post('/api/mission/command')
    async def api_mission_cmd(req: Request):
        body = await req.json()
        ros_node._mqtt_pub('mission/command', json.dumps(body), qos=1)
        return _ok(mission=body.get('command', ''))

    @app.get('/api/mission/list')
    async def api_mission_list():
        missions_dir = os.path.expanduser('~/missions')
        if not os.path.isdir(missions_dir):
            return _ok(missions=[])
        missions = sorted(f.replace('.json', '') for f in os.listdir(missions_dir)
                          if f.endswith('.json'))
        return _ok(missions=missions)

    # ── Precision Drive ────────────────────────────────────────────
    @app.post('/api/precision_drive/command')
    async def api_precision_drive_cmd(req: Request):
        body = await req.json()
        ros_node._mqtt_pub('precision_drive/command', json.dumps(body), qos=1)
        return _ok(precision_drive=body.get('action', ''))

    # ── TTS ────────────────────────────────────────────────────────
    @app.post('/api/tts/toggle')
    async def api_tts_toggle(req: Request):
        body = await req.json()
        enabled = bool(body.get('enabled', True))
        with ros_node._lock:
            ros_node._tts_enabled = enabled
        ros_node._mqtt_pub('tts/enable', 'on' if enabled else 'off', qos=1)
        return _ok(tts_enabled=enabled)

    @app.post('/api/tts/speak')
    async def api_tts_speak(req: Request):
        body = await req.json()
        text = body.get('text', '').strip()
        if text:
            ros_node._mqtt_pub('tts/command', text, qos=1)
        return _ok(spoken=text)

    # ── Multi-robot (stub — full implementation needs multiple MQTT prefixes) ──
    @app.get('/api/multi_robot/list')
    async def api_multi_robot_list():
        # For now, return only the current robot
        with ros_node._lock:
            status = ros_node._robot_status
            bat = ros_node._battery
        return _ok(robots=[{
            'id': ros_node._robot_id,
            'connected': ros_node._mqtt_connected,
            'state': status.get('state', 'unknown'),
            'battery': bat.get('percent', -1),
            'lastSeen': int(time.time() * 1000),
        }])

    @app.post('/api/multi_robot/call')
    async def api_multi_robot_call(req: Request):
        body = await req.json()
        target_id = body.get('robot_id', '')
        ros_node._mqtt_pub('call_robot', target_id, qos=1)
        return _ok(called=target_id)

    # ── Static files (JS/CSS assets for React SPA) ─────────────────
    assets_dir = os.path.join(static_dir, 'assets')
    if os.path.isdir(assets_dir):
        app.mount('/assets', StaticFiles(directory=assets_dir), name='assets')
    if os.path.isdir(static_dir):
        app.mount('/static', StaticFiles(directory=static_dir), name='static')

    # Wrap with Socket.IO ASGI middleware so /socket.io/ is handled
    return socketio.ASGIApp(sio, other_asgi_app=app)


# =============================================================
# Main
# =============================================================

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)
    try:
        uvicorn.run(app, host='0.0.0.0', port=node._port, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        if node._mqtt:
            node._mqtt.loop_stop()
            node._mqtt.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
