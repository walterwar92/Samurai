#!/usr/bin/env python3
"""
dashboard_node — Web dashboard + REST API for Samurai robot.

Runs on the compute laptop alongside YOLO/SLAM/Nav2.
Provides a real-time web interface at http://localhost:5000

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
    GET  /api/actuators               — claw + laser state
    GET  /api/battery                 — battery info
    GET  /api/speed_profile           — current speed profile
    GET  /api/camera/frame            — JPEG binary
    GET  /api/camera/frame.json       — base64 JPEG
    GET  /api/map/image               — PNG binary
    GET  /api/map/info                — map metadata
    GET  /api/map/list                — list saved maps
    GET  /api/log                     — voice/event log
    POST /api/fsm/command             — send voice command
    POST /api/actuators/claw          — open / close
    POST /api/actuators/laser         — on / off
    POST /api/speed_profile           — slow / normal / fast
    POST /api/patrol/command          — start / stop patrol
    POST /api/patrol/waypoints        — set waypoints
    POST /api/follow_me               — start / stop follow-me
    POST /api/path_recorder/command   — start / stop / play path
    GET  /api/path_recorder/list      — list saved paths
    POST /api/map/save                — save SLAM map
    POST /api/map/load                — load SLAM map

Subscribes:
    /yolo/annotated       (sensor_msgs/Image)       — annotated camera feed
    /robot_status         (std_msgs/String)          — FSM state JSON
    /ball_detection       (std_msgs/String)          — ball detection JSON
    /range                (sensor_msgs/Range)        — ultrasonic distance
    /imu/data             (sensor_msgs/Imu)          — IMU orientation + gyro + accel
    /map                  (nav_msgs/OccupancyGrid)   — SLAM map
    /odometry/filtered    (nav_msgs/Odometry)        — robot pose + velocity
    /voice_command        (std_msgs/String)          — voice commands (log)
    /scan                 (sensor_msgs/LaserScan)    — pseudo-laser scan
    /cmd_vel              (geometry_msgs/Twist)      — commanded velocity
    /battery_status       (std_msgs/String)          — battery JSON
    /temperature          (std_msgs/String)          — temperature JSON
    /watchdog_status      (std_msgs/String)          — watchdog JSON
    /speed_profile/active (std_msgs/String)          — current speed profile

Publishes:
    /voice_command          (std_msgs/String) — FSM commands via API
    /claw/command           (std_msgs/String) — claw open/close
    /laser/command          (std_msgs/String) — laser on/off
    /speed_profile          (std_msgs/String) — speed profile change
    /patrol/command         (std_msgs/String) — patrol control
    /patrol/waypoints       (std_msgs/String) — patrol waypoints JSON
    /follow_me/command      (std_msgs/String) — follow-me control
    /path_recorder/command  (std_msgs/String) — path recorder control
    /map_manager/save       (std_msgs/String) — save SLAM map
    /map_manager/load       (std_msgs/String) — load SLAM map
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Range, Imu, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
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

        # ── Shared state ────────────────────────────────────
        self._latest_frame = None
        self._robot_status = {}
        self._ball_detection = {}
        self._range_m = -1.0
        self._imu_ypr   = [0.0, 0.0, 0.0]   # yaw, pitch, roll (deg)
        self._imu_gyro  = [0.0, 0.0, 0.0]   # gx, gy, gz (rad/s)
        self._imu_accel = [0.0, 0.0, 0.0]   # ax, ay, az (m/s²)
        self._map_png = None
        self._map_info = {}
        self._robot_pose     = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._robot_velocity = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self._cmd_velocity   = {'linear_x': 0.0, 'angular_z': 0.0}
        self._scan_points = []
        self._voice_log = deque(maxlen=20)
        self._event_log = deque(maxlen=100)

        # New state
        self._battery      = {'voltage': -1.0, 'percentage': -1, 'status': 'unknown'}
        self._temperature  = {'value': -1.0, 'unit': 'C'}
        self._watchdog     = {}
        self._speed_profile = 'normal'

        # Actuator state (tracked internally — no feedback topic)
        self._claw_open = False
        self._laser_on  = False

        # ── QoS: надёжная доставка карты (transient_local) ───
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # ── QoS: сенсоры — BEST_EFFORT, только последнее ─────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ────────────────────────────────────
        self.create_subscription(CompressedImage, '/yolo/annotated/compressed', self._image_cb, 5)
        self.create_subscription(String,        '/robot_status',         self._status_cb,        10)
        self.create_subscription(String,        '/ball_detection',       self._detection_cb,     10)
        self.create_subscription(Range,         '/range',                self._range_cb,     sensor_qos)
        self.create_subscription(Imu,           '/imu/data',             self._imu_cb,       sensor_qos)
        self.create_subscription(OccupancyGrid, '/map',                  self._map_cb,     map_qos)
        self.create_subscription(Odometry,      '/odometry/filtered',    self._odom_cb,          10)
        self.create_subscription(String,        '/voice_command',        self._voice_cb,         10)
        self.create_subscription(LaserScan,     '/scan',                 self._scan_cb,          10)
        self.create_subscription(Twist,         '/cmd_vel',              self._cmdvel_cb,        10)
        self.create_subscription(String,        '/battery_status',       self._battery_cb,       10)
        self.create_subscription(String,        '/temperature',          self._temperature_cb,   10)
        self.create_subscription(String,        '/watchdog_status',      self._watchdog_cb,      10)
        self.create_subscription(String,        '/speed_profile/active', self._speed_profile_cb, 10)

        # ── Publishers ───────────────────────────────────────
        self._pub_voice      = self.create_publisher(String, '/voice_command',         10)
        self._pub_claw       = self.create_publisher(String, '/claw/command',          10)
        self._pub_laser      = self.create_publisher(String, '/laser/command',         10)
        self._pub_speed      = self.create_publisher(String, '/speed_profile',         10)
        self._pub_patrol_cmd = self.create_publisher(String, '/patrol/command',        10)
        self._pub_patrol_wp  = self.create_publisher(String, '/patrol/waypoints',      10)
        self._pub_follow     = self.create_publisher(String, '/follow_me/command',     10)
        self._pub_path_rec   = self.create_publisher(String, '/path_recorder/command', 10)
        self._pub_map_save   = self.create_publisher(String, '/map_manager/save',      10)
        self._pub_map_load   = self.create_publisher(String, '/map_manager/load',      10)

        ip = _get_local_ip()
        self.get_logger().info(f'Dashboard node started — http://{ip}:{self._port}')
        self.get_logger().info(f'REST API           — http://{ip}:{self._port}/api/status')

    # ── ROS2 Callbacks ─────────────────────────────────────────────

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
        yaw   = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        sinp  = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll  = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        with self._lock:
            self._imu_ypr   = [round(yaw, 1), round(pitch, 1), round(roll, 1)]
            self._imu_gyro  = [round(msg.angular_velocity.x, 4),
                               round(msg.angular_velocity.y, 4),
                               round(msg.angular_velocity.z, 4)]
            self._imu_accel = [round(msg.linear_acceleration.x, 4),
                               round(msg.linear_acceleration.y, 4),
                               round(msg.linear_acceleration.z, 4)]

    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 128, dtype=np.uint8)
        img[data == 0]   = [240, 240, 240]
        img[data > 50]   = [30,  30,  30]
        img = cv2.flip(img, 0)
        _, png = cv2.imencode('.png', img)
        with self._lock:
            self._map_png  = png.tobytes()
            self._map_info = {
                'width': w, 'height': h,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
            }

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._robot_pose = {'x': round(pos.x, 3), 'y': round(pos.y, 3), 'yaw': round(yaw, 3)}
            self._robot_velocity = {
                'linear_x':  round(msg.twist.twist.linear.x,  3),
                'linear_y':  round(msg.twist.twist.linear.y,  3),
                'angular_z': round(msg.twist.twist.angular.z, 3),
            }

    def _voice_cb(self, msg: String):
        entry = {'text': msg.data, 'time': time.strftime('%H:%M:%S'), 'type': 'voice'}
        with self._lock:
            self._voice_log.append(entry)
            self._event_log.append(entry)

    def _scan_cb(self, msg: LaserScan):
        points, angle = [], msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                points.append([round(r * math.cos(angle), 3), round(r * math.sin(angle), 3)])
            angle += msg.angle_increment
        with self._lock:
            self._scan_points = points

    def _cmdvel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_velocity = {
                'linear_x':  round(msg.linear.x,  3),
                'angular_z': round(msg.angular.z, 3),
            }

    def _battery_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {'raw': msg.data}
        with self._lock:
            self._battery = data

    def _temperature_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            try:
                data = {'value': float(msg.data), 'unit': 'C'}
            except ValueError:
                return
        with self._lock:
            self._temperature = data

    def _watchdog_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._watchdog = data
        except json.JSONDecodeError:
            pass

    def _speed_profile_cb(self, msg: String):
        with self._lock:
            self._speed_profile = msg.data.strip()

    # ── Publish helpers ────────────────────────────────────────────

    def _pub_str(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def send_command(self, command: str):
        self._pub_str(self._pub_voice, command)
        entry = {'text': command, 'time': time.strftime('%H:%M:%S'), 'type': 'api_command'}
        with self._lock:
            self._event_log.append(entry)

    def set_claw(self, state: str):
        self._pub_str(self._pub_claw, state)
        with self._lock:
            self._claw_open = (state == 'open')

    def set_laser(self, state: str):
        self._pub_str(self._pub_laser, state)
        with self._lock:
            self._laser_on = (state == 'on')

    def set_speed_profile(self, profile: str):
        self._pub_str(self._pub_speed, profile)

    def set_patrol_command(self, command: str):
        self._pub_str(self._pub_patrol_cmd, command)

    def set_patrol_waypoints(self, waypoints: list):
        self._pub_str(self._pub_patrol_wp, json.dumps(waypoints))

    def set_follow_me(self, command: str):
        self._pub_str(self._pub_follow, command)

    def set_path_recorder(self, payload: dict):
        self._pub_str(self._pub_path_rec, json.dumps(payload))

    def save_map(self, name: str):
        self._pub_str(self._pub_map_save, name)

    def load_map(self, name: str):
        self._pub_str(self._pub_map_load, name)

    # ── Data access (for Flask) ────────────────────────────────────

    def get_frame(self):
        with self._lock:
            return self._latest_frame

    def get_map_png(self):
        with self._lock:
            return self._map_png

    def get_state(self):
        """Full state dict for SocketIO push."""
        with self._lock:
            return {
                'status':       self._robot_status,
                'detection':    self._ball_detection,
                'range_m':      self._range_m,
                'imu': {
                    'yaw': self._imu_ypr[0], 'pitch': self._imu_ypr[1], 'roll': self._imu_ypr[2],
                    'gyro':  {'x': self._imu_gyro[0],  'y': self._imu_gyro[1],  'z': self._imu_gyro[2]},
                    'accel': {'x': self._imu_accel[0], 'y': self._imu_accel[1], 'z': self._imu_accel[2]},
                },
                'pose':         self._robot_pose,
                'velocity':     self._robot_velocity,
                'cmd_velocity': self._cmd_velocity,
                'map_info':     self._map_info,
                'scan_points':  self._scan_points,
                'voice_log':    list(self._voice_log),
                'battery':      self._battery,
                'temperature':  self._temperature,
                'watchdog':     self._watchdog,
                'speed_profile': self._speed_profile,
                'actuators': {
                    'claw':  'open'  if self._claw_open else 'closed',
                    'laser': 'on'    if self._laser_on  else 'off',
                },
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
                    'laser': 'on'   if self._laser_on  else 'off',
                },
                'battery':       dict(self._battery),
                'temperature':   dict(self._temperature),
                'watchdog':      dict(self._watchdog),
                'speed_profile': self._speed_profile,
                'map_info':      dict(self._map_info),
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


def create_app(ros_node: DashboardNode) -> FastAPI:
    base_dir   = _find_static_dir()
    static_dir = os.path.join(base_dir, 'static')

    app = FastAPI(title='Samurai Dashboard')
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_methods=['*'],
        allow_headers=['*'],
    )

    # Список активных WebSocket соединений для state_update push
    _ws_clients: list[WebSocket] = []
    _ws_lock = threading.Lock()

    # ── Background: push state to all WS clients (4 Hz) ──────────
    async def _ws_push_loop():
        while True:
            await asyncio.sleep(0.25)
            state_json = json.dumps(ros_node.get_state())
            dead = []
            with _ws_lock:
                clients = list(_ws_clients)
            for ws in clients:
                try:
                    await ws.send_text(state_json)
                except Exception:
                    dead.append(ws)
            if dead:
                with _ws_lock:
                    for ws in dead:
                        _ws_clients.remove(ws)

    @app.on_event('startup')
    async def startup():
        asyncio.create_task(_ws_push_loop())

    # ── SPA ───────────────────────────────────────────────────────
    index_html = os.path.join(static_dir, 'index.html')

    @app.get('/')
    @app.get('/dashboard')
    @app.get('/admin')
    async def serve_spa():
        return FileResponse(index_html)

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

    @app.get('/map.png')
    async def map_image_legacy():
        png = ros_node.get_map_png()
        return Response(content=png or b'\x89PNG\r\n\x1a\n', media_type='image/png')

    # ── WebSocket state push (совместим с Socket.IO клиентом) ─────
    @app.websocket('/ws/state')
    async def ws_state(ws: WebSocket):
        await ws.accept()
        with _ws_lock:
            _ws_clients.append(ws)
        try:
            while True:
                await ws.receive_text()  # держим соединение живым
        except WebSocketDisconnect:
            with _ws_lock:
                if ws in _ws_clients:
                    _ws_clients.remove(ws)

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
                    '/api/path_recorder/list', '/api/log'],
            'POST': ['/api/fsm/command', '/api/actuators/claw', '/api/actuators/laser',
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
            return _ok(claw='open' if ros_node._claw_open else 'closed',
                       laser='on'  if ros_node._laser_on  else 'off')

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
            return JSONResponse({'ok': False, 'error': 'Map not available yet'}, status_code=503)
        return Response(content=png, media_type='image/png')

    @app.get('/api/map/info')
    async def api_map_info():
        with ros_node._lock:
            info = dict(ros_node._map_info)
        if not info:
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

    @app.post('/api/fsm/command')
    async def api_fsm_command(req: Request):
        body = await req.json()
        command = body.get('command', body.get('text', '')).strip()
        if not command:
            return _err('"command" field is required')
        ros_node.send_command(command)
        return _ok(sent=command)

    @app.post('/api/actuators/claw')
    async def api_claw(req: Request):
        body  = await req.json()
        state = body.get('state', '').lower().strip()
        if state not in ('open', 'close', 'closed'):
            return _err('state must be "open" or "close"')
        cmd = 'open' if state == 'open' else 'close'
        ros_node.set_claw(cmd)
        return _ok(claw='open' if cmd == 'open' else 'closed')

    @app.post('/api/actuators/laser')
    async def api_laser(req: Request):
        body  = await req.json()
        state = body.get('state', '').lower().strip()
        if state not in ('on', 'off'):
            return _err('state must be "on" or "off"')
        ros_node.set_laser(state)
        return _ok(laser=state)

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
        if command not in ('start', 'stop', 'play'):
            return _err('command must be "start", "stop", or "play"')
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

    # ── Static files (SPA assets) ─────────────────────────────────
    if os.path.isdir(static_dir):
        app.mount('/', StaticFiles(directory=static_dir, html=True), name='static')

    return app


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
