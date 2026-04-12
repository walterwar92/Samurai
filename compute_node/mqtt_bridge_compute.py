#!/usr/bin/env python3
"""
mqtt_bridge_compute — MQTT ↔ ROS2 bridge running on the laptop (Docker).

Translates Pi MQTT sensor data → ROS2 topics for SLAM/Nav2/YOLO,
and ROS2 compute results → MQTT for Pi FSM.

MQTT → ROS2:
    samurai/robot1/odom        → /odom (Odometry) + TF odom→base_link
    samurai/robot1/imu         → /imu/data (Imu)
    samurai/robot1/camera      → /camera/image_raw/compressed (CompressedImage)
    samurai/robot1/range       → /range (Range)
    samurai/robot1/battery     → /battery (Float32) + /battery_percent (Float32)
    samurai/robot1/temperature → /cpu_temperature (Float32)
    samurai/robot1/status      → /robot_status (String)
    samurai/robot1/goal_pose   → /goal_pose (PoseStamped)

ROS2 → MQTT:
    /ball_detection          → samurai/robot1/ball_detection
    /yolo/detections         → samurai/robot1/detections
    /gesture/command         → samurai/robot1/gesture/command
    /cmd_vel                 → samurai/robot1/cmd_vel
    /voice_command           → samurai/robot1/voice_command
    /claw/command            → samurai/robot1/claw/command
    /speed_profile           → samurai/robot1/speed_profile
    /patrol/command          → samurai/robot1/patrol/command
    /follow_me/command       → samurai/robot1/follow_me/command
    /path_recorder/command   → samurai/robot1/path_recorder/command

Heartbeat (1 Hz):
    samurai/robot1/heartbeat — JSON {"ts": ..., "source": "compute_bridge"}
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range, CompressedImage
from std_msgs.msg import String, Float32, Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import paho.mqtt.client as mqtt


class MqttBridgeCompute(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_compute')

        _env_broker = os.environ.get('MQTT_BROKER', '192.168.1.100')
        _env_port = int(os.environ.get('MQTT_PORT', '1883'))
        _env_robot = os.environ.get('ROBOT_ID', 'robot1')
        self.declare_parameter('mqtt_broker', _env_broker)
        self.declare_parameter('mqtt_port', _env_port)
        self.declare_parameter('robot_id', _env_robot)

        broker = self.get_parameter('mqtt_broker').value
        port = self.get_parameter('mqtt_port').value
        self._robot_id = self.get_parameter('robot_id').value
        self._prefix = f'samurai/{self._robot_id}'

        # ── MQTT client ───────────────────────────────────────
        self._mqtt = mqtt.Client(client_id=f'samurai_compute_bridge')
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=30)
        self._mqtt.connect_async(broker, port)
        self._mqtt.loop_start()

        # ── QoS profiles ─────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(depth=10)

        # ── ROS2 Publishers (MQTT → ROS2) ────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', reliable_qos)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self._cam_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self._range_pub = self.create_publisher(Range, '/range', sensor_qos)
        self._battery_pub = self.create_publisher(Float32, '/battery', 10)
        self._battery_pct_pub = self.create_publisher(
            Float32, '/battery_percent', 10)
        self._temp_pub = self.create_publisher(Float32, '/cpu_temperature', 10)
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._slam_map_pub = self.create_publisher(String, '/slam_map', 10)
        self._path_status_pub = self.create_publisher(String, '/path_recorder/status', 10)
        self._path_data_pub = self.create_publisher(String, '/path_recorder/path', 10)

        # ── Remote YOLO publishers (MQTT GPU laptop → ROS2) ──
        # These publish detection data from the remote GPU laptop
        # into ROS2 so that dashboard and other nodes can use it.
        self._remote_det_pub = self.create_publisher(String, '/ball_detection', 10)
        self._remote_dets_pub = self.create_publisher(String, '/yolo/detections', 10)
        self._remote_ann_pub = self.create_publisher(
            CompressedImage, '/yolo/annotated/compressed', sensor_qos)
        self._remote_yolo_online = False

        # ── TF Broadcasters ──────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)
        self._publish_static_transforms()

        # ── ROS2 Subscribers (ROS2 → MQTT) ───────────────────
        # Compute results → Pi
        self.create_subscription(
            String, '/ball_detection', self._ball_det_cb, 10)
        self.create_subscription(
            String, '/yolo/detections', self._yolo_dets_cb, 10)
        self.create_subscription(
            String, '/gesture/command', self._gesture_cb, 10)

        # Dashboard/Nav2 commands → Pi
        self.create_subscription(
            Twist, '/cmd_vel', self._cmdvel_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/voice_command', self._voice_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/claw/command', self._claw_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/speed_profile', self._speed_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/patrol/command', self._patrol_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/follow_me/command', self._follow_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/path_recorder/command', self._path_rec_to_mqtt_cb, 10)
        self.create_subscription(
            String, '/reset_position', self._reset_pos_to_mqtt_cb, 10)

        # ── Heartbeat timer (1 Hz) → Pi knows laptop is alive ──
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_cb)

        self.get_logger().info(
            f'MQTT bridge started: {broker}:{port} id={self._robot_id}')

    # ── Static TF (base_link → sensor frames) ───────────────
    def _publish_static_transforms(self):
        static_broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            ('base_link', 'ultrasonic_link', (0.10, 0.0, 0.05), (0, 0, 0)),
            ('base_link', 'camera_link', (0.08, 0.0, 0.12), (0, 0.26, 0)),
            ('base_link', 'imu_link', (0.0, 0.0, 0.03), (0, 0, 0)),
        ]
        static_msgs = []
        for parent, child, (x, y, z), (roll, pitch, yaw) in transforms:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            # Euler → quaternion
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy
            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            static_msgs.append(t)
        static_broadcaster.sendTransform(static_msgs)

    # ── MQTT Callbacks ───────────────────────────────────────
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'MQTT connected (rc={rc})')
        client.subscribe(f'{self._prefix}/odom')
        client.subscribe(f'{self._prefix}/imu')
        client.subscribe(f'{self._prefix}/camera', qos=0)
        client.subscribe(f'{self._prefix}/range')
        client.subscribe(f'{self._prefix}/battery')
        client.subscribe(f'{self._prefix}/temperature')
        client.subscribe(f'{self._prefix}/status')
        client.subscribe(f'{self._prefix}/goal_pose')
        client.subscribe(f'{self._prefix}/slam_map')
        client.subscribe(f'{self._prefix}/path_recorder/status')
        client.subscribe(f'{self._prefix}/path_recorder/path')
        # Remote YOLO detections (from GPU laptop via MQTT)
        client.subscribe(f'{self._prefix}/ball_detection')
        client.subscribe(f'{self._prefix}/detections')
        client.subscribe(f'{self._prefix}/yolo/annotated', qos=0)
        client.subscribe(f'{self._prefix}/yolo/status')

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        suffix = topic[len(self._prefix) + 1:]  # strip "samurai/robot1/"

        try:
            if suffix == 'odom':
                self._handle_odom(msg.payload)
            elif suffix == 'imu':
                self._handle_imu(msg.payload)
            elif suffix == 'camera':
                self._handle_camera(msg.payload)
            elif suffix == 'range':
                self._handle_range(msg.payload)
            elif suffix == 'battery':
                self._handle_battery(msg.payload)
            elif suffix == 'temperature':
                self._handle_temperature(msg.payload)
            elif suffix == 'status':
                self._handle_status(msg.payload)
            elif suffix == 'goal_pose':
                self._handle_goal_pose(msg.payload)
            elif suffix == 'slam_map':
                self._handle_slam_map(msg.payload)
            elif suffix.startswith('path_recorder/'):
                self._handle_path_recorder(suffix, msg.payload)
            # Remote YOLO detections (from GPU laptop)
            elif suffix == 'ball_detection':
                self._handle_remote_ball_detection(msg.payload)
            elif suffix == 'detections':
                self._handle_remote_detections(msg.payload)
            elif suffix == 'yolo/annotated':
                self._handle_remote_annotated(msg.payload)
            elif suffix == 'yolo/status':
                self._handle_yolo_status(msg.payload)
        except Exception as exc:
            self.get_logger().error(f'Bridge error [{suffix}]: {exc}')

    # ── MQTT → ROS2 Handlers ─────────────────────────────────
    def _handle_odom(self, payload):
        d = json.loads(payload)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # odom x,y arrive in centimetres — ROS2 uses metres
        odom.pose.pose.position.x = d['x'] / 100.0
        odom.pose.pose.position.y = d['y'] / 100.0
        theta = d['theta']
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x = d.get('vx', 0.0)
        odom.twist.twist.angular.z = d.get('vz', 0.0)

        # Pose covariance (6x6 row-major, diagonal only)
        # x, y, z, roll, pitch, yaw
        pc = odom.pose.covariance
        pc[0]  = 0.01   # x variance (m²)
        pc[7]  = 0.01   # y variance
        pc[14] = 1e6    # z — not used (2D), set very high
        pc[21] = 1e6    # roll — not used
        pc[28] = 1e6    # pitch — not used
        pc[35] = 0.005  # yaw variance (rad²) — IMU-fused, good confidence

        # Twist covariance
        tc = odom.twist.covariance
        tc[0]  = 0.01   # vx variance
        tc[7]  = 1e6    # vy — not used
        tc[14] = 1e6    # vz — not used
        tc[21] = 1e6    # vroll — not used
        tc[28] = 1e6    # vpitch — not used
        tc[35] = 0.02   # vyaw variance

        self._odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = d['x'] / 100.0
        t.transform.translation.y = d['y'] / 100.0
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _handle_imu(self, payload):
        d = json.loads(payload)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # If Pi-side filter provides orientation (EKF roll/pitch/yaw),
        # convert to quaternion and send it. Otherwise mark as unavailable.
        ekf = d.get('ekf')
        if ekf and d.get('calibrated', False):
            roll  = math.radians(ekf.get('roll', 0.0))
            pitch = math.radians(ekf.get('pitch', 0.0))
            yaw   = math.radians(ekf.get('yaw', 0.0))
            # Euler → quaternion (ZYX convention)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            # Orientation covariance (low for yaw, high for roll/pitch)
            msg.orientation_covariance[0] = 0.1    # roll variance
            msg.orientation_covariance[4] = 0.1    # pitch variance
            msg.orientation_covariance[8] = 0.005  # yaw variance — calibrated
        else:
            msg.orientation_covariance[0] = -1.0   # orientation unavailable

        msg.angular_velocity.x = d['gx']
        msg.angular_velocity.y = d['gy']
        msg.angular_velocity.z = d['gz']
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration.x = d['ax']
        msg.linear_acceleration.y = d['ay']
        msg.linear_acceleration.z = d['az']
        msg.linear_acceleration_covariance[0] = 0.5   # higher — vibrating chassis
        msg.linear_acceleration_covariance[4] = 0.5
        msg.linear_acceleration_covariance[8] = 0.5
        self._imu_pub.publish(msg)

    def _handle_camera(self, payload):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        msg.data = payload  # raw JPEG bytes
        self._cam_pub.publish(msg)

    def _handle_range(self, payload):
        d = json.loads(payload)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 2.0
        msg.range = float(d.get('range', 2.0) if isinstance(d, dict) else d)
        self._range_pub.publish(msg)

    def _handle_battery(self, payload):
        d = json.loads(payload)
        v = Float32()
        v.data = float(d.get('voltage', 0.0))
        self._battery_pub.publish(v)
        p = Float32()
        p.data = float(d.get('percent', 0.0))
        self._battery_pct_pub.publish(p)

    def _handle_temperature(self, payload):
        msg = Float32()
        msg.data = float(payload)
        self._temp_pub.publish(msg)

    def _handle_status(self, payload):
        msg = String()
        msg.data = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        self._status_pub.publish(msg)

    def _handle_goal_pose(self, payload):
        d = json.loads(payload)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = d.get('x', 0.0)
        goal.pose.position.y = d.get('y', 0.0)
        theta = d.get('theta', 0.0)
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)
        self._goal_pub.publish(goal)

    def _handle_slam_map(self, payload):
        """Forward Pi-side SLAM map as ROS2 String (JSON)."""
        msg = String()
        msg.data = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        self._slam_map_pub.publish(msg)

    def _handle_path_recorder(self, suffix, payload):
        """Forward path recorder topics as ROS2 String."""
        msg = String()
        msg.data = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        if suffix == 'path_recorder/status':
            self._path_status_pub.publish(msg)
        elif suffix == 'path_recorder/path':
            self._path_data_pub.publish(msg)

    # ── Remote YOLO Handlers (GPU laptop → ROS2) ────────────
    def _handle_remote_ball_detection(self, payload):
        """Forward GPU laptop ball detection → ROS2 /ball_detection."""
        msg = String()
        msg.data = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        self._remote_det_pub.publish(msg)

    def _handle_remote_detections(self, payload):
        """Forward GPU laptop detection summary → ROS2 /yolo/detections."""
        msg = String()
        msg.data = payload.decode('utf-8') if isinstance(payload, bytes) else str(payload)
        self._remote_dets_pub.publish(msg)

    def _handle_remote_annotated(self, payload):
        """Forward GPU laptop annotated JPEG → ROS2 /yolo/annotated/compressed."""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        msg.data = payload
        self._remote_ann_pub.publish(msg)

    def _handle_yolo_status(self, payload):
        """Track GPU YOLO detector online/offline status."""
        try:
            d = json.loads(payload)
            self._remote_yolo_online = d.get('online', False)
            status = 'ONLINE' if self._remote_yolo_online else 'OFFLINE'
            self.get_logger().info(f'Remote GPU YOLO: {status}')
        except Exception:
            pass

    # ── Heartbeat → MQTT ────────────────────────────────────
    def _heartbeat_cb(self):
        import time
        self._mqtt.publish(
            f'{self._prefix}/heartbeat',
            json.dumps({'ts': time.time(), 'source': 'compute_bridge'}))

    # ── ROS2 → MQTT Callbacks ────────────────────────────────
    def _ball_det_cb(self, msg: String):
        # Skip if remote GPU YOLO is online (it publishes directly to MQTT)
        if self._remote_yolo_online:
            return
        self._mqtt.publish(
            f'{self._prefix}/ball_detection', msg.data)

    def _yolo_dets_cb(self, msg: String):
        if self._remote_yolo_online:
            return
        self._mqtt.publish(
            f'{self._prefix}/detections', msg.data)

    def _gesture_cb(self, msg: String):
        self._mqtt.publish(
            f'{self._prefix}/gesture/command', msg.data)

    # ── Dashboard/Nav2 Commands → MQTT (Pi) ──────────────────
    def _cmdvel_to_mqtt_cb(self, msg: Twist):
        payload = json.dumps({
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
        })
        self._mqtt.publish(f'{self._prefix}/cmd_vel', payload)

    def _voice_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/voice_command', msg.data)

    def _claw_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/claw/command', msg.data)

    def _speed_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/speed_profile', msg.data)

    def _patrol_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/patrol/command', msg.data)

    def _follow_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/follow_me/command', msg.data)

    def _path_rec_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/path_recorder/command', msg.data)

    def _reset_pos_to_mqtt_cb(self, msg: String):
        self._mqtt.publish(f'{self._prefix}/reset_position', msg.data)

    # ── Cleanup ──────────────────────────────────────────────
    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeCompute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
