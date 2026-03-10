#!/usr/bin/env python3
"""
fallback_nav_node — Reactive obstacle-avoidance when laptop disconnects.

Monitors /yolo/detections (published by the compute laptop) as a heartbeat.
If the laptop goes silent for LAPTOP_TIMEOUT_S seconds, switches to FALLBACK
mode and publishes simple reactive /cmd_vel commands using the ultrasonic sensor.
When the laptop reconnects (topic resumes), reverts to IDLE (yields /cmd_vel).

Fallback FSM:
  FORWARD  — range > SAFE_M:      drive slowly forward
  CAUTION  — STOP_M < range ≤ SAFE_M: slow down
  STOP     — range ≤ STOP_M:      stop, then rotate to find clear path
  ROTATE   — rotating: re-check range every 0.5s until > SAFE_M

The laptop's Nav2 / teleoperation always takes priority — this node only
publishes to /cmd_vel when the laptop is detected as offline.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String

# ── Tuneable parameters ───────────────────────────────────────────────────────
LAPTOP_TIMEOUT_S = 2.0   # seconds without /yolo/detections → assume disconnected
SAFE_M           = 0.40  # range above which we drive forward
STOP_M           = 0.20  # range below which we stop and rotate
FWD_SPEED        = 0.10  # m/s forward in FALLBACK
SLOW_SPEED       = 0.05  # m/s when in CAUTION zone
ROT_SPEED        = 0.8   # rad/s rotation speed when avoiding obstacle
CTRL_HZ          = 10    # control loop frequency (Hz)

# ── States ────────────────────────────────────────────────────────────────────
STATE_IDLE    = 'IDLE'      # laptop online — we are silent
STATE_FORWARD = 'FORWARD'   # fallback: drive forward
STATE_CAUTION = 'CAUTION'   # fallback: slow
STATE_STOP    = 'STOP'      # fallback: stopping
STATE_ROTATE  = 'ROTATE'    # fallback: rotating to find clear path


class FallbackNavNode(Node):
    def __init__(self):
        super().__init__('fallback_nav_node')

        self._state          = STATE_IDLE
        self._range          = 2.0   # last ultrasonic reading (metres)
        self._last_laptop_t  = self.get_clock().now()
        self._rotate_dir     = 1.0   # 1.0 = CCW, -1.0 = CW

        # BEST_EFFORT for sensor topics (same QoS as producers)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Heartbeat: laptop publishes /yolo/detections continuously
        self.create_subscription(
            String, '/yolo/detections', self._laptop_heartbeat_cb, 10)

        # Ultrasonic range
        self.create_subscription(
            Range, '/range', self._range_cb, sensor_qos)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop
        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        self.get_logger().info(
            'fallback_nav_node ready '
            f'(timeout={LAPTOP_TIMEOUT_S}s, safe={SAFE_M}m, stop={STOP_M}m)')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _laptop_heartbeat_cb(self, _msg):
        """Reset laptop liveness timer on every /yolo/detections message."""
        self._last_laptop_t = self.get_clock().now()
        if self._state != STATE_IDLE:
            self.get_logger().info('Laptop reconnected — yielding /cmd_vel')
            self._state = STATE_IDLE

    def _range_cb(self, msg: Range):
        """Update latest ultrasonic distance."""
        if math.isfinite(msg.range) and msg.range > 0:
            self._range = msg.range

    # ── Control loop ──────────────────────────────────────────────────────────
    def _control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_laptop_t).nanoseconds / 1e9

        # Decide operating mode
        if elapsed < LAPTOP_TIMEOUT_S:
            if self._state != STATE_IDLE:
                self._state = STATE_IDLE
                self.get_logger().info('Laptop online — fallback IDLE')
            return  # laptop is alive; do not publish

        # Laptop disconnected — take over /cmd_vel
        if self._state == STATE_IDLE:
            self.get_logger().warn(
                f'Laptop offline ({elapsed:.1f}s) — entering FALLBACK mode')
            self._state = STATE_FORWARD

        self._run_fallback_fsm()

    def _run_fallback_fsm(self):
        """Reactive FSM: forward / caution / stop+rotate."""
        r = self._range
        twist = Twist()

        if self._state == STATE_ROTATE:
            # Keep rotating until path is clear
            if r > SAFE_M:
                self.get_logger().info(
                    f'Path clear ({r:.2f}m) — resuming forward')
                self._state = STATE_FORWARD
            else:
                twist.angular.z = ROT_SPEED * self._rotate_dir
                self._cmd_pub.publish(twist)
                return

        if r > SAFE_M:
            # Clear path
            twist.linear.x = FWD_SPEED
            self._state = STATE_FORWARD

        elif r > STOP_M:
            # Approaching — slow down
            # Scale speed linearly from FWD_SPEED → 0 as range → STOP_M
            scale = (r - STOP_M) / (SAFE_M - STOP_M)
            twist.linear.x = SLOW_SPEED + scale * (FWD_SPEED - SLOW_SPEED)
            self._state = STATE_CAUTION

        else:
            # Obstacle! Stop then rotate
            if self._state != STATE_STOP and self._state != STATE_ROTATE:
                self.get_logger().warn(
                    f'Obstacle at {r:.2f}m — stopping, then rotating')
                # Alternate rotation direction each time
                self._rotate_dir *= -1.0
            self._state = STATE_ROTATE
            twist.angular.z = ROT_SPEED * self._rotate_dir

        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FallbackNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command before shutting down
        stop = Twist()
        node._cmd_pub.publish(stop)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
