#!/usr/bin/env python3
"""
motor_node — ROS2 node for 4-motor tracked chassis.

Subscribes:
  /cmd_vel        (geometry_msgs/Twist)  — linear.x  forward m/s
                                          — angular.z  rotation rad/s
  /speed_profile  (std_msgs/String)      — "slow" / "normal" / "fast"

Publishes:
  /odom                  (nav_msgs/Odometry)  — wheel odometry estimate (open-loop)
  /speed_profile/active  (std_msgs/String)    — current speed profile name

TF broadcast:  odom → base_link
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robot_pkg.hardware.motor_driver import MotorDriver

# Robot geometry (metres)
WHEEL_BASE = 0.17        # distance between left and right tracks

# Speed profiles: {name: (max_linear m/s, max_angular rad/s)}
SPEED_PROFILES = {
    'slow':   (0.10, 0.8),
    'normal': (0.20, 1.5),
    'fast':   (0.30, 2.0),
}
DEFAULT_PROFILE = 'normal'


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Hardware
        self._driver = MotorDriver()
        if self._driver.simulated:
            self.get_logger().warn('MotorDriver in SIMULATION mode — no real motors')
        else:
            self.get_logger().info('MotorDriver initialised (PCA9685 @ 0x5F)')

        # Speed profile
        self._profile = DEFAULT_PROFILE
        self._max_lin, self._max_ang = SPEED_PROFILES[DEFAULT_PROFILE]

        # State for open-loop odometry
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.get_clock().now()

        self._linear = 0.0
        self._angular = 0.0

        # ROS interfaces
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(String, '/speed_profile', self._profile_cb, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._profile_pub = self.create_publisher(String, '/speed_profile/active', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 20 Hz control + odometry loop
        self.create_timer(0.05, self._control_loop)
        # 1 Hz profile broadcast
        self.create_timer(1.0, self._publish_profile)

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self._linear = msg.linear.x
        self._angular = msg.angular.z

    def _profile_cb(self, msg: String):
        name = msg.data.strip().lower()
        if name in SPEED_PROFILES:
            self._profile = name
            self._max_lin, self._max_ang = SPEED_PROFILES[name]
            self.get_logger().info(
                f'Speed profile: {name} (lin={self._max_lin}, ang={self._max_ang})')

    # ------------------------------------------------------------------
    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # Convert m/s → percentage for driver
        lin_pct = (self._linear / self._max_lin) * 100.0 if self._max_lin > 0 else 0.0
        ang_pct = (self._angular / self._max_ang) * 100.0 if self._max_ang > 0 else 0.0
        self._driver.move(lin_pct, ang_pct)

        # Open-loop odometry integration
        dx = self._linear * math.cos(self._theta) * dt
        dy = self._linear * math.sin(self._theta) * dt
        dtheta = self._angular * dt
        self._x += dx
        self._y += dy
        self._theta += dtheta

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        odom.twist.twist.linear.x = self._linear
        odom.twist.twist.angular.z = self._angular
        self._odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = math.sin(self._theta / 2.0)
        t.transform.rotation.w = math.cos(self._theta / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _publish_profile(self):
        msg = String()
        msg.data = self._profile
        self._profile_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._driver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
