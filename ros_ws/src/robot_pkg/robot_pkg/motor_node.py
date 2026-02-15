#!/usr/bin/env python3
"""
motor_node — ROS2 node for 4-motor tracked chassis.

Subscribes:
  /cmd_vel  (geometry_msgs/Twist)  — linear.x  forward m/s
                                    — angular.z  rotation rad/s

Publishes:
  /odom  (nav_msgs/Odometry)       — wheel odometry estimate (open-loop)

TF broadcast:  odom → base_link
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from robot_pkg.hardware.motor_driver import MotorDriver

# Robot geometry (metres)
WHEEL_BASE = 0.17        # distance between left and right tracks
MAX_LINEAR_SPEED = 0.3   # m/s at 100 % throttle
MAX_ANGULAR_SPEED = 2.0  # rad/s at full spin


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Hardware
        self._driver = MotorDriver()
        self.get_logger().info('MotorDriver initialised (PCA9685 @ 0x5F)')

        # State for open-loop odometry
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.get_clock().now()

        self._linear = 0.0
        self._angular = 0.0

        # ROS interfaces
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 20 Hz control + odometry loop
        self.create_timer(0.05, self._control_loop)

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self._linear = msg.linear.x
        self._angular = msg.angular.z

    # ------------------------------------------------------------------
    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # Convert m/s → percentage for driver
        lin_pct = (self._linear / MAX_LINEAR_SPEED) * 100.0
        ang_pct = (self._angular / MAX_ANGULAR_SPEED) * 100.0
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
