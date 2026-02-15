#!/usr/bin/env python3
"""
servo_node — Claw servo control (PCA9685 channel 0).

Subscribes:
  /claw/command  (std_msgs/String)  "open" | "close" | "angle:<0-180>"
Publishes:
  /claw/state    (std_msgs/Float32) current angle
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from robot_pkg.hardware.servo_driver import ServoDriver


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self._servo = ServoDriver(channel=0)
        self._servo.init_position()

        self.create_subscription(String, '/claw/command', self._cmd_cb, 10)
        self._state_pub = self.create_publisher(Float32, '/claw/state', 10)
        self.create_timer(0.1, self._publish_state)  # 10 Hz
        self.get_logger().info('Claw servo node ready (channel 0)')

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            self._servo.open_claw()
            self.get_logger().info('Claw OPEN')
        elif cmd == 'close':
            self._servo.close_claw()
            self.get_logger().info('Claw CLOSE')
        elif cmd.startswith('angle:'):
            try:
                angle = float(cmd.split(':')[1])
                self._servo.set_angle(angle)
                self.get_logger().info(f'Claw angle={angle:.1f}°')
            except (ValueError, IndexError):
                self.get_logger().error(f'Invalid angle command: {cmd}')
        else:
            self.get_logger().warn(f'Unknown claw command: {cmd}')

    def _publish_state(self):
        msg = Float32()
        msg.data = self._servo.angle
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
