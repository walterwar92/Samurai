#!/usr/bin/env python3
"""
laser_node — Laser pointer control via GPIO17.

Subscribes:
  /laser/command  (std_msgs/Bool)  True=ON, False=OFF

Safety: auto-off after 10 s continuous burn.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from gpiozero import LED
    _HW = True
except (ImportError, RuntimeError):
    _HW = False

LASER_PIN = 17
AUTO_OFF_SEC = 10.0


class LaserNode(Node):
    def __init__(self):
        super().__init__('laser_node')

        if _HW:
            self._laser = LED(LASER_PIN)
        else:
            self._laser = None
            self.get_logger().warn('gpiozero unavailable — laser simulated')

        self._active = False
        self._on_time = 0.0

        self.create_subscription(Bool, '/laser/command', self._cmd_cb, 10)
        self.create_timer(0.5, self._safety_timer)
        self.get_logger().info(f'Laser node ready on GPIO{LASER_PIN}')

    def _cmd_cb(self, msg: Bool):
        if msg.data:
            self._turn_on()
        else:
            self._turn_off()

    def _turn_on(self):
        if not self._active:
            self._active = True
            self._on_time = 0.0
            if self._laser:
                self._laser.on()
            self.get_logger().info('Laser ON')

    def _turn_off(self):
        if self._active:
            self._active = False
            self._on_time = 0.0
            if self._laser:
                self._laser.off()
            self.get_logger().info('Laser OFF')

    def _safety_timer(self):
        if self._active:
            self._on_time += 0.5
            if self._on_time >= AUTO_OFF_SEC:
                self.get_logger().warn('Laser auto-off (safety timeout)')
                self._turn_off()

    def destroy_node(self):
        if self._laser:
            self._laser.off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
