#!/usr/bin/env python3
"""
ultrasonic_node — HC-SR04 distance sensor.

Hardware:  trigger=GPIO23, echo=GPIO24
Publishes: /range  (sensor_msgs/Range)  @ 20 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    from gpiozero import DistanceSensor
    _HW = True
except (ImportError, RuntimeError):
    _HW = False

TRIGGER_PIN = 23
ECHO_PIN = 24
MAX_RANGE = 2.0      # metres
MIN_RANGE = 0.02     # metres
FIELD_OF_VIEW = 0.26  # ~15 deg half-cone


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        if _HW:
            try:
                self._sensor = DistanceSensor(
                    echo=ECHO_PIN, trigger=TRIGGER_PIN, max_distance=MAX_RANGE,
                )
                self._simulated = False
                self.get_logger().info(
                    'HC-SR04 ready (trig=%d echo=%d)', TRIGGER_PIN, ECHO_PIN)
            except Exception as exc:
                self.get_logger().error('HC-SR04 init failed: %s — simulated', exc)
                self._sensor = None
                self._simulated = True
        else:
            self._sensor = None
            self._simulated = True
            self.get_logger().warn('gpiozero unavailable — ultrasonic simulated')

        self._pub = self.create_publisher(Range, '/range', 10)
        self.create_timer(0.05, self._publish)  # 20 Hz

    def _publish(self):
        if self._simulated:
            dist = MAX_RANGE
        else:
            dist = self._sensor.distance  # metres

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = FIELD_OF_VIEW
        msg.min_range = MIN_RANGE
        msg.max_range = MAX_RANGE
        msg.range = float(dist)
        self._pub.publish(msg)

    def destroy_node(self):
        if self._sensor:
            self._sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
