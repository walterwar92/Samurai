#!/usr/bin/env python3
"""
temperature_node — Raspberry Pi CPU temperature monitor.

Reads /sys/class/thermal/thermal_zone0/temp and publishes as degrees Celsius.

Publishes:
    /cpu_temperature  (std_msgs/Float32)  — CPU temp in °C
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

THERMAL_PATH = '/sys/class/thermal/thermal_zone0/temp'


class TemperatureNode(Node):
    def __init__(self):
        super().__init__('temperature_node')

        self._has_sensor = os.path.exists(THERMAL_PATH)
        if not self._has_sensor:
            self.get_logger().warn(
                f'{THERMAL_PATH} not found — simulated temperature')

        self._pub = self.create_publisher(Float32, '/cpu_temperature', 10)

        # 2 Hz
        self.create_timer(0.5, self._read_temp)

    def _read_temp(self):
        if self._has_sensor:
            try:
                with open(THERMAL_PATH, 'r') as f:
                    temp_c = int(f.read().strip()) / 1000.0
            except Exception:
                temp_c = -1.0
        else:
            temp_c = 45.0  # simulated

        msg = Float32()
        msg.data = round(temp_c, 1)
        self._pub.publish(msg)

        if temp_c > 80.0:
            self.get_logger().error(f'CPU THROTTLING: {temp_c:.1f}°C')
        elif temp_c > 70.0:
            self.get_logger().warn(f'CPU hot: {temp_c:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
