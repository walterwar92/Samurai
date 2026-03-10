#!/usr/bin/env python3
"""
temperature_node — Raspberry Pi CPU temperature monitor.

Reads /sys/class/thermal/thermal_zone0/temp and publishes as degrees Celsius.

Publishes:
    samurai/{robot_id}/temperature  — CPU temp in °C (float)
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

THERMAL_PATH = '/sys/class/thermal/thermal_zone0/temp'


class TemperatureNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('temperature_node', **kwargs)

        self._has_sensor = os.path.exists(THERMAL_PATH)
        if not self._has_sensor:
            self.log_warn('%s not found — simulated temperature', THERMAL_PATH)

        self.create_timer(0.5, self._read_temp)  # 2 Hz

    def _read_temp(self):
        if self._has_sensor:
            try:
                with open(THERMAL_PATH, 'r') as f:
                    temp_c = int(f.read().strip()) / 1000.0
            except Exception:
                temp_c = -1.0
        else:
            temp_c = 45.0  # simulated

        self.publish('temperature', round(temp_c, 1))

        if temp_c > 80.0:
            self.log_error('CPU THROTTLING: %.1f°C', temp_c)
        elif temp_c > 70.0:
            self.log_warn('CPU hot: %.1f°C', temp_c)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()

    node = TemperatureNode(broker=args.broker, port=args.port,
                           robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
