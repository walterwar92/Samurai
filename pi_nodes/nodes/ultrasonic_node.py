#!/usr/bin/env python3
"""
ultrasonic_node — HC-SR04 distance sensor.

Publishes:
    samurai/{robot_id}/range  — {range, ts} @ 20 Hz
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from gpiozero import DistanceSensor
    _HW = True
except (ImportError, RuntimeError):
    _HW = False

TRIGGER_PIN = 23
ECHO_PIN = 24
MAX_RANGE = 2.0
MIN_RANGE = 0.02


class UltrasonicNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('ultrasonic_node', **kwargs)

        self._sensor = None
        self._simulated = True

        if _HW:
            try:
                self._sensor = DistanceSensor(
                    echo=ECHO_PIN, trigger=TRIGGER_PIN,
                    max_distance=MAX_RANGE)
                self._simulated = False
                self.log_info('HC-SR04 ready (trig=%d echo=%d)',
                              TRIGGER_PIN, ECHO_PIN)
            except Exception as exc:
                self.log_error('HC-SR04 init failed: %s — simulated', exc)
        else:
            self.log_warn('gpiozero unavailable — ultrasonic simulated')

        self.create_timer(0.05, self._publish)  # 20 Hz

    def _publish(self):
        if self._simulated:
            dist = MAX_RANGE
        else:
            dist = self._sensor.distance

        self.publish('range', {
            'range': round(float(dist), 4),
            'ts': self.timestamp(),
        })

    def on_shutdown(self):
        if self._sensor:
            self._sensor.close()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = UltrasonicNode(broker=args.broker, port=args.port,
                          robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
