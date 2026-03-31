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
    PUBLISH_DELTA = 0.01   # m — publish only when change exceeds this
    FORCE_INTERVAL = 1.0   # s — force publish at least this often

    def __init__(self, **kwargs):
        super().__init__('ultrasonic_node', **kwargs)

        self._sensor = None
        self._simulated = True
        self._last_published = -1.0
        self._last_publish_time = 0.0
        # Pre-allocated message dict — updated in-place
        self._msg = {'range': 0.0, 'ts': 0.0}

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

        self.create_timer(0.05, self._publish)  # 20 Hz read rate

    def _publish(self):
        if self._simulated:
            dist = MAX_RANGE
        else:
            dist = self._sensor.distance

        # Publish only on significant change or after force interval
        now = self.timestamp()
        if (abs(dist - self._last_published) < self.PUBLISH_DELTA and
                now - self._last_publish_time < self.FORCE_INTERVAL):
            return

        self._last_published = dist
        self._last_publish_time = now
        m = self._msg
        m['range'] = round(float(dist), 4)
        m['ts'] = now
        self.publish('range', m)

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
