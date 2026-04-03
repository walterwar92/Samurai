#!/usr/bin/env python3
"""
ultrasonic_node — HC-SR04 distance sensor.

Publishes:
    samurai/{robot_id}/range  — {range, ts} @ 20 Hz
"""

import os
import sys
import time

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

# Consecutive read failures before attempting sensor re-init
MAX_FAILURES = 10


class UltrasonicNode(MqttNode):
    PUBLISH_DELTA = 0.01   # m — publish only when change exceeds this
    FORCE_INTERVAL = 1.0   # s — force publish at least this often

    def __init__(self, **kwargs):
        super().__init__('ultrasonic_node', **kwargs)

        self._sensor = None
        self._simulated = True
        self._last_published = -1.0
        self._last_publish_time = 0.0
        self._fail_count = 0
        self._last_valid = MAX_RANGE
        # Pre-allocated message dict — updated in-place
        self._msg = {'range': 0.0, 'ts': 0.0}

        self._init_sensor()

        self.create_timer(0.05, self._publish)  # 20 Hz read rate

    def _init_sensor(self):
        """Initialize or re-initialize HC-SR04 sensor."""
        if self._sensor is not None:
            try:
                self._sensor.close()
            except Exception:
                pass
            self._sensor = None

        if not _HW:
            self.log_warn('gpiozero unavailable — ultrasonic simulated')
            return

        try:
            self._sensor = DistanceSensor(
                echo=ECHO_PIN, trigger=TRIGGER_PIN,
                max_distance=MAX_RANGE)
            self._simulated = False
            self._fail_count = 0
            self.log_info('HC-SR04 ready (trig=%d echo=%d)',
                          TRIGGER_PIN, ECHO_PIN)
        except Exception as exc:
            self.log_error('HC-SR04 init failed: %s — simulated', exc)
            self._simulated = True

    def _publish(self):
        if self._simulated:
            dist = MAX_RANGE
        else:
            try:
                dist = self._sensor.distance
                # Validate range
                if dist < MIN_RANGE:
                    dist = MIN_RANGE
                elif dist > MAX_RANGE:
                    dist = MAX_RANGE
                self._last_valid = dist
                self._fail_count = 0
            except Exception as exc:
                self._fail_count += 1
                if self._fail_count <= 3 or self._fail_count % 50 == 0:
                    self.log_warn('HC-SR04 read error (%d): %s',
                                  self._fail_count, exc)
                # Use last valid reading to avoid data gap
                dist = self._last_valid

                # After too many failures, re-init sensor
                if self._fail_count >= MAX_FAILURES:
                    self.log_error('HC-SR04 %d consecutive failures — '
                                   're-initializing sensor', self._fail_count)
                    self._init_sensor()
                    return

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
