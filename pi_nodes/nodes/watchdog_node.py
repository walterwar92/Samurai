#!/usr/bin/env python3
"""
watchdog_node — Monitor health of all robot MQTT topics.

Publishes:
    samurai/{robot_id}/watchdog  — JSON health report @ 1 Hz
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

TIMEOUT_SEC = 5.0

MONITORED_TOPICS = [
    'odom', 'camera', 'imu', 'range',
    'ball_detection', 'cmd_vel', 'battery', 'temperature',
]


class WatchdogNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('watchdog_node', **kwargs)

        self._last_seen: dict[str, float] = {}

        for suffix in MONITORED_TOPICS:
            self._last_seen[suffix] = 0.0
            self.subscribe(suffix,
                           lambda t, d, s=suffix: self._topic_cb(s),
                           parse_json=False)

        self.create_timer(1.0, self._report)
        self.log_info('Watchdog monitoring %d topics', len(MONITORED_TOPICS))

    def _topic_cb(self, suffix: str):
        self._last_seen[suffix] = time.time()

    def _report(self):
        now = time.time()
        report = {}
        dead_topics = []

        for suffix in MONITORED_TOPICS:
            last = self._last_seen[suffix]
            if last == 0.0:
                alive = False
                age = -1.0
            else:
                age = round(now - last, 1)
                alive = age < TIMEOUT_SEC

            report[suffix] = {'alive': alive, 'last_seen_sec': age}
            if not alive and last > 0.0:
                dead_topics.append(suffix)

        if dead_topics:
            self.log_warn('Dead topics: %s', ', '.join(dead_topics))

        self.publish('watchdog', report)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = WatchdogNode(broker=args.broker, port=args.port,
                        robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
