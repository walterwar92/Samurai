#!/usr/bin/env python3
"""
watchdog_node — Monitor health of all robot MQTT topics.

Publishes:
    samurai/{robot_id}/watchdog  — JSON health report @ 1 Hz
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

TIMEOUT_SEC = 3.0       # topic dead after 3s (was 5s)
REPORT_INTERVAL = 0.5   # report at 2 Hz (was 1 Hz)

MONITORED_TOPICS = [
    'odom', 'camera', 'imu', 'range',
    'ball_detection', 'cmd_vel', 'battery', 'temperature',
]

# Critical topics — if these die, trigger emergency stop
CRITICAL_TOPICS = {'odom', 'imu'}


class WatchdogNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('watchdog_node', **kwargs)

        self._last_seen: dict[str, float] = {}
        self._emergency_sent = False
        grace_sec = cfg('watchdog.startup_grace_sec', 15.0)
        self._startup_grace = time.time() + grace_sec  # configurable grace period

        for suffix in MONITORED_TOPICS:
            self._last_seen[suffix] = 0.0
            self.subscribe(suffix,
                           lambda t, d, s=suffix: self._topic_cb(s),
                           parse_json=False)

        self.create_timer(REPORT_INTERVAL, self._report)
        self.log_info('Watchdog monitoring %d topics (critical: %s)',
                      len(MONITORED_TOPICS), ', '.join(CRITICAL_TOPICS))

    def _topic_cb(self, suffix: str):
        self._last_seen[suffix] = time.time()
        # Clear emergency flag when critical topics come back
        if suffix in CRITICAL_TOPICS and self._emergency_sent:
            self._emergency_sent = False
            self.log_info('Critical topic %s recovered', suffix)

    def _report(self):
        now = time.time()
        report = {}
        dead_topics = []
        critical_dead = []

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
                if suffix in CRITICAL_TOPICS:
                    critical_dead.append(suffix)

        if dead_topics:
            self.log_warn('Dead topics: %s', ', '.join(dead_topics))

        # Emergency stop if critical topics die (after grace period)
        if critical_dead and not self._emergency_sent and now > self._startup_grace:
            self.log_error('CRITICAL topics dead: %s — sending emergency stop!',
                           ', '.join(critical_dead))
            self.publish('cmd_vel',
                         {'linear_x': 0.0, 'angular_z': 0.0}, qos=1)
            self.publish('cmd_vel/manual',
                         {'linear_x': 0.0, 'angular_z': 0.0}, qos=1)
            self._emergency_sent = True

        report['mqtt'] = self._mqtt_connected
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
