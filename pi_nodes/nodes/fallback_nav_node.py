#!/usr/bin/env python3
"""
fallback_nav_node — Reactive obstacle-avoidance when laptop disconnects.

Monitors laptop heartbeat (detections topic). If silent for LAPTOP_TIMEOUT_S,
switches to FALLBACK mode and publishes cmd_vel using ultrasonic sensor.

Subscribes:
    samurai/{robot_id}/detections   — laptop heartbeat (YOLO detections)
    samurai/{robot_id}/range        — ultrasonic distance
Publishes:
    samurai/{robot_id}/cmd_vel      — motor commands in fallback mode
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

LAPTOP_TIMEOUT_S = 2.0
STARTUP_GRACE_S = 10.0  # don't enter fallback until laptop seen at least once
SAFE_M = 0.40
STOP_M = 0.20
FWD_SPEED = 0.10
SLOW_SPEED = 0.05
ROT_SPEED = 0.8
CTRL_HZ = 10

STATE_IDLE = 'IDLE'
STATE_FORWARD = 'FORWARD'
STATE_CAUTION = 'CAUTION'
STATE_STOP = 'STOP'
STATE_ROTATE = 'ROTATE'


class FallbackNavNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('fallback_nav_node', **kwargs)

        self._state = STATE_IDLE
        self._range = 2.0
        self._last_laptop_t = self.now_sec()
        self._rotate_dir = 1.0
        self._laptop_seen = False  # laptop must connect at least once

        self.subscribe('detections', self._laptop_heartbeat_cb)
        self.subscribe('range', self._range_cb)
        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        self.log_info('fallback_nav ready (timeout=%.1fs, grace=%.1fs, safe=%.2fm)',
                      LAPTOP_TIMEOUT_S, STARTUP_GRACE_S, SAFE_M)

    def _laptop_heartbeat_cb(self, topic, data):
        self._last_laptop_t = self.now_sec()
        if not self._laptop_seen:
            self._laptop_seen = True
            self.log_info('Laptop first seen — fallback armed')
        if self._state != STATE_IDLE:
            self.log_info('Laptop reconnected — yielding cmd_vel')
            self._state = STATE_IDLE

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = data.get('range', 2.0)
        else:
            try:
                r = float(data)
            except (ValueError, TypeError):
                return
        if math.isfinite(r) and r > 0:
            self._range = r

    def _control_loop(self):
        elapsed = self.now_sec() - self._last_laptop_t

        if elapsed < LAPTOP_TIMEOUT_S:
            if self._state != STATE_IDLE:
                self._state = STATE_IDLE
                self.log_info('Laptop online — fallback IDLE')
            return

        # Don't enter fallback if laptop was never seen (robot just booted)
        if not self._laptop_seen:
            return

        if self._state == STATE_IDLE:
            self.log_warn('Laptop offline (%.1fs) — entering FALLBACK mode',
                          elapsed)
            self._state = STATE_FORWARD

        self._run_fallback()

    def _run_fallback(self):
        r = self._range
        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        if self._state == STATE_ROTATE:
            if r > SAFE_M:
                self.log_info('Path clear (%.2fm) — resuming forward', r)
                self._state = STATE_FORWARD
            else:
                cmd['angular_z'] = ROT_SPEED * self._rotate_dir
                self.publish('cmd_vel', cmd, qos=1)
                return

        if r > SAFE_M:
            cmd['linear_x'] = FWD_SPEED
            self._state = STATE_FORWARD
        elif r > STOP_M:
            scale = (r - STOP_M) / (SAFE_M - STOP_M)
            cmd['linear_x'] = SLOW_SPEED + scale * (FWD_SPEED - SLOW_SPEED)
            self._state = STATE_CAUTION
        else:
            if self._state != STATE_STOP and self._state != STATE_ROTATE:
                self.log_warn('Obstacle at %.2fm — rotating', r)
                self._rotate_dir *= -1.0
            self._state = STATE_ROTATE
            cmd['angular_z'] = ROT_SPEED * self._rotate_dir

        self.publish('cmd_vel', cmd, qos=1)

    def on_shutdown(self):
        self.publish('cmd_vel', {'linear_x': 0.0, 'angular_z': 0.0}, qos=1)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = FallbackNavNode(broker=args.broker, port=args.port,
                           robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
