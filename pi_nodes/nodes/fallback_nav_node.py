#!/usr/bin/env python3
"""
fallback_nav_node — Safety watchdog: STOP robot when laptop disconnects.

Monitors laptop heartbeat topic. If silent for LAPTOP_TIMEOUT_S,
sends STOP (cmd_vel = 0,0) to prevent uncontrolled movement.

Optional reactive obstacle-avoidance mode (--fallback-drive) re-enables
the old behaviour of driving autonomously when laptop is offline.

Subscribes:
    samurai/{robot_id}/heartbeat    — laptop heartbeat (primary)
    samurai/{robot_id}/detections   — YOLO detections (secondary heartbeat)
    samurai/{robot_id}/range        — ultrasonic distance (for fallback-drive)
Publishes:
    samurai/{robot_id}/cmd_vel      — STOP command when laptop offline
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

LAPTOP_TIMEOUT_S = 5.0   # increased from 2s — less aggressive false positives
STARTUP_GRACE_S = 15.0   # wait for laptop to connect after boot
SAFE_M = 0.40
STOP_M = 0.20
FWD_SPEED = 0.10
SLOW_SPEED = 0.05
ROT_SPEED = 0.8
CTRL_HZ = 5              # reduced from 10Hz — STOP doesn't need high rate

STATE_IDLE = 'IDLE'
STATE_STOPPED = 'STOPPED'       # new: laptop offline, robot stopped
STATE_FORWARD = 'FORWARD'
STATE_CAUTION = 'CAUTION'
STATE_ROTATE = 'ROTATE'


class FallbackNavNode(MqttNode):
    def __init__(self, fallback_drive=False, **kwargs):
        super().__init__('fallback_nav_node', **kwargs)

        self._fallback_drive = fallback_drive  # if True: old autonomous mode
        self._state = STATE_IDLE
        self._range = 2.0
        self._last_laptop_t = self.now_sec()
        self._rotate_dir = 1.0
        self._laptop_seen = False
        self._stop_sent = False  # track if we already sent STOP

        # Primary heartbeat — dedicated lightweight topic
        self.subscribe('heartbeat', self._laptop_heartbeat_cb)
        # Secondary heartbeat — YOLO detections also count as "laptop alive"
        self.subscribe('detections', self._laptop_heartbeat_cb)
        self.subscribe('range', self._range_cb)
        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        mode = 'DRIVE' if fallback_drive else 'STOP'
        self.log_info('fallback_nav ready (mode=%s, timeout=%.1fs, grace=%.1fs)',
                      mode, LAPTOP_TIMEOUT_S, STARTUP_GRACE_S)

    def _laptop_heartbeat_cb(self, topic, data):
        self._last_laptop_t = self.now_sec()
        self._stop_sent = False
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
            self.log_warn('Laptop offline (%.1fs) — %s',
                          elapsed,
                          'entering FALLBACK DRIVE mode' if self._fallback_drive
                          else 'STOPPING robot')

        if self._fallback_drive:
            # Old behaviour: autonomous obstacle avoidance
            if self._state == STATE_IDLE:
                self._state = STATE_FORWARD
            self._run_fallback_drive()
        else:
            # New default: STOP the robot safely
            self._run_fallback_stop()

    def _run_fallback_stop(self):
        """Default mode: send STOP and hold until laptop reconnects."""
        if not self._stop_sent:
            self.publish('cmd_vel', {'linear_x': 0.0, 'angular_z': 0.0}, qos=1)
            self._stop_sent = True
            self._state = STATE_STOPPED
            self.log_warn('STOP sent — robot halted, waiting for laptop')

    def _run_fallback_drive(self):
        """Optional mode: reactive obstacle-avoidance when laptop is offline."""
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
            if self._state != STATE_ROTATE:
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
    parser.add_argument('--fallback-drive', action='store_true',
                        help='Enable autonomous driving when laptop offline '
                             '(default: STOP)')
    args = parser.parse_args()
    node = FallbackNavNode(
        fallback_drive=args.fallback_drive,
        broker=args.broker, port=args.port,
        robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
