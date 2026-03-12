#!/usr/bin/env python3
"""
motor_node — 4-motor tracked chassis control + open-loop odometry.

Subscribes:
    samurai/{robot_id}/cmd_vel       — {linear_x, angular_z}
    samurai/{robot_id}/speed_profile — "slow" / "normal" / "fast"
Publishes:
    samurai/{robot_id}/odom                 — {x, y, theta, vx, vz, ts} @ 20 Hz
    samurai/{robot_id}/speed_profile/active — profile name @ 1 Hz
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.motor_driver import MotorDriver

WHEEL_BASE = 0.17

SPEED_PROFILES = {
    'slow':   (0.10, 0.8),
    'normal': (0.20, 1.5),
    'fast':   (0.30, 2.0),
}
DEFAULT_PROFILE = 'normal'

# Physical maximums (= 'fast' profile) — used for normalization only
_PHYS_MAX_LIN, _PHYS_MAX_ANG = SPEED_PROFILES['fast']


class MotorNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('motor_node', **kwargs)

        self._driver = MotorDriver()
        if self._driver.simulated:
            self.log_warn('MotorDriver in SIMULATION mode')
        else:
            self.log_info('MotorDriver initialised (PCA9685 @ 0x5F)')

        self._profile = DEFAULT_PROFILE
        self._max_lin, self._max_ang = SPEED_PROFILES[DEFAULT_PROFILE]

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.now_sec()
        self._linear = 0.0
        self._angular = 0.0

        self.subscribe('cmd_vel', self._cmd_vel_cb, qos=1)
        self.subscribe('speed_profile', self._profile_cb, qos=1)
        self.create_timer(0.05, self._control_loop)    # 20 Hz
        self.create_timer(1.0, self._publish_profile)   # 1 Hz

    def _cmd_vel_cb(self, topic, data):
        if isinstance(data, dict):
            self._linear = float(data.get('linear_x', 0.0))
            self._angular = float(data.get('angular_z', 0.0))
        else:
            self.log_warn('Bad cmd_vel payload: %s', data)

    def _profile_cb(self, topic, data):
        name = str(data).strip().lower()
        if name in SPEED_PROFILES:
            self._profile = name
            self._max_lin, self._max_ang = SPEED_PROFILES[name]
            self.log_info('Speed profile: %s (lin=%.2f, ang=%.2f)',
                          name, self._max_lin, self._max_ang)

    def _control_loop(self):
        now = self.now_sec()
        dt = now - self._last_time
        self._last_time = now

        # m/s → percentage for driver
        # Clamp to profile limit, then normalize by physical maximum
        lin = max(-self._max_lin, min(self._max_lin, self._linear))
        ang = max(-self._max_ang, min(self._max_ang, self._angular))
        lin_pct = lin / _PHYS_MAX_LIN * 100.0
        ang_pct = ang / _PHYS_MAX_ANG * 100.0
        self._driver.move(lin_pct, ang_pct)

        # Open-loop odometry
        self._x += self._linear * math.cos(self._theta) * dt
        self._y += self._linear * math.sin(self._theta) * dt
        self._theta += self._angular * dt

        self.publish('odom', {
            'x': round(self._x, 4),
            'y': round(self._y, 4),
            'theta': round(self._theta, 4),
            'vx': round(self._linear, 3),
            'vz': round(self._angular, 3),
            'ts': self.timestamp(),
        })

    def _publish_profile(self):
        self.publish('speed_profile/active', self._profile)

    def on_shutdown(self):
        self._driver.shutdown()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = MotorNode(broker=args.broker, port=args.port,
                     robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
