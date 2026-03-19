#!/usr/bin/env python3
"""
motor_node — 4-motor tracked chassis control + IMU-fused odometry.

Subscribes:
    samurai/{robot_id}/cmd_vel       — {linear_x, angular_z}
    samurai/{robot_id}/speed_profile — "slow" / "normal" / "fast"
    samurai/{robot_id}/imu           — IMU data with EKF yaw for theta fusion
Publishes:
    samurai/{robot_id}/odom                 — {x, y, theta, vx, vz, ts} @ 20 Hz
    samurai/{robot_id}/speed_profile/active — profile name @ 1 Hz

Odometry improvements:
    - Dead zone: velocities below threshold treated as zero (anti-vibration)
    - IMU yaw fusion: theta from EKF instead of open-loop integration
    - cmd_vel timeout: auto-stop if no command received within 500ms
    - Waits for IMU calibration before publishing odometry
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

# Dead zone thresholds — commands below these are treated as zero
# Prevents odometry drift from vibrations and floating-point noise
DEADZONE_LINEAR  = 0.01   # m/s
DEADZONE_ANGULAR = 0.05   # rad/s

# Auto-stop if no cmd_vel received within this time (seconds)
CMD_VEL_TIMEOUT = 0.5


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

        # Odometry state — starts at (0,0,0), reset on IMU calibration
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.now_sec()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0  # timestamp of last cmd_vel

        # IMU fusion state
        self._imu_yaw_rad = None        # latest EKF yaw (radians)
        self._imu_calibrated = False     # wait for IMU calibration

        self.subscribe('cmd_vel', self._cmd_vel_cb, qos=1)
        self.subscribe('speed_profile', self._profile_cb, qos=1)
        self.subscribe('imu', self._imu_cb, qos=0)
        self.create_timer(0.05, self._control_loop)    # 20 Hz
        self.create_timer(1.0, self._publish_profile)   # 1 Hz

    def _cmd_vel_cb(self, topic, data):
        if isinstance(data, dict):
            self._linear = float(data.get('linear_x', 0.0))
            self._angular = float(data.get('angular_z', 0.0))
            self._last_cmd_time = self.now_sec()
        else:
            self.log_warn('Bad cmd_vel payload: %s', data)

    def _imu_cb(self, topic, data):
        """Receive IMU data, extract EKF yaw for odometry theta fusion."""
        if not isinstance(data, dict):
            return
        # Track calibration status
        if not self._imu_calibrated:
            if data.get('calibrated', False):
                self._imu_calibrated = True
                self.log_info('IMU calibrated — odometry active')
            return
        # Extract EKF yaw (degrees → radians)
        ekf = data.get('ekf')
        if ekf and 'yaw' in ekf:
            self._imu_yaw_rad = math.radians(ekf['yaw'])

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

        # Auto-stop on cmd_vel timeout
        if self._last_cmd_time > 0 and (now - self._last_cmd_time) > CMD_VEL_TIMEOUT:
            self._linear = 0.0
            self._angular = 0.0

        # Apply dead zone — treat tiny values as zero
        lin_cmd = self._linear
        ang_cmd = self._angular
        if abs(lin_cmd) < DEADZONE_LINEAR:
            lin_cmd = 0.0
        if abs(ang_cmd) < DEADZONE_ANGULAR:
            ang_cmd = 0.0

        # m/s → percentage for driver
        lin = max(-self._max_lin, min(self._max_lin, lin_cmd))
        ang = max(-self._max_ang, min(self._max_ang, ang_cmd))
        lin_pct = lin / _PHYS_MAX_LIN * 100.0
        ang_pct = ang / _PHYS_MAX_ANG * 100.0
        self._driver.move(lin_pct, ang_pct)

        # ── Odometry ──────────────────────────────────────────────────
        # Use IMU yaw for theta (much more accurate than open-loop)
        if self._imu_yaw_rad is not None:
            self._theta = self._imu_yaw_rad
        else:
            # Fallback: open-loop integration (only before IMU ready)
            self._theta += ang_cmd * dt

        # Only integrate position when actually moving (above dead zone)
        if abs(lin_cmd) >= DEADZONE_LINEAR:
            self._x += lin_cmd * math.cos(self._theta) * dt
            self._y += lin_cmd * math.sin(self._theta) * dt

        self.publish('odom', {
            'x': round(self._x, 4),
            'y': round(self._y, 4),
            'theta': round(self._theta, 4),
            'vx': round(lin_cmd, 3),
            'vz': round(ang_cmd, 3),
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
