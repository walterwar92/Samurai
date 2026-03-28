#!/usr/bin/env python3
"""
motor_node — 4-motor tracked chassis control + accel-fused odometry.

Subscribes:
    samurai/{robot_id}/cmd_vel       — {linear_x, angular_z}
    samurai/{robot_id}/speed_profile — "slow" / "normal" / "fast"
    samurai/{robot_id}/imu           — IMU data with EKF + accel for position
Publishes:
    samurai/{robot_id}/odom                 — {x, y, theta, vx, vz, ...} @ 20 Hz
    samurai/{robot_id}/speed_profile/active — profile name @ 1 Hz

Position estimation:
    Complementary filter fusing:
    - Wheel odometry (cmd_vel integration, long-term stable)
    - Accelerometer double-integration (gravity-free, short-term accurate)
    - IMU yaw for heading
    - ZUPT for zero-velocity detection
    - Earth rotation compensation (Coriolis)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.motor_driver import MotorDriver

try:
    from pi_nodes.filters.accel_position import AccelPositionEstimator
    _ACCEL_POS_AVAILABLE = True
except ImportError:
    _ACCEL_POS_AVAILABLE = False

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

WHEEL_BASE = 0.17

# Wheel odometry scale correction — calibrate by driving a known distance
# and comparing measured vs actual. >1.0 = robot moves further than reported.
WHEEL_SCALE_LINEAR = 1.0    # tune: measure 1m, adjust if odom reads differently
WHEEL_SCALE_ANGULAR = 1.0   # tune: measure 360° turn, adjust accordingly

SPEED_PROFILES = {
    'slow':   (0.10, 0.8),
    'normal': (0.20, 1.5),
    'fast':   (0.30, 2.0),
}
DEFAULT_PROFILE = 'normal'

# Physical maximums (= 'fast' profile) — used for normalization only
_PHYS_MAX_LIN, _PHYS_MAX_ANG = SPEED_PROFILES['fast']

# Dead zone thresholds — commands below these are treated as zero
DEADZONE_LINEAR  = cfg('odometry.deadzone_linear', 0.01)   # m/s
DEADZONE_ANGULAR = cfg('odometry.deadzone_angular', 0.05)  # rad/s

# Auto-stop if no cmd_vel received within this time (seconds)
# Увеличено с 0.5 до значения из конфига (1.0 по умолчанию) — WiFi MQTT может задерживать
CMD_VEL_TIMEOUT = cfg('odometry.cmd_vel_timeout', 1.0)


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

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0      # published forward velocity
        self._vz = 0.0      # published angular velocity
        self._last_time = self.now_sec()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0

        # IMU state
        self._imu_yaw_rad = None
        self._imu_calibrated = False

        # Accelerometer position estimator
        self._pos_estimator = None
        self._pos_estimator_ready = False
        if _ACCEL_POS_AVAILABLE:
            self._pos_estimator = AccelPositionEstimator()
            self.log_info('AccelPositionEstimator loaded')
        else:
            self.log_warn('AccelPositionEstimator not available — wheel-only odom')

        self.subscribe('cmd_vel', self._cmd_vel_cb, qos=1)
        self.subscribe('speed_profile', self._profile_cb, qos=1)
        self.subscribe('imu', self._imu_cb, qos=0)
        self.subscribe('reset_position', self._reset_position_cb, qos=1)
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
        """Process IMU data: extract yaw, feed accelerometer to position estimator."""
        if not isinstance(data, dict):
            return

        # Wait for calibration
        if not self._imu_calibrated:
            if data.get('calibrated', False):
                self._imu_calibrated = True
                # Initialize position estimator with gravity calibration
                if self._pos_estimator is not None:
                    gb = data.get('gravity_body', [0, 0, 9.81])
                    ekf = data.get('ekf', {})
                    home_roll = ekf.get('roll_rad', 0.0)
                    home_pitch = ekf.get('pitch_rad', 0.0)
                    self._pos_estimator.set_calibration(
                        gravity_body=tuple(gb),
                        home_roll=home_roll,
                        home_pitch=home_pitch,
                    )
                    self._pos_estimator_ready = True
                    self.log_info(
                        'AccelPosition calibrated: g_body=(%.3f,%.3f,%.3f)',
                        gb[0], gb[1], gb[2])
                self.log_info('IMU calibrated — odometry active')
            return

        # Extract EKF orientation
        ekf = data.get('ekf')
        if not ekf:
            return

        yaw_rad = ekf.get('yaw_rad', None)
        if yaw_rad is not None:
            self._imu_yaw_rad = yaw_rad

        # Feed accelerometer to position estimator
        if self._pos_estimator_ready:
            roll_rad = ekf.get('roll_rad', 0.0)
            pitch_rad = ekf.get('pitch_rad', 0.0)
            yaw_r = ekf.get('yaw_rad', 0.0)
            ax = data.get('ax', 0.0)
            ay = data.get('ay', 0.0)
            az = data.get('az', 0.0)
            gx = data.get('gx', 0.0)
            gy = data.get('gy', 0.0)
            gz = data.get('gz', 0.0)
            # dt from IMU (50 Hz → ~0.02s)
            dt = 0.02  # fixed IMU rate
            self._pos_estimator.update_imu(
                ax, ay, az, gx, gy, gz,
                roll_rad, pitch_rad, yaw_r, dt)

    def _reset_position_cb(self, topic, data):
        """Reset odometry to (0, 0, 0) — current pose becomes new home."""
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._vz = 0.0
        if self._pos_estimator is not None:
            self._pos_estimator.reset()
        self.log_info('Position reset to (0, 0, 0) — new home set')

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

        # Apply dead zone
        lin_cmd = self._linear
        ang_cmd = self._angular
        if abs(lin_cmd) < DEADZONE_LINEAR:
            lin_cmd = 0.0
        if abs(ang_cmd) < DEADZONE_ANGULAR:
            ang_cmd = 0.0

        # Drive motors
        lin = max(-self._max_lin, min(self._max_lin, lin_cmd))
        ang = max(-self._max_ang, min(self._max_ang, ang_cmd))
        lin_pct = lin / _PHYS_MAX_LIN * 100.0
        ang_pct = ang / _PHYS_MAX_ANG * 100.0
        self._driver.move(lin_pct, ang_pct)

        # ── Heading ───────────────────────────────────────────────
        if self._imu_yaw_rad is not None:
            self._theta = self._imu_yaw_rad
        else:
            self._theta += ang_cmd * dt

        # ── Position ──────────────────────────────────────────────
        cmd_is_moving = (abs(lin_cmd) >= DEADZONE_LINEAR or
                         abs(ang_cmd) >= DEADZONE_ANGULAR)

        # Apply wheel calibration scale factors
        lin_scaled = lin_cmd * WHEEL_SCALE_LINEAR

        if self._pos_estimator_ready:
            # Hint estimator about motor command state
            self._pos_estimator.set_cmd_moving(cmd_is_moving)
            # Feed calibrated wheel odometry to estimator
            self._pos_estimator.update_wheel_odom(lin_scaled, self._theta, dt)
            # Blend accelerometer + wheel odometry
            self._pos_estimator.blend()

            is_stationary = self._pos_estimator.is_stationary

            # Position from estimator (frozen when stationary)
            self._x = self._pos_estimator.x
            self._y = self._pos_estimator.y

            # Velocity: zero when stationary (no phantom drift)
            if is_stationary:
                self._vx = 0.0
                self._vz = 0.0
            else:
                self._vx = lin_scaled
                self._vz = ang_cmd

            # Get linear acceleration for publishing
            la_x, la_y = self._pos_estimator.linear_accel_world
        else:
            # Fallback: wheel-only odometry
            is_stationary = not cmd_is_moving
            if cmd_is_moving:
                self._x += lin_scaled * math.cos(self._theta) * dt
                self._y += lin_scaled * math.sin(self._theta) * dt
            self._vx = 0.0 if is_stationary else lin_scaled
            self._vz = 0.0 if is_stationary else ang_cmd
            la_x, la_y = 0.0, 0.0

        self.publish('odom', {
            'x': round(self._x, 4),
            'y': round(self._y, 4),
            'theta': round(self._theta, 4),
            'vx': round(self._vx, 3),
            'vz': round(self._vz, 3),
            'accel_x': round(la_x, 4),
            'accel_y': round(la_y, 4),
            'stationary': is_stationary,
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
