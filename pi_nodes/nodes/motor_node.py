#!/usr/bin/env python3
"""
motor_node — 4-motor tracked chassis control + accel-fused odometry.

Priority command mux (higher overrides lower, resumes after timeout):
  1. MANUAL   — cmd_vel/manual (user joystick/voice direct, highest priority)
  2. COLLISION — internal collision avoidance override
  3. AUTONOMOUS — cmd_vel (FSM, path_recorder, explorer — lowest priority)

Subscribes:
    samurai/{robot_id}/cmd_vel        — {linear_x, angular_z}  (autonomous)
    samurai/{robot_id}/cmd_vel/manual — {linear_x, angular_z}  (manual override)
    samurai/{robot_id}/speed_profile  — "slow" / "normal" / "fast"
    samurai/{robot_id}/imu            — IMU data with EKF + accel for position
Publishes:
    samurai/{robot_id}/odom                 — {x(cm), y(cm), theta, vx, vz, ...} @ 20 Hz
    samurai/{robot_id}/speed_profile/active — profile name @ 1 Hz
    samurai/{robot_id}/cmd_vel/active_source — "manual"/"collision"/"autonomous"/"none"
    samurai/{robot_id}/calibration/active   — {profile, scale_fwd, scale_bwd, motor_trim}

Calibration MQTT interface:
    samurai/{robot_id}/calibration/set           — {scale_fwd, scale_bwd, motor_trim}
    samurai/{robot_id}/calibration/profile/load  — "name" or {name: "..."}
    samurai/{robot_id}/calibration/profile/save  — {name: "...", description: "..."}
    samurai/{robot_id}/calibration/profile/delete — "name"
    samurai/{robot_id}/calibration/profile/list  — any (triggers response)

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
    from pi_nodes.calibration_profiles import (
        get_active, get_profile, save_profile, set_active,
        list_profiles, delete_profile,
    )
    _PROFILES_AVAILABLE = True
except ImportError:
    _PROFILES_AVAILABLE = False

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

WHEEL_BASE = 0.17

# Collision guard — stops forward motion when obstacle too close
COLLISION_GUARD_STOP_M  = 0.20   # full stop distance
COLLISION_GUARD_SLOW_M  = 0.40   # start slowing down

# Wheel odometry scale correction — defaults (overridden by active profile).
_DEFAULT_SCALE_FWD  = cfg('wheel_calibration.scale_linear_fwd', 1.235)
_DEFAULT_SCALE_BWD  = cfg('wheel_calibration.scale_linear_bwd', 0.988)
_DEFAULT_MOTOR_TRIM = cfg('wheel_calibration.motor_trim_pct', -12.003)
WHEEL_SCALE_ANGULAR = cfg('wheel_calibration.scale_angular', 1.0)

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
# 0.5s — быстрая остановка при потере связи (безопаснее чем 1.0)
CMD_VEL_TIMEOUT = cfg('odometry.cmd_vel_timeout', 0.5)

# Priority mux — manual override timeout (seconds).
# After this time without manual commands, autonomous control resumes.
MANUAL_OVERRIDE_TIMEOUT = cfg('motor.manual_override_timeout', 0.5)

# Collision avoidance — instead of just stopping, attempt to steer around
COLLISION_AVOID_ANGULAR = 0.6   # rad/s — turn speed when avoiding obstacle


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

        # ── Calibration coefficients (mutable, updated via MQTT) ─────
        self._scale_fwd = _DEFAULT_SCALE_FWD
        self._scale_bwd = _DEFAULT_SCALE_BWD
        self._motor_trim = _DEFAULT_MOTOR_TRIM   # % of max_angular
        self._cal_profile_name = 'default'
        self._load_active_profile()

        # Odometry state (internal: metres; published: centimetres)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0      # published forward velocity
        self._vz = 0.0      # published angular velocity
        self._speed = 0.0   # velocity magnitude (scalar)
        self._last_time = self.now_sec()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = 0.0

        # Collision guard
        self._collision_guard = False
        self._range_m = float('inf')

        # Priority mux: manual override
        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._last_manual_time = 0.0
        self._active_source = 'none'      # 'manual', 'collision', 'autonomous', 'none'
        self._prev_active_source = ''

        # Collision avoidance — steering direction for circumnavigation
        self._collision_avoid_dir = 1.0   # +1 = left, -1 = right
        self._collision_avoid_active = False

        # IMU state
        self._imu_yaw_rad = None
        self._imu_calibrated = False
        self._imu_gz = 0.0       # raw gyro Z for angular velocity
        self._imu_last_ts = None  # for actual dt computation

        # Accelerometer position estimator — pass VelocityEKF params from config
        self._pos_estimator = None
        self._pos_estimator_ready = False
        if _ACCEL_POS_AVAILABLE:
            vekf_params = {
                'motor_tau':   cfg('velocity_ekf.motor_tau', 0.25),
                'braking_tau': cfg('velocity_ekf.braking_tau', 0.08),
                'q_velocity':  cfg('velocity_ekf.q_velocity', 0.20),
                'q_bias':      cfg('velocity_ekf.q_bias', 0.003),
                'r_accel':     cfg('velocity_ekf.r_accel', 0.06),
                'r_zupt':      cfg('velocity_ekf.r_zupt', 0.0001),
            }
            self._pos_estimator = AccelPositionEstimator(
                vekf_params=vekf_params,
            )
            self.log_info('AccelPositionEstimator loaded (q_vel=%.3f, r_acc=%.3f)',
                          vekf_params['q_velocity'], vekf_params['r_accel'])
        else:
            self.log_warn('AccelPositionEstimator not available — wheel-only odom')

        self.subscribe('cmd_vel', self._cmd_vel_cb, qos=1)
        self.subscribe('cmd_vel/manual', self._cmd_vel_manual_cb, qos=1)
        self.subscribe('speed_profile', self._profile_cb, qos=1)
        self.subscribe('imu', self._imu_cb, qos=0)
        self.subscribe('reset_position', self._reset_position_cb, qos=1)
        self.subscribe('range', self._range_cb, qos=0)
        self.subscribe('collision_guard/enable', self._collision_guard_cb, qos=1)

        # ── Calibration MQTT interface ───────────────────────────────
        self.subscribe('calibration/set', self._cal_set_cb, qos=1)
        self.subscribe('calibration/profile/load', self._cal_profile_load_cb, qos=1)
        self.subscribe('calibration/profile/save', self._cal_profile_save_cb, qos=1)
        self.subscribe('calibration/profile/delete', self._cal_profile_delete_cb, qos=1)
        self.subscribe('calibration/profile/list', self._cal_profile_list_cb, qos=1)

        # Pre-allocate odom message template (avoid dict creation every 50ms)
        # x, y — centimetres; vx — m/s; speed — m/s
        self._odom_msg = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'vz': 0.0,
            'speed': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'stationary': False, 'ts': 0.0,
        }
        self.create_timer(0.05, self._control_loop)    # 20 Hz
        self.create_timer(1.0, self._publish_profile)   # 1 Hz

    # ── Calibration profile helpers ──────────────────────────────

    def _load_active_profile(self):
        """Load the active calibration profile from calibration_profiles.yaml."""
        if not _PROFILES_AVAILABLE:
            return
        name, coeffs = get_active()
        if coeffs:
            self._scale_fwd = coeffs.get('scale_fwd', self._scale_fwd)
            self._scale_bwd = coeffs.get('scale_bwd', self._scale_bwd)
            self._motor_trim = coeffs.get('motor_trim', self._motor_trim)
            self._cal_profile_name = name
            self.log_info('Calibration profile "%s": fwd=%.4f bwd=%.4f trim=%.3f%%',
                          name, self._scale_fwd, self._scale_bwd, self._motor_trim)
        else:
            self.log_info('No active profile — using config.yaml defaults')

    def _publish_calibration(self):
        """Publish current calibration state (retained)."""
        self.publish('calibration/active', {
            'profile': self._cal_profile_name,
            'scale_fwd': self._scale_fwd,
            'scale_bwd': self._scale_bwd,
            'motor_trim': self._motor_trim,
        }, retain=True)

    # ── Calibration MQTT callbacks ───────────────────────────────

    def _cal_set_cb(self, topic, data):
        """Set calibration coefficients directly: {scale_fwd, scale_bwd, motor_trim}."""
        if not isinstance(data, dict):
            return
        changed = False
        if 'scale_fwd' in data:
            self._scale_fwd = float(data['scale_fwd'])
            changed = True
        if 'scale_bwd' in data:
            self._scale_bwd = float(data['scale_bwd'])
            changed = True
        if 'motor_trim' in data:
            self._motor_trim = float(data['motor_trim'])
            changed = True
        if changed:
            self._cal_profile_name = 'custom'
            self.log_info('Calibration set: fwd=%.4f bwd=%.4f trim=%.3f%%',
                          self._scale_fwd, self._scale_bwd, self._motor_trim)
            self._publish_calibration()

    def _cal_profile_load_cb(self, topic, data):
        """Load a named profile: "profile_name" or {name: "profile_name"}."""
        if not _PROFILES_AVAILABLE:
            self.log_warn('calibration_profiles module not available')
            return
        name = data.get('name', data) if isinstance(data, dict) else str(data).strip()
        coeffs = get_profile(name)
        if coeffs is None:
            self.log_warn('Profile "%s" not found', name)
            self.publish('calibration/profile/error',
                         {'error': 'not_found', 'name': name})
            return
        self._scale_fwd = coeffs['scale_fwd']
        self._scale_bwd = coeffs['scale_bwd']
        self._motor_trim = coeffs['motor_trim']
        self._cal_profile_name = name
        set_active(name)
        self.log_info('Profile "%s" loaded: fwd=%.4f bwd=%.4f trim=%.3f%%',
                      name, self._scale_fwd, self._scale_bwd, self._motor_trim)
        self._publish_calibration()

    def _cal_profile_save_cb(self, topic, data):
        """Save current coefficients as profile: {name, description?}."""
        if not _PROFILES_AVAILABLE:
            return
        if not isinstance(data, dict):
            data = {'name': str(data).strip()}
        name = data.get('name', '').strip()
        if not name:
            return
        desc = data.get('description', '')
        save_profile(name, self._scale_fwd, self._scale_bwd,
                     self._motor_trim, desc)
        set_active(name)
        self._cal_profile_name = name
        self.log_info('Profile "%s" saved (fwd=%.4f bwd=%.4f trim=%.3f%%)',
                      name, self._scale_fwd, self._scale_bwd, self._motor_trim)
        self._publish_calibration()
        self.publish('calibration/profile/saved', {'name': name})

    def _cal_profile_delete_cb(self, topic, data):
        """Delete a profile: "name" or {name: "name"}."""
        if not _PROFILES_AVAILABLE:
            return
        name = data.get('name', data) if isinstance(data, dict) else str(data).strip()
        if delete_profile(name):
            self.log_info('Profile "%s" deleted', name)
            if self._cal_profile_name == name:
                self._load_active_profile()
                self._publish_calibration()
        else:
            self.publish('calibration/profile/error',
                         {'error': 'not_found', 'name': name})

    def _cal_profile_list_cb(self, topic, data):
        """Publish list of all profiles."""
        if not _PROFILES_AVAILABLE:
            self.publish('calibration/profile/all', {'profiles': {}, 'active': ''})
            return
        profiles = list_profiles()
        self.publish('calibration/profile/all', {
            'profiles': profiles,
            'active': self._cal_profile_name,
        })

    def _cmd_vel_cb(self, topic, data):
        """Autonomous cmd_vel (from FSM, path_recorder, explorer)."""
        if isinstance(data, dict):
            self._linear = float(data.get('linear_x', 0.0))
            self._angular = float(data.get('angular_z', 0.0))
            self._last_cmd_time = self.now_sec()
        else:
            self.log_warn('Bad cmd_vel payload: %s', data)

    def _cmd_vel_manual_cb(self, topic, data):
        """Manual override cmd_vel (from joystick, dashboard, voice direct commands)."""
        if isinstance(data, dict):
            self._manual_linear = float(data.get('linear_x', 0.0))
            self._manual_angular = float(data.get('angular_z', 0.0))
            self._last_manual_time = self.now_sec()
        else:
            self.log_warn('Bad cmd_vel/manual payload: %s', data)

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            self._range_m = float(data.get('range', float('inf')))
        else:
            try:
                self._range_m = float(data)
            except (TypeError, ValueError):
                pass

    def _collision_guard_cb(self, topic, data):
        val = str(data).strip().lower()
        enabled = val in ('on', 'true', '1')
        if enabled != self._collision_guard:
            self._collision_guard = enabled
            self.log_info('Collision guard: %s', 'ON' if enabled else 'OFF')
            self.publish('collision_guard/state',
                         'on' if enabled else 'off', retain=True)

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

        # Store gyro Z for angular velocity reporting
        gz = data.get('gz', 0.0)
        self._imu_gz = gz

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

            # Actual dt from IMU timestamps (instead of hardcoded 0.02)
            imu_ts = data.get('ts', 0.0)
            dt = 0.02  # fallback
            if imu_ts > 0 and self._imu_last_ts is not None:
                actual_dt = imu_ts - self._imu_last_ts
                if 0.001 < actual_dt < 0.2:  # sanity: 1ms to 200ms
                    dt = actual_dt
            if imu_ts > 0:
                self._imu_last_ts = imu_ts

            self._pos_estimator.update_imu(
                ax, ay, az, gx, gy, gz,
                roll_rad, pitch_rad, yaw_r, dt, imu_ts)

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

        # ── Priority mux: select command source ──────────────────
        manual_active = (self._last_manual_time > 0 and
                         (now - self._last_manual_time) <= MANUAL_OVERRIDE_TIMEOUT)

        # Auto-stop on autonomous cmd_vel timeout
        if self._last_cmd_time > 0 and (now - self._last_cmd_time) > CMD_VEL_TIMEOUT:
            self._linear = 0.0
            self._angular = 0.0

        # Select source: manual (highest) > autonomous (lowest)
        if manual_active:
            lin_cmd = self._manual_linear
            ang_cmd = self._manual_angular
            self._active_source = 'manual'
        else:
            lin_cmd = self._linear
            ang_cmd = self._angular
            self._active_source = 'autonomous' if (abs(lin_cmd) > 0 or abs(ang_cmd) > 0) else 'none'

        # Apply dead zone
        if abs(lin_cmd) < DEADZONE_LINEAR:
            lin_cmd = 0.0
        if abs(ang_cmd) < DEADZONE_ANGULAR:
            ang_cmd = 0.0

        # ── Collision guard (highest priority — overrides everything) ──
        if self._collision_guard and lin_cmd > 0:
            r = self._range_m
            if r < COLLISION_GUARD_STOP_M:
                # Full stop + active steering to go around obstacle
                lin_cmd = 0.0
                if not manual_active:
                    # Only auto-steer when not in manual mode
                    ang_cmd = COLLISION_AVOID_ANGULAR * self._collision_avoid_dir
                    self._active_source = 'collision'
                    if not self._collision_avoid_active:
                        self._collision_avoid_active = True
                        self.log_info('Collision avoidance: steering %s',
                                      'left' if self._collision_avoid_dir > 0 else 'right')
            elif r < COLLISION_GUARD_SLOW_M:
                factor = (r - COLLISION_GUARD_STOP_M) / (COLLISION_GUARD_SLOW_M - COLLISION_GUARD_STOP_M)
                lin_cmd *= max(0.0, factor)
                if not manual_active:
                    self._active_source = 'collision'
            else:
                if self._collision_avoid_active:
                    self._collision_avoid_active = False
                    # Alternate direction for next obstacle
                    self._collision_avoid_dir *= -1.0
                    self.log_info('Collision avoidance: clear')

        # Drive motors — apply motor_trim for straight-line correction
        lin = max(-self._max_lin, min(self._max_lin, lin_cmd))
        ang = max(-self._max_ang, min(self._max_ang, ang_cmd))
        lin_pct = lin / _PHYS_MAX_LIN * 100.0
        ang_pct = ang / _PHYS_MAX_ANG * 100.0

        # Motor trim: compensate motor asymmetry (from test_precision_drive.py).
        # Forward → +trim, backward → −trim (same physical fix, mirrored).
        if abs(lin_pct) > 1.0:
            trim = self._motor_trim if lin_pct >= 0 else -self._motor_trim
            ang_pct += trim

        self._driver.move(lin_pct, ang_pct)

        # ── Heading ───────────────────────────────────────────────
        if self._imu_yaw_rad is not None:
            self._theta = self._imu_yaw_rad
        else:
            self._theta += ang_cmd * dt

        # Cache trig (used multiple times below)
        cy = math.cos(self._theta)
        sy = math.sin(self._theta)

        # ── Position ──────────────────────────────────────────────
        cmd_is_moving = (abs(lin_cmd) >= DEADZONE_LINEAR or
                         abs(ang_cmd) >= DEADZONE_ANGULAR)

        # Apply direction-dependent wheel scale (fwd/bwd differ by ~20%)
        lin_scaled = lin_cmd * (self._scale_fwd if lin_cmd >= 0.0 else self._scale_bwd)

        if self._pos_estimator_ready:
            self._pos_estimator.set_cmd_moving(cmd_is_moving)
            self._pos_estimator.update_wheel_odom(lin_scaled, self._theta, dt)
            self._pos_estimator.blend()

            is_stationary = self._pos_estimator.is_stationary
            self._x = self._pos_estimator.x
            self._y = self._pos_estimator.y

            if is_stationary:
                self._vx = 0.0
                self._vz = 0.0
                self._speed = 0.0
            else:
                fused_vx = self._pos_estimator.vx
                fused_vy = self._pos_estimator.vy
                self._vx = fused_vx * cy + fused_vy * sy
                self._vz = self._imu_gz
                self._speed = math.sqrt(fused_vx * fused_vx + fused_vy * fused_vy)

            la_x, la_y = self._pos_estimator.linear_accel_world
        else:
            is_stationary = not cmd_is_moving
            if cmd_is_moving:
                new_vx = lin_scaled
                avg_vx = 0.5 * (self._vx + new_vx) if abs(self._vx) > 0 else new_vx
                self._x += avg_vx * cy * dt
                self._y += avg_vx * sy * dt
            self._vx = 0.0 if is_stationary else lin_scaled
            self._speed = abs(self._vx)
            if self._imu_yaw_rad is not None:
                self._vz = 0.0 if is_stationary else self._imu_gz
            else:
                self._vz = 0.0 if is_stationary else ang_cmd
            la_x, la_y = 0.0, 0.0

        # Update pre-allocated odom dict (avoid allocation every 50ms)
        # x, y — centimetres for precision; vx, speed — m/s
        m = self._odom_msg
        m['x'] = round(self._x * 100.0, 2)    # cm
        m['y'] = round(self._y * 100.0, 2)    # cm
        m['theta'] = round(self._theta, 4)
        m['vx'] = round(self._vx, 3)
        m['vz'] = round(self._vz, 3)
        m['speed'] = round(self._speed, 3)
        m['accel_x'] = round(la_x, 4)
        m['accel_y'] = round(la_y, 4)
        m['stationary'] = is_stationary
        m['ts'] = self.timestamp()
        if self._pos_estimator_ready:
            m['wheel_scale'] = round(self._pos_estimator.wheel_scale, 3)

        self.publish('odom', m)

    def _publish_profile(self):
        self.publish('speed_profile/active', self._profile)
        # Publish active source for UI feedback
        if self._active_source != self._prev_active_source:
            self.publish('cmd_vel/active_source', self._active_source, retain=True)
            self._prev_active_source = self._active_source
        # Publish calibration state periodically
        self._publish_calibration()

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
