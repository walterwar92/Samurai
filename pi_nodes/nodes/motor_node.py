#!/usr/bin/env python3
"""
motor_node — 2-motor tracked chassis control + dead-reckoning odometry.

Priority command mux (higher overrides lower, resumes after timeout):
  1. MANUAL   — cmd_vel/manual (user joystick/voice direct, highest priority)
  2. COLLISION — internal collision avoidance override
  3. AUTONOMOUS — cmd_vel (FSM, path_recorder, explorer — lowest priority)

Subscribes:
    samurai/{robot_id}/cmd_vel        — {linear_x, angular_z}  (autonomous)
    samurai/{robot_id}/cmd_vel/manual — {linear_x, angular_z}  (manual override)
    samurai/{robot_id}/speed_profile  — "slow" / "normal" / "fast"
    samurai/{robot_id}/imu            — IMU data for heading (yaw)
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
    - Motors ON:  dead-reckoning from commanded velocity × calibration scale
    - Motors OFF: IMU push detector — detects physical movement via raw
                  accelerometer, estimates displacement at fixed speed
    - Heading: IMU yaw (accurate, drift-free via EKF)
    - AccelPositionEstimator runs in background for diagnostics only
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.motor_driver import MotorDriver
from pi_nodes.control.collision_policy import CollisionPolicy
from pi_nodes.control.imu_push_detector import IMUPushDetector, PushParams

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

# Physical constant — config-driven so a different chassis doesn't require a
# code change. Default matches the gusenitsa rig (Adeept HAT V3.1).
WHEEL_BASE = cfg('wheel_calibration.wheel_base', 0.17)

# ── IMU passive push detection (motors OFF) ────────────────────
# Когда моторы выключены, IMU детектирует физическое перемещение.
# Не интегрируем ускорение — используем фиксированную оценку скорости.
IMU_PUSH_SPEED       = cfg('imu_push.speed', 0.12)           # м/с
IMU_DEVIATION_THR    = cfg('imu_push.deviation_threshold', 0.20)  # м/с²
IMU_CONFIRM_SAMPLES  = cfg('imu_push.confirm_samples', 2)
IMU_MAX_MOVE_TICKS   = cfg('imu_push.max_move_ticks', 30)    # 1.5с
IMU_BIAS_ALPHA_SLOW  = cfg('imu_push.bias_alpha_slow', 0.02)
IMU_BIAS_ALPHA_FAST  = cfg('imu_push.bias_alpha_fast', 0.15)
IMU_FAST_ADAPT_TICKS = cfg('imu_push.fast_adapt_ticks', 20)

# Collision guard — stops forward motion when obstacle too close.
# Tunable via config so a smaller arena or sensor with different optimal
# trip distance doesn't require a code change.
COLLISION_GUARD_STOP_M = cfg('motor.collision_guard_stop_m', 0.20)
COLLISION_GUARD_SLOW_M = cfg('motor.collision_guard_slow_m', 0.40)

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

# Collision avoidance — instead of just stopping, attempt to steer around.
COLLISION_AVOID_ANGULAR = cfg('motor.collision_avoid_angular', 0.6)


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

        # Collision guard — extracted to CollisionPolicy class for testability.
        # avoid_dir / avoid_active live inside the policy now.
        self._collision_guard = False
        self._range_m = float('inf')
        self._range_age_s = -1.0  # from ultrasonic_node payload; -1 = no reading yet
        self._collision = CollisionPolicy(
            stop_m=COLLISION_GUARD_STOP_M,
            slow_m=COLLISION_GUARD_SLOW_M,
            avoid_angular=COLLISION_AVOID_ANGULAR,
        )

        # Priority mux: manual override
        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._last_manual_time = 0.0
        self._active_source = 'none'      # 'manual', 'collision', 'autonomous', 'none'
        self._prev_active_source = ''

        # IMU state
        self._imu_yaw_rad = None
        self._imu_calibrated = False
        self._imu_gz = 0.0       # raw gyro Z for angular velocity
        self._imu_last_ts = None  # for actual dt computation

        # ── IMU push detector — extracted to IMUPushDetector class. ─────
        # gravity_body lives here (it comes from IMU calibration MQTT msg
        # and is used to compute the body-frame linear accel passed in).
        self._imu_gravity_body = None     # [gx, gy, gz] from calibration
        self._imu_raw_lin_bx = 0.0        # gravity-free body-frame accel
        self._imu_raw_lin_by = 0.0
        self._push = IMUPushDetector(PushParams(
            speed=IMU_PUSH_SPEED,
            deviation_threshold=IMU_DEVIATION_THR,
            confirm_samples=IMU_CONFIRM_SAMPLES,
            max_move_ticks=IMU_MAX_MOVE_TICKS,
            bias_alpha_slow=IMU_BIAS_ALPHA_SLOW,
            bias_alpha_fast=IMU_BIAS_ALPHA_FAST,
            fast_adapt_ticks=IMU_FAST_ADAPT_TICKS,
        ))

        # Accelerometer position estimator — pass VelocityEKF params from config
        self._pos_estimator = None
        self._pos_estimator_ready = False
        if _ACCEL_POS_AVAILABLE:
            vekf_params = {
                'friction_tau': cfg('velocity_ekf.friction_tau', 0.15),
                'q_velocity':   cfg('velocity_ekf.q_velocity', 0.20),
                'q_bias':       cfg('velocity_ekf.q_bias', 0.003),
                'r_accel':      cfg('velocity_ekf.r_accel', 0.06),
                'r_zupt':       cfg('velocity_ekf.r_zupt', 0.0001),
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
            self._range_age_s = float(data.get('age_s', 0.0))
        else:
            try:
                self._range_m = float(data)
                self._range_age_s = 0.0  # legacy producer — assume fresh
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
                # Save gravity_body for push detector
                gb = data.get('gravity_body', [0, 0, 9.81])
                self._imu_gravity_body = list(gb)
                # Initialize position estimator with gravity calibration
                if self._pos_estimator is not None:
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

        # ── Raw gravity-free accel for push detector (body frame) ──
        if self._imu_gravity_body is not None:
            ax_raw = data.get('ax', 0.0)
            ay_raw = data.get('ay', 0.0)
            self._imu_raw_lin_bx = ax_raw - self._imu_gravity_body[0]
            self._imu_raw_lin_by = ay_raw - self._imu_gravity_body[1]

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
        self._push.reset()
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

        # ── Collision guard — delegated to CollisionPolicy ──
        if self._collision_guard:
            decision = self._collision.apply(
                linear=lin_cmd,
                angular=ang_cmd,
                range_m=self._range_m,
                range_age_s=self._range_age_s,
                manual_active=manual_active,
                now_mono=now,
                log_warn=self.log_warn,
            )
            lin_cmd = decision.linear
            ang_cmd = decision.angular
            if decision.source is not None:
                self._active_source = decision.source

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

        # ── Position (dead-reckoning: motor commands + IMU heading) ──
        # Apply calibration scale to commanded velocity for odometry
        if abs(lin) > DEADZONE_LINEAR:
            scale = self._scale_fwd if lin >= 0 else self._scale_bwd
            v_actual = lin * scale
        else:
            v_actual = 0.0

        # Integrate position from commanded velocity + IMU heading
        self._x += v_actual * dt * cy
        self._y += v_actual * dt * sy

        # Published velocity / state
        self._vx = v_actual
        self._vz = self._imu_gz if self._imu_calibrated else ang
        self._speed = abs(v_actual)
        motors_idle = (abs(v_actual) < 0.001
                       and abs(ang) < DEADZONE_ANGULAR)

        # ── IMU push detection — delegated to IMUPushDetector ──
        # Only run when gravity calibration is available (gravity_body is the
        # body-frame gravity vector saved by IMU calibration).
        if self._imu_gravity_body is not None:
            push_vx, push_vy = self._push.update(
                raw_lin_bx=self._imu_raw_lin_bx,
                raw_lin_by=self._imu_raw_lin_by,
                theta=self._theta,
                motors_idle=motors_idle,
            )
            if motors_idle:
                self._x += push_vx * dt
                self._y += push_vy * dt
        else:
            push_vx = 0.0
            push_vy = 0.0

        is_stationary = motors_idle and push_vx == 0.0 and push_vy == 0.0

        # AccelPositionEstimator — background diagnostics only
        # (provides linear_accel_world for dashboard graphs)
        if self._pos_estimator_ready:
            self._pos_estimator.update_prediction(self._theta, dt)
            self._pos_estimator.blend()
        la_x, la_y = (self._pos_estimator.linear_accel_world
                       if self._pos_estimator_ready else (0.0, 0.0))

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
