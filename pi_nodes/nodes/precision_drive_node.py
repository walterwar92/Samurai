#!/usr/bin/env python3
"""
precision_drive_node — Smart motion correction for precise distance driving.

Drives the robot exactly N cm in a given direction with closed-loop control.
Detects disturbances (push, lift) via IMU and corrects the path.

Subscribes:
    samurai/{robot_id}/odom                  — position feedback
    samurai/{robot_id}/imu                   — disturbance detection
    samurai/{robot_id}/precision_drive/command — movement commands

Publishes:
    samurai/{robot_id}/cmd_vel               — motor commands
    samurai/{robot_id}/precision_drive/status — progress & state
    samurai/{robot_id}/precision_drive/result — completion result

Commands (JSON to precision_drive/command):
    Single leg:
        {"action": "drive", "distance_cm": 50, "direction": "forward"}
        {"action": "drive", "distance_cm": 30, "direction": "backward"}
        {"action": "turn", "angle_deg": 90}    # positive = CCW (left)

    Scenarios (multi-leg sequences):
        {"action": "scenario", "name": "cross",  "distance_cm": 50}
        {"action": "scenario", "name": "square", "distance_cm": 40}
        {"action": "scenario", "name": "line",   "distance_cm": 100}
        {"action": "scenario", "name": "zigzag", "distance_cm": 50}

    Control:
        {"action": "stop"}                      # abort current task
        {"action": "pause"}                     # pause (resume with "resume")
        {"action": "resume"}

Disturbance handling:
    - Push: lateral accel spike -> pause, re-align heading, continue
    - Lift: gravity loss (|a| << g) -> full stop, wait for ground, resume
"""

import math
import os
import sys
import time
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d


# ── Tuning constants (from config.yaml or defaults) ──────────────────

# Driving
DRIVE_SPEED       = cfg('precision_drive.drive_speed', 0.12)       # m/s
DRIVE_SPEED_SLOW  = cfg('precision_drive.drive_speed_slow', 0.06)  # m/s near target
DRIVE_SPEED_CRAWL = cfg('precision_drive.drive_speed_crawl', 0.04) # m/s final approach
SLOW_DIST_M       = cfg('precision_drive.slow_distance_m', 0.08)   # start slowing at 8cm
CRAWL_DIST_M      = cfg('precision_drive.crawl_distance_m', 0.02)  # crawl at 2cm
ARRIVE_TOL_M      = cfg('precision_drive.arrive_tolerance_m', 0.005) # 5mm arrival
OVERSHOOT_TOL_M   = cfg('precision_drive.overshoot_tolerance_m', 0.015)

# Turning
TURN_SPEED        = cfg('precision_drive.turn_speed', 0.5)         # rad/s
TURN_SPEED_SLOW   = cfg('precision_drive.turn_speed_slow', 0.15)   # rad/s near target
TURN_SLOW_RAD     = cfg('precision_drive.turn_slow_rad', 0.15)     # slow zone
TURN_TOL_RAD      = cfg('precision_drive.turn_tolerance_rad', 0.02) # ~1.1 deg

# Heading correction during straight drive (PI-controller)
HEADING_KP        = cfg('precision_drive.heading_kp', 2.5)
HEADING_KI        = cfg('precision_drive.heading_ki', 0.3)
HEADING_MAX_CORR  = cfg('precision_drive.heading_max_correction', 0.4) # rad/s

# Lateral drift correction
LATERAL_KP        = cfg('precision_drive.lateral_kp', 2.0)
LATERAL_MAX_CORR  = cfg('precision_drive.lateral_max_correction', 0.3) # rad/s

# Disturbance detection
PUSH_ACCEL_THRESHOLD = cfg('precision_drive.push_accel_threshold', 3.0)   # m/s^2
PUSH_COOLDOWN_S      = cfg('precision_drive.push_cooldown_s', 1.0)
LIFT_GRAVITY_LOW     = cfg('precision_drive.lift_gravity_low', 6.0)       # m/s^2
LIFT_GRAVITY_OK      = cfg('precision_drive.lift_gravity_ok', 8.5)        # m/s^2
LIFT_SAMPLES         = cfg('precision_drive.lift_samples', 5)
SETTLE_TIME_S        = cfg('precision_drive.settle_time_s', 0.3)          # reduced from 0.5

# Safety
TIMEOUT_PER_CM_S  = cfg('precision_drive.timeout_per_cm_s', 0.5)
MAX_LEG_TIMEOUT_S = cfg('precision_drive.max_leg_timeout_s', 30.0)

# Scenario transition — shorter pause between legs
SCENARIO_SETTLE_S = cfg('precision_drive.scenario_settle_s', 0.15)

# Odom smoothing — average last N readings for stable position
ODOM_SMOOTH_N = 3

CONTROL_HZ = 20


def _normalize_angle(a):
    """Normalize angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class PrecisionDriveNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('precision_drive_node', **kwargs)

        # ── Odometry state (from motor_node) ─────────────────────
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_stationary = False
        self._odom_ready = False
        self._odom_lock = threading.Lock()

        # Odom smoothing ring buffer
        self._odom_buf_x = []
        self._odom_buf_y = []

        # ── IMU raw state (for disturbance detection) ────────────
        self._imu_ax = 0.0
        self._imu_ay = 0.0
        self._imu_az = 9.81
        self._imu_calibrated = False
        self._imu_lock = threading.Lock()

        # ── State machine ────────────────────────────────────────
        self._state = 'idle'
        self._prev_state = 'idle'

        # ── Current leg target ───────────────────────────────────
        self._start_x = 0.0
        self._start_y = 0.0
        self._target_heading = 0.0
        self._target_distance = 0.0
        self._target_yaw = 0.0
        self._turn_direction = 1.0
        self._leg_start_time = 0.0

        # ── PI controller state (heading) ────────────────────────
        self._heading_integral = 0.0
        self._heading_integral_max = 0.3  # anti-windup

        # ── Position confirmation — require stable arrival ───────
        self._arrive_count = 0
        self._ARRIVE_CONFIRM = 3  # consecutive ticks within tolerance

        # ── Scenario queue ───────────────────────────────────────
        self._scenario_legs = []
        self._scenario_name = ''
        self._scenario_idx = 0

        # ── Disturbance tracking ─────────────────────────────────
        self._push_last_time = 0.0
        self._lift_low_count = 0
        self._settle_start = 0.0
        self._settle_duration = SETTLE_TIME_S

        # ── Pre-allocated messages ───────────────────────────────
        self._cmd_msg = {'linear_x': 0.0, 'angular_z': 0.0}
        self._status_msg = {
            'state': 'idle',
            'scenario': '',
            'leg': 0,
            'total_legs': 0,
            'distance_done_cm': 0.0,
            'distance_target_cm': 0.0,
            'heading_error_deg': 0.0,
            'lateral_error_cm': 0.0,
            'disturbance': 'none',
        }

        # ── Subscriptions ────────────────────────────────────────
        self.subscribe('odom', self._odom_cb, qos=0)
        self.subscribe('imu', self._imu_cb, qos=0)
        self.subscribe('precision_drive/command', self._command_cb, qos=1)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.create_timer(0.25, self._publish_status)

        self.log_info('PrecisionDriveNode ready (speed=%.2f m/s, tol=%.0f mm, '
                      'heading PI kp=%.1f ki=%.1f)',
                      DRIVE_SPEED, ARRIVE_TOL_M * 1000,
                      HEADING_KP, HEADING_KI)

    # ── Callbacks ────────────────────────────────────────────────

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        with self._odom_lock:
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)

            # Smooth odom with ring buffer
            self._odom_buf_x.append(x)
            self._odom_buf_y.append(y)
            if len(self._odom_buf_x) > ODOM_SMOOTH_N:
                self._odom_buf_x.pop(0)
                self._odom_buf_y.pop(0)

            self._odom_x = sum(self._odom_buf_x) / len(self._odom_buf_x)
            self._odom_y = sum(self._odom_buf_y) / len(self._odom_buf_y)
            self._odom_theta = data.get('theta', 0.0)
            self._odom_stationary = data.get('stationary', False)
            self._odom_ready = True

    def _imu_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        if not data.get('calibrated', False):
            return
        self._imu_calibrated = True
        with self._imu_lock:
            self._imu_ax = data.get('ax', 0.0)
            self._imu_ay = data.get('ay', 0.0)
            self._imu_az = data.get('az', 9.81)

    def _command_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        action = data.get('action', '')

        if action == 'stop':
            self._abort('stop command')
            return
        if action == 'pause':
            if self._state not in ('idle', 'done', 'paused_user'):
                self._prev_state = self._state
                self._state = 'paused_user'
                self._send_cmd(0.0, 0.0)
                self.log_info('Paused by user')
            return
        if action == 'resume':
            if self._state == 'paused_user':
                self._state = self._prev_state
                self.log_info('Resumed by user -> %s', self._state)
            return

        if self._state != 'idle':
            self.log_warn('Busy (state=%s), ignoring command', self._state)
            return

        if action == 'drive':
            dist_cm = float(data.get('distance_cm', 0))
            direction = data.get('direction', 'forward')
            speed = float(data.get('speed', DRIVE_SPEED))
            if dist_cm <= 0:
                self.log_warn('Invalid distance: %.1f cm', dist_cm)
                return
            self._start_single_drive(dist_cm, direction, speed)

        elif action == 'turn':
            angle_deg = float(data.get('angle_deg', 0))
            if abs(angle_deg) < 1:
                return
            self._start_single_turn(angle_deg)

        elif action == 'scenario':
            name = data.get('name', '')
            dist_cm = float(data.get('distance_cm', 50))
            self._start_scenario(name, dist_cm)

        else:
            self.log_warn('Unknown action: %s', action)

    # ── Scenario builders ────────────────────────────────────────

    def _build_scenario(self, name, dist_cm):
        """Build a list of legs for a scenario."""
        d = dist_cm
        if name == 'cross':
            return [
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'forward %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': -90,                  'label': 'turn right 90'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'right %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': 90,                   'label': 'turn left 90'},
                {'type': 'turn',  'deg': 90,                   'label': 'turn left 90'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'left %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': -90,                  'label': 'back to forward'},
                {'type': 'turn',  'deg': 180,                  'label': 'turn 180'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'backward %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'return to start'},
                {'type': 'turn',  'deg': 180,                  'label': 'face original'},
            ]
        elif name == 'square':
            legs = []
            for i in range(4):
                legs.append({'type': 'drive', 'cm': d, 'dir': 'forward',
                             'label': 'side %d (%dcm)' % (i+1, d)})
                legs.append({'type': 'turn', 'deg': -90,
                             'label': 'corner %d' % (i+1)})
            return legs
        elif name == 'line':
            return [
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'forward %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'return %dcm' % d},
            ]
        elif name == 'zigzag':
            legs = []
            for i in range(4):
                legs.append({'type': 'drive', 'cm': d, 'dir': 'forward',
                             'label': 'leg %d' % (i+1)})
                angle = 60 if (i % 2 == 0) else -60
                if i < 3:
                    legs.append({'type': 'turn', 'deg': angle,
                                 'label': 'zigzag turn'})
            legs.append({'type': 'turn', 'deg': 180, 'label': 'turn back'})
            legs.append({'type': 'drive', 'cm': d, 'dir': 'forward',
                         'label': 'return'})
            return legs
        else:
            return []

    def _start_scenario(self, name, dist_cm):
        legs = self._build_scenario(name, dist_cm)
        if not legs:
            self.log_warn('Unknown scenario: %s', name)
            self._publish_result(False, 'unknown scenario: %s' % name)
            return
        self._scenario_legs = legs
        self._scenario_name = name
        self._scenario_idx = 0
        self.log_info('Scenario "%s" started (%d legs, %.0f cm)', name, len(legs), dist_cm)
        self._start_next_leg()

    def _start_next_leg(self):
        if self._scenario_idx >= len(self._scenario_legs):
            self.log_info('Scenario "%s" completed!', self._scenario_name)
            self._publish_result(True, 'scenario %s done' % self._scenario_name)
            self._state = 'idle'
            self._scenario_legs = []
            return

        leg = self._scenario_legs[self._scenario_idx]
        self.log_info('Leg %d/%d: %s',
                      self._scenario_idx + 1, len(self._scenario_legs),
                      leg.get('label', ''))

        if leg['type'] == 'drive':
            self._start_single_drive(leg['cm'], leg['dir'], DRIVE_SPEED)
        elif leg['type'] == 'turn':
            self._start_single_turn(leg['deg'])

    # ── Single movement starters ─────────────────────────────────

    def _start_single_drive(self, dist_cm, direction, speed):
        with self._odom_lock:
            self._start_x = self._odom_x
            self._start_y = self._odom_y
            theta = self._odom_theta

        dist_m = dist_cm / 100.0

        if direction == 'forward':
            self._target_heading = theta
            self._target_distance = dist_m
        elif direction == 'backward':
            self._target_heading = theta
            self._target_distance = -dist_m
        elif direction == 'left':
            self._target_heading = _normalize_angle(theta + math.pi / 2)
            self._target_distance = dist_m
        elif direction == 'right':
            self._target_heading = _normalize_angle(theta - math.pi / 2)
            self._target_distance = dist_m
        else:
            self.log_warn('Unknown direction: %s', direction)
            return

        # Reset PI controller
        self._heading_integral = 0.0
        self._arrive_count = 0

        self._leg_start_time = time.monotonic()
        # If direction requires turning first, align first
        heading_err = abs(_normalize_angle(self._target_heading - theta))
        if heading_err > TURN_TOL_RAD and direction in ('left', 'right'):
            self._target_yaw = self._target_heading
            self._turn_direction = 1.0 if _normalize_angle(self._target_heading - theta) > 0 else -1.0
            self._state = 'aligning'
        else:
            self._state = 'driving'

        self.log_info('Drive: %.1f cm %s (heading=%.1f deg)',
                      dist_cm, direction, math.degrees(self._target_heading))

    def _start_single_turn(self, angle_deg):
        with self._odom_lock:
            theta = self._odom_theta
        angle_rad = math.radians(angle_deg)
        self._target_yaw = _normalize_angle(theta + angle_rad)
        self._turn_direction = 1.0 if angle_rad > 0 else -1.0
        self._arrive_count = 0
        self._leg_start_time = time.monotonic()
        self._state = 'turning'
        self.log_info('Turn: %.1f deg (target=%.1f deg)',
                      angle_deg, math.degrees(self._target_yaw))

    # ── Main control loop (20 Hz) ────────────────────────────────

    def _control_loop(self):
        if not self._odom_ready or not self._imu_calibrated:
            return

        # Check for disturbances (in any active state)
        if self._state in ('driving', 'aligning', 'turning'):
            disturbance = self._check_disturbances()
            if disturbance:
                return

        state = self._state

        if state == 'idle' or state == 'done':
            return
        elif state in ('paused_user', 'paused_push'):
            self._send_cmd(0.0, 0.0)
            return
        elif state == 'paused_lift':
            self._send_cmd(0.0, 0.0)
            self._handle_lift_recovery()
            return
        elif state == 'settling':
            self._send_cmd(0.0, 0.0)
            if time.monotonic() - self._settle_start >= self._settle_duration:
                if self._prev_state == '_next_leg':
                    self._state = 'idle'  # temp
                    self._start_next_leg()
                else:
                    self.log_info('Settled, resuming %s', self._prev_state)
                    self._state = self._prev_state
            return
        elif state == 'aligning':
            self._do_align()
        elif state == 'driving':
            self._do_drive()
        elif state == 'turning':
            self._do_turn()

    # ── Driving logic ────────────────────────────────────────────

    def _do_drive(self):
        with self._odom_lock:
            x = self._odom_x
            y = self._odom_y
            theta = self._odom_theta

        # Distance travelled along the target heading direction
        dx = x - self._start_x
        dy = y - self._start_y
        cos_h = math.cos(self._target_heading)
        sin_h = math.sin(self._target_heading)

        # Project displacement onto target heading (signed distance along path)
        dist_along = dx * cos_h + dy * sin_h
        # Lateral error: perpendicular distance from the ideal line
        dist_lateral = -dx * sin_h + dy * cos_h

        target_dist = self._target_distance
        is_reverse = target_dist < 0
        target_abs = abs(target_dist)
        dist_signed = dist_along if not is_reverse else -dist_along
        remaining = target_abs - dist_signed

        # Timeout check
        timeout = min(target_abs * 100 * TIMEOUT_PER_CM_S, MAX_LEG_TIMEOUT_S)
        if time.monotonic() - self._leg_start_time > timeout:
            self.log_warn('Leg timeout (%.1f s)', timeout)
            self._finish_leg(False, 'timeout')
            return

        # Arrival check — require multiple consecutive ticks within tolerance
        if remaining <= ARRIVE_TOL_M:
            self._arrive_count += 1
            if self._arrive_count >= self._ARRIVE_CONFIRM:
                self.log_info('Arrived! (error=%.1f mm, %d confirms)',
                              remaining * 1000, self._arrive_count)
                self._finish_leg(True, 'arrived (%.1f mm)' % (remaining * 1000))
                return
            # Keep sending crawl speed to maintain position lock
            self._send_cmd(0.0, 0.0)
            return
        else:
            self._arrive_count = 0

        # Overshoot check
        if remaining < -OVERSHOOT_TOL_M:
            self.log_warn('Overshoot by %.1f mm', -remaining * 1000)
            self._finish_leg(True, 'overshoot %.1f mm' % (-remaining * 1000))
            return

        # Speed profile: 3-zone ramp (full -> slow -> crawl)
        if remaining < CRAWL_DIST_M:
            speed = DRIVE_SPEED_CRAWL
        elif remaining < SLOW_DIST_M:
            # Linear ramp from crawl to full speed
            t = (remaining - CRAWL_DIST_M) / (SLOW_DIST_M - CRAWL_DIST_M)
            speed = DRIVE_SPEED_CRAWL + (DRIVE_SPEED - DRIVE_SPEED_CRAWL) * t
        else:
            speed = DRIVE_SPEED

        if is_reverse:
            speed = -speed

        # Heading correction (PI-controller)
        heading_err = _normalize_angle(self._target_heading - theta)
        dt = 1.0 / CONTROL_HZ

        # Proportional term
        ang_p = HEADING_KP * heading_err

        # Integral term (with anti-windup)
        self._heading_integral += heading_err * dt
        self._heading_integral = max(-self._heading_integral_max,
                                     min(self._heading_integral_max,
                                         self._heading_integral))
        ang_i = HEADING_KI * self._heading_integral

        ang_correction = ang_p + ang_i
        ang_correction = max(-HEADING_MAX_CORR, min(HEADING_MAX_CORR, ang_correction))

        # Lateral drift correction — steer back onto the original line
        lat_correction = -LATERAL_KP * dist_lateral
        lat_correction = max(-LATERAL_MAX_CORR, min(LATERAL_MAX_CORR, lat_correction))
        if is_reverse:
            lat_correction = -lat_correction

        angular = ang_correction + lat_correction
        max_ang = HEADING_MAX_CORR + LATERAL_MAX_CORR
        angular = max(-max_ang, min(max_ang, angular))

        self._send_cmd(speed, angular)

    def _do_align(self):
        """Turn to target heading before driving."""
        with self._odom_lock:
            theta = self._odom_theta

        err = _normalize_angle(self._target_yaw - theta)
        if abs(err) <= TURN_TOL_RAD:
            self._send_cmd(0.0, 0.0)
            with self._odom_lock:
                self._start_x = self._odom_x
                self._start_y = self._odom_y
            self._heading_integral = 0.0
            self._state = 'driving'
            self.log_info('Aligned (err=%.1f deg), driving', math.degrees(err))
            return

        # Proportional speed with minimum
        if abs(err) > TURN_SLOW_RAD:
            speed = TURN_SPEED
        else:
            speed = TURN_SPEED_SLOW + (TURN_SPEED - TURN_SPEED_SLOW) * (abs(err) / TURN_SLOW_RAD)
        angular = speed * (1.0 if err > 0 else -1.0)
        self._send_cmd(0.0, angular)

    def _do_turn(self):
        """Turn in place to target yaw."""
        with self._odom_lock:
            theta = self._odom_theta

        err = _normalize_angle(self._target_yaw - theta)

        # Confirmed arrival
        if abs(err) <= TURN_TOL_RAD:
            self._arrive_count += 1
            if self._arrive_count >= self._ARRIVE_CONFIRM:
                self._send_cmd(0.0, 0.0)
                self.log_info('Turn done (err=%.1f deg)', math.degrees(err))
                self._finish_leg(True, 'turn done (%.1f deg)' % math.degrees(err))
                return
            self._send_cmd(0.0, 0.0)
            return
        else:
            self._arrive_count = 0

        # Timeout
        timeout = min(abs(math.degrees(err)) * 0.1 + 5, MAX_LEG_TIMEOUT_S)
        if time.monotonic() - self._leg_start_time > timeout:
            self._finish_leg(False, 'turn timeout')
            return

        # Proportional speed with minimum
        if abs(err) > TURN_SLOW_RAD:
            speed = TURN_SPEED
        else:
            speed = TURN_SPEED_SLOW + (TURN_SPEED - TURN_SPEED_SLOW) * (abs(err) / TURN_SLOW_RAD)
        angular = speed * (1.0 if err > 0 else -1.0)
        self._send_cmd(0.0, angular)

    # ── Disturbance detection ────────────────────────────────────

    def _check_disturbances(self):
        """Check for push or lift. Returns True if disturbance detected."""
        now = time.monotonic()
        with self._imu_lock:
            ax = self._imu_ax
            ay = self._imu_ay
            az = self._imu_az

        a_mag = math.sqrt(ax * ax + ay * ay + az * az)

        # ── Lift detection: gravity disappears ───────────────────
        if a_mag < LIFT_GRAVITY_LOW:
            self._lift_low_count += 1
            if self._lift_low_count >= LIFT_SAMPLES:
                self._prev_state = self._state
                self._state = 'paused_lift'
                self._send_cmd(0.0, 0.0)
                self.log_warn('LIFT detected (|a|=%.2f m/s^2) — motors stopped', a_mag)
                return True
        else:
            self._lift_low_count = 0

        # ── Push detection: lateral accel spike ──────────────────
        if now - self._push_last_time < PUSH_COOLDOWN_S:
            return False

        with self._odom_lock:
            theta = self._odom_theta

        cos_h = math.cos(theta)
        sin_h = math.sin(theta)
        a_lateral = abs(-ax * sin_h + ay * cos_h)

        if a_lateral > PUSH_ACCEL_THRESHOLD:
            self._push_last_time = now
            self._prev_state = self._state
            self._state = 'settling'
            self._settle_start = now
            self._settle_duration = SETTLE_TIME_S
            self._send_cmd(0.0, 0.0)
            self.log_warn('PUSH detected (a_lat=%.2f m/s^2) — pausing to settle', a_lateral)
            return True

        return False

    def _handle_lift_recovery(self):
        """Wait until robot is back on the ground."""
        with self._imu_lock:
            ax = self._imu_ax
            ay = self._imu_ay
            az = self._imu_az

        a_mag = math.sqrt(ax * ax + ay * ay + az * az)

        if a_mag >= LIFT_GRAVITY_OK:
            self._lift_low_count = 0
            self._state = 'settling'
            self._settle_start = time.monotonic()
            self._settle_duration = SETTLE_TIME_S
            self.log_info('Back on ground (|a|=%.2f m/s^2) — settling...', a_mag)

    # ── Leg completion ───────────────────────────────────────────

    def _finish_leg(self, success, reason):
        self._send_cmd(0.0, 0.0)
        self._heading_integral = 0.0
        self._arrive_count = 0

        if self._scenario_legs:
            self._scenario_idx += 1
            if success:
                # Short pause between scenario legs
                self._state = 'settling'
                self._settle_start = time.monotonic()
                self._settle_duration = SCENARIO_SETTLE_S
                self._prev_state = '_next_leg'
            else:
                self.log_warn('Scenario "%s" leg %d failed: %s',
                              self._scenario_name, self._scenario_idx, reason)
                self._publish_result(False, 'leg %d: %s' % (self._scenario_idx, reason))
                self._state = 'idle'
                self._scenario_legs = []
        else:
            self._publish_result(success, reason)
            self._state = 'idle'

    def _abort(self, reason):
        self._send_cmd(0.0, 0.0)
        self._state = 'idle'
        self._scenario_legs = []
        self._heading_integral = 0.0
        self._arrive_count = 0
        self.log_info('Aborted: %s', reason)
        self._publish_result(False, 'aborted: %s' % reason)

    # ── Publishing helpers ───────────────────────────────────────

    def _send_cmd(self, linear, angular):
        m = self._cmd_msg
        m['linear_x'] = round(linear, 4)
        m['angular_z'] = round(angular, 4)
        self.publish('cmd_vel', m)

    def _publish_status(self):
        with self._odom_lock:
            x = self._odom_x
            y = self._odom_y
            theta = self._odom_theta

        s = self._status_msg
        s['state'] = self._state
        s['scenario'] = self._scenario_name
        s['leg'] = self._scenario_idx + 1 if self._scenario_legs else 0
        s['total_legs'] = len(self._scenario_legs)

        if self._state in ('driving', 'settling', 'paused_push', 'paused_lift', 'paused_user'):
            dx = x - self._start_x
            dy = y - self._start_y
            cos_h = math.cos(self._target_heading)
            sin_h = math.sin(self._target_heading)
            dist_along = dx * cos_h + dy * sin_h
            dist_lateral = -dx * sin_h + dy * cos_h

            target_abs = abs(self._target_distance)
            s['distance_done_cm'] = round(abs(dist_along) * 100, 1)
            s['distance_target_cm'] = round(target_abs * 100, 1)
            s['heading_error_deg'] = round(
                math.degrees(_normalize_angle(self._target_heading - theta)), 1)
            s['lateral_error_cm'] = round(dist_lateral * 100, 1)
        else:
            s['distance_done_cm'] = 0.0
            s['distance_target_cm'] = 0.0
            s['heading_error_deg'] = 0.0
            s['lateral_error_cm'] = 0.0

        if self._state == 'paused_lift':
            s['disturbance'] = 'lift'
        elif self._state == 'settling' and self._prev_state in ('driving', 'turning', 'aligning'):
            s['disturbance'] = 'push'
        else:
            s['disturbance'] = 'none'

        self.publish('precision_drive/status', s)

    def _publish_result(self, success, detail):
        self.publish('precision_drive/result', {
            'success': success,
            'detail': detail,
            'scenario': self._scenario_name,
            'ts': self.timestamp(),
        })


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = PrecisionDriveNode(broker=args.broker, port=args.port,
                              robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
