#!/usr/bin/env python3
"""
mpc_drive_node — Model Predictive Control for precise distance driving.

Drop-in replacement for precision_drive_node.  Uses a linearized
state-space model + receding-horizon QP instead of PI control.

Subscribes:
    samurai/{robot_id}/odom                  — position feedback
    samurai/{robot_id}/imu                   — disturbance detection
    samurai/{robot_id}/precision_drive/command — movement commands (same topic!)

Publishes:
    samurai/{robot_id}/cmd_vel               — motor commands
    samurai/{robot_id}/precision_drive/status — progress & state
    samurai/{robot_id}/precision_drive/result — completion result

Commands — identical to precision_drive_node:
    {"action": "drive",    "distance_cm": 50, "direction": "forward"}
    {"action": "turn",     "angle_deg": 90}
    {"action": "scenario", "name": "square", "distance_cm": 40}
    {"action": "stop"} / {"action": "pause"} / {"action": "resume"}
"""

import math
import os
import sys
import time
import threading

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.filters.mpc_controller import MPCController

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d


# ── Tuning from config.yaml (shared keys with precision_drive) ──────

# MPC-specific
MPC_NP          = cfg('mpc_drive.Np', 20)
MPC_NC          = cfg('mpc_drive.Nc', 8)
MPC_MOTOR_TAU   = cfg('mpc_drive.motor_tau', 0.25)
MPC_Q_X         = cfg('mpc_drive.q_x', 1.0)
MPC_Q_Y         = cfg('mpc_drive.q_y', 5.0)
MPC_Q_THETA     = cfg('mpc_drive.q_theta', 3.0)
MPC_R_V         = cfg('mpc_drive.r_v', 0.1)
MPC_R_OMEGA     = cfg('mpc_drive.r_omega', 0.1)

# Driving (reuse precision_drive keys where sensible)
DRIVE_SPEED       = cfg('precision_drive.drive_speed', 0.12)
ARRIVE_TOL_M      = cfg('precision_drive.arrive_tolerance_m', 0.005)
OVERSHOOT_TOL_M   = cfg('precision_drive.overshoot_tolerance_m', 0.015)
V_MAX             = cfg('mpc_drive.v_max', 0.15)
V_MIN             = cfg('mpc_drive.v_min', -0.15)
OMEGA_MAX         = cfg('mpc_drive.omega_max', 1.5)
OMEGA_MIN         = cfg('mpc_drive.omega_min', -1.5)

# Turning (proportional — MPC not suitable at v₀≈0)
TURN_SPEED      = cfg('precision_drive.turn_speed', 0.5)
TURN_SPEED_SLOW = cfg('precision_drive.turn_speed_slow', 0.15)
TURN_SLOW_RAD   = cfg('precision_drive.turn_slow_rad', 0.15)
TURN_TOL_RAD    = cfg('precision_drive.turn_tolerance_rad', 0.02)

# Disturbance
PUSH_ACCEL_THRESHOLD = cfg('precision_drive.push_accel_threshold', 3.0)
PUSH_COOLDOWN_S      = cfg('precision_drive.push_cooldown_s', 1.0)
LIFT_GRAVITY_LOW     = cfg('precision_drive.lift_gravity_low', 6.0)
LIFT_GRAVITY_OK      = cfg('precision_drive.lift_gravity_ok', 8.5)
LIFT_SAMPLES         = cfg('precision_drive.lift_samples', 5)
SETTLE_TIME_S        = cfg('precision_drive.settle_time_s', 0.3)

# Safety
TIMEOUT_PER_CM_S  = cfg('precision_drive.timeout_per_cm_s', 0.5)
MAX_LEG_TIMEOUT_S = cfg('precision_drive.max_leg_timeout_s', 30.0)
SCENARIO_SETTLE_S = cfg('precision_drive.scenario_settle_s', 0.15)

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ

# Odom smoothing
ODOM_SMOOTH_N = 3


def _normalize_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MPCDriveNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('mpc_drive_node', **kwargs)

        # ── MPC controller ───────────────────────────────────────
        self._mpc = MPCController(
            dt=DT, Np=MPC_NP, Nc=MPC_NC, motor_tau=MPC_MOTOR_TAU,
            q_x=MPC_Q_X, q_y=MPC_Q_Y, q_theta=MPC_Q_THETA,
            r_v=MPC_R_V, r_omega=MPC_R_OMEGA,
            v_max=V_MAX, v_min=V_MIN,
            omega_max=OMEGA_MAX, omega_min=OMEGA_MIN,
        )

        # ── Odometry state ───────────────────────────────────────
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_vx = 0.0
        self._odom_vz = 0.0
        self._odom_stationary = False
        self._odom_ready = False
        self._odom_lock = threading.Lock()
        self._odom_buf_x = []
        self._odom_buf_y = []

        # ── IMU (disturbance detection) ──────────────────────────
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
        self._target_distance = 0.0   # signed: >0 forward, <0 backward
        self._target_yaw = 0.0
        self._turn_direction = 1.0
        self._leg_start_time = 0.0

        # ── Arrival confirmation ─────────────────────────────────
        self._arrive_count = 0
        self._ARRIVE_CONFIRM = 3

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
            'state': 'idle', 'scenario': '', 'leg': 0, 'total_legs': 0,
            'distance_done_cm': 0.0, 'distance_target_cm': 0.0,
            'heading_error_deg': 0.0, 'lateral_error_cm': 0.0,
            'disturbance': 'none', 'controller': 'mpc',
        }

        # ── Subscriptions ────────────────────────────────────────
        self.subscribe('odom', self._odom_cb, qos=0)
        self.subscribe('imu', self._imu_cb, qos=0)
        self.subscribe('precision_drive/command', self._command_cb, qos=1)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(DT, self._control_loop)
        self.create_timer(0.25, self._publish_status)

        self.log_info('MPCDriveNode ready (Np=%d Nc=%d tau=%.2f Q=[%.1f,%.1f,%.1f] R=[%.1f,%.1f])',
                      MPC_NP, MPC_NC, MPC_MOTOR_TAU,
                      MPC_Q_X, MPC_Q_Y, MPC_Q_THETA, MPC_R_V, MPC_R_OMEGA)

    # ── Callbacks ────────────────────────────────────────────────

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        with self._odom_lock:
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            self._odom_buf_x.append(x)
            self._odom_buf_y.append(y)
            if len(self._odom_buf_x) > ODOM_SMOOTH_N:
                self._odom_buf_x.pop(0)
                self._odom_buf_y.pop(0)
            self._odom_x = sum(self._odom_buf_x) / len(self._odom_buf_x)
            self._odom_y = sum(self._odom_buf_y) / len(self._odom_buf_y)
            self._odom_theta = data.get('theta', 0.0)
            self._odom_vx = data.get('vx', 0.0)
            self._odom_vz = data.get('vz', 0.0)
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
                self.log_info('Resumed -> %s', self._state)
            return

        if self._state != 'idle':
            self.log_warn('Busy (state=%s), ignoring command', self._state)
            return

        if action == 'drive':
            dist_cm = float(data.get('distance_cm', 0))
            direction = data.get('direction', 'forward')
            if dist_cm <= 0:
                return
            self._start_single_drive(dist_cm, direction)
        elif action == 'turn':
            angle_deg = float(data.get('angle_deg', 0))
            if abs(angle_deg) < 1:
                return
            self._start_single_turn(angle_deg)
        elif action == 'scenario':
            name = data.get('name', '')
            dist_cm = float(data.get('distance_cm', 50))
            self._start_scenario(name, dist_cm)

    # ── Scenario builder (same as precision_drive) ───────────────

    def _build_scenario(self, name, dist_cm):
        d = dist_cm
        if name == 'cross':
            return [
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'forward %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': -90, 'label': 'turn right 90'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'right %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': 90,  'label': 'turn left 90'},
                {'type': 'turn',  'deg': 90,  'label': 'turn left 90'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'left %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'back %dcm' % d},
                {'type': 'turn',  'deg': -90, 'label': 'back to forward'},
                {'type': 'turn',  'deg': 180, 'label': 'turn 180'},
                {'type': 'drive', 'cm': d, 'dir': 'forward',  'label': 'backward %dcm' % d},
                {'type': 'drive', 'cm': d, 'dir': 'backward', 'label': 'return to start'},
                {'type': 'turn',  'deg': 180, 'label': 'face original'},
            ]
        elif name == 'square':
            legs = []
            for i in range(4):
                legs.append({'type': 'drive', 'cm': d, 'dir': 'forward',
                             'label': 'side %d (%dcm)' % (i + 1, d)})
                legs.append({'type': 'turn', 'deg': -90,
                             'label': 'corner %d' % (i + 1)})
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
                             'label': 'leg %d' % (i + 1)})
                angle = 60 if (i % 2 == 0) else -60
                if i < 3:
                    legs.append({'type': 'turn', 'deg': angle, 'label': 'zigzag turn'})
            legs.append({'type': 'turn', 'deg': 180, 'label': 'turn back'})
            legs.append({'type': 'drive', 'cm': d, 'dir': 'forward', 'label': 'return'})
            return legs
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
        self.log_info('Leg %d/%d: %s', self._scenario_idx + 1,
                      len(self._scenario_legs), leg.get('label', ''))
        if leg['type'] == 'drive':
            self._start_single_drive(leg['cm'], leg['dir'])
        elif leg['type'] == 'turn':
            self._start_single_turn(leg['deg'])

    # ── Single movement starters ─────────────────────────────────

    def _start_single_drive(self, dist_cm, direction):
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

        self._arrive_count = 0
        self._leg_start_time = time.monotonic()

        heading_err = abs(_normalize_angle(self._target_heading - theta))
        if heading_err > TURN_TOL_RAD and direction in ('left', 'right'):
            self._target_yaw = self._target_heading
            self._turn_direction = 1.0 if _normalize_angle(self._target_heading - theta) > 0 else -1.0
            self._state = 'aligning'
        else:
            self._state = 'driving'

        self.log_info('MPC drive: %.1f cm %s (heading=%.1f°)',
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
        self.log_info('Turn: %.1f° (target=%.1f°)', angle_deg, math.degrees(self._target_yaw))

    # ── Main control loop (20 Hz) ────────────────────────────────

    def _control_loop(self):
        if not self._odom_ready or not self._imu_calibrated:
            return

        if self._state in ('driving', 'aligning', 'turning'):
            if self._check_disturbances():
                return

        state = self._state

        if state in ('idle', 'done'):
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
                    self._state = 'idle'
                    self._start_next_leg()
                else:
                    self._state = self._prev_state
            return
        elif state == 'aligning':
            self._do_align()
        elif state == 'driving':
            self._do_drive_mpc()
        elif state == 'turning':
            self._do_turn()

    # ── MPC driving ──────────────────────────────────────────────

    def _do_drive_mpc(self):
        """Drive using MPC in path-local frame."""
        with self._odom_lock:
            x = self._odom_x
            y = self._odom_y
            theta = self._odom_theta
            vx = self._odom_vx
            vz = self._odom_vz

        # ── Transform to path-local frame ────────────────────────
        h = self._target_heading
        cos_h = math.cos(h)
        sin_h = math.sin(h)
        dx = x - self._start_x
        dy = y - self._start_y

        x_path = dx * cos_h + dy * sin_h         # distance along path
        y_path = -dx * sin_h + dy * cos_h         # lateral deviation
        theta_path = _normalize_angle(theta - h)   # heading deviation

        target = self._target_distance  # signed
        is_reverse = target < 0
        target_abs = abs(target)
        dist_signed = x_path if not is_reverse else -x_path
        remaining = target_abs - dist_signed

        # ── Timeout ──────────────────────────────────────────────
        timeout = min(target_abs * 100 * TIMEOUT_PER_CM_S, MAX_LEG_TIMEOUT_S)
        if time.monotonic() - self._leg_start_time > timeout:
            self.log_warn('Leg timeout (%.1f s)', timeout)
            self._finish_leg(False, 'timeout')
            return

        # ── Arrival check ────────────────────────────────────────
        if remaining <= ARRIVE_TOL_M:
            self._arrive_count += 1
            if self._arrive_count >= self._ARRIVE_CONFIRM:
                self.log_info('Arrived! (error=%.1f mm)', remaining * 1000)
                self._finish_leg(True, 'arrived (%.1f mm)' % (remaining * 1000))
                return
            self._send_cmd(0.0, 0.0)
            return
        else:
            self._arrive_count = 0

        # ── Overshoot ────────────────────────────────────────────
        if remaining < -OVERSHOOT_TOL_M:
            self.log_warn('Overshoot by %.1f mm', -remaining * 1000)
            self._finish_leg(True, 'overshoot %.1f mm' % (-remaining * 1000))
            return

        # ── Build path-local state vector ────────────────────────
        # For reverse: flip x_path sign so MPC sees positive remaining
        if is_reverse:
            x_local = -x_path
            v_local = -vx
            target_local = -target   # positive
        else:
            x_local = x_path
            v_local = vx
            target_local = target

        x_state = np.array([x_local, y_path, theta_path, v_local, vz])

        # ── Generate reference trajectory ────────────────────────
        Np = self._mpc.Np
        ref = np.zeros((Np, 3))
        for k in range(Np):
            # Reference: move at DRIVE_SPEED towards target, cap at target
            x_ref_k = x_local + DRIVE_SPEED * (k + 1) * DT
            ref[k, 0] = min(x_ref_k, target_local)   # cap at target
            ref[k, 1] = 0.0                           # zero lateral deviation
            ref[k, 2] = 0.0                           # zero heading deviation

        # ── Compute MPC ──────────────────────────────────────────
        v0 = max(abs(v_local), DRIVE_SPEED * 0.3)  # avoid v₀=0 singularity
        v_dem, omega_dem = self._mpc.compute(x_state, ref, v0)

        # For reverse: flip v_dem back
        if is_reverse:
            v_dem = -v_dem

        self._send_cmd(v_dem, omega_dem)

    # ── Turn (proportional — same as precision_drive) ────────────

    def _do_align(self):
        with self._odom_lock:
            theta = self._odom_theta
        err = _normalize_angle(self._target_yaw - theta)
        if abs(err) <= TURN_TOL_RAD:
            self._send_cmd(0.0, 0.0)
            with self._odom_lock:
                self._start_x = self._odom_x
                self._start_y = self._odom_y
            self._state = 'driving'
            self.log_info('Aligned (err=%.1f°), driving with MPC', math.degrees(err))
            return
        if abs(err) > TURN_SLOW_RAD:
            speed = TURN_SPEED
        else:
            speed = TURN_SPEED_SLOW + (TURN_SPEED - TURN_SPEED_SLOW) * (abs(err) / TURN_SLOW_RAD)
        self._send_cmd(0.0, speed * (1.0 if err > 0 else -1.0))

    def _do_turn(self):
        with self._odom_lock:
            theta = self._odom_theta
        err = _normalize_angle(self._target_yaw - theta)
        if abs(err) <= TURN_TOL_RAD:
            self._arrive_count += 1
            if self._arrive_count >= self._ARRIVE_CONFIRM:
                self._send_cmd(0.0, 0.0)
                self.log_info('Turn done (err=%.1f°)', math.degrees(err))
                self._finish_leg(True, 'turn done (%.1f deg)' % math.degrees(err))
                return
            self._send_cmd(0.0, 0.0)
            return
        else:
            self._arrive_count = 0
        timeout = min(abs(math.degrees(err)) * 0.1 + 5, MAX_LEG_TIMEOUT_S)
        if time.monotonic() - self._leg_start_time > timeout:
            self._finish_leg(False, 'turn timeout')
            return
        if abs(err) > TURN_SLOW_RAD:
            speed = TURN_SPEED
        else:
            speed = TURN_SPEED_SLOW + (TURN_SPEED - TURN_SPEED_SLOW) * (abs(err) / TURN_SLOW_RAD)
        self._send_cmd(0.0, speed * (1.0 if err > 0 else -1.0))

    # ── Disturbance detection ────────────────────────────────────

    def _check_disturbances(self):
        now = time.monotonic()
        with self._imu_lock:
            ax, ay, az = self._imu_ax, self._imu_ay, self._imu_az

        a_mag = math.sqrt(ax * ax + ay * ay + az * az)

        # Lift
        if a_mag < LIFT_GRAVITY_LOW:
            self._lift_low_count += 1
            if self._lift_low_count >= LIFT_SAMPLES:
                self._prev_state = self._state
                self._state = 'paused_lift'
                self._send_cmd(0.0, 0.0)
                self.log_warn('LIFT detected (|a|=%.2f) — motors stopped', a_mag)
                return True
        else:
            self._lift_low_count = 0

        # Push
        if now - self._push_last_time < PUSH_COOLDOWN_S:
            return False
        with self._odom_lock:
            theta = self._odom_theta
        a_lateral = abs(-ax * math.sin(theta) + ay * math.cos(theta))
        if a_lateral > PUSH_ACCEL_THRESHOLD:
            self._push_last_time = now
            self._prev_state = self._state
            self._state = 'settling'
            self._settle_start = now
            self._settle_duration = SETTLE_TIME_S
            self._send_cmd(0.0, 0.0)
            self.log_warn('PUSH detected (a_lat=%.2f) — settling', a_lateral)
            return True
        return False

    def _handle_lift_recovery(self):
        with self._imu_lock:
            ax, ay, az = self._imu_ax, self._imu_ay, self._imu_az
        a_mag = math.sqrt(ax * ax + ay * ay + az * az)
        if a_mag >= LIFT_GRAVITY_OK:
            self._lift_low_count = 0
            self._state = 'settling'
            self._settle_start = time.monotonic()
            self._settle_duration = SETTLE_TIME_S
            self.log_info('Back on ground (|a|=%.2f) — settling', a_mag)

    # ── Leg completion ───────────────────────────────────────────

    def _finish_leg(self, success, reason):
        self._send_cmd(0.0, 0.0)
        self._arrive_count = 0
        if self._scenario_legs:
            self._scenario_idx += 1
            if success:
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
            x, y, theta = self._odom_x, self._odom_y, self._odom_theta

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
            s['distance_done_cm'] = round(abs(dist_along) * 100, 1)
            s['distance_target_cm'] = round(abs(self._target_distance) * 100, 1)
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
            'controller': 'mpc',
            'ts': self.timestamp(),
        })


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = MPCDriveNode(broker=args.broker, port=args.port,
                        robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
