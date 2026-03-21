#!/usr/bin/env python3
"""
path_recorder_node — Records robot path and replays it in reverse for "return home".

Records odometry waypoints while the robot moves. On "replay" command,
drives the robot back along the same path in reverse order.

Subscribes:
    samurai/{robot_id}/odom                 — {x, y, theta, vx, vz, ...}
    samurai/{robot_id}/path_recorder/command — "record" / "stop" / "replay" / "clear"
Publishes:
    samurai/{robot_id}/cmd_vel              — drive commands during replay
    samurai/{robot_id}/path_recorder/status — {state, waypoints_count, ...}
    samurai/{robot_id}/path_recorder/path   — full path as [[x,y,theta], ...]
"""

import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Configuration ─────────────────────────────────────────────
RECORD_INTERVAL_M = 0.03      # record waypoint every 3 cm moved (finer path)
RECORD_INTERVAL_RAD = 0.08    # or every ~4.5° turned (finer turns)
REPLAY_GOAL_TOLERANCE = 0.04  # m — close enough to intermediate waypoint
REPLAY_HOME_TOLERANCE = 0.015 # m — tight tolerance for final home point (1.5 cm)
REPLAY_LINEAR_SPEED = 0.20    # m/s — replay drive speed
REPLAY_MIN_LINEAR = 0.05      # m/s — minimum linear speed to avoid stalls
REPLAY_ANGULAR_SPEED = 0.8    # rad/s — max replay turn speed
REPLAY_ANGULAR_KP = 2.5       # proportional gain for angular correction
REPLAY_LOOKAHEAD_M = 0.10     # m — look-ahead distance for pure pursuit
MAX_WAYPOINTS = 8000          # memory safety limit (finer recording = more pts)
CONTROL_HZ = 15               # replay control loop frequency (smoother control)

# ── Final approach (last waypoint) ───────────────────────────
FINAL_APPROACH_RADIUS = 0.12  # m — switch to slow precision mode within this
FINAL_LINEAR_SPEED = 0.08     # m/s — crawl speed for final approach
FINAL_ANGULAR_KP = 3.0        # stronger angular correction in final phase
PATHS_DIR = os.path.expanduser('~/paths')


class PathRecorderNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('path_recorder_node', **kwargs)

        # State
        self._state = 'idle'  # idle / recording / replaying
        self._path = []       # list of (x, y, theta) tuples
        self._replay_index = 0
        self._last_record_x = 0.0
        self._last_record_y = 0.0
        self._last_record_theta = 0.0

        # Current pose from odometry
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._pose_valid = False

        # Subscribers
        self.subscribe('odom', self._odom_cb)
        self.subscribe('path_recorder/command', self._command_cb, qos=1)

        # Control loop for replay
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        # Status publisher (1 Hz)
        self.create_timer(1.0, self._publish_status)

        self.log_info('Path recorder ready (record_interval=%.2fm, max=%d waypoints)',
                      RECORD_INTERVAL_M, MAX_WAYPOINTS)

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        self._x = data.get('x', 0.0)
        self._y = data.get('y', 0.0)
        self._theta = data.get('theta', 0.0)
        stationary = data.get('stationary', True)
        self._pose_valid = True

        # Auto-start recording when robot moves (always have a path for "home")
        if self._state == 'idle' and not stationary:
            self._start_recording()
            self.log_info('Auto-recording: robot is moving')

        # Record waypoint if recording and moved enough
        if self._state == 'recording':
            self._maybe_record_waypoint()

    def _command_cb(self, topic, data):
        if isinstance(data, dict):
            cmd = data.get('command', '').lower().strip()
        else:
            cmd = str(data).lower().strip()

        if cmd in ('record', 'start'):
            name = data.get('name', '') if isinstance(data, dict) else ''
            self._start_recording()
        elif cmd == 'stop':
            name = data.get('name', '') if isinstance(data, dict) else ''
            self._stop(name)
        elif cmd in ('replay', 'play'):
            name = data.get('name', '') if isinstance(data, dict) else ''
            if name:
                self._load_path(name)
            self._start_replay()
        elif cmd == 'clear':
            self._clear_path()
        elif cmd == 'save':
            name = data.get('name', 'path') if isinstance(data, dict) else 'path'
            self._save_path(name)
        elif cmd == 'load':
            name = data.get('name', 'path') if isinstance(data, dict) else 'path'
            self._load_path(name)
        else:
            self.log_warn('Unknown path_recorder command: %s', cmd)

    # ── Recording ──────────────────────────────────────────────
    def _start_recording(self):
        if self._state == 'replaying':
            self._stop_driving()
        self._state = 'recording'
        self._path = []
        if self._pose_valid:
            self._path.append((self._x, self._y, self._theta))
            self._last_record_x = self._x
            self._last_record_y = self._y
            self._last_record_theta = self._theta
        self.log_info('Path recording started')

    def _maybe_record_waypoint(self):
        if len(self._path) >= MAX_WAYPOINTS:
            return

        dx = self._x - self._last_record_x
        dy = self._y - self._last_record_y
        dist = math.sqrt(dx * dx + dy * dy)
        dtheta = abs(self._angle_diff(self._theta, self._last_record_theta))

        if dist >= RECORD_INTERVAL_M or dtheta >= RECORD_INTERVAL_RAD:
            self._path.append((self._x, self._y, self._theta))
            self._last_record_x = self._x
            self._last_record_y = self._y
            self._last_record_theta = self._theta

    # ── Replay (reverse path following) ────────────────────────
    def _start_replay(self):
        if not self._path:
            self.log_warn('No path recorded — cannot replay')
            return
        if self._state == 'recording':
            # Add final waypoint
            if self._pose_valid:
                self._path.append((self._x, self._y, self._theta))

        self._state = 'replaying'
        # Reverse the path for return-home
        self._replay_index = len(self._path) - 1
        self.log_info('Path replay started (%d waypoints, reversed)', len(self._path))

    def _control_loop(self):
        if self._state != 'replaying':
            return
        if not self._pose_valid:
            return
        if self._replay_index < 0:
            self.log_info('Path replay complete — arrived home')
            self._stop_driving()
            self._state = 'idle'
            self._publish_status()
            return

        # ── Skip waypoints that are already behind us (look-ahead) ──
        while self._replay_index > 0:
            tx, ty, _ = self._path[self._replay_index]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < REPLAY_LOOKAHEAD_M:
                self._replay_index -= 1
            else:
                break

        # ── Determine target ──
        is_final = (self._replay_index == 0)
        tx, ty, t_theta = self._path[self._replay_index]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)
        tolerance = REPLAY_HOME_TOLERANCE if is_final else REPLAY_GOAL_TOLERANCE

        if dist < tolerance:
            self._replay_index -= 1
            if is_final:
                # Final home point reached — do precision alignment
                self.log_info('Home reached (error=%.3fm)', dist)
                self._stop_driving()
                self._state = 'idle'
                self._publish_status()
            return

        # ── Pure pursuit: simultaneous linear + angular ──
        bearing = math.atan2(dy, dx)
        angle_error = self._angle_diff(bearing, self._theta)

        # Final approach mode: within FINAL_APPROACH_RADIUS of home point
        in_final_approach = is_final and dist < FINAL_APPROACH_RADIUS

        if in_final_approach:
            # ── Precision final approach ──
            # Stronger angular correction, slower speed, proportional to distance
            angular = FINAL_ANGULAR_KP * angle_error
            angular = max(-REPLAY_ANGULAR_SPEED, min(REPLAY_ANGULAR_SPEED, angular))

            cos_factor = max(0.0, math.cos(angle_error))
            # Proportional slowdown: linear speed scales with remaining distance
            dist_ratio = dist / FINAL_APPROACH_RADIUS  # 0..1
            linear = FINAL_LINEAR_SPEED * cos_factor * dist_ratio
            linear = max(REPLAY_MIN_LINEAR * 0.6 * cos_factor, linear)

            # Rotate in place if angle error > 45° during final approach
            if abs(angle_error) > 0.8:
                linear = 0.0
        else:
            # ── Normal pure pursuit ──
            # Angular: proportional control, clamped
            angular = REPLAY_ANGULAR_KP * angle_error
            angular = max(-REPLAY_ANGULAR_SPEED, min(REPLAY_ANGULAR_SPEED, angular))

            # Linear: scale down when turning hard or near target
            cos_factor = max(0.0, math.cos(angle_error))
            dist_factor = min(1.0, dist / 0.15)

            linear = REPLAY_LINEAR_SPEED * cos_factor * dist_factor
            linear = max(REPLAY_MIN_LINEAR * cos_factor, linear)

            # If angle error is very large (>90°), stop and rotate in place
            if abs(angle_error) > 1.2:
                linear = 0.0

        self.publish('cmd_vel', {
            'linear_x': round(linear, 3),
            'angular_z': round(angular, 3),
        })

    # ── Stop / Clear ───────────────────────────────────────────
    def _stop(self, name: str = ''):
        if self._state == 'replaying':
            self._stop_driving()
        old = self._state
        self._state = 'idle'
        self.log_info('Path recorder stopped (was %s, %d waypoints saved)',
                      old, len(self._path))
        # Auto-save if name was provided and we have waypoints
        if name and self._path:
            self._save_path(name)

    def _clear_path(self):
        self._stop()
        self._path = []
        self.log_info('Path cleared')

    def _stop_driving(self):
        self.publish('cmd_vel', {'linear_x': 0.0, 'angular_z': 0.0})

    # ── Save / Load ────────────────────────────────────────────
    def _save_path(self, name: str):
        os.makedirs(PATHS_DIR, exist_ok=True)
        filepath = os.path.join(PATHS_DIR, f'{name}.json')
        data = {
            'name': name,
            'waypoints': len(self._path),
            'path': [[round(x, 4), round(y, 4), round(t, 4)]
                     for x, y, t in self._path],
            'saved_at': time.time(),
        }
        with open(filepath, 'w') as f:
            json.dump(data, f)
        self.log_info('Path saved: %s (%d waypoints)', filepath, len(self._path))

    def _load_path(self, name: str):
        filepath = os.path.join(PATHS_DIR, f'{name}.json')
        if not os.path.exists(filepath):
            self.log_warn('Path file not found: %s', filepath)
            return
        with open(filepath, 'r') as f:
            data = json.load(f)
        self._path = [(w[0], w[1], w[2]) for w in data.get('path', [])]
        self.log_info('Path loaded: %s (%d waypoints)', name, len(self._path))

    # ── Status / Path publishing ───────────────────────────────
    def _publish_status(self):
        status = {
            'state': self._state,
            'waypoints_count': len(self._path),
            'replay_index': self._replay_index if self._state == 'replaying' else -1,
            'replay_remaining': max(0, self._replay_index + 1)
                                if self._state == 'replaying' else 0,
        }
        self.publish('path_recorder/status', status)

        # Publish path (for map overlay) — only if path exists and not too large
        if self._path and len(self._path) <= 1000:
            path_data = [[round(x, 3), round(y, 3)] for x, y, _ in self._path]
            self.publish('path_recorder/path', path_data)

    # ── Helpers ────────────────────────────────────────────────
    @staticmethod
    def _angle_diff(target, current):
        """Shortest signed angle difference (target - current), in [-pi, pi]."""
        d = target - current
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def on_shutdown(self):
        if self._state == 'replaying':
            self._stop_driving()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = PathRecorderNode(broker=args.broker, port=args.port,
                            robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
