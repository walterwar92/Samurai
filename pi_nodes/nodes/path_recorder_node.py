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
RECORD_INTERVAL_M = 0.05      # record waypoint every 5 cm moved
RECORD_INTERVAL_RAD = 0.10    # or every ~6° turned
REPLAY_GOAL_TOLERANCE = 0.08  # m — close enough to waypoint
REPLAY_ANGLE_TOLERANCE = 0.15 # rad — close enough heading (~8.6°)
REPLAY_LINEAR_SPEED = 0.12    # m/s — replay drive speed
REPLAY_ANGULAR_SPEED = 0.6    # rad/s — replay turn speed
REPLAY_ALIGN_FIRST = True     # rotate to face next waypoint before driving
MAX_WAYPOINTS = 5000          # memory safety limit
CONTROL_HZ = 10               # replay control loop frequency
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
        self._pose_valid = True

        # Record waypoint if recording and moved enough
        if self._state == 'recording':
            self._maybe_record_waypoint()

    def _command_cb(self, topic, data):
        if isinstance(data, dict):
            cmd = data.get('command', '').lower().strip()
        else:
            cmd = str(data).lower().strip()

        if cmd == 'record':
            self._start_recording()
        elif cmd == 'stop':
            self._stop()
        elif cmd == 'replay':
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

        # Target waypoint
        tx, ty, t_theta = self._path[self._replay_index]

        # Distance and bearing to target
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < REPLAY_GOAL_TOLERANCE:
            # Reached this waypoint, move to next (going backwards through path)
            self._replay_index -= 1
            return

        # Bearing to target
        bearing = math.atan2(dy, dx)
        angle_error = self._angle_diff(bearing, self._theta)

        # If we need to turn significantly, rotate first
        if abs(angle_error) > REPLAY_ANGLE_TOLERANCE:
            angular = REPLAY_ANGULAR_SPEED if angle_error > 0 else -REPLAY_ANGULAR_SPEED
            # Slow rotation for small errors
            if abs(angle_error) < 0.3:
                angular *= abs(angle_error) / 0.3
            self.publish('cmd_vel', {
                'linear_x': 0.0,
                'angular_z': round(angular, 3),
            })
        else:
            # Drive toward waypoint
            linear = min(REPLAY_LINEAR_SPEED, dist * 1.5)
            # Small angular correction while driving
            angular = angle_error * 1.5
            angular = max(-REPLAY_ANGULAR_SPEED, min(REPLAY_ANGULAR_SPEED, angular))
            self.publish('cmd_vel', {
                'linear_x': round(linear, 3),
                'angular_z': round(angular, 3),
            })

    # ── Stop / Clear ───────────────────────────────────────────
    def _stop(self):
        if self._state == 'replaying':
            self._stop_driving()
        old = self._state
        self._state = 'idle'
        self.log_info('Path recorder stopped (was %s, %d waypoints saved)',
                      old, len(self._path))

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
