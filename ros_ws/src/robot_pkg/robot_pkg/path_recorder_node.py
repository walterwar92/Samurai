#!/usr/bin/env python3
"""
path_recorder_node — Record and replay robot paths.

Records odometry poses and replays them as Nav2 goals.
Storage: ~/samurai_paths/

Subscribes:
    /odometry/filtered          (Odometry)  — robot pose
    /path_recorder/command      (String)    — "record"/"stop"/"replay"/"save:<name>"/"load:<name>"

Publishes:
    /goal_pose                  (PoseStamped) — Nav2 goals during replay
    /path_recorder/status       (String)      — JSON {state, points_count, current_idx}
    /path_recorder/path_list    (String)      — JSON list of saved path names
"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

PATHS_DIR = os.path.expanduser('~/samurai_paths')
MIN_DIST = 0.20   # metres between recorded points
MIN_ROT = 0.30    # radians between recorded points
GOAL_TOL = 0.20   # metres tolerance for replay goal reached


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')

        os.makedirs(PATHS_DIR, exist_ok=True)

        self._state = 'idle'  # idle / recording / replaying
        self._path = []
        self._replay_idx = 0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0

        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._status_pub = self.create_publisher(String, '/path_recorder/status', 10)
        self._list_pub = self.create_publisher(String, '/path_recorder/path_list', 10)

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(String, '/path_recorder/command', self._command_cb, 10)

        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f'Path recorder — storage: {PATHS_DIR}')

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

        if self._state == 'recording':
            dist = math.hypot(self._robot_x - self._last_x,
                              self._robot_y - self._last_y)
            rot = abs(self._robot_yaw - self._last_yaw)
            if dist >= MIN_DIST or rot >= MIN_ROT:
                self._path.append({
                    'x': round(self._robot_x, 3),
                    'y': round(self._robot_y, 3),
                    'yaw': round(self._robot_yaw, 3),
                })
                self._last_x = self._robot_x
                self._last_y = self._robot_y
                self._last_yaw = self._robot_yaw

    def _command_cb(self, msg: String):
        cmd = msg.data.strip()

        if cmd == 'record':
            self._state = 'recording'
            self._path = []
            self._last_x = self._robot_x
            self._last_y = self._robot_y
            self._last_yaw = self._robot_yaw
            self.get_logger().info('Recording path...')

        elif cmd == 'stop':
            self._state = 'idle'
            self.get_logger().info(f'Stopped — {len(self._path)} points recorded')

        elif cmd == 'replay':
            if not self._path:
                self.get_logger().warn('No path to replay')
                return
            self._state = 'replaying'
            self._replay_idx = 0
            self.get_logger().info(f'Replaying {len(self._path)} points')

        elif cmd.startswith('save:'):
            name = cmd[5:].strip()
            if name and self._path:
                path = os.path.join(PATHS_DIR, f'{name}.json')
                with open(path, 'w') as f:
                    json.dump(self._path, f)
                self.get_logger().info(f'Path saved: {name} ({len(self._path)} pts)')

        elif cmd.startswith('load:'):
            name = cmd[5:].strip()
            path = os.path.join(PATHS_DIR, f'{name}.json')
            if os.path.exists(path):
                with open(path, 'r') as f:
                    self._path = json.load(f)
                self.get_logger().info(f'Path loaded: {name} ({len(self._path)} pts)')
            else:
                self.get_logger().warn(f'Path not found: {name}')

        elif cmd == 'list':
            paths = [f[:-5] for f in os.listdir(PATHS_DIR) if f.endswith('.json')]
            paths.sort()
            out = String()
            out.data = json.dumps(paths)
            self._list_pub.publish(out)

    def _tick(self):
        if self._state != 'replaying' or not self._path:
            return

        if self._replay_idx >= len(self._path):
            self._state = 'idle'
            self.get_logger().info('Replay complete')
            return

        wp = self._path[self._replay_idx]
        dist = math.hypot(wp['x'] - self._robot_x, wp['y'] - self._robot_y)

        if dist < GOAL_TOL:
            self._replay_idx += 1
            return

        # Send goal
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
        self._goal_pub.publish(goal)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self._state,
            'points_count': len(self._path),
            'current_idx': self._replay_idx if self._state == 'replaying' else -1,
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
