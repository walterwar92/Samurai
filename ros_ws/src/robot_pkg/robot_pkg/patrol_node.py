#!/usr/bin/env python3
"""
patrol_node — Autonomous waypoint patrol.

Cycles through waypoints, sending Nav2 goals. Loops after last waypoint.

Subscribes:
    /patrol/waypoints   (String)   — JSON list of {x, y, yaw}
    /patrol/command      (String)   — "start" / "stop" / "pause"
    /odometry/filtered   (Odometry) — current robot pose

Publishes:
    /goal_pose      (PoseStamped)  — Nav2 goal
    /patrol/status  (String)       — JSON {active, paused, current_waypoint_idx, total}
"""

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

GOAL_TOLERANCE = 0.20  # metres


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self._waypoints = []
        self._current_idx = 0
        self._active = False
        self._paused = False
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._goal_sent = False

        # Publishers
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._status_pub = self.create_publisher(String, '/patrol/status', 10)

        # Subscribers
        self.create_subscription(String, '/patrol/waypoints', self._waypoints_cb, 10)
        self.create_subscription(String, '/patrol/command', self._command_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)

        # 2 Hz tick
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Patrol node started')

    def _waypoints_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self._waypoints = data
                self._current_idx = 0
                self._goal_sent = False
                self.get_logger().info(f'Received {len(data)} waypoints')
        except json.JSONDecodeError:
            pass

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if not self._waypoints:
                self.get_logger().warn('No waypoints set')
                return
            self._active = True
            self._paused = False
            self._goal_sent = False
            self.get_logger().info('Patrol started')
        elif cmd == 'stop':
            self._active = False
            self._paused = False
            self._current_idx = 0
            self._goal_sent = False
            self.get_logger().info('Patrol stopped')
        elif cmd == 'pause':
            self._paused = not self._paused
            self.get_logger().info(f'Patrol {"paused" if self._paused else "resumed"}')

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _tick(self):
        if not self._active or self._paused or not self._waypoints:
            return

        wp = self._waypoints[self._current_idx]
        tx, ty = wp.get('x', 0.0), wp.get('y', 0.0)

        # Send goal if not yet sent
        if not self._goal_sent:
            self._send_goal(tx, ty, wp.get('yaw', 0.0))
            self._goal_sent = True

        # Check if reached
        dist = math.hypot(tx - self._robot_x, ty - self._robot_y)
        if dist < GOAL_TOLERANCE:
            self.get_logger().info(
                f'Waypoint {self._current_idx + 1}/{len(self._waypoints)} reached')
            self._current_idx = (self._current_idx + 1) % len(self._waypoints)
            self._goal_sent = False

    def _send_goal(self, x: float, y: float, yaw: float):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(goal)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'active': self._active,
            'paused': self._paused,
            'current_waypoint_idx': self._current_idx,
            'total': len(self._waypoints),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
