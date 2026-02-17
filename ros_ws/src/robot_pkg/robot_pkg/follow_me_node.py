#!/usr/bin/env python3
"""
follow_me_node — Follow a person detected by YOLO.

Uses "person" class detections to track and follow a human at a target distance.

Subscribes:
    /yolo/detections    (String)  — JSON list of all YOLO detections
    /follow_me/command  (String)  — "start" / "stop"

Publishes:
    /cmd_vel           (Twist)   — velocity commands
    /follow_me/status  (String)  — JSON {active, tracking, distance}
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

TARGET_DISTANCE = 1.0    # metres to maintain from person
PERSON_HEIGHT_M = 1.7    # average person height for distance estimation
FOCAL_LENGTH_PX = 500.0  # camera focal length estimate
IMG_CX = 320             # image centre x
LOST_TIMEOUT = 2.0       # seconds before stopping


class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')

        self._active = False
        self._tracking = False
        self._last_person_time = 0.0
        self._person_distance = 0.0

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/follow_me/status', 10)

        self.create_subscription(String, '/yolo/detections', self._detections_cb, 10)
        self.create_subscription(String, '/follow_me/command', self._command_cb, 10)

        self.create_timer(0.1, self._tick)  # 10 Hz
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Follow-me node started')

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self._active = True
            self.get_logger().info('Follow-me activated')
        elif cmd == 'stop':
            self._active = False
            self._tracking = False
            self._cmd_pub.publish(Twist())  # stop
            self.get_logger().info('Follow-me deactivated')

    def _detections_cb(self, msg: String):
        if not self._active:
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # Find largest "person" detection (closest)
        best = None
        best_area = 0
        for det in detections:
            if det.get('class', '') == 'person':
                area = det.get('w', 0) * det.get('h', 0)
                if area > best_area:
                    best = det
                    best_area = area

        if best is not None:
            self._last_person_time = time.time()
            self._tracking = True

            # Estimate distance from bounding box height
            bbox_h = best.get('h', 1)
            if bbox_h > 10:
                self._person_distance = (PERSON_HEIGHT_M * FOCAL_LENGTH_PX) / bbox_h
            else:
                self._person_distance = TARGET_DISTANCE

            # Steering: centre person in frame
            person_cx = best.get('x', IMG_CX) + best.get('w', 0) / 2.0
            error_x = (person_cx - IMG_CX) / IMG_CX  # -1 to +1

            # Speed: proportional to distance error
            dist_error = self._person_distance - TARGET_DISTANCE

            twist = Twist()
            twist.angular.z = -error_x * 0.8
            if dist_error > 0.15:
                twist.linear.x = min(0.20, dist_error * 0.3)
            elif dist_error < -0.15:
                twist.linear.x = max(-0.10, dist_error * 0.2)

            self._cmd_pub.publish(twist)

    def _tick(self):
        if not self._active:
            return
        if self._tracking and (time.time() - self._last_person_time) > LOST_TIMEOUT:
            self._tracking = False
            self._cmd_pub.publish(Twist())  # stop
            self.get_logger().warn('Person lost — stopping')

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'active': self._active,
            'tracking': self._tracking,
            'distance': round(self._person_distance, 2),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
