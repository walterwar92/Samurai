#!/usr/bin/env python3
"""
watchdog_node — Monitor health of all robot nodes by tracking topic activity.

Subscribes to key topics and reports which are alive/dead.

Publishes:
    /watchdog  (std_msgs/String)  — JSON: {topic: {alive, last_seen_sec}}
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, Range, Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

TIMEOUT_SEC = 5.0  # topic considered dead after this

MONITORED_TOPICS = {
    '/odom': Odometry,
    '/camera/image_raw': Image,
    '/imu/data': Imu,
    '/range': Range,
    '/scan': LaserScan,
    '/ball_detection': String,
    '/cmd_vel': Twist,
    '/battery': Float32,
    '/cpu_temperature': Float32,
}


class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog_node')

        self._last_seen = {}
        self._pub = self.create_publisher(String, '/watchdog', 10)

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        for topic, msg_type in MONITORED_TOPICS.items():
            self._last_seen[topic] = 0.0
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._topic_cb(t),
                best_effort,
            )

        # 1 Hz health report
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            f'Watchdog monitoring {len(MONITORED_TOPICS)} topics')

    def _topic_cb(self, topic: str):
        self._last_seen[topic] = time.time()

    def _report(self):
        now = time.time()
        report = {}
        dead_topics = []

        for topic in MONITORED_TOPICS:
            last = self._last_seen[topic]
            if last == 0.0:
                alive = False
                age = -1.0
            else:
                age = round(now - last, 1)
                alive = age < TIMEOUT_SEC

            report[topic] = {'alive': alive, 'last_seen_sec': age}
            if not alive and last > 0.0:
                dead_topics.append(topic)

        if dead_topics:
            self.get_logger().warn(f'Dead topics: {", ".join(dead_topics)}')

        msg = String()
        msg.data = json.dumps(report)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
