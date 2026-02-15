#!/usr/bin/env python3
"""
depth_to_scan_node — Converts camera image + ultrasonic range into a
pseudo LaserScan for slam_toolbox / cartographer compatibility.

Since we only have a single-point ultrasonic sensor and a monocular camera,
this node creates a minimal scan message by:
1. Taking ultrasonic range as the centre beam.
2. Filling side beams as max_range (unknown).

This allows slam_toolbox to ingest /scan topic.

Subscribes:
  /range               (sensor_msgs/Range)
Publishes:
  /scan                (sensor_msgs/LaserScan)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan

NUM_BEAMS = 180       # fake scan with 180 beams over 180°
ANGLE_MIN = -math.pi / 2.0
ANGLE_MAX = math.pi / 2.0
MAX_RANGE = 3.0
MIN_RANGE = 0.02


class DepthToScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_scan_node')
        self._latest_range = MAX_RANGE
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(0.1, self._publish_scan)  # 10 Hz
        self.get_logger().info('DepthToScan node started')

    def _range_cb(self, msg: Range):
        self._latest_range = msg.range

    def _publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'ultrasonic_link'
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX
        scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / NUM_BEAMS
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = MIN_RANGE
        scan.range_max = MAX_RANGE

        # Fill all beams with max_range, centre beam gets ultrasonic value
        ranges = [MAX_RANGE] * NUM_BEAMS
        centre = NUM_BEAMS // 2
        # Spread ultrasonic reading over ~15° cone (about 15 beams)
        cone_beams = int(0.26 / scan.angle_increment)  # ~15° half-cone
        for i in range(max(0, centre - cone_beams),
                       min(NUM_BEAMS, centre + cone_beams + 1)):
            ranges[i] = min(self._latest_range, MAX_RANGE)

        scan.ranges = ranges
        self._pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
