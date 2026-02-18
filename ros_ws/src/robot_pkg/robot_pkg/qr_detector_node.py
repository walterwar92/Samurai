#!/usr/bin/env python3
"""
qr_detector_node — QR code detection from camera feed.

Uses OpenCV QRCodeDetector (no extra dependencies).

Subscribes:
    /camera/image_raw  (Image)  — camera feed

Publishes:
    /qr_detection  (String)  — JSON {data, timestamp} or {} if none
"""

import json
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._frame_count = 0
        self._skip_frames = 3  # process every Nth frame

        self._pub = self.create_publisher(String, '/qr_detection', 10)

        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 5)

        self.get_logger().info('QR detector node started')

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self._skip_frames != 0:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        try:
            data, points, _ = self._detector.detectAndDecode(frame)
        except Exception:
            data = None
            points = None

        out = String()
        if data:
            self.get_logger().info(f'QR detected: {data[:50]}')
            out.data = json.dumps({
                'data': data,
                'timestamp': time.time(),
            })
        else:
            out.data = '{}'

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
