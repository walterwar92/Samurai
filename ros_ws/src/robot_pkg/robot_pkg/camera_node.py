#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera → ROS2 image topic.

Uses picamera2 (libcamera backend).
Publishes: /camera/image_raw  (sensor_msgs/Image)  @ 30 fps
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('hflip', False)
        self.declare_parameter('vflip', False)

        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Picamera2 setup
        self._cam = Picamera2()
        config = self._cam.create_preview_configuration(
            main={'size': (w, h), 'format': 'RGB888'},
            buffer_count=4,
        )
        self._cam.configure(config)
        self._cam.start()
        self.get_logger().info(f'Camera started {w}x{h} @ {fps} fps')

        self.create_timer(1.0 / fps, self._capture)

    def _capture(self):
        frame = self._cam.capture_array()  # RGB888
        # Convert RGB → BGR for cv_bridge compatibility
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        msg = self._bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self._pub.publish(msg)

    def destroy_node(self):
        self._cam.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
