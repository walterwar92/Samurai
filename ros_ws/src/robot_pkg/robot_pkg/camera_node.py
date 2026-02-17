#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera -> ROS2 image topic.

Uses picamera2 (libcamera backend).
Publishes: /camera/image_raw  (sensor_msgs/Image)  @ 30 fps
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from picamera2 import Picamera2
    _HW = True
except ImportError:
    _HW = False


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

        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self._cam = None
        self._bridge = None

        if not _HW:
            self.get_logger().error(
                'picamera2/cv2/numpy/cv_bridge not available — camera disabled')
            return

        try:
            self._bridge = CvBridge()
            self._cam = Picamera2()
            config = self._cam.create_preview_configuration(
                main={'size': (w, h), 'format': 'RGB888'},
                buffer_count=4,
            )
            self._cam.configure(config)
            self._cam.start()
            self.get_logger().info(f'Camera started {w}x{h} @ {fps} fps')
            self.create_timer(1.0 / fps, self._capture)
        except Exception as exc:
            self.get_logger().error(f'Camera init failed: {exc} — camera disabled')
            self._cam = None

    def _capture(self):
        if self._cam is None:
            return
        frame = self._cam.capture_array()  # RGB888
        # Convert RGB -> BGR for cv_bridge compatibility
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        msg = self._bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cam is not None:
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
