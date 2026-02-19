#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera -> ROS2 image topic.

Uses picamera2 (libcamera backend).
Publishes: /camera/image_raw/compressed  (sensor_msgs/CompressedImage)  @ 30 fps

Compressed JPEG reduces DDS network traffic by ~20x vs raw Image
(~40 KB/frame vs ~900 KB/frame at 640x480).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
    from picamera2 import Picamera2
    _HW = True
except ImportError:
    _HW = False

# JPEG quality: 75 is a good balance (≈40 KB/frame, visually lossless for YOLO)
JPEG_QUALITY = 75


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', JPEG_QUALITY)
        self.declare_parameter('hflip', False)
        self.declare_parameter('vflip', False)

        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self._quality = self.get_parameter('jpeg_quality').value

        # BEST_EFFORT: старые кадры не нужны, только последний
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', cam_qos)
        self._cam = None

        if not _HW:
            self.get_logger().error(
                'picamera2/cv2/numpy not available — camera disabled')
            return

        try:
            self._cam = Picamera2()
            config = self._cam.create_preview_configuration(
                main={'size': (w, h), 'format': 'RGB888'},
                buffer_count=4,
            )
            self._cam.configure(config)
            self._cam.start()
            self.get_logger().info(
                f'Camera started {w}x{h} @ {fps} fps → /camera/image_raw/compressed (JPEG {self._quality}%)')
            self.create_timer(1.0 / fps, self._capture)
        except Exception as exc:
            self.get_logger().error(f'Camera init failed: {exc} — camera disabled')
            self._cam = None

    def _capture(self):
        if self._cam is None:
            return
        frame = self._cam.capture_array()  # RGB888
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, enc = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        msg.data = enc.tobytes()
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
