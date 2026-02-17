#!/usr/bin/env python3
"""
depth_to_scan_node — Monocular depth estimation → LaserScan for SLAM.

Uses MiDaS Small to estimate dense depth from the camera image.
The ultrasonic sensor provides absolute scale calibration.
Publishes a LaserScan covering the camera's horizontal FOV.

Runs on the compute laptop (i9-13900H) alongside YOLO/Nav2.

Subscribes:
    /camera/image_raw  (sensor_msgs/Image)   — monocular camera
    /range             (sensor_msgs/Range)    — ultrasonic distance (scale ref)
Publishes:
    /scan              (sensor_msgs/LaserScan) — dense pseudo-scan for SLAM
"""

import math
import threading

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range, LaserScan
from cv_bridge import CvBridge


class DepthToScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_scan_node')

        self.declare_parameter('camera_hfov', 1.085)    # ~62° RPi CSI v2
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('num_beams', 180)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('scan_row_ratio', 0.5)    # 0.5 = middle row
        self.declare_parameter('scale_smoothing', 0.8)   # EMA for scale factor

        self._hfov = self.get_parameter('camera_hfov').value
        self._max_range = self.get_parameter('max_range').value
        self._min_range = self.get_parameter('min_range').value
        self._num_beams = self.get_parameter('num_beams').value
        self._scan_row_ratio = self.get_parameter('scan_row_ratio').value
        self._alpha = self.get_parameter('scale_smoothing').value
        rate = self.get_parameter('publish_rate').value

        # --- Load MiDaS Small ---
        self.get_logger().info('Loading MiDaS Small model (first run downloads ~50 MB)...')
        self._model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True)
        self._model.eval()
        self._transform = torch.hub.load(
            'intel-isl/MiDaS', 'transforms', trust_repo=True
        ).small_transform
        self.get_logger().info('MiDaS Small loaded')

        # --- State ---
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._us_range = self._max_range
        self._scale = 0.0   # calibrated scale factor (0 = not yet calibrated)

        # --- ROS interfaces ---
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 1)
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'DepthToScan ready  hfov={math.degrees(self._hfov):.0f}°  '
            f'beams={self._num_beams}  rate={rate} Hz'
        )

    # ── Callbacks ─────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock:
            self._latest_frame = frame

    def _range_cb(self, msg: Range):
        if msg.range >= self._min_range:
            with self._lock:
                self._us_range = msg.range

    # ── Main loop ─────────────────────────────────────────────

    def _tick(self):
        with self._lock:
            frame = self._latest_frame
            us_range = self._us_range

        if frame is None:
            return

        # --- Run MiDaS ---
        input_batch = self._transform(frame)
        with torch.no_grad():
            inv_depth = self._model(input_batch)          # (1, H_out, W_out)

        inv_depth = inv_depth.squeeze().cpu().numpy()     # (H, W) relative inverse depth

        h, w = inv_depth.shape
        row_idx = int(h * self._scan_row_ratio)
        row_idx = max(0, min(row_idx, h - 1))

        # Extract horizontal scanline
        scan_row = inv_depth[row_idx, :]                  # (W,)

        # --- Calibrate scale using ultrasonic at centre ---
        centre_val = scan_row[w // 2]
        if centre_val > 1e-3 and us_range < self._max_range:
            new_scale = us_range * centre_val
            if self._scale < 1e-3:
                self._scale = new_scale                   # first calibration
            else:
                self._scale = self._alpha * self._scale + (1.0 - self._alpha) * new_scale

        if self._scale < 1e-3:
            return  # not calibrated yet, skip

        # --- Convert inverse depth → metric depth ---
        with np.errstate(divide='ignore', invalid='ignore'):
            metric = np.where(scan_row > 1e-3, self._scale / scan_row, self._max_range)
        metric = np.clip(metric, self._min_range, self._max_range)

        # --- Resample to num_beams ---
        indices = np.linspace(0, len(metric) - 1, self._num_beams).astype(int)
        ranges = metric[indices]

        # Image columns go left→right, LaserScan angle_min (right) → angle_max (left)
        ranges = ranges[::-1].copy()

        # --- Publish LaserScan ---
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'camera_link'
        scan.angle_min = -self._hfov / 2.0
        scan.angle_max = self._hfov / 2.0
        scan.angle_increment = self._hfov / self._num_beams
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self._min_range
        scan.range_max = self._max_range
        scan.ranges = ranges.tolist()

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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
