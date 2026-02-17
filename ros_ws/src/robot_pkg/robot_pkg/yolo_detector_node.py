#!/usr/bin/env python3
"""
yolo_detector_node — Ball detection with YOLOv8n / YOLO11n.

Runs on the compute laptop (i9-13900H).
Subscribes: /camera/image_raw  (sensor_msgs/Image)
Publishes:  /ball_detection     (std_msgs/String)  JSON per detection
            /yolo/annotated     (sensor_msgs/Image) annotated frame

Colour classification: uses HSV analysis inside each bounding box.
"""

import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO

# HSV colour ranges for ball classification
COLOUR_RANGES = {
    'red':    [((0, 100, 100), (10, 255, 255)),
               ((160, 100, 100), (180, 255, 255))],
    'orange': [((10, 100, 100), (25, 255, 255))],
    'yellow': [((25, 100, 100), (35, 255, 255))],
    'green':  [((35, 100, 100), (85, 255, 255))],
    'blue':   [((85, 100, 100), (130, 255, 255))],
    'white':  [((0, 0, 200), (180, 30, 255))],
    'black':  [((0, 0, 0), (180, 255, 50))],
}

# Known ball diameter for distance estimation (metres)
BALL_DIAMETER_M = 0.04
FOCAL_LENGTH_PX = 500.0  # approximate for 640px wide CSI camera


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('device', 'cpu')

        model_path = self.get_parameter('model').value
        self._conf = self.get_parameter('confidence').value
        device = self.get_parameter('device').value

        self._bridge = CvBridge()
        self._model = YOLO(model_path)
        self._model.to(device)
        self.get_logger().info(f'YOLO model loaded: {model_path} on {device}')

        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 5)
        self._det_pub = self.create_publisher(String, '/ball_detection', 10)
        self._ann_pub = self.create_publisher(Image, '/yolo/annotated', 5)

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self._model(frame, conf=self._conf, verbose=False)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self._model.names.get(cls_id, 'unknown')

                # Only process "sports ball" class (id=32 in COCO) or all
                # For custom-trained models, adjust accordingly
                w = x2 - x1
                h = y2 - y1

                # Classify colour via HSV
                roi_hsv = hsv_frame[y1:y2, x1:x2]
                colour = self._classify_colour(roi_hsv)

                # Estimate distance from apparent size
                apparent_px = max(w, h)
                dist_est = -1.0
                if apparent_px > 10:
                    dist_est = (BALL_DIAMETER_M * FOCAL_LENGTH_PX) / apparent_px

                # Publish detection as JSON
                det = {
                    'colour': colour,
                    'class': cls_name,
                    'x': x1, 'y': y1, 'w': w, 'h': h,
                    'conf': round(conf, 3),
                    'distance': round(dist_est, 3),
                }
                det_msg = String()
                det_msg.data = json.dumps(det)
                self._det_pub.publish(det_msg)

                # Annotate frame
                label = f'{colour} {conf:.2f} {dist_est:.2f}m'
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish annotated image
        ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        ann_msg.header = msg.header
        self._ann_pub.publish(ann_msg)

    def _classify_colour(self, roi_hsv: np.ndarray) -> str:
        if roi_hsv.size == 0:
            return 'unknown'
        best_colour = 'unknown'
        best_ratio = 0.0
        total_px = roi_hsv.shape[0] * roi_hsv.shape[1]
        for colour, ranges in COLOUR_RANGES.items():
            mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(roi_hsv, np.array(lo), np.array(hi))
            ratio = np.count_nonzero(mask) / total_px
            if ratio > best_ratio and ratio > 0.15:
                best_ratio = ratio
                best_colour = colour
        return best_colour


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
