#!/usr/bin/env python3
"""
yolo_detector_node — Ball detection with YOLOv8n / YOLO11n.

Runs on the compute laptop (i9-13900H).
Subscribes: /camera/image_raw/compressed  (sensor_msgs/CompressedImage)
Publishes:  /ball_detection               (std_msgs/String)  JSON per detection
            /yolo/annotated/compressed    (sensor_msgs/CompressedImage) annotated JPEG

Using CompressedImage reduces DDS traffic by ~20x vs raw Image.
Colour classification: uses HSV analysis inside each bounding box.
"""

import json
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

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

        onnx_path = model_path.replace('.pt', '.onnx')
        if model_path.endswith('.pt') and not os.path.exists(onnx_path):
            self.get_logger().info(f'Exporting {model_path} → {onnx_path} (ONNX, imgsz=320)...')
            tmp = YOLO(model_path)
            tmp.export(format='onnx', imgsz=320, optimize=True, simplify=True)
            self.get_logger().info('ONNX export complete')

        if os.path.exists(onnx_path):
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] \
                        if device == 'cuda' else ['CPUExecutionProvider']
            self._ort = ort.InferenceSession(onnx_path, providers=providers)
            self._input_name = self._ort.get_inputs()[0].name
            self._imgsz = 320
            self._use_onnx = True
            _tmp = YOLO(model_path) if os.path.exists(model_path) else None
            self._names = _tmp.names if _tmp else {0: 'object'}
            self.get_logger().info(f'ONNX Runtime session: {onnx_path} on {providers[0]}')
        else:
            self._model = YOLO(model_path)
            self._model.to(device)
            self._use_onnx = False
            self._names = self._model.names
            self.get_logger().info(f'YOLO (PyTorch) loaded: {model_path} on {device}')

        # BEST_EFFORT: старые кадры не нужны для детекции
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self._image_cb, cam_qos)
        self._det_pub = self.create_publisher(String, '/ball_detection', 10)
        self._ann_pub = self.create_publisher(CompressedImage, '/yolo/annotated/compressed', cam_qos)

    def _infer(self, frame: np.ndarray):
        if self._use_onnx:
            return self._infer_onnx(frame)
        results = self._model(frame, conf=self._conf, verbose=False)
        boxes = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                boxes.append((x1, y1, x2, y2, float(box.conf[0]), int(box.cls[0])))
        return boxes

    def _infer_onnx(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        img = cv2.resize(frame, (self._imgsz, self._imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[np.newaxis]
        outputs = self._ort.run(None, {self._input_name: img})[0]
        boxes = []
        preds = outputs[0].T
        for pred in preds:
            cx, cy, bw, bh = pred[:4]
            scores = pred[4:]
            cls_id = int(np.argmax(scores))
            conf = float(scores[cls_id])
            if conf < self._conf:
                continue
            x1 = int((cx - bw / 2) / self._imgsz * w)
            y1 = int((cy - bh / 2) / self._imgsz * h)
            x2 = int((cx + bw / 2) / self._imgsz * w)
            y2 = int((cy + bh / 2) / self._imgsz * h)
            boxes.append((max(0, x1), max(0, y1), min(w, x2), min(h, y2), conf, cls_id))
        return boxes

    def _image_cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        raw_boxes = self._infer(frame)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()

        for (x1, y1, x2, y2, conf, cls_id) in raw_boxes:
            cls_name = self._names.get(cls_id, 'unknown')
            w = x2 - x1
            h = y2 - y1

            roi_hsv = hsv_frame[y1:y2, x1:x2]
            colour = self._classify_colour(roi_hsv)

            apparent_px = max(w, h)
            dist_est = -1.0
            if apparent_px > 10:
                dist_est = (BALL_DIAMETER_M * FOCAL_LENGTH_PX) / apparent_px

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

            label = f'{colour} {conf:.2f} {dist_est:.2f}m'
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish annotated image as CompressedImage (JPEG)
        ok, enc = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            ann_msg = CompressedImage()
            ann_msg.header = msg.header
            ann_msg.format = 'jpeg'
            ann_msg.data = enc.tobytes()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
