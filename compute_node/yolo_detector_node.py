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

try:
    import yaml
    _CFG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')
    if os.path.exists(_CFG_PATH):
        with open(_CFG_PATH) as _f:
            _YAML = yaml.safe_load(_f) or {}
    else:
        _YAML = {}
except Exception:
    _YAML = {}

# HSV colour ranges — loaded from config.yaml with hardcoded fallback
_DEFAULT_HSV = {
    'red':    [((0, 70, 70), (10, 255, 255)),
               ((160, 70, 70), (180, 255, 255))],
    'orange': [((10, 80, 80), (25, 255, 255))],
    'yellow': [((22, 80, 80), (38, 255, 255))],
    'green':  [((35, 60, 60), (85, 255, 255))],
    'blue':   [((85, 60, 60), (135, 255, 255))],
    'white':  [((0, 0, 180), (180, 40, 255))],
    'black':  [((0, 0, 0), (180, 255, 60))],
}

def _load_hsv_from_config():
    """Parse hsv_colours from config.yaml into COLOUR_RANGES format."""
    raw = _YAML.get('hsv_colours')
    if not raw or not isinstance(raw, dict):
        return _DEFAULT_HSV
    result = {}
    for colour, range_list in raw.items():
        parsed = []
        for r in range_list:
            if isinstance(r, list) and len(r) == 6:
                parsed.append((tuple(r[:3]), tuple(r[3:])))
        if parsed:
            result[colour] = parsed
    return result if result else _DEFAULT_HSV

COLOUR_RANGES = _load_hsv_from_config()

# Known ball diameter for distance estimation (metres)
BALL_DIAMETER_M = 0.04
FOCAL_LENGTH_PX = 500.0  # approximate for 640px wide CSI camera

# COCO classes where HSV ball-colour classification makes sense
BALL_CLASSES = {'sports ball', 'frisbee', 'ball', 'apple', 'orange', 'banana',
                'cup', 'bottle', 'vase', 'teddy bear', 'kite'}


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model', 'yolo11s.pt')
        self.declare_parameter('confidence', 0.40)
        self.declare_parameter('device', 'cpu')

        # Detection enabled flag — controlled via MQTT from dashboard
        self._enabled = True
        # Frame drop: skip incoming frames if inference is still running
        self._processing = False

        model_path = self.get_parameter('model').value
        self._conf = self.get_parameter('confidence').value
        device = self.get_parameter('device').value

        onnx_path = model_path.replace('.pt', '.onnx')
        if model_path.endswith('.pt') and not os.path.exists(onnx_path):
            self.get_logger().info(f'Exporting {model_path} → {onnx_path} (ONNX, imgsz=416)...')
            tmp = YOLO(model_path)
            tmp.export(format='onnx', imgsz=416, optimize=True, simplify=True)
            self.get_logger().info('ONNX export complete')

        if os.path.exists(onnx_path):
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] \
                        if device == 'cuda' else ['CPUExecutionProvider']
            n_threads = max(1, os.cpu_count() // 2) if os.cpu_count() else 4
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = n_threads
            sess_opts.inter_op_num_threads = max(1, n_threads // 2)
            sess_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
            self._ort = ort.InferenceSession(onnx_path, sess_options=sess_opts, providers=providers)
            self._input_name = self._ort.get_inputs()[0].name
            self._imgsz = 416
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
        self._dets_pub = self.create_publisher(String, '/yolo/detections', 10)
        self._ann_pub = self.create_publisher(CompressedImage, '/yolo/annotated/compressed', cam_qos)

        # Subscribe to detection enable/disable topic
        self.create_subscription(String, '/yolo/enable', self._enable_cb, 10)

    def _enable_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('true', '1', 'on', 'enable'):
            self._enabled = True
            self.get_logger().info('Detection ENABLED')
        elif cmd in ('false', '0', 'off', 'disable'):
            self._enabled = False
            self.get_logger().info('Detection DISABLED')

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
        # Drop frame if previous inference still running (avoid queue buildup)
        if self._processing:
            return
        self._processing = True
        try:
            self._process_frame(msg)
        finally:
            self._processing = False

    def _process_frame(self, msg: CompressedImage):
        np_arr = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # When disabled: pass through raw frame without detection
        if not self._enabled:
            ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                ann_msg = CompressedImage()
                ann_msg.header = msg.header
                ann_msg.format = 'jpeg'
                ann_msg.data = enc.tobytes()
                self._ann_pub.publish(ann_msg)
            # Publish empty detections (heartbeat)
            dets_msg = String()
            dets_msg.data = json.dumps({'objects': [], 'count': 0})
            self._dets_pub.publish(dets_msg)
            return

        raw_boxes = self._infer(frame)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()
        all_detections = []

        for (x1, y1, x2, y2, conf, cls_id) in raw_boxes:
            cls_name = self._names.get(cls_id, 'unknown')
            w = x2 - x1
            h = y2 - y1

            # Ball-like objects: precise HSV; others: dominant centre colour
            if cls_name in BALL_CLASSES:
                roi_hsv = hsv_frame[y1:y2, x1:x2]
                colour = self._classify_colour_hsv(roi_hsv)
            else:
                colour = self._classify_colour_dominant(frame[y1:y2, x1:x2])

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
            all_detections.append(det)

            det_msg = String()
            det_msg.data = json.dumps(det)
            self._det_pub.publish(det_msg)

            label = f'{colour} {conf:.2f} {dist_est:.2f}m'
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish summary detections (always — even empty list serves as heartbeat)
        dets_msg = String()
        dets_msg.data = json.dumps({'objects': all_detections, 'count': len(all_detections)})
        self._dets_pub.publish(dets_msg)

        # Publish annotated image as CompressedImage (JPEG)
        ok, enc = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            ann_msg = CompressedImage()
            ann_msg.header = msg.header
            ann_msg.format = 'jpeg'
            ann_msg.data = enc.tobytes()
            self._ann_pub.publish(ann_msg)

    def _classify_colour_hsv(self, roi_hsv: np.ndarray) -> str:
        """HSV classification for simple solid-colour objects (balls)."""
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
            if ratio > best_ratio and ratio > 0.10:
                best_ratio = ratio
                best_colour = colour
        return best_colour

    @staticmethod
    def _classify_colour_dominant(roi_bgr: np.ndarray) -> str:
        """Dominant colour from center 50% of ROI (for complex objects)."""
        if roi_bgr.size == 0:
            return ''
        h, w = roi_bgr.shape[:2]
        cy1, cy2 = h // 4, 3 * h // 4
        cx1, cx2 = w // 4, 3 * w // 4
        center = roi_bgr[max(cy1,0):max(cy2,1), max(cx1,0):max(cx2,1)]
        if center.size == 0:
            center = roi_bgr
        mean_bgr = center.mean(axis=(0, 1))
        b, g, r = mean_bgr
        hsv_px = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0][0]
        hue, sat, val = int(hsv_px[0]), int(hsv_px[1]), int(hsv_px[2])
        if val < 50:
            return 'black'
        if sat < 30:
            return 'white' if val > 200 else 'gray'
        if hue < 10 or hue >= 160:
            return 'red'
        if hue < 25:
            return 'orange'
        if hue < 38:
            return 'yellow'
        if hue < 85:
            return 'green'
        if hue < 135:
            return 'blue'
        if hue < 160:
            return 'purple'
        return ''


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
