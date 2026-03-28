#!/usr/bin/env python3
"""
yolo_detector_mqtt — Standalone MQTT YOLO detector for GPU laptop.

Runs on a separate laptop with a GPU (NVIDIA CUDA).
Connects directly to the MQTT broker on the robot (Raspberry Pi).
No ROS2 or Docker required — pure Python.

Data flow:
    Pi camera → MQTT (samurai/robot1/camera) → this node → YOLO (GPU) →
    → MQTT (samurai/robot1/ball_detection, detections, yolo/annotated)

The CPU laptop (with SLAM/Nav2/dashboard) receives detection results
via the same MQTT broker — no direct connection between laptops needed.

Usage:
    pip install ultralytics paho-mqtt opencv-python-headless numpy PyYAML
    # For GPU:
    pip install onnxruntime-gpu   # or just onnxruntime for CPU fallback

    python yolo_detector_mqtt.py --broker 192.168.1.100
    python yolo_detector_mqtt.py --broker 192.168.1.100 --model yolo11n.pt --device cuda
    python yolo_detector_mqtt.py --broker 192.168.1.100 --model yolo11m.pt --conf 0.35

Environment variables (alternative to CLI args):
    MQTT_BROKER  — broker IP (default: 192.168.1.100)
    MQTT_PORT    — broker port (default: 1883)
    ROBOT_ID     — robot identifier (default: robot1)
"""

import argparse
import json
import logging
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('yolo_gpu')

# ── HSV colour ranges for ball classification ──────────────────────
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

BALL_DIAMETER_M = 0.04
FOCAL_LENGTH_PX = 500.0


class YoloDetectorMqtt:
    """Standalone MQTT-based YOLO detector for GPU laptop."""

    def __init__(self, broker: str, port: int, robot_id: str,
                 model_path: str, confidence: float, device: str,
                 publish_annotated: bool = True,
                 annotated_quality: int = 70):
        self._prefix = f'samurai/{robot_id}'
        self._conf = confidence
        self._publish_annotated = publish_annotated
        self._annotated_quality = annotated_quality
        self._enabled = True
        self._running = True

        # Stats
        self._frame_count = 0
        self._det_count = 0
        self._last_fps_time = time.time()
        self._fps = 0.0

        # ── Load YOLO model ────────────────────────────────────
        self._load_model(model_path, device)

        # ── MQTT client ────────────────────────────────────────
        self._mqtt = mqtt.Client(client_id=f'samurai_yolo_gpu_{robot_id}')
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=30)

        # Will message — other nodes know GPU detector went offline
        self._mqtt.will_set(
            f'{self._prefix}/yolo/status',
            json.dumps({'online': False, 'source': 'gpu_laptop'}),
            qos=1, retain=True)

        log.info('Connecting to MQTT broker %s:%d ...', broker, port)
        self._mqtt.connect_async(broker, port)
        self._mqtt.loop_start()

    def _load_model(self, model_path: str, device: str):
        """Load YOLO model with GPU support (ONNX or PyTorch)."""
        from ultralytics import YOLO

        # Try ONNX first for best performance
        onnx_path = model_path.replace('.pt', '.onnx')

        if model_path.endswith('.pt') and not os.path.exists(onnx_path):
            # Check if .pt exists locally
            if not os.path.exists(model_path):
                # Try common locations
                for base in ['.', os.path.dirname(__file__), os.path.expanduser('~/models')]:
                    candidate = os.path.join(base, model_path)
                    if os.path.exists(candidate):
                        model_path = candidate
                        onnx_path = model_path.replace('.pt', '.onnx')
                        break

            log.info('Exporting %s -> ONNX (imgsz=416) ...', model_path)
            tmp = YOLO(model_path)
            tmp.export(format='onnx', imgsz=416, optimize=True, simplify=True)
            log.info('ONNX export complete')

        if os.path.exists(onnx_path):
            import onnxruntime as ort
            if device == 'cuda':
                providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            else:
                providers = ['CPUExecutionProvider']
            self._ort = ort.InferenceSession(onnx_path, providers=providers)
            self._input_name = self._ort.get_inputs()[0].name
            self._imgsz = 416
            self._use_onnx = True
            # Get class names from .pt if available
            _tmp = YOLO(model_path) if os.path.exists(model_path) else None
            self._names = _tmp.names if _tmp else {0: 'object'}
            active_provider = self._ort.get_providers()[0]
            log.info('ONNX Runtime: %s | Provider: %s', onnx_path, active_provider)
            if device == 'cuda' and 'CUDA' not in active_provider:
                log.warning('CUDA requested but not available! Falling back to CPU.')
        else:
            self._model = YOLO(model_path)
            self._model.to(device)
            self._use_onnx = False
            self._names = self._model.names
            log.info('PyTorch YOLO loaded: %s on %s', model_path, device)

    # ── MQTT callbacks ─────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed (rc=%d)', rc)
            return
        log.info('MQTT connected to broker')
        # Subscribe to camera frames from Pi
        client.subscribe(f'{self._prefix}/camera', qos=0)
        # Subscribe to enable/disable control
        client.subscribe(f'{self._prefix}/detection/enable', qos=1)
        # Announce online status
        client.publish(
            f'{self._prefix}/yolo/status',
            json.dumps({'online': True, 'source': 'gpu_laptop',
                        'ts': time.time()}),
            qos=1, retain=True)
        log.info('Subscribed to %s/camera — waiting for frames ...', self._prefix)

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        suffix = topic[len(self._prefix) + 1:]

        if suffix == 'camera':
            if self._enabled:
                self._process_frame(msg.payload)
            else:
                # Pass through raw frame and empty detections
                self._publish_empty_detections()
        elif suffix == 'detection/enable':
            cmd = msg.payload.decode('utf-8', errors='ignore').strip().lower()
            self._enabled = cmd in ('on', 'true', '1', 'enable')
            log.info('Detection %s', 'ENABLED' if self._enabled else 'DISABLED')

    # ── Frame processing ───────────────────────────────────────
    def _process_frame(self, payload: bytes):
        """Decode JPEG, run YOLO, publish results."""
        np_arr = np.frombuffer(payload, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Run inference
        raw_boxes = self._infer(frame)

        # Classify colours and estimate distance
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy() if self._publish_annotated else None
        all_detections = []

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
            all_detections.append(det)

            # Publish individual detection
            self._mqtt.publish(
                f'{self._prefix}/ball_detection',
                json.dumps(det))

            # Draw on annotated frame
            if annotated is not None:
                label = f'{colour} {conf:.2f} {dist_est:.2f}m'
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish summary (always — empty list = heartbeat)
        self._mqtt.publish(
            f'{self._prefix}/detections',
            json.dumps({'objects': all_detections, 'count': len(all_detections)}))

        # Publish annotated frame
        if annotated is not None:
            ok, enc = cv2.imencode(
                '.jpg', annotated,
                [cv2.IMWRITE_JPEG_QUALITY, self._annotated_quality])
            if ok:
                self._mqtt.publish(
                    f'{self._prefix}/yolo/annotated',
                    enc.tobytes())

        # Update stats
        self._frame_count += 1
        self._det_count += len(all_detections)
        now = time.time()
        dt = now - self._last_fps_time
        if dt >= 5.0:
            self._fps = self._frame_count / dt
            log.info('FPS: %.1f | Frames: %d | Detections: %d',
                     self._fps, self._frame_count, self._det_count)
            self._frame_count = 0
            self._det_count = 0
            self._last_fps_time = now

    def _publish_empty_detections(self):
        """Publish empty detection result (heartbeat when disabled)."""
        self._mqtt.publish(
            f'{self._prefix}/detections',
            json.dumps({'objects': [], 'count': 0}))

    # ── YOLO inference ─────────────────────────────────────────
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

    # ── Colour classification ──────────────────────────────────
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

    # ── Lifecycle ──────────────────────────────────────────────
    def run(self):
        """Block until stopped (Ctrl+C or SIGTERM)."""
        log.info('GPU YOLO detector running. Press Ctrl+C to stop.')
        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        self.stop()

    def stop(self):
        """Clean shutdown."""
        self._running = False
        # Announce offline
        self._mqtt.publish(
            f'{self._prefix}/yolo/status',
            json.dumps({'online': False, 'source': 'gpu_laptop'}),
            qos=1, retain=True)
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        log.info('GPU YOLO detector stopped.')


def main():
    parser = argparse.ArgumentParser(
        description='Samurai GPU YOLO Detector (standalone MQTT)')
    parser.add_argument('--broker', default=os.environ.get('MQTT_BROKER', '192.168.1.100'),
                        help='MQTT broker IP (Pi address)')
    parser.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')),
                        help='MQTT broker port')
    parser.add_argument('--robot-id', default=os.environ.get('ROBOT_ID', 'robot1'),
                        help='Robot identifier')
    parser.add_argument('--model', default='yolo11n.pt',
                        help='YOLO model path (.pt or .onnx)')
    parser.add_argument('--conf', type=float, default=0.40,
                        help='Detection confidence threshold')
    parser.add_argument('--device', default='cuda',
                        help='Inference device: cuda or cpu')
    parser.add_argument('--no-annotated', action='store_true',
                        help='Disable publishing annotated frames (saves bandwidth)')
    parser.add_argument('--quality', type=int, default=70,
                        help='Annotated JPEG quality (0-100)')

    args = parser.parse_args()

    # Handle SIGTERM gracefully (Docker, systemd)
    detector = YoloDetectorMqtt(
        broker=args.broker,
        port=args.port,
        robot_id=args.robot_id,
        model_path=args.model,
        confidence=args.conf,
        device=args.device,
        publish_annotated=not args.no_annotated,
        annotated_quality=args.quality,
    )

    signal.signal(signal.SIGTERM, lambda *_: detector.stop())
    detector.run()


if __name__ == '__main__':
    main()
