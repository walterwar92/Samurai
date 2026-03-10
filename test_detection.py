#!/usr/bin/env python3
"""
test_detection.py — Standalone object detection test using laptop webcam.

No ROS2 required. Uses the same YOLOv8n + HSV colour logic as yolo_detector_node.py.

Usage:
    python test_detection.py [--camera 0] [--model yolov8n.pt] [--conf 0.45] [--width 640]

Controls:
    q / Esc — quit
    s       — save current frame to disk
    c       — toggle colour classification overlay
    +/-     — increase/decrease confidence threshold
"""

import argparse
import time
import os
import sys
import cv2
import numpy as np

# ── HSV colour ranges (same as yolo_detector_node.py) ────────────────────────
COLOUR_RANGES = {
    'red':    [((0,  100, 100), (10,  255, 255)),
               ((160, 100, 100), (180, 255, 255))],
    'orange': [((10,  100, 100), (25,  255, 255))],
    'yellow': [((25,  100, 100), (35,  255, 255))],
    'green':  [((35,  100, 100), (85,  255, 255))],
    'blue':   [((85,  100, 100), (130, 255, 255))],
    'white':  [((0,   0,   200), (180, 30,  255))],
    'black':  [((0,   0,   0),  (180, 255,  50))],
}

COLOUR_BGR = {
    'red':    (0,   0,   255),
    'orange': (0,   140, 255),
    'yellow': (0,   220, 255),
    'green':  (0,   200,  50),
    'blue':   (255, 100,   0),
    'white':  (220, 220, 220),
    'black':  (60,   60,  60),
    'unknown':(0,   255,   0),
}

BALL_DIAMETER_M  = 0.04   # metres (same as node)
FOCAL_LENGTH_PX  = 500.0  # approximate for 640 px wide camera


# ── Colour classifier ─────────────────────────────────────────────────────────
def classify_colour(roi_bgr: np.ndarray) -> str:
    if roi_bgr.size == 0:
        return 'unknown'
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total_px = roi_hsv.shape[0] * roi_hsv.shape[1]
    best, best_ratio = 'unknown', 0.0
    for colour, ranges in COLOUR_RANGES.items():
        mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(roi_hsv, np.array(lo), np.array(hi))
        ratio = np.count_nonzero(mask) / total_px
        if ratio > best_ratio and ratio > 0.15:
            best_ratio = ratio
            best = colour
    return best


# ── YOLO inference wrapper ────────────────────────────────────────────────────
def load_model(model_path: str, device: str):
    try:
        from ultralytics import YOLO
    except ImportError:
        print('[ERROR] ultralytics not installed. Run: pip install ultralytics')
        sys.exit(1)

    # Try ONNX for faster CPU inference (same logic as yolo_detector_node.py)
    onnx_path = model_path.replace('.pt', '.onnx')
    if model_path.endswith('.pt') and not os.path.exists(onnx_path):
        print(f'[INFO] Exporting {model_path} → {onnx_path} (ONNX, imgsz=320)...')
        tmp = YOLO(model_path)
        tmp.export(format='onnx', imgsz=320, optimize=True, simplify=True)
        print('[INFO] ONNX export done')

    if os.path.exists(onnx_path):
        try:
            import onnxruntime as ort
            providers = (['CUDAExecutionProvider', 'CPUExecutionProvider']
                         if device == 'cuda' else ['CPUExecutionProvider'])
            session = ort.InferenceSession(onnx_path, providers=providers)
            pt_model = YOLO(model_path) if os.path.exists(model_path) else None
            names = pt_model.names if pt_model else {0: 'object'}
            print(f'[INFO] ONNX Runtime: {onnx_path} ({providers[0]})')
            return ('onnx', session, names)
        except ImportError:
            print('[WARN] onnxruntime not installed — falling back to PyTorch')

    model = YOLO(model_path)
    model.to(device)
    print(f'[INFO] YOLO (PyTorch): {model_path} on {device}')
    return ('pt', model, model.names)


def infer(backend, frame: np.ndarray, conf: float, imgsz: int = 320):
    kind, obj, names = backend
    h, w = frame.shape[:2]
    boxes = []

    if kind == 'onnx':
        img = cv2.resize(frame, (imgsz, imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[np.newaxis]
        outputs = obj.run(None, {obj.get_inputs()[0].name: img})[0]
        preds = outputs[0].T
        for pred in preds:
            cx, cy, bw, bh = pred[:4]
            scores = pred[4:]
            cls_id = int(np.argmax(scores))
            score = float(scores[cls_id])
            if score < conf:
                continue
            x1 = int(max(0, (cx - bw / 2) / imgsz * w))
            y1 = int(max(0, (cy - bh / 2) / imgsz * h))
            x2 = int(min(w, (cx + bw / 2) / imgsz * w))
            y2 = int(min(h, (cy + bh / 2) / imgsz * h))
            boxes.append((x1, y1, x2, y2, score, cls_id))
    else:
        results = obj(frame, conf=conf, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                boxes.append((x1, y1, x2, y2, float(box.conf[0]), int(box.cls[0])))

    return boxes, names


# ── Drawing helpers ───────────────────────────────────────────────────────────
def draw_detections(frame: np.ndarray, boxes, names, show_colour: bool):
    annotated = frame.copy()
    for (x1, y1, x2, y2, conf, cls_id) in boxes:
        cls_name = names.get(cls_id, 'unknown')
        w_box = x2 - x1
        h_box = y2 - y1

        colour = 'unknown'
        if show_colour:
            roi = frame[y1:y2, x1:x2]
            colour = classify_colour(roi)

        apparent_px = max(w_box, h_box)
        dist = (BALL_DIAMETER_M * FOCAL_LENGTH_PX) / apparent_px if apparent_px > 10 else -1.0

        box_color = COLOUR_BGR.get(colour, (0, 255, 0))

        cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)

        parts = [f'{cls_name}', f'{conf:.2f}']
        if show_colour and colour != 'unknown':
            parts.append(colour)
        if dist > 0:
            parts.append(f'{dist:.2f}m')
        label = ' | '.join(parts)

        (lw, lh), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_y = max(y1 - 4, lh + 4)
        cv2.rectangle(annotated, (x1, label_y - lh - 4), (x1 + lw + 4, label_y + baseline), box_color, -1)
        cv2.putText(annotated, label, (x1 + 2, label_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    return annotated


def draw_hud(frame: np.ndarray, fps: float, conf: float, show_colour: bool,
             n_det: int, model_path: str):
    h, w = frame.shape[:2]
    lines = [
        f'Model: {os.path.basename(model_path)}',
        f'FPS:   {fps:.1f}',
        f'Conf:  {conf:.2f}  (+/- to adjust)',
        f'Colour: {"ON" if show_colour else "OFF"}  (c)',
        f'Dets:  {n_det}',
        'q/Esc — quit   s — save frame',
    ]
    y = 18
    for line in lines:
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 255), 1, cv2.LINE_AA)
        y += 18


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Laptop camera object detection test')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera index (default 0 = front webcam)')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='YOLO model file (default: yolov8n.pt)')
    parser.add_argument('--conf', type=float, default=0.45,
                        help='Detection confidence threshold (default 0.45)')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'cuda'],
                        help='Inference device (default: cpu)')
    parser.add_argument('--width', type=int, default=640,
                        help='Camera capture width (default 640)')
    parser.add_argument('--height', type=int, default=480,
                        help='Camera capture height (default 480)')
    args = parser.parse_args()

    # ── Open camera ──────────────────────────────────────────────────────────
    print(f'[INFO] Opening camera {args.camera} ({args.width}x{args.height})...')
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f'[ERROR] Cannot open camera {args.camera}')
        sys.exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'[INFO] Camera opened: {actual_w}x{actual_h}')

    # ── Load model ────────────────────────────────────────────────────────────
    backend = load_model(args.model, args.device)

    # ── State ─────────────────────────────────────────────────────────────────
    conf        = args.conf
    show_colour = True
    fps         = 0.0
    frame_count = 0
    t_fps       = time.time()
    save_idx    = 0

    print('[INFO] Starting detection loop. Press q or Esc to quit.')
    cv2.namedWindow('Samurai — Object Detection Test', cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('[WARN] Frame grab failed — retrying...')
            time.sleep(0.05)
            continue

        # ── Inference ────────────────────────────────────────────────────────
        t0 = time.perf_counter()
        boxes, names = infer(backend, frame, conf)
        infer_ms = (time.perf_counter() - t0) * 1000

        # ── FPS ──────────────────────────────────────────────────────────────
        frame_count += 1
        elapsed = time.time() - t_fps
        if elapsed >= 0.5:
            fps = frame_count / elapsed
            frame_count = 0
            t_fps = time.time()

        # ── Draw ─────────────────────────────────────────────────────────────
        annotated = draw_detections(frame, boxes, names, show_colour)
        draw_hud(annotated, fps, conf, show_colour, len(boxes), args.model)

        # Inference time in corner
        cv2.putText(annotated, f'infer {infer_ms:.0f}ms',
                    (actual_w - 110, actual_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        cv2.imshow('Samurai — Object Detection Test', annotated)

        # ── Keys ─────────────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):          # q or Esc
            break
        elif key == ord('s'):              # save frame
            fname = f'capture_{save_idx:04d}.jpg'
            cv2.imwrite(fname, annotated)
            print(f'[INFO] Saved: {fname}')
            save_idx += 1
        elif key == ord('c'):              # toggle colour
            show_colour = not show_colour
            print(f'[INFO] Colour classification: {"ON" if show_colour else "OFF"}')
        elif key == ord('+') or key == ord('='):
            conf = min(conf + 0.05, 0.95)
            print(f'[INFO] Confidence: {conf:.2f}')
        elif key == ord('-'):
            conf = max(conf - 0.05, 0.05)
            print(f'[INFO] Confidence: {conf:.2f}')

    cap.release()
    cv2.destroyAllWindows()
    print('[INFO] Done.')


if __name__ == '__main__':
    main()
