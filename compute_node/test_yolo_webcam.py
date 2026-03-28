#!/usr/bin/env python3
"""
test_yolo_webcam.py — Тест YOLO + HSV распознавания на веб-камере ноутбука.

Без ROS2 — standalone скрипт. Показывает аннотированное видео в окне OpenCV.
Также поднимает мини-сервер на :5001 с MJPEG + WebSocket для просмотра в браузере.

Использование:
    python compute_node/test_yolo_webcam.py
    python compute_node/test_yolo_webcam.py --model yolo11n.pt --camera 0 --conf 0.35
    python compute_node/test_yolo_webcam.py --no-gui    # без окна OpenCV (headless)

Клавиши (в окне OpenCV):
    q / ESC — выход
    s       — сохранить текущий кадр (snapshot_*.jpg)
    d       — вкл/выкл детекцию
    c       — переключить показ HSV-цвета
"""

import argparse
import json
import os
import sys
import time
import threading

# ─── Потоки: env vars ДОЛЖНЫ быть до импорта numpy/torch/onnx ────
def _pre_setup_threads():
    """Parse --threads early and set env vars BEFORE heavy imports."""
    threads = 0
    for i, a in enumerate(sys.argv):
        if a == '--threads' and i + 1 < len(sys.argv):
            try:
                threads = int(sys.argv[i + 1])
            except ValueError:
                pass
    if threads <= 0:
        threads = max(1, (os.cpu_count() or 4) // 2)
    os.environ['OMP_NUM_THREADS'] = str(threads)
    os.environ['MKL_NUM_THREADS'] = str(threads)
    os.environ['OPENBLAS_NUM_THREADS'] = str(threads)
    os.environ['NUMEXPR_NUM_THREADS'] = str(threads)
    os.environ['VECLIB_MAXIMUM_THREADS'] = str(threads)
    return threads

_THREADS = _pre_setup_threads()

# Теперь безопасно импортировать (numpy/torch увидят env vars)
import cv2
import numpy as np

def _setup_threads(threads: int):
    """Configure multi-core inference AFTER imports."""
    cv2.setNumThreads(threads)
    try:
        import torch
        torch.set_num_threads(threads)
        # interop_threads можно менять только до первого forward pass
        try:
            torch.set_num_interop_threads(max(1, threads // 2))
        except RuntimeError:
            pass  # уже зафиксировано — ок
    except ImportError:
        pass
    print(f'[THREADS] CPU ядер: {os.cpu_count()} | Используется потоков: {threads}')

# ─── HSV colour ranges (из yolo_detector_node.py) ────────────────
COLOUR_RANGES = {
    'red':    [((0, 70, 70), (10, 255, 255)),
               ((160, 70, 70), (180, 255, 255))],
    'orange': [((10, 80, 80), (25, 255, 255))],
    'yellow': [((22, 80, 80), (38, 255, 255))],
    'green':  [((35, 60, 60), (85, 255, 255))],
    'blue':   [((85, 60, 60), (135, 255, 255))],
    'white':  [((0, 0, 180), (180, 40, 255))],
    'black':  [((0, 0, 0), (180, 255, 60))],
}

COLOUR_BGR = {
    'red': (0, 0, 255), 'orange': (0, 140, 255), 'yellow': (0, 255, 255),
    'green': (0, 200, 0), 'blue': (255, 100, 0), 'white': (255, 255, 255),
    'black': (80, 80, 80), 'gray': (160, 160, 160), 'purple': (180, 50, 180),
    'unknown': (128, 128, 128),
}

BALL_DIAMETER_M = 0.04
FOCAL_LENGTH_PX = 500.0

# COCO классы для которых HSV-цвет имеет смысл (мячи, простые объекты)
BALL_CLASSES = {'sports ball', 'frisbee', 'ball', 'apple', 'orange', 'banana',
                'cup', 'bottle', 'vase', 'teddy bear', 'kite'}


def classify_colour_hsv(roi_hsv: np.ndarray) -> str:
    """HSV-классификация для простых однотонных объектов (мячи)."""
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


def classify_colour_dominant(roi_bgr: np.ndarray) -> str:
    """Определение доминантного цвета по центральной области ROI (для любых объектов).
    Берёт центральные 50% пикселей чтобы избежать фона."""
    if roi_bgr.size == 0:
        return ''
    h, w = roi_bgr.shape[:2]
    # Центральная область 50%
    y1, y2 = h // 4, 3 * h // 4
    x1, x2 = w // 4, 3 * w // 4
    center = roi_bgr[max(y1,0):max(y2,1), max(x1,0):max(x2,1)]
    if center.size == 0:
        center = roi_bgr

    # Средний цвет
    mean_bgr = center.mean(axis=(0, 1))
    b, g, r = mean_bgr

    # Конвертируем в HSV для классификации
    hsv_pixel = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0][0]
    h_val, s_val, v_val = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])

    # Ахроматические
    if v_val < 50:
        return 'black'
    if s_val < 30:
        if v_val > 200:
            return 'white'
        return 'gray'

    # Хроматические по hue
    if h_val < 10 or h_val >= 160:
        return 'red'
    if h_val < 25:
        return 'orange'
    if h_val < 38:
        return 'yellow'
    if h_val < 85:
        return 'green'
    if h_val < 135:
        return 'blue'
    if h_val < 160:
        return 'purple'
    return ''


class YoloWebcamTester:
    def __init__(self, model_path: str, camera_id: int, confidence: float,
                 device: str, gui: bool, web_port: int, threads: int = 0):
        self.camera_id = camera_id
        self.conf = confidence
        self.gui = gui
        self.web_port = web_port

        # Setup multi-core (env vars already set in _pre_setup_threads)
        self._threads = threads if threads > 0 else _THREADS
        _setup_threads(self._threads)
        self.detection_enabled = True
        self.show_colour = True
        self.running = True
        self.frame_count = 0
        self.fps = 0.0
        self.last_detections = []
        self._annotated_jpg = None
        self._lock = threading.Lock()

        # Load YOLO
        print(f'[YOLO] Загрузка модели: {model_path} (device={device})')
        from ultralytics import YOLO

        # Check for ONNX version
        onnx_path = model_path.replace('.pt', '.onnx')
        self._use_onnx = False

        if model_path.endswith('.pt') and os.path.exists(onnx_path):
            print(f'[YOLO] Найдена ONNX версия: {onnx_path}')
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] \
                if device == 'cuda' else ['CPUExecutionProvider']
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = self._threads
            sess_opts.inter_op_num_threads = max(1, self._threads // 2)
            sess_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
            self._ort = ort.InferenceSession(onnx_path, sess_options=sess_opts, providers=providers)
            self._input_name = self._ort.get_inputs()[0].name
            self._imgsz = 416
            self._use_onnx = True
            _tmp = YOLO(model_path)
            self._names = _tmp.names
            print(f'[YOLO] ONNX Runtime: {providers[0]}')
        else:
            self._model = YOLO(model_path)
            self._model.to(device)
            self._names = self._model.names
            print(f'[YOLO] PyTorch: {device}')

        print(f'[YOLO] Классы: {list(self._names.values())[:10]}...')
        print(f'[YOLO] Confidence порог: {self.conf}')

    def _infer(self, frame: np.ndarray):
        if self._use_onnx:
            return self._infer_onnx(frame)
        results = self._model(frame, conf=self.conf, verbose=False)
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
            if conf < self.conf:
                continue
            x1 = int((cx - bw / 2) / self._imgsz * w)
            y1 = int((cy - bh / 2) / self._imgsz * h)
            x2 = int((cx + bw / 2) / self._imgsz * w)
            y2 = int((cy + bh / 2) / self._imgsz * h)
            boxes.append((max(0, x1), max(0, y1), min(w, x2), min(h, y2), conf, cls_id))
        return boxes

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        if not self.detection_enabled:
            # Show "PAUSED" overlay
            cv2.putText(frame, 'DETECTION OFF', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return frame

        raw_boxes = self._infer(frame)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()
        detections = []

        for (x1, y1, x2, y2, conf, cls_id) in raw_boxes:
            cls_name = self._names.get(cls_id, 'unknown')
            w = x2 - x1
            h = y2 - y1

            # Выбор метода определения цвета в зависимости от класса
            colour = ''
            if self.show_colour:
                if cls_name in BALL_CLASSES:
                    # Мячи и простые объекты — точный HSV
                    roi_hsv = hsv_frame[y1:y2, x1:x2]
                    colour = classify_colour_hsv(roi_hsv)
                else:
                    # Люди, машины и т.д. — доминантный цвет центра
                    roi_bgr = frame[y1:y2, x1:x2]
                    colour = classify_colour_dominant(roi_bgr)

            apparent_px = max(w, h)
            dist_est = -1.0
            if apparent_px > 10:
                dist_est = (BALL_DIAMETER_M * FOCAL_LENGTH_PX) / apparent_px

            det = {
                'class': cls_name, 'colour': colour,
                'conf': round(conf, 3), 'distance': round(dist_est, 3),
                'x': x1, 'y': y1, 'w': w, 'h': h,
            }
            detections.append(det)

            # Draw bounding box
            box_colour = COLOUR_BGR.get(colour, (0, 255, 0))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_colour, 2)

            # Label
            parts = [cls_name]
            if colour and colour not in ('unknown', ''):
                parts.append(colour)
            parts.append(f'{conf:.2f}')
            if dist_est > 0:
                parts.append(f'{dist_est:.2f}m')
            label = ' '.join(parts)

            # Background for text
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), box_colour, -1)
            cv2.putText(annotated, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        self.last_detections = detections

        # FPS + detection count overlay
        info = f'FPS: {self.fps:.1f} | Objects: {len(detections)}'
        cv2.putText(annotated, info, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return annotated

    def _start_web_server(self):
        """Mini Flask server for browser viewing."""
        try:
            from flask import Flask, Response
        except ImportError:
            print('[WEB] Flask не установлен — веб-просмотр недоступен')
            return

        web = Flask(__name__)

        def gen_mjpeg():
            while self.running:
                with self._lock:
                    jpg = self._annotated_jpg
                if jpg:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                time.sleep(0.033)

        @web.route('/')
        def index():
            return f'''<!DOCTYPE html><html><head><title>YOLO Webcam Test</title></head>
            <body style="background:#111;color:#eee;font-family:sans-serif;text-align:center">
            <h2>YOLO Webcam Test</h2>
            <img src="/video_feed" style="max-width:100%;border:2px solid #333">
            <p>Открыто на порту {self.web_port}</p>
            </body></html>'''

        @web.route('/video_feed')
        def video_feed():
            return Response(gen_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @web.route('/api/detections')
        def api_detections():
            return {'ok': True, 'detections': self.last_detections}

        print(f'[WEB] Сервер запущен: http://localhost:{self.web_port}')
        web.run(host='0.0.0.0', port=self.web_port, threaded=True,
                use_reloader=False)

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f'[ERROR] Не удалось открыть камеру {self.camera_id}')
            sys.exit(1)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f'[CAM] Камера {self.camera_id}: {w}x{h}')
        print(f'[INFO] Клавиши: q=выход, d=вкл/выкл детекцию, c=цвет, s=сохранить кадр')

        # Start web server in background
        web_thread = threading.Thread(target=self._start_web_server, daemon=True)
        web_thread.start()

        fps_counter = 0
        fps_time = time.time()

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print('[WARN] Нет кадра с камеры, повтор...')
                time.sleep(0.1)
                continue

            annotated = self.process_frame(frame)

            # Encode for web server
            ok, jpg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self._lock:
                    self._annotated_jpg = jpg.tobytes()

            # FPS calculation
            fps_counter += 1
            elapsed = time.time() - fps_time
            if elapsed >= 1.0:
                self.fps = fps_counter / elapsed
                fps_counter = 0
                fps_time = time.time()

            self.frame_count += 1

            # Show in GUI window
            if self.gui:
                cv2.imshow('YOLO Webcam Test | q=quit d=detect c=colour s=save', annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):  # q or ESC
                    break
                elif key == ord('d'):
                    self.detection_enabled = not self.detection_enabled
                    print(f'[INFO] Детекция: {"ВКЛ" if self.detection_enabled else "ВЫКЛ"}')
                elif key == ord('c'):
                    self.show_colour = not self.show_colour
                    print(f'[INFO] HSV цвет: {"ВКЛ" if self.show_colour else "ВЫКЛ"}')
                elif key == ord('s'):
                    fname = f'snapshot_{self.frame_count}.jpg'
                    cv2.imwrite(fname, annotated)
                    print(f'[INFO] Сохранён: {fname}')

            # Print detections to console periodically
            if self.frame_count % 30 == 0 and self.last_detections:
                print(f'[DET] {len(self.last_detections)} объектов: '
                      + ', '.join(f'{d["class"]}({d["colour"]}) {d["conf"]:.2f}'
                                  for d in self.last_detections[:5]))

        self.running = False
        cap.release()
        if self.gui:
            cv2.destroyAllWindows()
        print('[INFO] Завершено')


def main():
    parser = argparse.ArgumentParser(description='YOLO Webcam Test — Samurai')
    parser.add_argument('--model', default='yolo11s.pt',
                        help='Путь к модели YOLO (default: yolo11s.pt)')
    parser.add_argument('--camera', type=int, default=0,
                        help='ID камеры (default: 0)')
    parser.add_argument('--conf', type=float, default=0.35,
                        help='Порог уверенности (default: 0.35)')
    parser.add_argument('--device', default='cpu',
                        help='Устройство: cpu / cuda (default: cpu)')
    parser.add_argument('--no-gui', action='store_true',
                        help='Без окна OpenCV (только веб-сервер)')
    parser.add_argument('--port', type=int, default=5001,
                        help='Порт веб-сервера (default: 5001)')
    parser.add_argument('--threads', type=int, default=0,
                        help='Кол-во ядер CPU (default: половина доступных)')
    args = parser.parse_args()

    tester = YoloWebcamTester(
        model_path=args.model,
        camera_id=args.camera,
        confidence=args.conf,
        device=args.device,
        gui=not args.no_gui,
        web_port=args.port,
        threads=args.threads,
    )
    tester.run()


if __name__ == '__main__':
    main()
