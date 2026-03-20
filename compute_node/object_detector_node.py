#!/usr/bin/env python3
"""
object_detector_node — Standalone MQTT-based object detector.

Runs on the compute laptop — NO ROS2 required.

Subscribes:
    samurai/{robot_id}/camera   — binary JPEG frames (from camera_node)
    samurai/{robot_id}/range    — ultrasonic distance {range, ts}
    samurai/{robot_id}/odom     — robot pose {x, y, theta}

Publishes:
    samurai/{robot_id}/detections      — все объекты + дистанция + мировые координаты
    samurai/{robot_id}/ball_detection  — лучшая детекция мяча (обратная совместимость с FSM)
    samurai/{robot_id}/detected_frame  — аннотированный JPEG (для дашборда)

Конвейер детекции:
    1. YOLO (yolov8n / yolo11n) если модель доступна, иначе HSV blob detection
    2. Классификация цвета по HSV для каждого bbox
    3. Оценка дистанции:
         — Монокулярная: (known_height_m × focal_px) / bbox_height_px
         — Ультразвуковая: если объект по центру кадра И показание свежее
         — Комбинированная: взвешенное среднее обоих методов
    4. Мировые координаты: поза робота + дистанция + угол камеры

Использование:
    python compute_node/object_detector_node.py --broker 192.168.1.50
    python compute_node/object_detector_node.py --broker raspberrypi.local --robot-id robot1
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Параметры камеры ────────────────────────────────────────────────────────
CAM_WIDTH        = 640
CAM_HEIGHT       = 480
FOCAL_LENGTH_PX  = 500.0   # приблизительно для CSI-камеры 640px
CAM_FOV_DEG      = 60.0    # горизонтальный FOV
CAM_FOV_RAD      = math.radians(CAM_FOV_DEG)

# ── Реальные размеры объектов (высота, м) для монокулярной глубины ──────────
OBJECT_SIZES = {
    'sports ball':  0.04,
    'ball':         0.04,
    'tennis ball':  0.067,
    'orange':       0.08,
    'apple':        0.08,
    'bottle':       0.25,
    'cup':          0.12,
    'person':       1.70,
    'chair':        0.90,
    'cat':          0.30,
    'dog':          0.40,
    'car':          1.50,
    'bicycle':      1.10,
    'object':       0.04,   # YOLO generic class → считаем мячом
}
DEFAULT_OBJECT_SIZE_M = 0.10  # для неизвестных классов

# ── HSV диапазоны цветов ────────────────────────────────────────────────────
COLOUR_RANGES = {
    'red':    [((0,   100, 100), (10,  255, 255)),
               ((160, 100, 100), (180, 255, 255))],
    'orange': [((10,  100, 100), (25,  255, 255))],
    'yellow': [((25,  100, 100), (35,  255, 255))],
    'green':  [((35,  100, 100), (85,  255, 255))],
    'blue':   [((85,  100, 100), (130, 255, 255))],
    'white':  [((0,   0,   200), (180, 30,  255))],
    'black':  [((0,   0,   0),   (180, 255, 50))],
}

# ── Параметры blob-детекции (HSV fallback) ──────────────────────────────────
MIN_BLOB_AREA    = 300      # пикс²
MAX_BLOB_AREA    = 60000    # пикс²
MIN_CIRCULARITY  = 0.45     # для шарообразных объектов

# ── Параметры оценки дистанции ──────────────────────────────────────────────
CONF_THRESHOLD          = 0.40
ULTRASONIC_MAX_AGE_S    = 0.5   # секунд — макс возраст показания ультразвука
ULTRASONIC_CENTER_THR   = 0.25  # |cx_norm| < порог → объект по центру, можно использовать УЗ
ULTRASONIC_WEIGHT       = 0.65  # доля ультразвука при смешивании

# ── Цвета аннотации ─────────────────────────────────────────────────────────
COLOUR_BGR = {
    'red':     (0,   0,   220),
    'orange':  (0,   128, 255),
    'yellow':  (0,   220, 220),
    'green':   (0,   200, 0),
    'blue':    (220, 80,  0),
    'white':   (220, 220, 220),
    'black':   (60,  60,  60),
    'unknown': (0,   255, 0),
}


class ObjectDetectorNode(MqttNode):
    """Standalone MQTT object detector (YOLO + HSV colour + distance estimation)."""

    def __init__(self, **kwargs):
        super().__init__('object_detector', **kwargs)

        # Состояние робота
        self._robot_x     = 0.0
        self._robot_y     = 0.0
        self._robot_theta = 0.0
        self._pose_valid  = False

        # Состояние ультразвука
        self._range_m  = 2.0
        self._range_ts = 0.0

        # YOLO модель
        self._yolo       = None
        self._yolo_names = {}
        self._use_yolo   = False
        self._load_yolo()

        # Подписки
        self.subscribe('camera', self._camera_cb, parse_json=False)
        self.subscribe('range',  self._range_cb)
        self.subscribe('odom',   self._odom_cb)

        mode = 'YOLO' if self._use_yolo else 'HSV blob (без YOLO)'
        self.log_info('Детектор объектов запущен — режим: %s', mode)

    # ── Загрузка YOLO ───────────────────────────────────────────────────────
    def _load_yolo(self):
        """Загружает YOLO модель; при отсутствии переходит в HSV-режим."""
        candidates = [
            'yolo11n.pt',
            'yolov8n.pt',
            os.path.expanduser('~/models/yolo11n.pt'),
            os.path.expanduser('~/models/yolov8n.pt'),
            os.path.join(os.path.dirname(__file__), '..', 'yolov8n.pt'),
        ]
        try:
            from ultralytics import YOLO  # noqa: PLC0415
            for path in candidates:
                if os.path.exists(path):
                    self._yolo = YOLO(path)
                    self._yolo_names = self._yolo.names
                    self._use_yolo = True
                    self.log_info('YOLO загружен: %s', os.path.basename(path))
                    return
            # Попытка автоматической загрузки yolov8n (маленькая, быстрая)
            self.log_info('Локальная YOLO модель не найдена — скачиваю yolov8n...')
            self._yolo = YOLO('yolov8n.pt')
            self._yolo_names = self._yolo.names
            self._use_yolo = True
            self.log_info('yolov8n скачан и загружен')
        except Exception as exc:
            self.log_warn('YOLO недоступен (%s) — используется HSV blob детекция', exc)
            self._use_yolo = False

    # ── MQTT callbacks ──────────────────────────────────────────────────────
    def _odom_cb(self, topic, data):
        if isinstance(data, dict):
            self._robot_x     = float(data.get('x', 0.0))
            self._robot_y     = float(data.get('y', 0.0))
            self._robot_theta = float(data.get('theta', 0.0))
            self._pose_valid  = True

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = float(data.get('range', 2.0))
            if 0.02 <= r <= 2.0:
                self._range_m  = r
                self._range_ts = time.monotonic()

    def _camera_cb(self, topic, data):
        """Обрабатывает входящий JPEG-кадр с камеры."""
        if not isinstance(data, (bytes, bytearray)):
            return
        np_arr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Детекция
        if self._use_yolo:
            detections = self._detect_yolo(frame)
        else:
            detections = self._detect_hsv(frame)

        # Снэпшот состояния (thread-safe копия)
        robot_x     = self._robot_x
        robot_y     = self._robot_y
        robot_theta = self._robot_theta
        range_m     = self._range_m
        range_age   = time.monotonic() - self._range_ts
        uz_valid    = range_age < ULTRASONIC_MAX_AGE_S

        annotated = frame.copy()

        for det in detections:
            cx = det['x'] + det['w'] / 2.0
            cy = det['y'] + det['h'] / 2.0
            cx_norm = (cx - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0)   # [-1, 1]

            dist, method = self._estimate_distance(
                cls_name=det['class'],
                bbox_h_px=det['h'],
                cx_norm=cx_norm,
                ultrasonic_m=range_m,
                uz_valid=uz_valid,
            )
            det['distance']    = round(dist, 3)
            det['dist_method'] = method

            # Мировые координаты
            if self._pose_valid and dist > 0:
                wx, wy = self._project_world(robot_x, robot_y, robot_theta, cx, dist)
                det['world_x'] = round(wx, 3)
                det['world_y'] = round(wy, 3)
            else:
                det['world_x'] = None
                det['world_y'] = None

            # Аннотация кадра
            x1, y1 = det['x'], det['y']
            x2, y2 = x1 + det['w'], y1 + det['h']
            bgr = COLOUR_BGR.get(det['colour'], (0, 255, 0))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), bgr, 2)
            lbl = f"{det['colour']} {det['class']} {dist:.2f}m [{method}]"
            cv2.putText(annotated, lbl, (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, bgr, 1)

        # Публикация всех детекций
        self.publish('detections', {
            'objects': detections,
            'count':   len(detections),
            'ts':      time.time(),
        })

        # Обратная совместимость: лучший мяч → ball_detection (для FSM)
        ball = self._best_ball(detections)
        if ball:
            self.publish('ball_detection', ball)

        # Аннотированный кадр для дашборда
        ok, enc = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.publish('detected_frame', bytes(enc))

    # ── Методы детекции ─────────────────────────────────────────────────────
    def _detect_yolo(self, frame: np.ndarray) -> list:
        """YOLO inference + HSV классификация цвета."""
        results = self._yolo(frame, conf=CONF_THRESHOLD, verbose=False)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                w, h   = x2 - x1, y2 - y1
                if w < 5 or h < 5:
                    continue
                conf     = float(box.conf[0])
                cls_id   = int(box.cls[0])
                cls_name = self._yolo_names.get(cls_id, 'object')
                colour   = self._classify_colour(hsv[y1:y2, x1:x2])
                detections.append({
                    'class':  cls_name,
                    'colour': colour,
                    'x': x1, 'y': y1, 'w': w, 'h': h,
                    'conf': round(conf, 3),
                })
        return detections

    def _detect_hsv(self, frame: np.ndarray) -> list:
        """HSV blob-детекция — fallback без YOLO."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        detections = []

        for colour, ranges in COLOUR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (MIN_BLOB_AREA <= area <= MAX_BLOB_AREA):
                    continue
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4.0 * math.pi * area / (perimeter * perimeter)
                if circularity < MIN_CIRCULARITY:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                # Confidence: чем круглее и крупнее — тем выше
                conf = min(0.95, circularity * 0.8 + (area / MAX_BLOB_AREA) * 0.2)
                detections.append({
                    'class':  'ball',
                    'colour': colour,
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'conf': round(conf, 3),
                })

        # Крупнейшие первыми
        detections.sort(key=lambda d: d['w'] * d['h'], reverse=True)
        return detections

    # ── Оценка дистанции ────────────────────────────────────────────────────
    def _estimate_distance(self, cls_name: str, bbox_h_px: int,
                           cx_norm: float, ultrasonic_m: float,
                           uz_valid: bool) -> tuple:
        """
        Возвращает (distance_m, method_str).

        Методы:
          'mono'    — только монокулярная глубина
          'ultra'   — только ультразвук (объект точно по центру)
          'blend'   — взвешенное среднее mono + ultra
        """
        if bbox_h_px <= 5:
            return -1.0, 'none'

        # Монокулярная: distance = (known_height_m * focal_px) / bbox_height_px
        obj_h = OBJECT_SIZES.get(cls_name, DEFAULT_OBJECT_SIZE_M)
        mono  = (obj_h * FOCAL_LENGTH_PX) / bbox_h_px

        # Ультразвук если объект близко к центру кадра и показание свежее
        if uz_valid and abs(cx_norm) < ULTRASONIC_CENTER_THR and ultrasonic_m < 1.9:
            if abs(cx_norm) < 0.10:
                # Очень по центру — доверяем ультразвуку больше
                dist = 0.80 * ultrasonic_m + 0.20 * mono
                return round(dist, 3), 'ultra'
            dist = ULTRASONIC_WEIGHT * ultrasonic_m + (1 - ULTRASONIC_WEIGHT) * mono
            return round(dist, 3), 'blend'

        return round(mono, 3), 'mono'

    # ── Проекция в мировые координаты ──────────────────────────────────────
    def _project_world(self, rx: float, ry: float, theta: float,
                       cx_px: float, dist_m: float) -> tuple:
        """
        Переводит пиксельный центр объекта + дистанцию в мировые (x, y).

        cx_px  — пиксельная координата X центра bbox
        dist_m — оценённая дистанция до объекта
        """
        # Горизонтальный угол объекта относительно оси камеры
        angle_offset = ((cx_px - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0)) * (CAM_FOV_RAD / 2.0)
        obj_angle = theta + angle_offset
        wx = rx + dist_m * math.cos(obj_angle)
        wy = ry + dist_m * math.sin(obj_angle)
        return wx, wy

    # ── Вспомогательные ─────────────────────────────────────────────────────
    def _classify_colour(self, roi_hsv: np.ndarray) -> str:
        """Определяет доминирующий цвет объекта по HSV-маске."""
        if roi_hsv.size == 0:
            return 'unknown'
        total_px = roi_hsv.shape[0] * roi_hsv.shape[1]
        if total_px == 0:
            return 'unknown'
        best, best_ratio = 'unknown', 0.0
        for colour, ranges in COLOUR_RANGES.items():
            mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(roi_hsv, np.array(lo), np.array(hi))
            ratio = np.count_nonzero(mask) / total_px
            if ratio > best_ratio and ratio > 0.15:
                best_ratio, best = ratio, colour
        return best

    def _best_ball(self, detections: list) -> dict | None:
        """Лучшая детекция мяча для обратной совместимости с FSM."""
        if not detections:
            return None
        ball_classes = ('ball', 'sports ball', 'object')
        balls = [d for d in detections if d['class'] in ball_classes]
        pool  = balls if balls else detections
        return max(pool, key=lambda d: d['conf'])


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Samurai — Standalone MQTT Object Detector (no ROS2)')
    parser.add_argument('--broker',   default='127.0.0.1',
                        help='IP адрес MQTT брокера (Raspberry Pi)')
    parser.add_argument('--port',     type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()

    node = ObjectDetectorNode(
        broker=args.broker,
        port=args.port,
        robot_id=args.robot_id,
    )
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
