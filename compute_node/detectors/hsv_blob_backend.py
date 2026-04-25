"""
HSVBlobBackend — детектор круглых цветных объектов без YOLO.

Используется как fallback когда ultralytics не установлен.
Логика взята из object_detector_node.py._detect_hsv():
  - Для каждого цвета строит HSV-маску
  - Морфологические операции (open/close) для очистки шума
  - Поиск контуров → фильтр по area + circularity
  - Выводит Detection с уже заполненным colour (т.к. этот бэкенд знает цвет
    из самой логики детекции)
"""
from __future__ import annotations

import math

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

from .base import Detection, DetectorBackend, FrameContext
from .hsv import HSVClassifier, load_hsv_ranges_from_config


class HSVBlobBackend(DetectorBackend):
    """
    Детектирует цветные круглые блобы (мячи) без нейросетей.

    Параметры:
      min_area, max_area     — размер блоба в пикселях²
      min_circularity        — 4πA/P² (1.0 = идеальный круг)
      class_name             — имя класса для всех найденных блобов (default 'ball')
    """

    def __init__(self,
                 ranges: dict | None = None,
                 min_area: int = 300,
                 max_area: int = 60000,
                 min_circularity: float = 0.45,
                 class_name: str = 'ball'):
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) недоступен — HSVBlobBackend невозможен')
        self._ranges = ranges if ranges is not None else load_hsv_ranges_from_config()
        self._min_area = min_area
        self._max_area = max_area
        self._min_circ = min_circularity
        self._class_name = class_name
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def infer(self, ctx: FrameContext) -> list[Detection]:
        hsv = ctx.ensure_hsv()
        detections: list[Detection] = []

        for colour, ranges in self._ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (self._min_area <= area <= self._max_area):
                    continue
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4.0 * math.pi * area / (perimeter * perimeter)
                if circularity < self._min_circ:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                # Confidence: чем круглее и крупнее — тем выше
                conf = min(0.95, circularity * 0.8 + (area / self._max_area) * 0.2)
                detections.append(Detection(
                    cls=self._class_name,
                    colour=colour,    # бэкенд знает цвет напрямую
                    x=int(x), y=int(y), w=int(w), h=int(h),
                    conf=round(conf, 3),
                ))

        # Крупнейшие первыми (для FSM, который берёт best ball)
        detections.sort(key=lambda d: d.w * d.h, reverse=True)
        return detections


# Удобный конструктор: использует тот же config.yaml hsv_colours что HSVClassifier
def from_config() -> HSVBlobBackend:
    return HSVBlobBackend(ranges=load_hsv_ranges_from_config())
