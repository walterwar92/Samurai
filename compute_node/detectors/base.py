"""
Detection dataclass + DetectorBackend ABC + DetectionPipeline orchestrator.

Detection — единая структура объекта на всём пайплайне (бэкенд → enrich → publisher).
DetectorBackend — абстракция модели детекции (YOLO, HSV blob, заглушка для тестов).
DetectionPipeline — собирает бэкенд + классификатор цвета + оценщик дистанции + проектор мира.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np


@dataclass
class Detection:
    """Единая структура детекции на всём пайплайне."""
    cls: str = 'object'
    colour: str = 'unknown'
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0
    conf: float = 0.0
    distance: float = -1.0
    dist_method: str = 'none'         # 'mono' | 'ultra' | 'blend' | 'none'
    world_x: Optional[float] = None
    world_y: Optional[float] = None

    def to_dict(self) -> dict:
        """JSON-friendly dict (с обратной совместимостью полей старого API)."""
        d = asdict(self)
        # Старый API ожидает 'class', а не 'cls' (Python keyword)
        d['class'] = d.pop('cls')
        return d

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0


@dataclass
class RobotPose:
    """Снимок позы робота в момент кадра — для проекции мировых координат."""
    x: float = 0.0       # m
    y: float = 0.0       # m
    theta: float = 0.0   # rad
    valid: bool = False


@dataclass
class FrameContext:
    """Контекст одного кадра: BGR + HSV + поза + последнее показание ультразвука."""
    bgr: np.ndarray
    hsv: Optional[np.ndarray] = None
    pose: RobotPose = field(default_factory=RobotPose)
    ultrasonic_m: float = 2.0      # последнее показание ультразвука (м)
    ultrasonic_age_s: float = 999.0  # возраст показания
    timestamp: float = 0.0

    @property
    def width(self) -> int:
        return self.bgr.shape[1]

    @property
    def height(self) -> int:
        return self.bgr.shape[0]

    def ensure_hsv(self) -> np.ndarray:
        """Лениво вычисляет HSV-копию (один раз на кадр)."""
        if self.hsv is None:
            import cv2
            self.hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)
        return self.hsv


class DetectorBackend(ABC):
    """Интерфейс бэкенда детекции (YOLO PyTorch, YOLO ONNX, HSV blob, etc.)."""

    @abstractmethod
    def infer(self, ctx: FrameContext) -> list[Detection]:
        """Принимает FrameContext, возвращает список детекций (без enrichment)."""
        raise NotImplementedError

    @property
    def name(self) -> str:
        return self.__class__.__name__


class DetectionPipeline:
    """
    Orchestrator: бэкенд → enrich (color, distance, world) → готовые Detection.

    Использование:
        pipeline = DetectionPipeline(
            backend=YoloBackend('yolo11n.pt', device='cuda'),
            hsv_classifier=HSVClassifier.from_config(),
            distance_estimator=DistanceEstimator(),
            world_projector=WorldProjector(focal_px=500, fov_rad=math.radians(60)),
        )
        detections = pipeline.process(ctx)
    """

    def __init__(self,
                 backend: DetectorBackend,
                 hsv_classifier=None,
                 distance_estimator=None,
                 world_projector=None,
                 enrich_colour: bool = True):
        self.backend = backend
        self.hsv_classifier = hsv_classifier
        self.distance_estimator = distance_estimator
        self.world_projector = world_projector
        self.enrich_colour = enrich_colour

    def process(self, ctx: FrameContext) -> list[Detection]:
        """Полный пайплайн: detection + color + distance + world coords."""
        detections = self.backend.infer(ctx)

        if self.enrich_colour and self.hsv_classifier is not None:
            hsv = ctx.ensure_hsv()
            for d in detections:
                # Если бэкенд уже посчитал колор (HSV blob detector) — не перезаписываем
                if d.colour in ('unknown', '', None):
                    roi = hsv[d.y:d.y + d.h, d.x:d.x + d.w]
                    d.colour = self.hsv_classifier.classify(roi)

        if self.distance_estimator is not None:
            for d in detections:
                cx_norm = (d.cx - ctx.width / 2.0) / max(1.0, ctx.width / 2.0)
                dist, method = self.distance_estimator.estimate(
                    cls_name=d.cls, bbox_h_px=d.h,
                    cx_norm=cx_norm,
                    ultrasonic_m=ctx.ultrasonic_m,
                    ultrasonic_age_s=ctx.ultrasonic_age_s,
                )
                d.distance = dist
                d.dist_method = method

        if self.world_projector is not None and ctx.pose.valid:
            for d in detections:
                if d.distance > 0:
                    wx, wy = self.world_projector.project(
                        rx=ctx.pose.x, ry=ctx.pose.y, theta=ctx.pose.theta,
                        cx_px=d.cx, dist_m=d.distance,
                        img_width=ctx.width,
                    )
                    d.world_x = round(wx, 3)
                    d.world_y = round(wy, 3)

        return detections
