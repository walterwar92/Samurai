"""
compute_node.detectors — единый пакет компонентов детекции объектов.

Архитектура:
    FrameSource → Backend (YOLO/HSV) → enrich (HSV color, distance, world) → Publisher

Использование (high-level):
    from compute_node.detectors import (
        Detection, DetectionPipeline,
        YoloBackend, HSVBlobBackend,
        HSVClassifier, DistanceEstimator, WorldProjector,
        MQTTFrameSource, ROS2FrameSource,
        MQTTPublisher, ROS2Publisher,
    )

CLI entry-point: compute_node/detector.py (в корне compute_node/, не в этом пакете).
"""

from .base import Detection, DetectionPipeline, DetectorBackend, FrameContext, RobotPose
from .hsv import HSVClassifier, load_hsv_ranges_from_config
from .hsv_blob_backend import HSVBlobBackend
from .distance import DistanceEstimator
from .world import WorldProjector

# YoloBackend импортируется лениво (нужен ultralytics — не всегда установлен)
def _load_yolo_backend():
    from .yolo_backend import YoloBackend
    return YoloBackend


__all__ = [
    'Detection',
    'DetectionPipeline',
    'DetectorBackend',
    'FrameContext',
    'RobotPose',
    'HSVClassifier',
    'load_hsv_ranges_from_config',
    'HSVBlobBackend',
    'DistanceEstimator',
    'WorldProjector',
]
