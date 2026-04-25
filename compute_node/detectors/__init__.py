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

from .base import Detection, DetectionPipeline, DetectorBackend
from .hsv import HSVClassifier, load_hsv_ranges_from_config
from .distance import DistanceEstimator
from .world import WorldProjector

__all__ = [
    'Detection',
    'DetectionPipeline',
    'DetectorBackend',
    'HSVClassifier',
    'load_hsv_ranges_from_config',
    'DistanceEstimator',
    'WorldProjector',
]
