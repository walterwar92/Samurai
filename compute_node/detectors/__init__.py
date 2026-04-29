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
from .frame_sources import FrameSource, MQTTFrameSource


# H264TCPFrameSource — lazy import (требует PyAV)
def _load_h264_source():
    from .frame_sources import H264TCPFrameSource
    return H264TCPFrameSource
from .publishers import (
    DetectionPublisher, MQTTPublisher, HybridPublisher,
    draw_annotations, encode_jpeg,
)

# YoloBackend / ROS2FrameSource / ROS2Publisher — lazy imports
# (требуют опциональных зависимостей: ultralytics, rclpy)
def _load_yolo_backend():
    from .yolo_backend import YoloBackend
    return YoloBackend


def _load_ros2_frame_source():
    from .frame_sources import ROS2FrameSource
    return ROS2FrameSource


def _load_ros2_publisher():
    from .publishers import ROS2Publisher
    return ROS2Publisher


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
    'FrameSource',
    'MQTTFrameSource',
    'DetectionPublisher',
    'MQTTPublisher',
    'HybridPublisher',
    'draw_annotations',
    'encode_jpeg',
]
