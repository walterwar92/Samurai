"""
Publisher — куда публиковать результаты детекции.

Два варианта:
  - MQTTPublisher: samurai/{id}/{ball_detection, detections, yolo/annotated, ...}
  - ROS2Publisher: /ball_detection (String JSON) + /yolo/annotated/compressed

Дополнительно: HybridPublisher шлёт в оба источника одновременно
(если детектор работает в ROS2-режиме но хочет дублировать в MQTT).
"""
from __future__ import annotations

import json
import logging
import time
from abc import ABC, abstractmethod
from typing import Optional

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

from .base import Detection

log = logging.getLogger(__name__)


class DetectionPublisher(ABC):
    """ABC: куда публиковать результаты детекции."""

    @abstractmethod
    def publish_detection(self, det: Detection): ...
    @abstractmethod
    def publish_summary(self, detections: list[Detection]): ...
    @abstractmethod
    def publish_annotated(self, jpeg_bytes: bytes): ...

    def publish_status(self, online: bool):
        """Опционально — анонс online/offline. По умолчанию no-op."""
        pass

    def publish_best_ball(self, detections: list[Detection]):
        """Лучший мяч → ball_detection (для FSM на Pi). По умолчанию first ball."""
        if not detections:
            return
        ball_classes = ('ball', 'sports ball', 'object')
        balls = [d for d in detections if d.cls in ball_classes]
        pool = balls if balls else detections
        best = max(pool, key=lambda d: d.conf)
        self.publish_detection(best)

    @property
    def name(self) -> str:
        return self.__class__.__name__


# ─────────────────────────────────────────────────────────────────────────────
# MQTT
# ─────────────────────────────────────────────────────────────────────────────
class MQTTPublisher(DetectionPublisher):
    """
    Публикует:
      samurai/{id}/ball_detection   — лучший мяч (best_ball)
      samurai/{id}/detections       — все объекты + count
      samurai/{id}/yolo/annotated   — аннотированный JPEG (если включено)
      samurai/{id}/detected_frame   — дубль аннотации для дашборда (legacy compat)
      samurai/{id}/yolo/status      — online/offline (retain=true)
    """

    def __init__(self,
                 mqtt_client,
                 robot_id: str = 'robot1',
                 publish_annotated: bool = True,
                 source: str = 'detector'):
        self._client = mqtt_client
        self._prefix = f'samurai/{robot_id}'
        self._publish_annotated = publish_annotated
        self._source = source

    def publish_detection(self, det: Detection):
        try:
            self._client.publish(
                f'{self._prefix}/ball_detection',
                json.dumps(det.to_dict()))
        except Exception as e:
            log.error('publish_detection failed: %s', e)

    def publish_summary(self, detections: list[Detection]):
        payload = {
            'objects': [d.to_dict() for d in detections],
            'count': len(detections),
            'ts': time.time(),
        }
        try:
            self._client.publish(
                f'{self._prefix}/detections',
                json.dumps(payload))
        except Exception as e:
            log.error('publish_summary failed: %s', e)

    def publish_annotated(self, jpeg_bytes: bytes):
        if not self._publish_annotated:
            return
        try:
            # Новый канал
            self._client.publish(f'{self._prefix}/yolo/annotated', jpeg_bytes)
            # Legacy compat (object_detector_node публиковал в detected_frame)
            self._client.publish(f'{self._prefix}/detected_frame', jpeg_bytes)
        except Exception as e:
            log.error('publish_annotated failed: %s', e)

    def publish_status(self, online: bool):
        payload = {'online': online, 'source': self._source, 'ts': time.time()}
        try:
            self._client.publish(
                f'{self._prefix}/yolo/status',
                json.dumps(payload), qos=1, retain=True)
        except Exception as e:
            log.error('publish_status failed: %s', e)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2
# ─────────────────────────────────────────────────────────────────────────────
class ROS2Publisher(DetectionPublisher):
    """
    Публикует:
      /ball_detection (std_msgs/String JSON)
      /yolo/detections (std_msgs/String JSON)
      /yolo/annotated/compressed (sensor_msgs/CompressedImage)
    """

    def __init__(self, node, annotated_quality: int = 75,
                 publish_annotated: bool = True):
        try:
            from std_msgs.msg import String  # type: ignore
            from sensor_msgs.msg import CompressedImage  # type: ignore
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # type: ignore
        except ImportError as e:
            raise RuntimeError('ROS2Publisher требует rclpy + std_msgs + sensor_msgs') from e

        self._node = node
        self._publish_annotated = publish_annotated
        self._jpeg_quality = annotated_quality
        self._String = String
        self._CompressedImage = CompressedImage

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._det_pub = node.create_publisher(String, '/ball_detection', 10)
        self._dets_pub = node.create_publisher(String, '/yolo/detections', 10)
        self._ann_pub = node.create_publisher(
            CompressedImage, '/yolo/annotated/compressed', cam_qos)

    def publish_detection(self, det: Detection):
        msg = self._String()
        msg.data = json.dumps(det.to_dict())
        self._det_pub.publish(msg)

    def publish_summary(self, detections: list[Detection]):
        msg = self._String()
        msg.data = json.dumps({
            'objects': [d.to_dict() for d in detections],
            'count': len(detections),
        })
        self._dets_pub.publish(msg)

    def publish_annotated(self, jpeg_bytes: bytes):
        if not self._publish_annotated:
            return
        msg = self._CompressedImage()
        msg.format = 'jpeg'
        msg.data = jpeg_bytes
        self._ann_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Hybrid
# ─────────────────────────────────────────────────────────────────────────────
class HybridPublisher(DetectionPublisher):
    """Публикует в несколько целей одновременно. Полезно для перехода ROS2 ↔ MQTT."""

    def __init__(self, *publishers: DetectionPublisher):
        self._publishers = publishers

    def publish_detection(self, det: Detection):
        for p in self._publishers:
            p.publish_detection(det)

    def publish_summary(self, detections: list[Detection]):
        for p in self._publishers:
            p.publish_summary(detections)

    def publish_annotated(self, jpeg_bytes: bytes):
        for p in self._publishers:
            p.publish_annotated(jpeg_bytes)

    def publish_status(self, online: bool):
        for p in self._publishers:
            p.publish_status(online)


# ─────────────────────────────────────────────────────────────────────────────
# Annotation drawing helper
# ─────────────────────────────────────────────────────────────────────────────
COLOUR_BGR_DEFAULT = {
    'red':     (0,   0,   220),
    'orange':  (0,   128, 255),
    'yellow':  (0,   220, 220),
    'green':   (0,   200, 0),
    'blue':    (220, 80,  0),
    'white':   (220, 220, 220),
    'black':   (60,  60,  60),
    'unknown': (0,   255, 0),
}


def draw_annotations(frame, detections: list[Detection],
                     colour_bgr: Optional[dict] = None):
    """
    Рисует bbox'ы и подписи на копии кадра. Возвращает аннотированный np.ndarray.
    """
    if cv2 is None:
        return frame
    palette = colour_bgr if colour_bgr is not None else COLOUR_BGR_DEFAULT
    annotated = frame.copy()
    for d in detections:
        x1, y1 = d.x, d.y
        x2, y2 = x1 + d.w, y1 + d.h
        bgr = palette.get(d.colour, palette.get('unknown', (0, 255, 0)))
        cv2.rectangle(annotated, (x1, y1), (x2, y2), bgr, 2)
        if d.distance > 0:
            label = f'{d.colour} {d.cls} {d.conf:.2f} {d.distance:.2f}m [{d.dist_method}]'
        else:
            label = f'{d.colour} {d.cls} {d.conf:.2f}'
        cv2.putText(annotated, label, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, bgr, 1)
    return annotated


def encode_jpeg(frame, quality: int = 70) -> Optional[bytes]:
    """Кодирует BGR в JPEG bytes, None при ошибке."""
    if cv2 is None:
        return None
    ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return bytes(enc) if ok else None
