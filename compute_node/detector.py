#!/usr/bin/env python3
"""
detector.py — единый CLI для всех вариантов детекции.

Заменяет три файла:
  - yolo_detector_node.py (ROS2)
  - yolo_detector_mqtt.py (standalone GPU MQTT)
  - object_detector_node.py (CPU + HSV fallback через MQTT)

Использование:
  # CPU + HSV fallback через MQTT (как старый object_detector_node)
  python compute_node/detector.py --source mqtt --broker raspberrypi.local

  # GPU YOLO через MQTT (как старый yolo_detector_mqtt)
  python compute_node/detector.py --source mqtt --device cuda --model yolo11n.pt

  # ROS2 YOLO (как старый yolo_detector_node)
  python compute_node/detector.py --source ros --backend yolo

  # Калибратор HSV
  python compute_node/detector.py --calibrate red

Все опции — `python compute_node/detector.py --help`.
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import time
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('detector')

# Make sure parent dir is on sys.path so 'compute_node.detectors' resolves
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_THIS_DIR)
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from compute_node.detectors import (  # noqa: E402
    Detection, DetectionPipeline, FrameContext,
    HSVClassifier, HSVBlobBackend,
    DistanceEstimator, WorldProjector,
    MQTTFrameSource, MQTTPublisher,
    draw_annotations, encode_jpeg,
)


def _build_backend(backend: str, model: str, device: str, conf: float, frame_drop: bool):
    """Создаёт DetectorBackend по флагам."""
    if backend == 'yolo':
        try:
            from compute_node.detectors.yolo_backend import YoloBackend
            return YoloBackend(
                model_path=model, device=device,
                confidence=conf, frame_drop=frame_drop,
            )
        except (ImportError, RuntimeError) as e:
            log.warning('YOLO недоступен (%s) — fallback на HSV blob', e)
            return HSVBlobBackend()
    elif backend == 'hsv':
        return HSVBlobBackend()
    else:
        raise ValueError(f'Неизвестный backend: {backend}')


def _build_pipeline(backend, focal_px: float, fov_deg: float):
    """Собирает DetectionPipeline с HSV classifier + distance + world."""
    return DetectionPipeline(
        backend=backend,
        hsv_classifier=HSVClassifier.from_config(),
        distance_estimator=DistanceEstimator(focal_px=focal_px),
        world_projector=WorldProjector(fov_rad=math.radians(fov_deg)),
        enrich_colour=True,
    )


def _run_mqtt_mode(args):
    """MQTT source + MQTT publisher (ноут / GPU laptop)."""
    backend = _build_backend(args.backend, args.model, args.device, args.conf, args.frame_drop)
    pipeline = _build_pipeline(backend, args.focal_px, args.fov_deg)

    will_topic = f'samurai/{args.robot_id}/yolo/status'
    will_payload = b'{"online": false, "source": "detector"}'

    source = MQTTFrameSource(
        broker=args.broker, port=args.port,
        robot_id=args.robot_id,
        client_id=f'samurai_detector_{os.getpid()}',
        mqtt_user=args.mqtt_user, mqtt_pwd=args.mqtt_pass,
        will_topic=will_topic, will_payload=will_payload,
    )
    publisher = MQTTPublisher(
        mqtt_client=source.client,
        robot_id=args.robot_id,
        publish_annotated=not args.no_annotated,
        source='detector',
    )

    # Detection enable/disable subscription
    enabled = {'value': True}

    def _on_enable(client, userdata, msg):
        cmd = msg.payload.decode('utf-8', errors='ignore').strip().lower()
        enabled['value'] = cmd in ('on', 'true', '1', 'enable')
        log.info('Detection %s', 'ENABLED' if enabled['value'] else 'DISABLED')

    source.client.message_callback_add(
        f'samurai/{args.robot_id}/detection/enable', _on_enable)

    def _on_connect(client, userdata, flags, rc):
        # Доп. подписка на enable (отдельно от camera/range/odom MQTTFrameSource)
        if rc == 0:
            client.subscribe(f'samurai/{args.robot_id}/detection/enable', qos=1)
            publisher.publish_status(online=True)
    # Цепляем дополнительный handler — paho поддерживает только один on_connect,
    # поэтому делаем его поверх старого
    _orig_on_connect = source.client.on_connect

    def _combined_on_connect(c, u, f, rc):
        if _orig_on_connect:
            _orig_on_connect(c, u, f, rc)
        _on_connect(c, u, f, rc)
    source.client.on_connect = _combined_on_connect

    def _on_frame(ctx: FrameContext):
        if not enabled['value']:
            # Disabled mode — пустой список + сырой кадр (heartbeat)
            publisher.publish_summary([])
            jpeg = encode_jpeg(ctx.bgr, quality=args.quality)
            if jpeg is not None:
                publisher.publish_annotated(jpeg)
            return
        detections = pipeline.process(ctx)
        publisher.publish_best_ball(detections)
        publisher.publish_summary(detections)
        if not args.no_annotated:
            annotated = draw_annotations(ctx.bgr, detections)
            jpeg = encode_jpeg(annotated, quality=args.quality)
            if jpeg is not None:
                publisher.publish_annotated(jpeg)

    source.on_frame(_on_frame)
    source.start()

    log.info('Detector running (source=mqtt, backend=%s, broker=%s:%d)',
             backend.name, args.broker, args.port)
    log.info('Subscribed to camera/range/odom/detection-enable. Ctrl+C to stop.')

    _stop = {'flag': False}
    def _shutdown(signum, frame):
        _stop['flag'] = True
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    last_stats = time.time()
    frame_count_at_last = 0
    try:
        while not _stop['flag']:
            time.sleep(1.0)
            now = time.time()
            if now - last_stats >= 30:
                # Можно добавить FPS-статистику если нужно
                last_stats = now
    finally:
        publisher.publish_status(online=False)
        source.stop()
        log.info('Detector stopped')


def _run_ros2_mode(args):
    """ROS2 source + ROS2 publisher (внутри Docker compute)."""
    try:
        from compute_node.detectors.frame_sources import ROS2FrameSource
        from compute_node.detectors.publishers import ROS2Publisher
    except (ImportError, RuntimeError) as e:
        log.error('ROS2 mode недоступен: %s', e)
        return 1

    backend = _build_backend(args.backend, args.model, args.device, args.conf, args.frame_drop)
    pipeline = _build_pipeline(backend, args.focal_px, args.fov_deg)

    source = ROS2FrameSource(node_name='samurai_detector')
    source.start()  # должно создать node перед ROS2Publisher
    publisher = ROS2Publisher(
        node=source.node,
        annotated_quality=args.quality,
        publish_annotated=not args.no_annotated,
    )

    def _on_frame(ctx: FrameContext):
        detections = pipeline.process(ctx)
        publisher.publish_best_ball(detections)
        publisher.publish_summary(detections)
        if not args.no_annotated:
            annotated = draw_annotations(ctx.bgr, detections)
            jpeg = encode_jpeg(annotated, quality=args.quality)
            if jpeg is not None:
                publisher.publish_annotated(jpeg)

    source.on_frame(_on_frame)

    log.info('Detector running (source=ros2, backend=%s)', backend.name)

    _stop = {'flag': False}
    def _shutdown(signum, frame):
        _stop['flag'] = True
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while not _stop['flag']:
            time.sleep(1.0)
    finally:
        source.stop()
        log.info('Detector stopped')

    return 0


def _run_calibrator(args):
    """HSV калибратор (OpenCV GUI)."""
    from compute_node.detectors.hsv_calibrator import HSVCalibrator, DEFAULT_COLOURS

    colour = args.calibrate if args.calibrate not in (None, 'auto', '') else 'red'
    if colour not in DEFAULT_COLOURS:
        log.warning('Unknown colour "%s" — fallback red. Available: %s',
                    colour, DEFAULT_COLOURS)
        colour = 'red'

    static_img = None
    source = None
    if args.image:
        try:
            import cv2
            static_img = cv2.imread(args.image)
        except ImportError:
            log.error('cv2 не установлен')
            return 1
        if static_img is None:
            log.error('Не удалось загрузить %s', args.image)
            return 1
    else:
        source = MQTTFrameSource(
            broker=args.broker, port=args.port,
            robot_id=args.robot_id,
            client_id='samurai_hsv_calibrator',
            mqtt_user=args.mqtt_user, mqtt_pwd=args.mqtt_pass,
        )

    cal = HSVCalibrator(colour=colour, source=source, static_image=static_img)
    cal.run()
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Samurai unified detector (заменяет yolo_detector_node, '
                    'yolo_detector_mqtt, object_detector_node)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры:
  # CPU + HSV fallback через MQTT (как object_detector_node)
  detector.py --source mqtt --backend hsv --broker raspberrypi.local

  # GPU YOLO через MQTT (как yolo_detector_mqtt)
  detector.py --source mqtt --backend yolo --device cuda --model yolo11n.pt

  # ROS2 YOLO (как yolo_detector_node, для внутри Docker)
  detector.py --source ros --backend yolo

  # HSV калибратор для подстройки под освещение
  detector.py --calibrate red
  detector.py --calibrate yellow --image photo.jpg
""")

    # ── Mode ──────────────────────────────────────────────────
    parser.add_argument('--source', choices=('mqtt', 'ros'), default='mqtt',
                        help='Источник кадров (default: mqtt)')
    parser.add_argument('--backend', choices=('yolo', 'hsv'), default='yolo',
                        help='Модель детекции (default: yolo, fallback hsv)')
    parser.add_argument('--calibrate', nargs='?', const='red', default=None,
                        metavar='COLOUR',
                        help='Запустить HSV калибратор (вместо детекции). Default: red')
    parser.add_argument('--image', default=None,
                        help='Статичное изображение для калибратора (вместо live)')

    # ── YOLO ──────────────────────────────────────────────────
    parser.add_argument('--model', default='yolo11n.pt',
                        help='YOLO модель (default: yolo11n.pt)')
    parser.add_argument('--device', default='cuda',
                        help='cuda или cpu (default: cuda — fallback на cpu если нет GPU)')
    parser.add_argument('--conf', type=float, default=0.40,
                        help='Порог уверенности (default: 0.40)')
    parser.add_argument('--frame-drop', action='store_true',
                        help='Пропускать кадры если предыдущий inference активен')

    # ── Publishing ────────────────────────────────────────────
    parser.add_argument('--no-annotated', action='store_true',
                        help='Не публиковать аннотированные кадры (экономия bandwidth)')
    parser.add_argument('--quality', type=int, default=70,
                        help='JPEG quality аннотации (0-100, default: 70)')

    # ── Camera intrinsics ─────────────────────────────────────
    parser.add_argument('--focal-px', type=float, default=500.0,
                        help='Фокусное расстояние камеры (px, default: 500)')
    parser.add_argument('--fov-deg', type=float, default=60.0,
                        help='Горизонтальный FOV (degrees, default: 60)')

    # ── MQTT ──────────────────────────────────────────────────
    parser.add_argument('--broker', default=os.environ.get('MQTT_BROKER', '127.0.0.1'),
                        help='MQTT broker IP (default: env MQTT_BROKER или 127.0.0.1)')
    parser.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')),
                        help='MQTT broker port (default: 1883)')
    parser.add_argument('--robot-id', default=os.environ.get('ROBOT_ID', 'robot1'),
                        help='Robot identifier (default: robot1)')
    parser.add_argument('--mqtt-user', default=None,
                        help='MQTT user (default: ENV/file/config)')
    parser.add_argument('--mqtt-pass', default=None,
                        help='MQTT password (default: ENV/file/config)')

    args = parser.parse_args()

    if args.calibrate is not None:
        return _run_calibrator(args) or 0

    if args.source == 'mqtt':
        _run_mqtt_mode(args)
    elif args.source == 'ros':
        return _run_ros2_mode(args) or 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
