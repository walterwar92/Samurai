#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera -> MQTT JPEG stream.

Publishes:
    samurai/{robot_id}/camera  — JPEG binary @ 15 fps (configurable)

Uses RGB888 pixel format — on this hardware Picamera2 RGB888
outputs BGR byte order, which cv2.imencode accepts directly.
"""

import os
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

try:
    import cv2
    from picamera2 import Picamera2
    _HW = True
except ImportError:
    _HW = False

# Максимальное время ожидания кадра от камеры (сек).
_CAPTURE_TIMEOUT_S = 1.0


class CameraNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('camera_node', **kwargs)

        self._quality = cfg('mqtt.camera_jpeg_quality', 70)
        self._fps = cfg('mqtt.camera_fps', 20)
        self._w, self._h = 640, 480

        self._cam = None
        self._capturing = False      # non-blocking flag instead of Lock
        self._capture_start = 0.0    # when current capture started
        self._error_count = 0
        self._total_restarts = 0
        self._max_restarts = 10      # max auto-restarts before giving up
        self._pool = ThreadPoolExecutor(max_workers=1,
                                        thread_name_prefix='cam_cap')
        # Pre-allocate JPEG encode params (avoid list creation per frame)
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._quality] if _HW else []

        if not _HW:
            self.log_error('picamera2/cv2 not available — camera disabled')
            return

        self._start_camera(self._w, self._h, self._fps)

    def _start_camera(self, w: int, h: int, fps: int):
        """Инициализировать и запустить Picamera2. Можно вызывать повторно при сбое."""
        try:
            if self._cam is not None:
                try:
                    self._cam.stop()
                except Exception:
                    pass
            self._cam = Picamera2()
            # RGB888 — на данном железе Picamera2 отдаёт BGR byte order,
            # что совпадает с ожиданиями cv2.imencode (конвертация не нужна)
            config = self._cam.create_preview_configuration(
                main={'size': (w, h), 'format': 'RGB888'},
                buffer_count=4,
            )
            self._cam.configure(config)
            self._cam.start()
            self._error_count = 0
            self.log_info('Camera started %dx%d @ %d fps (JPEG q=%d, RGB888)',
                          w, h, fps, self._quality)
            self.create_timer(1.0 / fps, self._capture)
        except Exception as exc:
            self.log_error('Camera init failed: %s — disabled', exc)
            self._cam = None

    def _try_restart_camera(self):
        """Attempt to restart camera after errors."""
        self._total_restarts += 1
        if self._total_restarts > self._max_restarts:
            self.log_error('Camera restart limit (%d) reached — giving up',
                           self._max_restarts)
            return
        self.log_warn('Restarting camera (attempt %d/%d)...',
                      self._total_restarts, self._max_restarts)
        time.sleep(1.0)  # brief pause before restart
        self._start_camera(self._w, self._h, self._fps)

    def _capture(self):
        if self._cam is None:
            return

        # Non-blocking: skip if previous capture still in flight
        if self._capturing:
            elapsed = time.monotonic() - self._capture_start
            if elapsed > _CAPTURE_TIMEOUT_S:
                self._capturing = False
                self._error_count += 1
                self.log_warn('Camera capture timed out (%d/5)', self._error_count)
                if self._error_count >= 5:
                    self.log_error('Camera hung repeatedly — attempting restart')
                    self._cam = None
                    threading.Thread(target=self._try_restart_camera,
                                     daemon=True).start()
            return

        self._capturing = True
        self._capture_start = time.monotonic()

        self._pool.submit(self._do_capture)

    def _do_capture(self):
        try:
            frame = self._cam.capture_array()
            if frame is None:
                return

            self._error_count = 0
            ok, enc = cv2.imencode('.jpg', frame, self._encode_params)
            if ok:
                self.publish('camera', enc.tobytes())
        except Exception as exc:
            self._error_count += 1
            self.log_error('Camera capture error: %s (%d/5)',
                           exc, self._error_count)
            if self._error_count >= 5:
                self.log_error('Too many errors — attempting restart')
                self._cam = None
                self._pool.submit(self._try_restart_camera)
        finally:
            self._capturing = False

    def on_shutdown(self):
        self._pool.shutdown(wait=False)
        if self._cam is not None:
            try:
                self._cam.stop()
            except Exception:
                pass


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = CameraNode(broker=args.broker, port=args.port,
                      robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
