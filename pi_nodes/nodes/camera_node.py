#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera -> MQTT JPEG stream.

Publishes:
    samurai/{robot_id}/camera  — JPEG binary @ 15 fps (configurable)
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

try:
    import cv2
    import numpy as np
    from picamera2 import Picamera2
    _HW = True
except ImportError:
    _HW = False


class CameraNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('camera_node', **kwargs)

        self._quality = cfg('mqtt.camera_jpeg_quality', 75)
        fps = cfg('mqtt.camera_fps', 15)
        w, h = 640, 480

        self._cam = None
        if not _HW:
            self.log_error('picamera2/cv2/numpy not available — camera disabled')
            return

        try:
            self._cam = Picamera2()
            config = self._cam.create_preview_configuration(
                main={'size': (w, h), 'format': 'RGB888'},
                buffer_count=4,
            )
            self._cam.configure(config)
            self._cam.start()
            self.log_info('Camera started %dx%d @ %d fps (JPEG %d%%)',
                          w, h, fps, self._quality)
            self.create_timer(1.0 / fps, self._capture)
        except Exception as exc:
            self.log_error('Camera init failed: %s — disabled', exc)
            self._cam = None

    def _capture(self):
        if self._cam is None:
            return
        frame = self._cam.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, enc = cv2.imencode('.jpg', frame_bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            self.publish('camera', enc.tobytes())

    def on_shutdown(self):
        if self._cam is not None:
            self._cam.stop()


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
