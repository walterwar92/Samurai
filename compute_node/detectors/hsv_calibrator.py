"""
HSVCalibrator — интерактивная подстройка HSV-диапазонов под текущее освещение.

OpenCV GUI с trackbars (H/S/V low/high) и live preview маски.
Источник кадров — любой FrameSource (MQTTFrameSource для live с робота)
или статичное изображение (для калибровки по фотографии).

Запуск:
    samurai detector --calibrate              # выбор цвета через GUI
    samurai detector --calibrate red          # сразу калибровка red
    samurai detector --calibrate red --image photo.jpg  # из файла

Управление:
    Trackbars — подбор диапазона
    Колесо мыши на превью — выбрать цвет (sample HSV)
    s — сохранить в config.yaml (секция hsv_colours)
    n — следующий цвет
    q / ESC — выход

Single-range support (для большинства цветов хватает одного диапазона).
Для red нужен dual-range (0-10 + 160-180) — есть отдельная подсказка.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

from .base import FrameContext
from .hsv import load_hsv_ranges_from_config

log = logging.getLogger(__name__)

WINDOW_PREVIEW = 'HSV Calibrator — Preview'
WINDOW_MASK = 'HSV Calibrator — Mask'
WINDOW_RESULT = 'HSV Calibrator — Result'
WINDOW_CONTROLS = 'HSV Calibrator — Controls'

DEFAULT_COLOURS = ['red', 'orange', 'yellow', 'green', 'blue', 'white', 'black']


class HSVCalibrator:
    """Interactive OpenCV-based HSV range tuner."""

    def __init__(self,
                 colour: str = 'red',
                 source: Optional[object] = None,
                 static_image: Optional[np.ndarray] = None,
                 config_path: Optional[str] = None):
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) не установлен — калибратор невозможен')
        self._colour = colour
        self._source = source
        self._static = static_image
        self._latest_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False

        # Загружаем текущие диапазоны из config.yaml как стартовые значения
        self._ranges = load_hsv_ranges_from_config(config_path)
        self._config_path = config_path or os.path.join(
            os.path.dirname(__file__), '..', '..', 'config.yaml')

    # ── Frame ingestion ───────────────────────────────────────
    def _on_frame(self, ctx: FrameContext):
        with self._lock:
            self._latest_frame = ctx.bgr.copy()

    def _get_frame(self) -> Optional[np.ndarray]:
        if self._static is not None:
            return self._static
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    # ── GUI ───────────────────────────────────────────────────
    def _create_trackbars(self):
        cv2.namedWindow(WINDOW_CONTROLS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_CONTROLS, 400, 200)

        # Стартовые значения из существующего диапазона
        ranges = self._ranges.get(self._colour, [((0, 100, 100), (10, 255, 255))])
        (h1, s1, v1), (h2, s2, v2) = ranges[0]  # берём первый диапазон

        cv2.createTrackbar('H_low',  WINDOW_CONTROLS, h1, 180, lambda _: None)
        cv2.createTrackbar('S_low',  WINDOW_CONTROLS, s1, 255, lambda _: None)
        cv2.createTrackbar('V_low',  WINDOW_CONTROLS, v1, 255, lambda _: None)
        cv2.createTrackbar('H_high', WINDOW_CONTROLS, h2, 180, lambda _: None)
        cv2.createTrackbar('S_high', WINDOW_CONTROLS, s2, 255, lambda _: None)
        cv2.createTrackbar('V_high', WINDOW_CONTROLS, v2, 255, lambda _: None)

    def _read_trackbars(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        h1 = cv2.getTrackbarPos('H_low',  WINDOW_CONTROLS)
        s1 = cv2.getTrackbarPos('S_low',  WINDOW_CONTROLS)
        v1 = cv2.getTrackbarPos('V_low',  WINDOW_CONTROLS)
        h2 = cv2.getTrackbarPos('H_high', WINDOW_CONTROLS)
        s2 = cv2.getTrackbarPos('S_high', WINDOW_CONTROLS)
        v2 = cv2.getTrackbarPos('V_high', WINDOW_CONTROLS)
        return (h1, s1, v1), (h2, s2, v2)

    # ── Lifecycle ─────────────────────────────────────────────
    def run(self):
        if self._source is not None:
            self._source.on_frame(self._on_frame)
            self._source.start()
            log.info('Waiting for first frame...')
            t0 = time.time()
            while self._get_frame() is None and time.time() - t0 < 10:
                time.sleep(0.1)
            if self._get_frame() is None and self._static is None:
                log.error('No frame received after 10s — нет live-источника?')
                self._source.stop()
                return

        self._running = True
        self._create_trackbars()
        log.info('HSVCalibrator running. Colour=%s', self._colour)
        log.info('Controls: s=save, n=next colour, q/ESC=quit')

        try:
            while self._running:
                frame = self._get_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue

                lo, hi = self._read_trackbars()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
                result = cv2.bitwise_and(frame, frame, mask=mask)

                # Overlay info
                preview = frame.copy()
                ratio = float(np.count_nonzero(mask)) / max(1, mask.size)
                cv2.putText(preview,
                            f'{self._colour}: lo={lo} hi={hi} | ratio={ratio*100:.1f}%',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
                cv2.putText(preview, 's=save  n=next  q=quit',
                            (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow(WINDOW_PREVIEW, preview)
                cv2.imshow(WINDOW_MASK, mask)
                cv2.imshow(WINDOW_RESULT, result)

                key = cv2.waitKey(30) & 0xFF
                if key == ord('q') or key == 27:  # ESC
                    break
                elif key == ord('s'):
                    self._save_to_config(self._colour, lo, hi)
                elif key == ord('n'):
                    self._next_colour()
        finally:
            cv2.destroyAllWindows()
            if self._source is not None:
                self._source.stop()

    def _next_colour(self):
        try:
            i = DEFAULT_COLOURS.index(self._colour)
        except ValueError:
            i = -1
        i = (i + 1) % len(DEFAULT_COLOURS)
        self._colour = DEFAULT_COLOURS[i]
        log.info('Switched to colour: %s', self._colour)
        # Reset trackbars to new colour's existing range
        cv2.destroyWindow(WINDOW_CONTROLS)
        self._create_trackbars()

    def _save_to_config(self, colour: str,
                        lo: tuple[int, int, int],
                        hi: tuple[int, int, int]):
        """
        Сохраняет диапазон в config.yaml секцию hsv_colours.
        Формат: список flat 6-tuples (совместимо с обоими существующими форматами).
        """
        try:
            import yaml
        except ImportError:
            log.error('PyYAML не установлен — нельзя сохранить')
            return

        try:
            with open(self._config_path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}
        except yaml.YAMLError as e:
            log.error('config.yaml невалиден: %s', e)
            return

        if 'hsv_colours' not in data or not isinstance(data['hsv_colours'], dict):
            data['hsv_colours'] = {}

        # Сохраняем как flat 6-list (наиболее простой формат)
        data['hsv_colours'][colour] = [list(lo) + list(hi)]

        try:
            with open(self._config_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, default_flow_style=None,
                               allow_unicode=True, sort_keys=False)
            log.info('Saved %s to %s: lo=%s hi=%s', colour, self._config_path, lo, hi)
        except Exception as e:
            log.error('Save failed: %s', e)

        # Update internal cache
        self._ranges[colour] = [(lo, hi)]


def main():
    """Standalone CLI: python -m compute_node.detectors.hsv_calibrator [colour]"""
    import argparse
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] %(levelname)s %(message)s',
        datefmt='%H:%M:%S')

    parser = argparse.ArgumentParser(description='Samurai HSV calibrator')
    parser.add_argument('colour', nargs='?', default='red',
                        choices=DEFAULT_COLOURS,
                        help='Colour to calibrate (default: red)')
    parser.add_argument('--image', help='Static image path (для калибровки по фото)')
    parser.add_argument('--broker', default='127.0.0.1',
                        help='MQTT broker IP (live source)')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()

    static_img = None
    source = None

    if args.image:
        if cv2 is None:
            print('cv2 not installed', flush=True)
            return 1
        static_img = cv2.imread(args.image)
        if static_img is None:
            print(f'Не удалось загрузить {args.image}', flush=True)
            return 1
    else:
        from .frame_sources import MQTTFrameSource
        source = MQTTFrameSource(
            broker=args.broker, port=args.port,
            robot_id=args.robot_id,
            client_id='samurai_hsv_calibrator')

    cal = HSVCalibrator(colour=args.colour, source=source, static_image=static_img)
    cal.run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
