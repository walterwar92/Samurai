#!/usr/bin/env python3
"""
tools/collect_frames.py — сбор кадров с Pi для обучения YOLO.

Подключается к Pi H.264 TCP потоку (через H264TCPFrameSource), показывает
live preview, по нажатию клавиши сохраняет текущий кадр в
dataset/raw/<color>/.

Использование:
    python tools/collect_frames.py --pi 192.168.4.1
    python tools/collect_frames.py --pi 192.168.4.1 --out dataset/raw

Hotkeys (в OpenCV окне):
    r       — save red ball frame
    g       — save green ball frame
    b       — save blue ball frame
    SPACE   — save background frame (без мяча, negative samples)
    q / ESC — quit

При повторном запуске счётчики продолжаются с существующих файлов в папках.
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# Bootstrap project imports
_THIS = Path(__file__).resolve()
_ROOT = _THIS.parent.parent
sys.path.insert(0, str(_ROOT))

from compute_node.detectors.base import FrameContext  # noqa: E402
from compute_node.detectors.frame_sources import H264TCPFrameSource  # noqa: E402

# Mapping cv2.waitKey return code → имя класса (папка)
CLASS_KEYS = {
    ord('r'): 'red',
    ord('g'): 'green',
    ord('b'): 'blue',
    ord(' '): 'background',
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Сбор кадров с Pi для обучения YOLO',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--pi', default='192.168.4.1',
                        help='IP Pi MQTT (default: 192.168.4.1)')
    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT port (default: 1883)')
    parser.add_argument('--robot-id', default='robot1')
    parser.add_argument('--out', default='dataset/raw',
                        help='Папка для сохранения (default: dataset/raw)')
    args = parser.parse_args()

    # Создание папок под классы
    out_root = Path(args.out)
    for name in ('red', 'green', 'blue', 'background'):
        (out_root / name).mkdir(parents=True, exist_ok=True)
    print(f'Saving to: {out_root.resolve()}')

    # Подключение к Pi H.264 потоку
    print(f'Connecting to Pi {args.pi}:{args.port} for discovery ...')
    source = H264TCPFrameSource(
        broker=args.pi,
        port=args.port,
        robot_id=args.robot_id,
        client_id=f'samurai_collector_{os.getpid()}',
    )

    # Shared latest frame между TCP-потоком и main-thread'ом cv2 GUI
    latest: dict = {'bgr': None}
    lock = threading.Lock()

    def _on_frame(ctx: FrameContext) -> None:
        with lock:
            latest['bgr'] = ctx.bgr.copy()

    source.on_frame(_on_frame)
    source.start()

    # Счётчики: продолжаем с того что уже есть
    counts = {n: len(list((out_root / n).glob('*.jpg')))
              for n in ('red', 'green', 'blue', 'background')}

    print('\nHotkeys: r=red  g=green  b=blue  SPACE=background  q/ESC=quit')
    print(f'Existing: red={counts["red"]} green={counts["green"]} '
          f'blue={counts["blue"]} bg={counts["background"]}\n')

    window = 'Samurai Frame Collector'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    last_save_msg = ''
    last_save_ts = 0.0
    waiting_shown = False

    try:
        while True:
            with lock:
                frame = latest['bgr']

            if frame is None:
                # Первый кадр ещё не пришёл — показываем заглушку
                placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, 'Waiting for Pi camera ...',
                            (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (255, 255, 255), 2)
                cv2.imshow(window, placeholder)
                if not waiting_shown:
                    print('Waiting for first frame ...')
                    waiting_shown = True
            else:
                display = frame.copy()
                h, w = display.shape[:2]

                # HUD: счётчики
                hud = (f"r:{counts['red']:3d}  g:{counts['green']:3d}  "
                       f"b:{counts['blue']:3d}  bg:{counts['background']:3d}")
                cv2.rectangle(display, (0, 0), (w, 28), (0, 0, 0), -1)
                cv2.putText(display, hud, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Flash на полсекунды после сохранения
                if last_save_msg and (time.time() - last_save_ts) < 0.5:
                    cv2.rectangle(display, (0, 28), (w, 58),
                                  (40, 180, 40), -1)
                    cv2.putText(display, last_save_msg, (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                cv2.imshow(window, display)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            if key in CLASS_KEYS and frame is not None:
                cls = CLASS_KEYS[key]
                counts[cls] += 1
                ts_ms = int(time.time() * 1000)
                fname = out_root / cls / f'img_{ts_ms}_{counts[cls]:05d}.jpg'
                if cv2.imwrite(str(fname), frame):
                    last_save_msg = f'saved {cls}/{fname.name}'
                    last_save_ts = time.time()
                    print(f'  + {cls}: {fname.relative_to(_ROOT)} '
                          f'(total {cls}: {counts[cls]})')
                else:
                    counts[cls] -= 1
                    print(f'  ! failed to save {fname}')

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        source.stop()
        print('\nFinal counts:')
        for name in ('red', 'green', 'blue', 'background'):
            print(f'  {name}: {counts[name]}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
