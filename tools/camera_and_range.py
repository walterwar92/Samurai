#!/usr/bin/env python3
"""
camera_and_range.py — Получает изображение с камеры и данные ультразвука.

Подключается к MQTT-брокеру на Pi, сохраняет кадры в папку frames/
и выводит данные дальномера в консоль в реальном времени.

Использование:
    python3 tools/camera_and_range.py <BROKER_IP>
    python3 tools/camera_and_range.py <BROKER_IP> --robot-id robot1
    python3 tools/camera_and_range.py <BROKER_IP> --no-save   # только консоль

Требования:
    pip install paho-mqtt opencv-python
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: pip install paho-mqtt")
    sys.exit(1)

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    print("WARN: opencv не найден — кадры будут сохраняться как .jpg без предпросмотра")
    print("      pip install opencv-python")
    HAS_CV2 = False


def parse_args():
    p = argparse.ArgumentParser(description='Камера + ультразвук с робота')
    p.add_argument('broker', help='IP адрес Pi (например 100.99.98.32)')
    p.add_argument('--port', type=int, default=1883)
    p.add_argument('--robot-id', default='robot1')
    p.add_argument('--no-save', action='store_true',
                   help='Не сохранять кадры на диск')
    p.add_argument('--save-every', type=int, default=30,
                   help='Сохранять каждый N-й кадр (default: 30)')
    p.add_argument('--output-dir', default='frames',
                   help='Папка для сохранения кадров (default: frames/)')
    return p.parse_args()


class CameraRangeViewer:
    def __init__(self, args):
        self.prefix = f'samurai/{args.robot_id}'
        self.save = not args.no_save
        self.save_every = args.save_every
        self.output_dir = args.output_dir

        self._frame_count = 0
        self._saved_count = 0
        self._range_m = None
        self._last_range_t = None
        self._connected = False
        self._start_t = time.time()

        if self.save:
            os.makedirs(self.output_dir, exist_ok=True)
            print(f"[INFO] Кадры сохраняются в: {os.path.abspath(self.output_dir)}/")

        self.client = mqtt.Client(client_id='samurai_cam_viewer')
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        print(f"[INFO] Подключение к {args.broker}:{args.port} ...")
        self.client.connect(args.broker, args.port, keepalive=10)

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f"[ERROR] Не удалось подключиться, rc={rc}")
            return
        self._connected = True
        print(f"[OK]   Подключён к брокеру")
        client.subscribe(f'{self.prefix}/camera', qos=0)
        client.subscribe(f'{self.prefix}/range', qos=0)
        print(f"[INFO] Подписан на:")
        print(f"       {self.prefix}/camera")
        print(f"       {self.prefix}/range")
        print()
        print("  Ультразвук:  будет выводиться ниже")
        print("  Камера:      кадры считаются и сохраняются")
        print("  Ctrl+C — выход")
        print("-" * 50)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            print(f"\n[WARN] Потеряно соединение (rc={rc}), переподключение...")

    def _on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self.prefix) + 1:]

        if suffix == 'range':
            self._handle_range(msg.payload)
        elif suffix == 'camera':
            self._handle_camera(msg.payload)

    def _handle_range(self, payload):
        try:
            d = json.loads(payload)
            r = float(d.get('range', d) if isinstance(d, dict) else d)
        except Exception:
            try:
                r = float(payload)
            except Exception:
                return

        self._range_m = r
        self._last_range_t = time.time()

        # Индикатор расстояния
        if r < 0.20:
            indicator = "🔴 ОЧЕНЬ БЛИЗКО"
        elif r < 0.40:
            indicator = "🟡 Близко"
        elif r < 1.00:
            indicator = "🟢 Норма"
        else:
            indicator = "⚪ Далеко"

        bar_len = min(40, int(r * 20))
        bar = '█' * bar_len + '░' * (40 - bar_len)
        elapsed = time.time() - self._start_t
        print(f"\r[{elapsed:6.1f}s] Ультразвук: {r:5.3f} м  {indicator}  [{bar}]", end='', flush=True)

    def _handle_camera(self, payload):
        self._frame_count += 1

        if not self.save:
            if self._frame_count % 30 == 0:
                elapsed = time.time() - self._start_t
                fps_approx = self._frame_count / elapsed if elapsed > 0 else 0
                print(f"\n[{elapsed:6.1f}s] Камера: получено {self._frame_count} кадров  (~{fps_approx:.1f} fps)")
            return

        # Сохранять каждый N-й кадр
        if self._frame_count % self.save_every != 0:
            return

        ts = datetime.now().strftime('%H%M%S_%f')[:9]
        filename = os.path.join(self.output_dir, f'frame_{ts}.jpg')

        if HAS_CV2:
            # Декодируем и сохраняем с информацией
            np_arr = np.frombuffer(payload, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                # Не JPEG — сохраняем как есть
                with open(filename, 'wb') as f:
                    f.write(payload)
            else:
                # Добавляем текст с данными дальномера
                h, w = frame.shape[:2]
                range_text = f"Range: {self._range_m:.3f}m" if self._range_m is not None else "Range: N/A"
                time_text = datetime.now().strftime('%H:%M:%S')
                cv2.putText(frame, range_text, (10, h - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, time_text, (10, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imwrite(filename, frame)
        else:
            with open(filename, 'wb') as f:
                f.write(payload)

        self._saved_count += 1
        elapsed = time.time() - self._start_t
        fps_approx = self._frame_count / elapsed if elapsed > 0 else 0
        print(f"\n[{elapsed:6.1f}s] Камера: кадр #{self._frame_count} сохранён → {filename}  (~{fps_approx:.1f} fps)")

    def run(self):
        self.client.loop_start()

        # Ждём подключения
        for _ in range(50):
            if self._connected:
                break
            time.sleep(0.1)
        else:
            print("[ERROR] Не удалось подключиться за 5 секунд")
            print("        Проверьте IP адрес и что Mosquitto запущен на Pi")
            self.client.loop_stop()
            return

        try:
            while True:
                time.sleep(1.0)
                # Предупреждение если нет данных
                elapsed = time.time() - self._start_t
                if elapsed > 5.0 and self._frame_count == 0 and self._range_m is None:
                    print(f"\n[WARN] {elapsed:.0f}s — нет данных ни от камеры, ни от ультразвука")
                    print("       Убедитесь что Pi запущен: ./start_robot_mqtt.sh")
        except KeyboardInterrupt:
            pass

        self.client.loop_stop()
        self.client.disconnect()

        elapsed = time.time() - self._start_t
        print(f"\n\n{'='*50}")
        print(f"ИТОГ за {elapsed:.1f} секунд:")
        print(f"  Кадров получено:  {self._frame_count}")
        print(f"  Кадров сохранено: {self._saved_count}")
        if self._range_m is not None:
            print(f"  Последнее расстояние: {self._range_m:.3f} м")
        if self.save and self._saved_count > 0:
            print(f"  Папка с кадрами: {os.path.abspath(self.output_dir)}/")
        print('='*50)


def main():
    args = parse_args()
    viewer = CameraRangeViewer(args)
    viewer.run()


if __name__ == '__main__':
    main()
