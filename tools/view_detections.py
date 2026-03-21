#!/usr/bin/env python3
"""
view_detections.py — Просмотр аннотированного кадра с детекциями объектов.

Использование:
    python tools/view_detections.py --broker 192.168.1.50
    python tools/view_detections.py --broker raspberrypi.local
"""

import argparse
import sys
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker',   default='127.0.0.1')
    parser.add_argument('--port',     type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()

    last_frame = [None]
    last_det   = [None]
    last_ts    = [0.0]

    def on_frame(client, userdata, msg):
        arr = np.frombuffer(msg.payload, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            last_frame[0] = frame
            last_ts[0] = time.monotonic()

    def on_detections(client, userdata, msg):
        import json
        try:
            last_det[0] = json.loads(msg.payload)
        except Exception:
            pass

    client = mqtt.Client(client_id='det_viewer', protocol=mqtt.MQTTv311)

    def on_connect(c, *_):
        c.subscribe(f'samurai/{args.robot_id}/detected_frame')
        c.subscribe(f'samurai/{args.robot_id}/detections')
        print(f'[✓] Подключён к {args.broker}:{args.port}  (robot_id={args.robot_id})')
        print('     Ожидаю кадры... Нажми Q для выхода.')

    client.on_connect = on_connect
    client.message_callback_add(f'samurai/{args.robot_id}/detected_frame', on_frame)
    client.message_callback_add(f'samurai/{args.robot_id}/detections',     on_detections)

    try:
        client.connect(args.broker, args.port)
    except Exception as e:
        sys.exit(f'[✗] Не удалось подключиться: {e}')

    client.loop_start()

    print(f'[→] Подключаюсь к {args.broker}:{args.port}...')

    while True:
        frame = last_frame[0]
        det   = last_det[0]

        if frame is not None:
            # Оверлей со статистикой в нижней части кадра
            overlay = frame.copy()
            age = time.monotonic() - last_ts[0]
            count = det.get('count', 0) if det else 0
            status = f'Объектов: {count}  |  Кадр: {age*1000:.0f}мс назад'
            cv2.rectangle(overlay, (0, frame.shape[0] - 22), (frame.shape[1], frame.shape[0]),
                          (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
            cv2.putText(frame, status, (6, frame.shape[0] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1)

            cv2.imshow('Samurai — Object Detections', frame)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), ord('Q'), 27):  # Q или Esc
            break

    client.loop_stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
