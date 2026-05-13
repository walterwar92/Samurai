#!/usr/bin/env python3
"""
camera_node — Raspberry Pi CSI Camera → H.264 TCP stream.

Раньше публиковал JPEG кадры в MQTT (~2.5 МБ/с при q=65, 640x480, 20fps).
Теперь — H.264 (Pi hardware encoder) через прямой TCP сокет.
Bandwidth снижен до ~200-500 КБ/с при том же визуальном качестве.

TCP server:
    Port: cfg('mqtt.camera_h264_port', 8554)
    Format: Annex B raw H.264 NAL units (start codes 0x000001)
    Late-joiners: получают накопленный init buffer (SPS/PPS + первый IDR)
                  чтобы декодер сразу мог начать без ожидания следующего keyframe

MQTT discovery (retained, опубликовано при connect):
    samurai/{robot_id}/camera/endpoint
    {
      "protocol": "tcp", "host": "<pi_ip>", "port": 8554,
      "codec": "h264", "format": "annex-b",
      "width": 640, "height": 480, "fps": 20, "bitrate": 2000000
    }

Клиенты (compute_node/detectors/H264TCPFrameSource, dashboard /ws/h264 proxy)
читают этот retained topic чтобы найти Pi без хардкодинга IP.
"""

import json
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode, get_local_ip

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

try:
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FileOutput
    _HW = True
except ImportError:
    _HW = False


class TCPStreamServer:
    """
    Multiplexing TCP сервер для H.264 потока.

    Принимает множественные клиентские соединения и шлёт каждому одинаковый
    поток NAL units. Поддерживает late-joiners через init_buffer (SPS/PPS +
    первый IDR), чтобы новый клиент мог начать декодирование сразу.
    """

    # Сколько байт начала потока сохранить для late-joiners.
    # SPS+PPS обычно ~50 байт, первый IDR ~50KB, плюс запас.
    INIT_BUFFER_SIZE = 128 * 1024
    SOCKET_BUFFER = 256 * 1024

    def __init__(self, port: int, log_fn):
        self._port = port
        self._sock: socket.socket | None = None
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._running = False
        self._accept_thread: threading.Thread | None = None
        self._log = log_fn
        self._init_buffer = bytearray()

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', self._port))
        self._sock.listen(8)
        self._sock.settimeout(1.0)  # позволяет accept выйти при stop()
        self._running = True
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            name='camera_tcp_accept',
            daemon=True)
        self._accept_thread.start()
        self._log('H.264 TCP server listening on :%d', self._port)

    def stop(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        with self._lock:
            for client in self._clients:
                try:
                    client.close()
                except OSError:
                    pass
            self._clients.clear()

    def _accept_loop(self):
        while self._running:
            try:
                client_sock, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                                   self.SOCKET_BUFFER)
            self._log('New H.264 client: %s', addr)

            with self._lock:
                # Отправляем накопленный init_buffer (SPS/PPS + первый IDR).
                # Без этого декодер вынужден ждать следующего keyframe.
                if self._init_buffer:
                    try:
                        client_sock.sendall(bytes(self._init_buffer))
                    except OSError as e:
                        self._log('Init send failed for %s: %s', addr, e)
                        try:
                            client_sock.close()
                        except OSError:
                            pass
                        continue
                self._clients.append(client_sock)

    def write_frame(self, data: bytes):
        """Раздать NAL unit всем подключенным клиентам."""
        if not data:
            return

        # Накопить начало потока для late-joiners
        if len(self._init_buffer) < self.INIT_BUFFER_SIZE:
            need = self.INIT_BUFFER_SIZE - len(self._init_buffer)
            self._init_buffer.extend(data[:need])

        with self._lock:
            dead = []
            for client in self._clients:
                try:
                    client.sendall(data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(client)
            for d in dead:
                try:
                    d.close()
                except OSError:
                    pass
                self._clients.remove(d)
                self._log('H.264 client disconnected (now %d)',
                          len(self._clients))

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)


class _StreamFileWrapper:
    """File-like объект для FileOutput → шлёт байты в TCPStreamServer."""

    def __init__(self, server: TCPStreamServer):
        self._server = server

    def write(self, data):
        self._server.write_frame(bytes(data))

    def flush(self):
        pass

    def close(self):
        pass


class CameraNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('camera_node', **kwargs)

        self._h264_port = int(cfg('mqtt.camera_h264_port', 8554))
        self._bitrate = int(cfg('mqtt.camera_h264_bitrate', 2_000_000))
        self._iperiod = int(cfg('mqtt.camera_h264_iperiod', 30))
        self._fps = int(cfg('mqtt.camera_fps', 20))
        self._w, self._h = 640, 480

        self._tcp: TCPStreamServer | None = None
        self._cam: Picamera2 | None = None
        self._encoder = None
        self._discovery_topic = 'camera/endpoint'

        if not _HW:
            self.log_error('picamera2 not available — camera disabled')
            return

        self._tcp = TCPStreamServer(self._h264_port, self.log_info)
        self._tcp.start()

        self._start_camera()
        # Periodic discovery republish (раз в 5с — на случай retained drift
        # или подключения новых broker'ов)
        self.create_timer(5.0, self._publish_discovery)

    def _start_camera(self):
        try:
            self._cam = Picamera2()
            config = self._cam.create_video_configuration(
                main={'size': (self._w, self._h), 'format': 'YUV420'},
                controls={'FrameRate': self._fps},
            )
            self._cam.configure(config)

            # profile='baseline' — Constrained Baseline (avc1.42E0xx).
            # Универсально поддерживается всеми браузерами через WebCodecs,
            # вкл. старые Chrome и Edge без аппаратного High-decoder. По
            # умолчанию vc4-hw-encode выдаёт High profile → не везде
            # декодируется; baseline — самый совместимый вариант.
            # Для 640×480 @ 20 fps quality difference незаметная.
            self._encoder = H264Encoder(
                bitrate=self._bitrate,
                iperiod=self._iperiod,
                profile='baseline',
                # repeat=True даёт SPS/PPS перед каждым IDR — late-joiners
                # видят headers сразу, не ждут следующего конфига
                repeat=True,
            )
            output = FileOutput(_StreamFileWrapper(self._tcp))
            self._cam.start_recording(self._encoder, output)
            self.log_info(
                'Camera started: %dx%d @ %d fps, H.264 baseline '
                'bitrate=%d iperiod=%d',
                self._w, self._h, self._fps, self._bitrate, self._iperiod)
        except TypeError as exc:
            # Старая picamera2 без profile= — повторяем без него.
            self.log_warn('H.264 profile=baseline не поддерживается '
                          'этой picamera2: %s. Fallback на default profile '
                          '(может потребоваться High-decoder в браузере).',
                          exc)
            try:
                self._encoder = H264Encoder(
                    bitrate=self._bitrate,
                    iperiod=self._iperiod,
                    repeat=True,
                )
                output = FileOutput(_StreamFileWrapper(self._tcp))
                self._cam.start_recording(self._encoder, output)
                self.log_info(
                    'Camera started (fallback): %dx%d @ %d fps, '
                    'default profile, bitrate=%d iperiod=%d',
                    self._w, self._h, self._fps, self._bitrate, self._iperiod)
            except Exception as exc2:
                self.log_error('Camera/H.264 init failed (fallback): %s', exc2)
                self._cam = None
        except Exception as exc:
            self.log_error('Camera/H.264 init failed: %s', exc)
            self._cam = None

    # Override — публикуем discovery после успешного MQTT connect.
    def _on_connect(self, client, userdata, flags, rc):
        super()._on_connect(client, userdata, flags, rc)
        if rc == 0:
            self._publish_discovery()

    def _publish_discovery(self):
        """Retained discovery message — клиенты находят Pi по этому топику.

        Defense-in-depth: если get_local_ip() вернул loopback (Pi в AP-режиме
        без default-route + сломанный interface enumeration) — НЕ публикуем
        вообще. Лучше пустой retained, чем bogus host=127.0.0.1, на который
        ноут пытается коннектиться к собственному localhost:8554. Громкий
        WARNING в лог сразу подскажет, какой env-override выставить.
        """
        if not self._mqtt_connected or self._tcp is None:
            return
        host = get_local_ip(broker_hint=self._broker)
        if host.startswith('127.'):
            self.log_warn(
                'camera/endpoint NOT published — get_local_ip returned '
                'loopback %r. Pi likely in AP-mode без default-route и '
                'interface enumeration не сработала. Set SAMURAI_PI_IP=<wlan-IP> '
                'env var (e.g. 192.168.4.1) и рестартни camera_node.', host)
            return
        endpoint = {
            'protocol': 'tcp',
            'host': host,
            'port': self._h264_port,
            'codec': 'h264',
            'format': 'annex-b',
            'width': self._w,
            'height': self._h,
            'fps': self._fps,
            'bitrate': self._bitrate,
            'iperiod': self._iperiod,
            'clients': self._tcp.client_count,
        }
        self.log_info('camera/endpoint -> tcp://%s:%d (clients=%d)',
                      host, self._h264_port, self._tcp.client_count)
        self.publish(self._discovery_topic, endpoint, qos=1, retain=True)

    def on_shutdown(self):
        # Очищаем retained discovery message чтобы клиенты знали что offline
        if self._mqtt_connected:
            try:
                self._client.publish(
                    self.topic(self._discovery_topic),
                    payload=b'', qos=1, retain=True)
            except Exception:
                pass

        if self._cam is not None:
            try:
                self._cam.stop_recording()
                self._cam.stop()
            except Exception:
                pass

        if self._tcp is not None:
            self._tcp.stop()


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
