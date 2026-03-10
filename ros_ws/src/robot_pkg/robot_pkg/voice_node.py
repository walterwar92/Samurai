#!/usr/bin/env python3
"""
voice_node — Offline Russian speech recognition via Vosk API.

Listens to microphone, recognises commands, publishes to /voice_command.
Commands:
  'получи [цвет] шарик'  — acquire ball of given colour
  'вызови вторую машину'  — call second robot
  Any other phrase         — published as-is for FSM interpretation

Vosk model path: ~/vosk-model-ru  (download vosk-model-small-ru-0.22)

VAD (Voice Activity Detection) via webrtcvad:
  Тишина пропускается до передачи в Vosk — экономия ~70% CPU.
"""

import json
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import pyaudio
    import vosk
    _HW = True
except ImportError:
    _HW = False

try:
    import webrtcvad
    _VAD = True
except ImportError:
    _VAD = False

# Default model path uses ~ so it works for any username (not just 'pi').
# Override via ROS2 parameter: --ros-args -p model_path:=/custom/path/model
VOSK_MODEL_PATH = os.path.expanduser('~/vosk-model-ru')
SAMPLE_RATE = 16000
CHUNK_SIZE = 4000          # сэмплов за чтение (250ms)
VAD_FRAME_BYTES = 960      # 30ms фрейм при 16kHz int16 (480 сэмплов × 2 байта)
VAD_AGGRESSIVENESS = 2     # 0=мягкий, 3=строгий


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.declare_parameter('model_path', VOSK_MODEL_PATH)
        model_path = os.path.expanduser(self.get_parameter('model_path').value)

        self._pub = self.create_publisher(String, '/voice_command', 10)

        if not _HW:
            self.get_logger().error('vosk/pyaudio not available — voice disabled')
            return

        # VAD для фильтрации тишины
        if _VAD:
            self._vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
            self.get_logger().info('VAD enabled (webrtcvad) — silence will be skipped')
        else:
            self._vad = None
            self.get_logger().warn('webrtcvad not installed — VAD disabled (pip install webrtcvad)')

        self.get_logger().info(f'Loading Vosk model from {model_path} ...')
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, SAMPLE_RATE)

        self._audio = pyaudio.PyAudio()
        self._stream = self._audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        # Background recognition thread
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('Voice recognition started (Russian, offline)')

    def _has_speech(self, data: bytes) -> bool:
        """Проверяет наличие речи в чанке через webrtcvad."""
        if self._vad is None:
            return True
        # Разбиваем на 30ms фреймы и проверяем каждый
        for i in range(0, len(data) - VAD_FRAME_BYTES + 1, VAD_FRAME_BYTES):
            frame = data[i:i + VAD_FRAME_BYTES]
            if len(frame) == VAD_FRAME_BYTES and self._vad.is_speech(frame, SAMPLE_RATE):
                return True
        return False

    def _listen_loop(self):
        while self._running:
            try:
                data = self._stream.read(CHUNK_SIZE, exception_on_overflow=False)
            except OSError as exc:
                self.get_logger().warning(f'Audio read error: {exc}')
                continue
            # Пропускаем тишину — экономим CPU Vosk
            if not self._has_speech(data):
                continue
            if self._recognizer.AcceptWaveform(data):
                try:
                    result = json.loads(self._recognizer.Result())
                except json.JSONDecodeError as exc:
                    self.get_logger().warning(f'Vosk JSON parse error: {exc}')
                    continue
                text = result.get('text', '').strip()
                if text:
                    self.get_logger().info(f'Heard: "{text}"')
                    msg = String()
                    msg.data = text
                    self._pub.publish(msg)

    def destroy_node(self):
        self._running = False
        # Wait up to 2 s for the recognition thread to notice _running=False
        # and exit cleanly before we tear down the audio device.
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if _HW:
            self._stream.stop_stream()
            self._stream.close()
            self._audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
