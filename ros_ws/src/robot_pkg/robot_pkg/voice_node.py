#!/usr/bin/env python3
"""
voice_node — Offline Russian speech recognition via Vosk API.

Listens to microphone, recognises commands, publishes to /voice_command.
Commands:
  'получи [цвет] шарик'  — acquire ball of given colour
  'вызови вторую машину'  — call second robot
  Any other phrase         — published as-is for FSM interpretation

Vosk model path: ~/vosk-model-ru  (download vosk-model-small-ru-0.22)
"""

import json
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

VOSK_MODEL_PATH = '/home/pi/vosk-model-ru'
SAMPLE_RATE = 16000
CHUNK_SIZE = 4000


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.declare_parameter('model_path', VOSK_MODEL_PATH)
        model_path = self.get_parameter('model_path').value

        self._pub = self.create_publisher(String, '/voice_command', 10)

        if not _HW:
            self.get_logger().error('vosk/pyaudio not available — voice disabled')
            return

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

    def _listen_loop(self):
        while self._running:
            data = self._stream.read(CHUNK_SIZE, exception_on_overflow=False)
            if self._recognizer.AcceptWaveform(data):
                result = json.loads(self._recognizer.Result())
                text = result.get('text', '').strip()
                if text:
                    self.get_logger().info(f'Heard: "{text}"')
                    msg = String()
                    msg.data = text
                    self._pub.publish(msg)

    def destroy_node(self):
        self._running = False
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
