#!/usr/bin/env python3
"""
voice_node — Offline Russian speech recognition via Vosk API.

Publishes:
    samurai/{robot_id}/voice_command  — recognized text
"""

import json
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda k, d=None: d

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

VOSK_MODEL_PATH = os.path.expanduser(cfg('voice.model_path', '~/vosk-model-ru'))
SAMPLE_RATE = cfg('voice.sample_rate', 16000)
CHUNK_SIZE = cfg('voice.chunk_size', 4000)
VAD_FRAME_BYTES = cfg('voice.vad_frame_bytes', 960)
VAD_AGGRESSIVENESS = cfg('voice.vad_aggressiveness', 2)


class VoiceNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('voice_node', **kwargs)

        self._audio = None
        self._stream = None
        self._listen_running = False
        self._thread = None

        if not _HW:
            self.log_error('vosk/pyaudio not available — voice disabled')
            return

        if _VAD:
            self._vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
            self.log_info('VAD enabled (webrtcvad)')
        else:
            self._vad = None
            self.log_warn('webrtcvad not installed — VAD disabled')

        self.log_info('Loading Vosk model from %s ...', VOSK_MODEL_PATH)
        self._model = vosk.Model(VOSK_MODEL_PATH)
        self._recognizer = vosk.KaldiRecognizer(self._model, SAMPLE_RATE)

        self._audio = pyaudio.PyAudio()
        self._stream = self._audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        self._listen_running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.log_info('Voice recognition started (Russian, offline)')

    def _has_speech(self, data: bytes) -> bool:
        if self._vad is None:
            return True
        for i in range(0, len(data) - VAD_FRAME_BYTES + 1, VAD_FRAME_BYTES):
            frame = data[i:i + VAD_FRAME_BYTES]
            if len(frame) == VAD_FRAME_BYTES and self._vad.is_speech(frame, SAMPLE_RATE):
                return True
        return False

    def _listen_loop(self):
        while self._listen_running:
            try:
                data = self._stream.read(CHUNK_SIZE, exception_on_overflow=False)
            except OSError as exc:
                self.log_warn('Audio read error: %s', exc)
                continue
            if not self._has_speech(data):
                continue
            if self._recognizer.AcceptWaveform(data):
                try:
                    result = json.loads(self._recognizer.Result())
                except json.JSONDecodeError:
                    continue
                text = result.get('text', '').strip()
                if text:
                    self.log_info('Heard: "%s"', text)
                    self.publish('voice_command', text, qos=1)

    def on_shutdown(self):
        self._listen_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
        if self._audio:
            self._audio.terminate()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = VoiceNode(broker=args.broker, port=args.port,
                     robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
