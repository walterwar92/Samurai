#!/usr/bin/env python3
"""
tts_node — Voice announcements for robot events.

Subscribes to various MQTT topics and generates spoken reports
using pyttsx3 (offline) with espeak fallback.

Subscribes:
    samurai/{robot_id}/ball_detection  — announce detected ball color & distance
    samurai/{robot_id}/status          — announce FSM state changes
    samurai/{robot_id}/range           — warn about close obstacles
    samurai/{robot_id}/battery         — warn about low battery
    samurai/{robot_id}/tts/command     — speak arbitrary text
    samurai/{robot_id}/tts/enable      — enable/disable TTS (payload: true/false)
Publishes:
    (none)
"""

import json
import os
import subprocess
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# Cooldown: don't repeat the same message within this interval
MESSAGE_COOLDOWN_S = 10.0

# Obstacle warning threshold (meters)
OBSTACLE_WARN_M = 0.20

# Battery warning threshold (percent)
BATTERY_WARN_PCT = 20

# Russian state name mapping
STATE_NAMES_RU = {
    'idle':         'ожидание',
    'search':       'поиск',
    'approach':     'подъезд',
    'grab':         'захват',
    'return_home':  'возврат домой',
    'deliver':      'доставка',
    'patrol':       'патрулирование',
    'follow':       'следование',
    'manual':       'ручное управление',
    'error':        'ошибка',
    'charging':     'зарядка',
}

# Color name mapping
COLOR_NAMES_RU = {
    'red':    'красный',
    'blue':   'синий',
    'green':  'зелёный',
    'yellow': 'жёлтый',
    'orange': 'оранжевый',
    'white':  'белый',
    'black':  'чёрный',
}


class TTSEngine:
    """Wrapper: tries pyttsx3, falls back to espeak subprocess."""

    def __init__(self, logger):
        self._log = logger
        self._engine = None
        self._use_espeak = False
        self._lock = threading.Lock()
        self._init_engine()

    def _init_engine(self):
        try:
            import pyttsx3
            self._engine = pyttsx3.init()
            # Try to set Russian voice
            voices = self._engine.getProperty('voices')
            for v in voices:
                if 'ru' in v.id.lower() or 'russian' in v.name.lower():
                    self._engine.setProperty('voice', v.id)
                    self._log('pyttsx3: Russian voice set — %s', v.name)
                    break
            else:
                self._log('pyttsx3: no Russian voice found, using default')
            self._engine.setProperty('rate', 150)
            self._log('TTS engine: pyttsx3')
        except Exception as e:
            self._log('pyttsx3 unavailable (%s), falling back to espeak', e)
            self._engine = None
            self._use_espeak = True

    def speak(self, text):
        with self._lock:
            if self._use_espeak:
                self._speak_espeak(text)
            else:
                self._speak_pyttsx3(text)

    def _speak_pyttsx3(self, text):
        try:
            self._engine.say(text)
            self._engine.runAndWait()
        except Exception as e:
            self._log('pyttsx3 error: %s — switching to espeak', e)
            self._use_espeak = True
            self._speak_espeak(text)

    def _speak_espeak(self, text):
        try:
            subprocess.run(
                ['espeak', '-v', 'ru', text],
                timeout=15,
                capture_output=True,
            )
        except FileNotFoundError:
            self._log('espeak not found — TTS unavailable')
        except subprocess.TimeoutExpired:
            self._log('espeak timed out for: %s', text[:50])
        except Exception as e:
            self._log('espeak error: %s', e)


class TTSNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('tts_node', **kwargs)

        self._enabled = True
        self._last_state = None
        self._cooldowns = {}          # message_key → last_spoken_time
        self._queue = deque(maxlen=20)
        self._queue_event = threading.Event()
        self._shutdown = False

        self._tts = TTSEngine(self.log_info)

        # Subscribe to topics
        self.subscribe('ball_detection', self._ball_detection_cb)
        self.subscribe('status', self._status_cb)
        self.subscribe('range', self._range_cb)
        self.subscribe('battery', self._battery_cb)
        self.subscribe('tts/command', self._command_cb)
        self.subscribe('tts/enable', self._enable_cb)

        # Speech worker thread
        self._worker = threading.Thread(target=self._speech_worker,
                                        daemon=True, name='tts_worker')
        self._worker.start()

        self.log_info('tts_node ready (enabled=%s)', self._enabled)

    # ── topic callbacks ─────────────────────────────────────────────

    def _ball_detection_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        color = data.get('color', 'unknown')
        distance = data.get('distance', 0)
        color_ru = COLOR_NAMES_RU.get(color, color)
        text = f'Обнаружен {color_ru} мяч на расстоянии {distance:.1f} метров'
        self._enqueue(f'ball_{color}', text)

    def _status_cb(self, topic, data):
        if isinstance(data, dict):
            state = data.get('state', '')
        elif isinstance(data, str):
            try:
                parsed = json.loads(data)
                state = parsed.get('state', data)
            except (json.JSONDecodeError, AttributeError):
                state = data
        else:
            return

        state_lower = state.lower().strip()
        if state_lower == self._last_state:
            return
        self._last_state = state_lower
        state_ru = STATE_NAMES_RU.get(state_lower, state)
        text = f'Переход в состояние {state_ru}'
        self._enqueue(f'state_{state_lower}', text)

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = data.get('range', 2.0)
        else:
            try:
                r = float(data)
            except (ValueError, TypeError):
                return
        if r < OBSTACLE_WARN_M:
            self._enqueue('obstacle_warn', 'Препятствие впереди')

    def _battery_cb(self, topic, data):
        if isinstance(data, dict):
            pct = data.get('percent', 100)
        else:
            try:
                pct = float(data)
            except (ValueError, TypeError):
                return
        if pct < BATTERY_WARN_PCT:
            self._enqueue('battery_low', 'Батарея разряжена')

    def _command_cb(self, topic, data):
        if isinstance(data, dict):
            text = data.get('text', '')
        elif isinstance(data, str):
            text = data
        else:
            return
        if text:
            self._enqueue(None, text)  # No cooldown key — always speak

    def _enable_cb(self, topic, data):
        if isinstance(data, dict):
            val = data.get('enabled', data.get('value', True))
        elif isinstance(data, str):
            val = data.lower().strip() in ('true', '1', 'on', 'yes')
        elif isinstance(data, bool):
            val = data
        else:
            val = bool(data)
        self._enabled = val
        self.log_info('TTS %s', 'enabled' if val else 'disabled')

    # ── speech queue ────────────────────────────────────────────────

    def _enqueue(self, cooldown_key, text):
        if not self._enabled:
            return
        now = time.monotonic()
        if cooldown_key is not None:
            last = self._cooldowns.get(cooldown_key, 0)
            if now - last < MESSAGE_COOLDOWN_S:
                return
            self._cooldowns[cooldown_key] = now
        self._queue.append(text)
        self._queue_event.set()

    def _speech_worker(self):
        while not self._shutdown:
            self._queue_event.wait(timeout=1.0)
            self._queue_event.clear()
            while self._queue and not self._shutdown:
                text = self._queue.popleft()
                self.log_info('Speaking: %s', text)
                self._tts.speak(text)

    def on_shutdown(self):
        self._shutdown = True
        self._queue_event.set()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='TTS voice reports node')
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = TTSNode(
        broker=args.broker, port=args.port,
        robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
