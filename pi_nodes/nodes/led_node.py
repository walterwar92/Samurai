#!/usr/bin/env python3
"""
led_node — WS2812B NeoPixel LED panel control via MQTT.

Subscribes:
    samurai/{robot_id}/led/command  — JSON {mode, color?, brightness?}

Modes:
    off       — all LEDs off
    solid     — solid color (use 'color' field: 'red','green','blue','white',...)
    red/green/blue/white/yellow/orange/cyan/magenta — shortcuts for solid
    blink     — blink color 3 times
    pulse     — smooth fade in/out
    rainbow   — rainbow cycle animation
    police    — alternating red/blue flash

Publishes:
    samurai/{robot_id}/led/state — {mode, color, simulated} @ on change
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.led_driver import LedDriver, COLORS, _wheel


class LedNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('led_node', **kwargs)

        self._driver = LedDriver()
        self._current_mode = 'off'
        self._current_color = 'off'
        self._anim_thread = None
        self._anim_stop = threading.Event()

        self.subscribe('led/command', self._cmd_cb, qos=1)

        # Turn off on start
        self._driver.off()
        self._publish_state()
        self.log_info('LED node ready (%d LEDs, simulated=%s)',
                      self._driver.count, self._driver.simulated)

    def _cmd_cb(self, topic, data):
        if isinstance(data, str):
            mode = data.strip().lower()
            color_name = ''
            brightness = -1.0
        elif isinstance(data, dict):
            mode = str(data.get('mode', 'off')).strip().lower()
            color_name = str(data.get('color', '')).strip().lower()
            brightness = float(data.get('brightness', -1.0))
        else:
            return

        self.log_info('LED command: mode=%s color=%s', mode, color_name)

        # Stop any running animation
        self._stop_animation()

        # Resolve color
        if mode in COLORS:
            color_name = mode
            mode = 'solid'

        color = COLORS.get(color_name, COLORS.get('white', (255, 255, 255)))

        if mode == 'off':
            self._driver.off()
            self._current_mode = 'off'
            self._current_color = 'off'

        elif mode == 'solid':
            self._driver.set_all(color)
            self._current_mode = 'solid'
            self._current_color = color_name or 'white'

        elif mode == 'blink':
            self._current_mode = 'blink'
            self._current_color = color_name or 'white'
            self._start_animation(self._anim_blink, color)

        elif mode == 'pulse':
            self._current_mode = 'pulse'
            self._current_color = color_name or 'white'
            self._start_animation(self._anim_pulse, color)

        elif mode == 'rainbow':
            self._current_mode = 'rainbow'
            self._current_color = 'rainbow'
            self._start_animation(self._anim_rainbow)

        elif mode == 'police':
            self._current_mode = 'police'
            self._current_color = 'police'
            self._start_animation(self._anim_police)

        else:
            self.log_warn('Unknown LED mode: %s', mode)
            return

        self._publish_state()

    def _publish_state(self):
        self.publish('led/state', {
            'mode': self._current_mode,
            'color': self._current_color,
            'simulated': self._driver.simulated,
            'count': self._driver.count,
        }, qos=1, retain=True)

    # ── Animation helpers ──────────────────────────────────────
    def _stop_animation(self):
        if self._anim_thread and self._anim_thread.is_alive():
            self._anim_stop.set()
            self._anim_thread.join(timeout=2.0)
        self._anim_stop.clear()

    def _start_animation(self, func, *args):
        self._anim_thread = threading.Thread(
            target=func, args=args, daemon=True)
        self._anim_thread.start()

    def _anim_blink(self, color):
        off = (0, 0, 0)
        while not self._anim_stop.is_set():
            self._driver.set_all(color)
            if self._anim_stop.wait(0.3):
                return
            self._driver.set_all(off)
            if self._anim_stop.wait(0.3):
                return

    def _anim_pulse(self, color):
        r, g, b = color
        steps = 25
        delay = 0.03
        while not self._anim_stop.is_set():
            # Fade in
            for i in range(steps + 1):
                if self._anim_stop.is_set():
                    return
                k = i / steps
                self._driver.set_all((int(r * k), int(g * k), int(b * k)))
                time.sleep(delay)
            # Fade out
            for i in range(steps, -1, -1):
                if self._anim_stop.is_set():
                    return
                k = i / steps
                self._driver.set_all((int(r * k), int(g * k), int(b * k)))
                time.sleep(delay)
            time.sleep(0.1)

    def _anim_rainbow(self):
        n = self._driver.count
        j = 0
        while not self._anim_stop.is_set():
            colors = [_wheel((i * 256 // n + j) & 255) for i in range(n)]
            self._driver.set_colors(colors)
            j = (j + 1) % 256
            time.sleep(0.02)

    def _anim_police(self):
        n = self._driver.count
        half = n // 2
        red = (255, 0, 0)
        blue = (0, 0, 255)
        off = (0, 0, 0)
        while not self._anim_stop.is_set():
            # Left red, right off
            colors = [red if i < half else off for i in range(n)]
            self._driver.set_colors(colors)
            if self._anim_stop.wait(0.15):
                return
            # All off
            self._driver.off()
            if self._anim_stop.wait(0.05):
                return
            # Left off, right blue
            colors = [off if i < half else blue for i in range(n)]
            self._driver.set_colors(colors)
            if self._anim_stop.wait(0.15):
                return
            self._driver.off()
            if self._anim_stop.wait(0.05):
                return

    def on_shutdown(self):
        self._stop_animation()
        self._driver.off()
        self._driver.deinit()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = LedNode(broker=args.broker, port=args.port,
                   robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
