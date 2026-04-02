#!/usr/bin/env python3
"""
head_node — Camera head servo control (PCA9685 channel 4).

Single servo for horizontal pan. Centers on startup (looks forward).
Supports smooth movement and absolute angle commands.
Future: object tracking mode.

Subscribes:
    samurai/{robot_id}/head/command — JSON {"angle": 0-180}
                                     or "center" to reset to home
Publishes:
    samurai/{robot_id}/head/state  — JSON {"angle": float} @ 10 Hz
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.servo_driver import ServoDriver

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # noqa: E731


class HeadNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('head_node', **kwargs)

        # Config — CH4=голова
        self._channel = cfg('servos.head.channel', 4)
        self._home = cfg('servos.head.home', 90)
        self._min = cfg('servos.head.min', 0)
        self._max = cfg('servos.head.max', 180)
        self._speed = cfg('servos.head.speed', 2.0)

        # Locked = голова не двигается, остаётся в текущем положении.
        # При запуске locked=True → серво НЕ дёргается, остаётся как есть.
        # Разблокировка через head/command: {"command": "unlock"}
        self._locked = cfg('servos.head.locked', True)

        # Servo driver — при locked=True не посылаем PWM при старте,
        # голова остаётся в физическом положении при включении.
        self._servo = ServoDriver(channel=self._channel,
                                  init_angle=self._home,
                                  start_disabled=self._locked)

        # Target & current angle
        self._target = self._home
        self._angle = self._home

        if self._locked:
            # НЕ отправляем команду серво — оставляем в том положении,
            # в котором голова физически находится при включении.
            # PCA9685 не будет генерировать PWM пока мы не вызовем set_angle.
            self._servo_initialized = False
        else:
            # Разблокирована — ставим в home при старте
            self._servo.set_angle(self._home)
            self._servo_initialized = True

        # MQTT
        self.subscribe('head/command', self._cmd_cb)
        self.create_timer(0.02, self._smooth_move)   # 50 Hz servo update
        self.create_timer(0.1, self._publish_state)   # 10 Hz state

        sim = self._servo.simulated
        if sim:
            self.log_warn('Head servo in SIMULATION mode')
        else:
            self.log_info('Head node ready (ch%d, home=%d, locked=%s)',
                          self._channel, self._home, self._locked)

    def _cmd_cb(self, topic, data):
        # MqttNode with parse_json=True already decodes JSON → dict.
        # But "center" string stays as str.
        if isinstance(data, str):
            cmd_lower = data.strip().lower()
            if cmd_lower == 'center':
                self._unlock_if_needed()
                self._target = self._home
                self.log_info('Head → CENTER')
                return
            if cmd_lower == 'unlock':
                self._unlock_if_needed()
                self.log_info('Head unlocked')
                return
            if cmd_lower == 'lock':
                self._locked = True
                self.log_info('Head locked at %.1f°', self._angle)
                return
            try:
                data = json.loads(data)
            except (json.JSONDecodeError, ValueError):
                self.log_warn('Invalid head command: %s', data)
                return

        if not isinstance(data, dict):
            self.log_warn('Invalid head command type: %s', type(data))
            return

        d = data

        if d.get('command') == 'center':
            self._unlock_if_needed()
            self._target = self._home
            self.log_info('Head → CENTER')
            return

        if d.get('command') == 'unlock':
            self._unlock_if_needed()
            self.log_info('Head unlocked')
            return

        if d.get('command') == 'lock':
            self._locked = True
            self.log_info('Head locked at %.1f°', self._angle)
            return

        if 'angle' in d:
            self._unlock_if_needed()
            self._target = max(self._min,
                               min(self._max, float(d['angle'])))
            self.log_info('Head target=%.1f', self._target)

    def _unlock_if_needed(self):
        """Unlock head and initialize servo if first move."""
        self._locked = False
        if not self._servo_initialized:
            self._servo.set_angle(self._angle)
            self._servo_initialized = True

    def _smooth_move(self):
        """Move servo towards target by _speed degrees per tick."""
        if self._locked:
            return
        self._angle = self._step(self._angle, self._target)
        self._servo.set_angle(self._angle)

    def _step(self, current: float, target: float) -> float:
        diff = target - current
        if abs(diff) < 0.5:
            return target
        step = min(abs(diff), self._speed)
        return current + step if diff > 0 else current - step

    def _publish_state(self):
        self.publish('head/state', json.dumps({
            'angle': round(self._angle, 1),
        }))


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = HeadNode(broker=args.broker, port=args.port,
                    robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
