#!/usr/bin/env python3
"""
head_node — Camera head pan/tilt servo control (PCA9685 channels 5, 6).

Always centers on startup (looks forward). Supports smooth movement
and absolute angle commands. Future: object tracking mode.

Subscribes:
    samurai/{robot_id}/head/command — JSON {"pan": 0-180, "tilt": 0-180}
                                     or "center" to reset to home
Publishes:
    samurai/{robot_id}/head/state  — JSON {"pan": float, "tilt": float} @ 10 Hz
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

        # Config
        self._pan_channel = cfg('servos.head.pan_channel', 5)
        self._tilt_channel = cfg('servos.head.tilt_channel', 6)
        self._pan_home = cfg('servos.head.pan_home', 90)
        self._tilt_home = cfg('servos.head.tilt_home', 90)
        self._pan_min = cfg('servos.head.pan_min', 0)
        self._pan_max = cfg('servos.head.pan_max', 180)
        self._tilt_min = cfg('servos.head.tilt_min', 30)
        self._tilt_max = cfg('servos.head.tilt_max', 150)
        self._speed = cfg('servos.head.speed', 2.0)

        # Servo drivers
        self._pan_servo = ServoDriver(channel=self._pan_channel)
        self._tilt_servo = ServoDriver(channel=self._tilt_channel)

        # Target angles (for smooth movement)
        self._pan_target = self._pan_home
        self._tilt_target = self._tilt_home

        # Current angles
        self._pan_angle = self._pan_home
        self._tilt_angle = self._tilt_home

        # Center on startup
        self._pan_servo.set_angle(self._pan_home)
        self._tilt_servo.set_angle(self._tilt_home)

        # MQTT
        self.subscribe('head/command', self._cmd_cb)
        self.create_timer(0.02, self._smooth_move)   # 50 Hz servo update
        self.create_timer(0.1, self._publish_state)   # 10 Hz state

        sim = self._pan_servo.simulated
        if sim:
            self.log_warn('Head servos in SIMULATION mode')
        else:
            self.log_info('Head node ready (pan=ch%d, tilt=ch%d, home=%d/%d)',
                          self._pan_channel, self._tilt_channel,
                          self._pan_home, self._tilt_home)

    def _cmd_cb(self, topic, data):
        text = str(data).strip()

        if text.lower() == 'center':
            self._pan_target = self._pan_home
            self._tilt_target = self._tilt_home
            self.log_info('Head → CENTER')
            return

        try:
            d = json.loads(text)
        except (json.JSONDecodeError, ValueError):
            self.log_warn('Invalid head command: %s', text)
            return

        if 'pan' in d:
            self._pan_target = max(self._pan_min,
                                   min(self._pan_max, float(d['pan'])))
        if 'tilt' in d:
            self._tilt_target = max(self._tilt_min,
                                    min(self._tilt_max, float(d['tilt'])))

        self.log_info('Head target pan=%.1f tilt=%.1f',
                      self._pan_target, self._tilt_target)

    def _smooth_move(self):
        """Move servos towards target by _speed degrees per tick."""
        self._pan_angle = self._step(self._pan_angle, self._pan_target)
        self._tilt_angle = self._step(self._tilt_angle, self._tilt_target)
        self._pan_servo.set_angle(self._pan_angle)
        self._tilt_servo.set_angle(self._tilt_angle)

    def _step(self, current: float, target: float) -> float:
        diff = target - current
        if abs(diff) < 0.5:
            return target
        step = min(abs(diff), self._speed)
        return current + step if diff > 0 else current - step

    def _publish_state(self):
        self.publish('head/state', json.dumps({
            'pan': round(self._pan_angle, 1),
            'tilt': round(self._tilt_angle, 1),
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
