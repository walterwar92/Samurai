#!/usr/bin/env python3
"""
laser_node — Laser pointer control via GPIO17.

Subscribes:
    samurai/{robot_id}/laser/command  — true/false
Safety: auto-off after 10 s continuous burn.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from gpiozero import LED
    _HW = True
except (ImportError, RuntimeError):
    _HW = False

LASER_PIN = 17
AUTO_OFF_SEC = 10.0


class LaserNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('laser_node', **kwargs)

        if _HW:
            self._laser = LED(LASER_PIN)
        else:
            self._laser = None
            self.log_warn('gpiozero unavailable — laser simulated')

        self._active = False
        self._on_time = 0.0

        self.subscribe('laser/command', self._cmd_cb)
        self.create_timer(0.5, self._safety_timer)
        self.log_info('Laser node ready on GPIO%d', LASER_PIN)

    def _cmd_cb(self, topic, data):
        on = data if isinstance(data, bool) else str(data).lower() in ('true', '1')
        if on:
            self._turn_on()
        else:
            self._turn_off()

    def _turn_on(self):
        if not self._active:
            self._active = True
            self._on_time = 0.0
            if self._laser:
                self._laser.on()
            self.log_info('Laser ON')

    def _turn_off(self):
        if self._active:
            self._active = False
            self._on_time = 0.0
            if self._laser:
                self._laser.off()
            self.log_info('Laser OFF')

    def _safety_timer(self):
        if self._active:
            self._on_time += 0.5
            if self._on_time >= AUTO_OFF_SEC:
                self.log_warn('Laser auto-off (safety timeout)')
                self._turn_off()

    def on_shutdown(self):
        if self._laser:
            self._laser.off()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = LaserNode(broker=args.broker, port=args.port,
                     robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
