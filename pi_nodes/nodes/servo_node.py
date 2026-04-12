#!/usr/bin/env python3
"""
servo_node — Claw servo control (PCA9685 channel 0).

Subscribes:
    samurai/{robot_id}/claw/command  — "open" | "close" | "angle:<0-180>"
Publishes:
    samurai/{robot_id}/claw/state   — float angle @ 10 Hz
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.servo_driver import ServoDriver


class ServoNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('servo_node', **kwargs)

        self._servo = ServoDriver(channel=0, start_disabled=True)

        self.subscribe('claw/command', self._cmd_cb)
        self.create_timer(0.1, self._publish_state)  # 10 Hz

        if self._servo.simulated:
            self.log_warn('Claw servo in SIMULATION mode')
        else:
            self.log_info('Claw servo node ready (channel 0)')

    def _cmd_cb(self, topic, data):
        cmd = str(data).strip().lower()
        if cmd == 'open':
            self._servo.open_claw()
            self.log_info('Claw OPEN')
        elif cmd == 'close':
            self._servo.close_claw()
            self.log_info('Claw CLOSE')
        elif cmd.startswith('angle:'):
            try:
                angle = float(cmd.split(':')[1])
                self._servo.set_angle(angle)
                self.log_info('Claw angle=%.1f°', angle)
            except (ValueError, IndexError):
                self.log_error('Invalid angle command: %s', cmd)
        else:
            self.log_warn('Unknown claw command: %s', cmd)

    def _publish_state(self):
        self.publish('claw/state', round(self._servo.angle, 1))


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = ServoNode(broker=args.broker, port=args.port,
                     robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
