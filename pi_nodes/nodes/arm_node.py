#!/usr/bin/env python3
"""
arm_node — Robotic arm servo control (PCA9685 channels 1-4).

4 independent joints with angle limits from config.yaml.

Subscribes:
    samurai/{robot_id}/arm/command — JSON:
        {"joint": 1, "angle": 90}           — single joint
        {"joints": [90, 90, 90, 90]}         — all joints at once
        "home"                                — reset all to home
Publishes:
    samurai/{robot_id}/arm/state  — JSON {"j1":..,"j2":..,"j3":..,"j4":..} @ 10 Hz
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


class ArmNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('arm_node', **kwargs)

        # Config
        self._channels = cfg('servos.arm.channels', [1, 2, 3, 4])
        self._home_angles = cfg('servos.arm.home_angles', [90, 90, 90, 90])
        self._min_angles = cfg('servos.arm.min_angles', [0, 0, 0, 0])
        self._max_angles = cfg('servos.arm.max_angles', [180, 180, 180, 180])
        self._labels = cfg('servos.arm.labels',
                           ['Joint1', 'Joint2', 'Joint3', 'Joint4'])
        self._num_joints = len(self._channels)

        # Create servo drivers for each joint
        self._servos: list[ServoDriver] = []
        self._angles: list[float] = []
        for i in range(self._num_joints):
            s = ServoDriver(channel=self._channels[i])
            s.set_angle(self._home_angles[i])
            self._servos.append(s)
            self._angles.append(float(self._home_angles[i]))

        # MQTT
        self.subscribe('arm/command', self._cmd_cb)
        self.create_timer(0.1, self._publish_state)  # 10 Hz

        sim = any(s.simulated for s in self._servos)
        if sim:
            self.log_warn('Arm servos in SIMULATION mode')
        else:
            self.log_info('Arm node ready (%d joints, channels=%s)',
                          self._num_joints, self._channels)

    def _set_joint(self, idx: int, angle: float):
        """Set joint angle with limits."""
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        self._servos[idx].set_angle(angle)
        self._angles[idx] = angle

    def _cmd_cb(self, topic, data):
        text = str(data).strip()

        if text.lower() == 'home':
            for i in range(self._num_joints):
                self._set_joint(i, self._home_angles[i])
            self.log_info('Arm → HOME')
            return

        try:
            d = json.loads(text)
        except (json.JSONDecodeError, ValueError):
            self.log_warn('Invalid arm command: %s', text)
            return

        # Single joint: {"joint": 1, "angle": 90} (1-indexed)
        if 'joint' in d and 'angle' in d:
            idx = int(d['joint']) - 1  # 1-indexed → 0-indexed
            angle = float(d['angle'])
            self._set_joint(idx, angle)
            self.log_info('Arm joint %d → %.1f°', idx + 1, self._angles[idx])
            return

        # All joints: {"joints": [90, 90, 90, 90]}
        if 'joints' in d:
            angles = d['joints']
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Arm all joints → %s', self._angles)
            return

        self.log_warn('Unknown arm command format: %s', text)

    def _publish_state(self):
        state = {}
        for i in range(self._num_joints):
            state[f'j{i+1}'] = round(self._angles[i], 1)
        self.publish('arm/state', json.dumps(state))


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = ArmNode(broker=args.broker, port=args.port,
                   robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
