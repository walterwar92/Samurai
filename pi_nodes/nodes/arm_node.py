#!/usr/bin/env python3
"""
arm_node — Robotic arm servo control (PCA9685 channels 0-3).

4 independent joints with angle limits from config.yaml.
Supports freeze (hold position), presets, and locked startup.

Subscribes:
    samurai/{robot_id}/arm/command — JSON:
        {"joint": 1, "angle": 90}           — single joint (1-indexed)
        {"joints": [90, 90, 90, 90]}         — all joints at once
        "home"                                — reset all to home

        {"command": "home"}                  — reset all to home
        {"command": "unlock"}                — unlock (start servo control)
        {"command": "freeze"}                — freeze all joints
        {"command": "freeze", "joint": 1}    — freeze single joint
        {"command": "unfreeze"}              — unfreeze all joints
        {"command": "unfreeze", "joint": 1}  — unfreeze single joint

        {"command": "save_preset", "name": "grab"}   — save current as preset
        {"command": "load_preset", "name": "grab"}   — load preset
        {"command": "delete_preset", "name": "grab"} — delete preset
        {"command": "list_presets"}                   — list presets

Publishes:
    samurai/{robot_id}/arm/state  — JSON @ 10 Hz:
        {"j1":..,"j2":..,"j3":..,"j4":.., "frozen":[..], "locked": bool}
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.servo_driver import ServoDriver
from pi_nodes.hardware.servo_presets import ServoPresets

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # noqa: E731


class ArmNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('arm_node', **kwargs)

        # Config — CH0=основание, CH1=сустав1, CH2=сустав2, CH3=клешня
        self._channels = cfg('servos.arm.channels', [0, 1, 2, 3])
        self._home_angles = cfg('servos.arm.home_angles', [0, 120, 0, 0])
        self._min_angles = cfg('servos.arm.min_angles', [0, 0, 0, 0])
        self._max_angles = cfg('servos.arm.max_angles', [120, 145, 180, 180])
        self._labels = cfg('servos.arm.labels',
                           ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня'])
        self._num_joints = len(self._channels)

        # Locked = arm doesn't move on startup, stays in physical position.
        # Unlock via arm/command: {"command": "unlock"} or "unlock"
        self._locked = cfg('servos.arm.locked', True)

        # Create servo drivers — start_disabled=True so no PWM on boot
        self._servos: list[ServoDriver] = []
        self._angles: list[float] = list(map(float, self._home_angles))
        for i in range(self._num_joints):
            s = ServoDriver(channel=self._channels[i],
                            init_angle=self._home_angles[i],
                            start_disabled=True)
            self._servos.append(s)

        self._servo_initialized = False

        # If not locked, immediately initialize servos to home
        if not self._locked:
            self._unlock()

        # Preset manager
        self._presets = ServoPresets()

        # MQTT
        self.subscribe('arm/command', self._cmd_cb)
        self.create_timer(0.1, self._publish_state)  # 10 Hz

        sim = any(s.simulated for s in self._servos)
        if sim:
            self.log_warn('Arm servos in SIMULATION mode')
        else:
            self.log_info('Arm node ready (%d joints, channels=%s, locked=%s)',
                          self._num_joints, self._channels, self._locked)

    def _unlock(self):
        """Unlock arm and initialize servos to home angles."""
        self._locked = False
        if not self._servo_initialized:
            for i in range(self._num_joints):
                self._servos[i].set_angle(self._home_angles[i], force=True)
                self._angles[i] = float(self._home_angles[i])
            self._servo_initialized = True

    def _set_joint(self, idx: int, angle: float):
        """Set joint angle with limits."""
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        self._servos[idx].set_angle(angle)
        if not self._servos[idx].frozen:
            self._angles[idx] = angle

    def _cmd_cb(self, topic, data):
        if isinstance(data, str):
            cmd_lower = data.strip().lower()
            if cmd_lower == 'home':
                self._unlock_if_needed()
                for i in range(self._num_joints):
                    self._set_joint(i, self._home_angles[i])
                self.log_info('Arm → HOME')
                return
            if cmd_lower == 'unlock':
                self._unlock_if_needed()
                self.log_info('Arm unlocked')
                return
            if cmd_lower == 'freeze':
                self._unlock_if_needed()
                for s in self._servos:
                    s.freeze()
                self.log_info('Arm ALL joints FROZEN')
                return
            if cmd_lower == 'unfreeze':
                for s in self._servos:
                    s.unfreeze()
                self.log_info('Arm ALL joints UNFROZEN')
                return
            try:
                data = json.loads(data)
            except (json.JSONDecodeError, ValueError):
                self.log_warn('Invalid arm command: %s', data)
                return

        if not isinstance(data, dict):
            self.log_warn('Invalid arm command type: %s', type(data))
            return

        d = data
        cmd = d.get('command', '')

        # --- Commands ---
        if cmd == 'home':
            self._unlock_if_needed()
            for i in range(self._num_joints):
                self._set_joint(i, self._home_angles[i])
            self.log_info('Arm → HOME')
            return

        if cmd == 'unlock':
            self._unlock_if_needed()
            self.log_info('Arm unlocked')
            return

        if cmd == 'freeze':
            self._unlock_if_needed()
            joint = d.get('joint')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._servos[idx].freeze()
                    self.log_info('Arm joint %d FROZEN at %.1f°',
                                  idx + 1, self._angles[idx])
            else:
                for s in self._servos:
                    s.freeze()
                self.log_info('Arm ALL joints FROZEN')
            return

        if cmd == 'unfreeze':
            joint = d.get('joint')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._servos[idx].unfreeze()
                    self.log_info('Arm joint %d UNFROZEN', idx + 1)
            else:
                for s in self._servos:
                    s.unfreeze()
                self.log_info('Arm ALL joints UNFROZEN')
            return

        if cmd == 'save_preset':
            name = d.get('name', '').strip()
            if not name:
                self.log_warn('save_preset: name required')
                return
            self._presets.save_preset('arm', name, list(self._angles))
            self.log_info('Preset saved: arm/%s = %s', name, self._angles)
            return

        if cmd == 'load_preset':
            name = d.get('name', '').strip()
            if not name:
                self.log_warn('load_preset: name required')
                return
            angles = self._presets.load_preset('arm', name)
            if angles is None:
                self.log_warn('Preset not found: arm/%s', name)
                return
            self._unlock_if_needed()
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Preset loaded: arm/%s → %s', name, self._angles)
            return

        if cmd == 'delete_preset':
            name = d.get('name', '').strip()
            if not name:
                self.log_warn('delete_preset: name required')
                return
            if self._presets.delete_preset('arm', name):
                self.log_info('Preset deleted: arm/%s', name)
            else:
                self.log_warn('Preset not found: arm/%s', name)
            return

        if cmd == 'list_presets':
            names = self._presets.list_presets('arm')
            self.publish('arm/presets', json.dumps(names))
            self.log_info('Arm presets: %s', names)
            return

        # --- Direct angle commands ---

        # Single joint: {"joint": 1, "angle": 90} (1-indexed)
        if 'joint' in d and 'angle' in d:
            self._unlock_if_needed()
            idx = int(d['joint']) - 1
            angle = float(d['angle'])
            self._set_joint(idx, angle)
            self.log_info('Arm joint %d → %.1f°', idx + 1, self._angles[idx])
            return

        # All joints: {"joints": [90, 90, 90, 90]}
        if 'joints' in d:
            self._unlock_if_needed()
            angles = d['joints']
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Arm all joints → %s', self._angles)
            return

        self.log_warn('Unknown arm command format: %s', d)

    def _unlock_if_needed(self):
        """Unlock arm if locked."""
        if self._locked:
            self._unlock()

    def _publish_state(self):
        state = {}
        for i in range(self._num_joints):
            state[f'j{i+1}'] = round(self._angles[i], 1)
        state['frozen'] = [s.frozen for s in self._servos]
        state['locked'] = self._locked
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
