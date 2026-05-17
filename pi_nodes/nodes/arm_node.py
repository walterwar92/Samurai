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
import math
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.hardware.servo_driver import ServoDriver
from pi_nodes.hardware.servo_presets import ServoPresets

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # noqa: E731


class ArmNode(MqttNode):
    # Период интерполяционного таймера (с). 50Гц — баланс между плавностью
    # и нагрузкой на I²C; ServoDriver.HOLD_TIME=0.5с при таком темпе даёт
    # ~25 idle-тиков перед release PWM, что норм.
    _TICK_DT = 0.02

    def __init__(self, **kwargs):
        super().__init__('arm_node', **kwargs)

        # Config — CH0=основание, CH1=сустав1, CH2=сустав2, CH3=клешня
        self._channels = cfg('servos.arm.channels', [0, 1, 2, 3])
        self._home_angles = cfg('servos.arm.home_angles', [0, 120, 0, 0])
        self._min_angles = cfg('servos.arm.min_angles', [0, 0, 0, 0])
        self._max_angles = cfg('servos.arm.max_angles', [120, 145, 180, 180])
        # Per-channel inversion (физически инвертированные серво).
        # Логический угол: 0..180 (как видит UI). Physical = max - logical (если invert=true).
        # Это позволяет держать API/UI в "учебных" координатах (0=home, 180=крайнее),
        # независимо от того, как монтирован конкретный серво. По умолчанию — никаких инверсий.
        self._invert = cfg('servos.arm.invert_angles', [False] * len(self._channels))
        self._labels = cfg('servos.arm.labels',
                           ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня'])
        self._num_joints = len(self._channels)

        # Locked = arm doesn't move on startup, stays in physical position.
        # Unlock via arm/command: {"command": "unlock"} or "unlock"
        self._locked = cfg('servos.arm.locked', True)

        # Create servo drivers — start_disabled=True so no PWM on boot.
        # init_angle для ServoDriver принимает физический угол, поэтому
        # для инвертированных каналов нужно физически выставить max - home.
        # Это не имеет эффекта при start_disabled=True (PWM не идёт), но при
        # последующем set_angle(force=True) важно правильно посчитать.
        self._servos: list[ServoDriver] = []
        # Target — куда хотим. Current — где реально сервопривод сейчас.
        # Интерполятор шагает current → target с max_speed_deg_per_sec.
        # Все источники углов (joint+angle, joints[], load_preset, home)
        # ставят только target; сглаживание сквозное.
        self._target_angles: list[float] = list(map(float, self._home_angles))
        self._current_angles: list[float] = list(map(float, self._home_angles))
        # Максимальная угловая скорость для интерполяции (°/сек).
        # Защита: max_speed=0 «зависил» бы руку — clamp до 1.0.
        self._max_speed: float = max(
            1.0, float(cfg('servos.arm.max_speed_deg_per_sec', 120.0)))
        # Lock for _target_angles / _current_angles / interpolator step.
        # Защищает от race-условий между MQTT callback thread (команды
        # ставят target, вызывают freeze) и timer thread @ 50Гц
        # (_interpolate_tick читает frozen, шагает current, шлёт PWM).
        # Без lock'а freeze в момент tick'а может оставить
        # _current_angles[i] на 2.4° впереди реального PWM до unfreeze.
        self._state_lock = threading.Lock()
        for i in range(self._num_joints):
            init_phys = self._to_physical(i, float(self._home_angles[i]))
            s = ServoDriver(channel=self._channels[i],
                            init_angle=init_phys,
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
        # Интерполятор: шагаем _current → _target @ 50Гц.
        # PCA9685 I²C-команды принимаются без проблем; ServoDriver.set_angle
        # уже клампит 0..180 и обрабатывает frozen.
        self.create_timer(self._TICK_DT, self._interpolate_tick)

        sim = any(s.simulated for s in self._servos)
        if sim:
            self.log_warn('Arm servos in SIMULATION mode')
        else:
            self.log_info('Arm node ready (%d joints, channels=%s, locked=%s)',
                          self._num_joints, self._channels, self._locked)

    def _to_physical(self, idx: int, logical: float) -> float:
        """Translate logical UI angle → physical servo angle (применяет инверсию)."""
        if 0 <= idx < self._num_joints and self._invert[idx]:
            return float(self._max_angles[idx]) - float(logical) + float(self._min_angles[idx])
        return float(logical)

    def _unlock(self):
        """Unlock arm and initialize servos to home angles."""
        self._locked = False
        if not self._servo_initialized:
            with self._state_lock:
                for i in range(self._num_joints):
                    phys = self._to_physical(i, self._home_angles[i])
                    self._servos[i].set_angle(phys, force=True)
                    self._target_angles[i] = self._current_angles[i] = float(self._home_angles[i])
            self._servo_initialized = True

    def _set_joint(self, idx: int, angle: float):
        """Set joint TARGET angle (логический) с лимитами.

        Реальный PWM шлёт _interpolate_tick @ 50Гц, плавно шагая current
        к target с max_speed_deg_per_sec. Если сустав frozen — target всё
        равно обновляется (чтобы после unfreeze сразу поехать к нему).
        """
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        with self._state_lock:
            self._target_angles[idx] = angle

    def _interpolate_tick(self):
        """Шаг интерполяции: current → target со скоростью _max_speed.

        Вызывается таймером @ 50Гц. Для каждого сустава:
        - если frozen → пропускаем (PWM-удержание уже у драйвера);
        - если |delta| <= max_step → snap current к target (без overshoot);
        - иначе current += sign(delta) * max_step.
        Реальный PWM выставляется через ServoDriver.set_angle (с учётом
        инверсии — _to_physical).
        """
        dt = self._TICK_DT
        max_step = self._max_speed * dt
        with self._state_lock:
            for i in range(self._num_joints):
                if self._servos[i].frozen:
                    continue
                delta = self._target_angles[i] - self._current_angles[i]
                if delta == 0.0:
                    continue
                if abs(delta) <= max_step:
                    self._current_angles[i] = self._target_angles[i]
                else:
                    self._current_angles[i] += math.copysign(max_step, delta)
                phys = self._to_physical(i, self._current_angles[i])
                self._servos[i].set_angle(phys)

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
                                  idx + 1, self._target_angles[idx])
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
            with self._state_lock:
                snapshot = list(self._current_angles)
            self._presets.save_preset('arm', name, snapshot)
            self.log_info('Preset saved: arm/%s = %s', name, snapshot)
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
            self.log_info('Preset loaded: arm/%s → %s', name, self._target_angles)
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
            self.log_info('Arm joint %d → %.1f°', idx + 1, self._target_angles[idx])
            return

        # All joints: {"joints": [90, 90, 90, 90]}
        if 'joints' in d:
            self._unlock_if_needed()
            angles = d['joints']
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Arm all joints → %s', self._target_angles)
            return

        self.log_warn('Unknown arm command format: %s', d)

    def _unlock_if_needed(self):
        """Unlock arm if locked."""
        if self._locked:
            self._unlock()

    def _publish_state(self):
        with self._state_lock:
            angles_snapshot = list(self._current_angles)
            frozen = [s.frozen for s in self._servos]
        state = {}
        for i in range(self._num_joints):
            state[f'j{i+1}'] = round(angles_snapshot[i], 1)
        state['frozen'] = frozen
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
