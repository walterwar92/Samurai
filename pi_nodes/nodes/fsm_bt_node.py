#!/usr/bin/env python3
"""
fsm_bt_node — Behaviour Tree альтернатива классическому fsm_node.py (#1).

Использует pi_nodes/bt/ (py_trees) вместо if/elif. Логика поведения та же
(охота за мячом, manual override, return home), но композиция декларативная
через дерево selector/sequence.

Запуск:
    # Параллельно с обычным fsm_node — НЕ запускать оба одновременно,
    # они оба публикуют cmd_vel и будут конкурировать.
    python -m pi_nodes.robot_launcher --nodes motor,imu,...,fsm_bt,...

Подписки (тот же набор что у fsm_node):
    voice_command   — парсинг целевого цвета и команд
    ball_detection  — обновляет blackboard.detection
    range           — blackboard.range_m
    odom            — blackboard.pose
    cmd_vel/manual  — устанавливает manual_override (BT тогда idle)
    gesture/command — то же что voice
    call_robot      — установка target от другого робота

Публикации:
    cmd_vel         — из BT через bb.send_cmd_vel
    claw/command    — из BT через bb.send_claw
    status          — снапшот с bt_status и активной веткой

Не реализовано (требует отдельных state — TODO):
    PATROLLING / FOLLOWING / PATH_REPLAY — это специальные режимы,
    в FSM они переключают другие ноды через MQTT-команды. В BT-варианте
    нужен отдельный sub-tree который активируется voice-командами.
    Пока эти команды просто публикуют patrol/command, follow_me/command
    и т.д. — но сам BT их не учитывает (manual_override-семантика).
"""
from __future__ import annotations

import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode
from pi_nodes.bt import BehaviourTreeRunner, RobotBlackboard

# ── Reuse тех же regex что у классического FSM ──────────────────────────
_MAX_CMD_LEN = 200
_P_CALLING = re.compile(r'вызови.{0,20}машин')
_P_GRAB    = re.compile(r'получи|возьми|найди')
_P_STOP    = re.compile(r'стоп|остановись|стой')
_P_HOME    = re.compile(r'домой|вернись')
_P_PATROL  = re.compile(r'патрул|обход')
_P_FOLLOW  = re.compile(r'следуй|за мной')
_P_RECORD  = re.compile(r'запиши путь|запись')
_P_REPLAY  = re.compile(r'воспроизведи|повтори путь')
_P_RESET   = re.compile(r'сбрось позицию|сброс позиции|обнули позицию|нулевая позиция')
_P_FORWARD = re.compile(r'вперёд|вперед|прямо|едь вперёд|двигайся вперёд')
_P_BACK    = re.compile(r'назад|едь назад|двигайся назад|сдай назад')
_P_LEFT    = re.compile(r'налево|влево|поверни влево|повернись влево')
_P_RIGHT   = re.compile(r'направо|вправо|поверни вправо|повернись вправо')

_COLOURS_RU = {
    'красн': 'red', 'синий': 'blue', 'синего': 'blue',
    'зелен': 'green', 'желт': 'yellow', 'бел': 'white',
    'черн': 'black', 'оранж': 'orange',
}

# Сколько секунд после последнего cmd_vel/manual считать manual override
# активным (после этого BT возвращает контроль).
MANUAL_OVERRIDE_TIMEOUT_S = 1.0

# Минимальная уверенность LLM для исполнения voice/intent (#2, 2026-04).
_INTENT_MIN_CONFIDENCE = 0.5

# Tick FSM на 10 Hz (как fsm_node)
TICK_INTERVAL_S = 0.1


class FSMBTNode(MqttNode):
    """Behaviour-Tree-driven автономия. Параллельная альтернатива FSMNode."""

    def __init__(self, **kwargs):
        super().__init__('fsm_bt_node', **kwargs)

        # Blackboard + дерево
        self._bb = RobotBlackboard()
        # Подключаем action callbacks к MQTT publish:
        self._bb.send_cmd_vel = self._send_cmd_vel
        self._bb.send_claw = self._send_claw
        self._bb.log = self._log

        self._runner = BehaviourTreeRunner(self._bb)

        self._last_manual_ts = 0.0
        # Pre-allocated dict для cmd_vel — переиспользуется в hot path
        self._cmd_msg = {'linear_x': 0.0, 'angular_z': 0.0}

        # Подписки — ровно тот же набор что у FSMNode
        self.subscribe('voice_command', self._voice_cb, qos=1)
        # voice/intent — структурированный intent от compute_node/llm_voice
        self.subscribe('voice/intent', self._intent_cb, qos=1)
        self.subscribe('ball_detection', self._ball_cb)
        self.subscribe('range', self._range_cb)
        self.subscribe('odom', self._odom_cb)
        self.subscribe('cmd_vel/manual', self._manual_cb)
        self.subscribe('gesture/command', self._gesture_cb, qos=1)
        self.subscribe('call_robot', self._call_recv_cb, qos=1)

        self.create_timer(TICK_INTERVAL_S, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.log_info('FSM-BT node started (py_trees backend)')

    # ── Voice / gesture parsers (та же логика что в fsm_node) ──────────
    def _voice_cb(self, topic, data):
        raw = str(data)
        if not raw or len(raw) > _MAX_CMD_LEN:
            self.log_warn('Voice command rejected: length=%d', len(raw))
            return
        text = raw.lower().strip()
        self.log_info('Voice: "%s"', text)

        if _P_CALLING.search(text):
            other_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
            self.publish_raw(f'samurai/{other_id}/call_robot', {
                'colour': self._bb.target_colour,
                'action': self._bb.target_action or 'grab',
            }, qos=1)
            self.log_info('Called other robot')
            return

        if _P_GRAB.search(text):
            colour = self._extract_colour(text)
            self._bb.set_target(colour, 'grab')
            self.log_info('Target set: grab %s ball', colour or 'any')
            return

        if _P_STOP.search(text):
            self._bb.clear_target()
            self._send_cmd_vel(0.0, 0.0)
            return

        # Ручное движение → cmd_vel/manual (имитирует что фронт сделал)
        if _P_FORWARD.search(text):
            self._publish_manual(0.15, 0.0)
            return
        if _P_BACK.search(text):
            self._publish_manual(-0.15, 0.0)
            return
        if _P_LEFT.search(text):
            self._publish_manual(0.0, 0.5)
            return
        if _P_RIGHT.search(text):
            self._publish_manual(0.0, -0.5)
            return

        if _P_HOME.search(text):
            # Имитация RETURNING: отметим grabbed=True (тогда DeliverHome ветка
            # активируется и поедет home_xy; после прибытия ClearTarget сбросит).
            self._bb.grabbed = True
            self.log_info('Return home requested')
            return

        if _P_PATROL.search(text):
            self.publish('patrol/command', 'start', qos=1)
            self.log_info('Patrol delegated to patrol_node')
            return

        if _P_FOLLOW.search(text):
            self.publish('follow_me/command', 'start', qos=1)
            self.log_info('Follow-me delegated')
            return

        if _P_RECORD.search(text):
            self.publish('path_recorder/command', 'record', qos=1)
            self.log_info('Path recording started')
            return

        if _P_REPLAY.search(text):
            self.publish('path_recorder/command', 'replay', qos=1)
            self.log_info('Path replay started')
            return

        if _P_RESET.search(text):
            self.publish('reset_position', 'reset', qos=1)
            self.log_info('Position reset')
            return

    def _intent_cb(self, topic, data):
        """Структурированный intent от LLM (#2). Если confidence высокий —
        выполняем action, иначе игнорим (regex-fallback из _voice_cb)."""
        if not isinstance(data, dict):
            try:
                data = json.loads(str(data))
            except (json.JSONDecodeError, TypeError):
                return
        try:
            confidence = float(data.get('confidence', 0.0))
        except (ValueError, TypeError):
            confidence = 0.0
        if confidence < _INTENT_MIN_CONFIDENCE:
            return

        action = str(data.get('action', 'idle'))
        colour = data.get('colour') or ''
        direction = data.get('direction')
        raw = str(data.get('raw_text', ''))[:_MAX_CMD_LEN]
        self.log_info(
            'LLM intent: action=%s colour=%s dir=%s conf=%.2f raw="%s"',
            action, colour, direction, confidence, raw,
        )

        if action == 'grab':
            self._bb.set_target(colour, 'grab')
        elif action == 'stop':
            self._bb.clear_target()
            self._send_cmd_vel(0.0, 0.0)
        elif action == 'home':
            self._bb.grabbed = True  # активирует DeliverHome ветку
        elif action == 'patrol':
            self.publish('patrol/command', 'start', qos=1)
        elif action == 'follow':
            self.publish('follow_me/command', 'start', qos=1)
        elif action == 'record_path':
            self.publish('path_recorder/command', 'record', qos=1)
        elif action == 'replay_path':
            self.publish('path_recorder/command', 'replay', qos=1)
        elif action == 'reset_position':
            self.publish('reset_position', 'reset', qos=1)
        elif action == 'call_other_robot':
            other_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
            self.publish_raw(f'samurai/{other_id}/call_robot', {
                'colour': self._bb.target_colour,
                'action': self._bb.target_action or 'grab',
            }, qos=1)
        elif action == 'move':
            speeds = {
                'forward': (0.15, 0.0),
                'back':    (-0.15, 0.0),
                'left':    (0.0, 0.5),
                'right':   (0.0, -0.5),
            }
            if direction in speeds:
                lin, ang = speeds[direction]
                self._publish_manual(lin, ang)
        # transition в BT-варианте отсутствует (нет явных state'ов)
        # action == 'idle' / unknown — игнорим

    def _gesture_cb(self, topic, data):
        gesture = str(data).strip()
        if not gesture:
            return
        if gesture == 'stop':
            self._bb.clear_target()
            self._send_cmd_vel(0.0, 0.0)
        elif gesture == 'forward':
            self._publish_manual(0.15, 0.0)
        elif gesture == 'grab':
            self._bb.set_target('', 'grab')
        elif gesture == 'follow':
            self.publish('follow_me/command', 'start', qos=1)
        elif gesture == 'point_left':
            self._publish_manual(0.0, 0.5)
        elif gesture == 'point_right':
            self._publish_manual(0.0, -0.5)

    def _extract_colour(self, text: str) -> str:
        text_norm = text.replace('ё', 'е')
        for rus, eng in _COLOURS_RU.items():
            if rus in text_norm:
                return eng
        return ''

    # ── Sensor callbacks ───────────────────────────────────────────────
    def _ball_cb(self, topic, data):
        if isinstance(data, dict):
            self._bb.update_detection(data)
        else:
            try:
                self._bb.update_detection(json.loads(str(data)))
            except (json.JSONDecodeError, TypeError):
                self._bb.update_detection(None)

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = data.get('range', float('inf'))
        else:
            try:
                r = float(data)
            except (ValueError, TypeError):
                return
        self._bb.update_range(float(r))

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        x = float(data.get('x', 0.0)) / 100.0  # см → м
        y = float(data.get('y', 0.0)) / 100.0
        theta = float(data.get('theta', 0.0))
        self._bb.update_pose(x, y, theta)

    def _manual_cb(self, topic, data):
        """cmd_vel/manual от фронта/voice — переключаем BT в idle на ~1с."""
        if not isinstance(data, dict):
            return
        lin = float(data.get('linear_x', 0.0))
        ang = float(data.get('angular_z', 0.0))
        # Если manual публикует ноль — это часто остановка автономии,
        # тоже считаем override (BT не вмешивается на короткое время).
        self._last_manual_ts = time.monotonic()
        self._bb.set_manual_override(True)
        # Сразу прокидываем в cmd_vel (BT в idle на этот тик не помешает)
        self._cmd_msg['linear_x'] = round(lin, 3)
        self._cmd_msg['angular_z'] = round(ang, 3)
        self.publish('cmd_vel', self._cmd_msg)

    def _call_recv_cb(self, topic, data):
        self.log_info('Incoming call: %s', data)
        if isinstance(data, dict):
            self._bb.set_target(
                data.get('colour', ''),
                data.get('action', 'grab'),
            )

    # ── Tick + status ──────────────────────────────────────────────────
    def _tick(self):
        # Обновим manual_override таймаут
        if (
            self._bb.manual_override
            and time.monotonic() - self._last_manual_ts > MANUAL_OVERRIDE_TIMEOUT_S
        ):
            self._bb.set_manual_override(False)
        self._runner.tick()

    def _publish_status(self):
        snap = self._runner.snapshot_state()
        # Совместимость с фронтом: он ожидает поле `state`
        snap['state'] = (
            'BT:' + snap.get('bt_active', '').rsplit(' > ', 1)[-1]
            if snap.get('bt_active') else 'IDLE'
        )
        self.publish('status', snap, qos=1, retain=True)

    # ── Publish helpers ────────────────────────────────────────────────
    def _send_cmd_vel(self, linear_x: float, angular_z: float):
        self._cmd_msg['linear_x'] = round(linear_x, 3)
        self._cmd_msg['angular_z'] = round(angular_z, 3)
        self.publish('cmd_vel', self._cmd_msg)

    def _send_claw(self, is_open: bool):
        self.publish('claw/command', 'open' if is_open else 'close', qos=1)

    def _publish_manual(self, linear_x: float, angular_z: float):
        """Эхо в cmd_vel/manual чтобы фронт видел ручную команду."""
        self.publish('cmd_vel/manual',
                     {'linear_x': round(linear_x, 3),
                      'angular_z': round(angular_z, 3)})

    def _log(self, msg: str, level: str = 'info'):
        if level == 'warn':
            self.log_warn(msg)
        elif level == 'error':
            self.log_error(msg) if hasattr(self, 'log_error') else self.log_warn(msg)
        else:
            self.log_info(msg)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = FSMBTNode(broker=args.broker, port=args.port, robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
