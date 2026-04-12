#!/usr/bin/env python3
"""
mission_node — Запись и воспроизведение полных миссий (путь + действия).

Расширяет концепцию path_recorder: записывает не только маршрут, но и все
действия робота (клешня, голосовые команды, движения головы).
При воспроизведении повторяет маршрут + выполняет действия в правильное время.

Subscribes:
    samurai/{robot_id}/odom              — позиция для записи маршрута
    samurai/{robot_id}/status            — состояния FSM
    samurai/{robot_id}/claw/state        — действия клешни
    samurai/{robot_id}/head/command      — движения головы
    samurai/{robot_id}/voice_command     — голосовые команды
    samurai/{robot_id}/mission/command   — управление: record/stop/play/save/load/list
Publishes:
    samurai/{robot_id}/cmd_vel           — движение при воспроизведении
    samurai/{robot_id}/mission/status    — {state, name, events, progress}
    samurai/{robot_id}/voice_command     — повтор голосовых команд
    samurai/{robot_id}/claw/command      — повтор клешни
    samurai/{robot_id}/head/command      — повтор головы
"""

import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Конфигурация ─────────────────────────────────────────────
WAYPOINT_INTERVAL_M = 0.05    # записывать waypoint каждые 5 см
WAYPOINT_INTERVAL_RAD = 0.10  # или каждые ~6°
REPLAY_LINEAR_SPEED = 0.18    # м/с
REPLAY_ANGULAR_KP = 2.5       # P-коэффициент
REPLAY_ANGULAR_MAX = 0.8      # рад/с макс
REPLAY_GOAL_TOL = 0.04        # м — точность waypoint
REPLAY_MIN_LINEAR = 0.04      # м/с
CONTROL_HZ = 15
MAX_EVENTS = 50000
MISSIONS_DIR = os.path.expanduser('~/missions')


class MissionNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('mission_node', **kwargs)

        self._state = 'idle'  # idle / recording / replaying
        self._mission_name = ''
        self._events = []        # [{t, type, ...}]
        self._replay_idx = 0
        self._replay_action_idx = 0
        self._record_start = 0.0
        self._replay_start = 0.0

        # Одометрия
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_wp_x = 0.0
        self._last_wp_y = 0.0
        self._last_wp_theta = 0.0
        self._pose_valid = False

        # Подписки — данные
        self.subscribe('odom', self._odom_cb)
        self.subscribe('mission/command', self._cmd_cb, qos=1)

        # Подписки — запись действий
        self.subscribe('claw/state', self._action_cb_factory('claw/command'))
        self.subscribe('head/command', self._action_cb_factory('head/command'))
        self.subscribe('voice_command', self._action_cb_factory('voice_command'))

        # Контроль
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.create_timer(1.0, self._publish_status)

        self.log_info('Mission node ready (missions dir: %s)', MISSIONS_DIR)

    def _action_cb_factory(self, topic_suffix):
        """Создаёт callback для записи действия определённого типа."""
        def _cb(topic, data):
            if self._state != 'recording':
                return
            t = time.monotonic() - self._record_start
            # Сериализуем данные
            if isinstance(data, dict):
                payload = data
            elif isinstance(data, bytes):
                try:
                    payload = json.loads(data)
                except Exception:
                    payload = data.decode('utf-8', errors='replace')
            else:
                payload = str(data)

            self._events.append({
                't': round(t, 3),
                'type': 'action',
                'topic': topic_suffix,
                'data': payload,
            })
        return _cb

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        self._x = data.get('x', 0.0) / 100.0      # cm → m
        self._y = data.get('y', 0.0) / 100.0      # cm → m
        self._theta = data.get('theta', 0.0)
        self._pose_valid = True

        if self._state == 'recording':
            self._maybe_record_waypoint()

    def _maybe_record_waypoint(self):
        if len(self._events) >= MAX_EVENTS:
            return
        dx = self._x - self._last_wp_x
        dy = self._y - self._last_wp_y
        dist = math.sqrt(dx * dx + dy * dy)
        dtheta = abs(self._angle_diff(self._theta, self._last_wp_theta))

        if dist >= WAYPOINT_INTERVAL_M or dtheta >= WAYPOINT_INTERVAL_RAD:
            t = time.monotonic() - self._record_start
            self._events.append({
                't': round(t, 3),
                'type': 'waypoint',
                'x': round(self._x, 4),
                'y': round(self._y, 4),
                'theta': round(self._theta, 4),
            })
            self._last_wp_x = self._x
            self._last_wp_y = self._y
            self._last_wp_theta = self._theta

    def _cmd_cb(self, topic, data):
        if isinstance(data, dict):
            cmd = data.get('command', '').strip().lower()
            name = data.get('name', '')
        else:
            cmd = str(data).strip().lower()
            name = ''

        if cmd in ('record', 'start'):
            self._start_recording(name)
        elif cmd == 'stop':
            self._stop(name)
        elif cmd in ('play', 'replay'):
            if name:
                self._load_mission(name)
            self._start_replay()
        elif cmd == 'save':
            self._save_mission(name or 'mission')
        elif cmd == 'load':
            self._load_mission(name or 'mission')
        elif cmd == 'list':
            self._list_missions()
        elif cmd == 'clear':
            self._events = []
            self._state = 'idle'
            self.log_info('Mission cleared')

    def _start_recording(self, name=''):
        if self._state == 'replaying':
            self._stop_driving()
        self._state = 'recording'
        self._events = []
        self._mission_name = name or f'mission_{int(time.time())}'
        self._record_start = time.monotonic()
        if self._pose_valid:
            self._last_wp_x = self._x
            self._last_wp_y = self._y
            self._last_wp_theta = self._theta
            self._events.append({
                't': 0.0,
                'type': 'waypoint',
                'x': round(self._x, 4),
                'y': round(self._y, 4),
                'theta': round(self._theta, 4),
            })
        self.log_info('Mission recording started: %s', self._mission_name)

    def _start_replay(self):
        if not self._events:
            self.log_warn('No mission to replay')
            return
        # Собираем только waypoints для навигации
        self._waypoints = [e for e in self._events if e['type'] == 'waypoint']
        self._actions = [e for e in self._events if e['type'] == 'action']
        if not self._waypoints:
            self.log_warn('Mission has no waypoints')
            return
        self._state = 'replaying'
        self._replay_idx = 0
        self._replay_action_idx = 0
        self._replay_start = time.monotonic()
        self.log_info('Mission replay started (%d waypoints, %d actions)',
                      len(self._waypoints), len(self._actions))

    def _control_loop(self):
        if self._state != 'replaying':
            return
        if not self._pose_valid:
            return

        # Проверяем и выполняем действия по времени
        self._execute_timed_actions()

        # Навигация к текущему waypoint
        if self._replay_idx >= len(self._waypoints):
            self.log_info('Mission replay complete')
            self._stop_driving()
            self._state = 'idle'
            return

        wp = self._waypoints[self._replay_idx]
        tx, ty = wp['x'], wp['y']
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < REPLAY_GOAL_TOL:
            self._replay_idx += 1
            return

        # Pure pursuit
        bearing = math.atan2(dy, dx)
        angle_error = self._angle_diff(bearing, self._theta)

        angular = REPLAY_ANGULAR_KP * angle_error
        angular = max(-REPLAY_ANGULAR_MAX, min(REPLAY_ANGULAR_MAX, angular))

        cos_factor = max(0.0, math.cos(angle_error))
        dist_factor = min(1.0, dist / 0.12)
        linear = REPLAY_LINEAR_SPEED * cos_factor * dist_factor
        linear = max(REPLAY_MIN_LINEAR * cos_factor, linear)

        if abs(angle_error) > 1.0:
            linear = 0.0

        self.publish('cmd_vel', {
            'linear_x': round(linear, 3),
            'angular_z': round(angular, 3),
        })

    def _execute_timed_actions(self):
        """Выполняет действия, время которых уже наступило."""
        elapsed = time.monotonic() - self._replay_start

        while self._replay_action_idx < len(self._actions):
            action = self._actions[self._replay_action_idx]
            if action['t'] > elapsed:
                break  # ещё не время

            # Выполняем действие
            topic = action['topic']
            data = action['data']
            self.publish(topic, data, qos=1)
            self.log_info('Mission action: %s → %s', topic, str(data)[:60])
            self._replay_action_idx += 1

    def _stop(self, name=''):
        if self._state == 'replaying':
            self._stop_driving()
        old = self._state
        self._state = 'idle'
        self.log_info('Mission stopped (was %s, %d events)', old, len(self._events))
        if name and self._events:
            self._save_mission(name)

    def _stop_driving(self):
        self.publish('cmd_vel', {'linear_x': 0.0, 'angular_z': 0.0})

    def _save_mission(self, name):
        os.makedirs(MISSIONS_DIR, exist_ok=True)
        filepath = os.path.join(MISSIONS_DIR, f'{name}.json')
        data = {
            'name': name,
            'events': self._events,
            'events_count': len(self._events),
            'waypoints': sum(1 for e in self._events if e['type'] == 'waypoint'),
            'actions': sum(1 for e in self._events if e['type'] == 'action'),
            'total_time': self._events[-1]['t'] if self._events else 0,
            'saved_at': time.time(),
        }
        with open(filepath, 'w') as f:
            json.dump(data, f)
        self.log_info('Mission saved: %s (%d events)', filepath, len(self._events))

    def _load_mission(self, name):
        filepath = os.path.join(MISSIONS_DIR, f'{name}.json')
        if not os.path.exists(filepath):
            self.log_warn('Mission not found: %s', filepath)
            return
        with open(filepath, 'r') as f:
            data = json.load(f)
        self._events = data.get('events', [])
        self._mission_name = name
        self.log_info('Mission loaded: %s (%d events)', name, len(self._events))

    def _list_missions(self):
        os.makedirs(MISSIONS_DIR, exist_ok=True)
        files = sorted(f.replace('.json', '') for f in os.listdir(MISSIONS_DIR)
                       if f.endswith('.json'))
        self.publish('mission/list', {'missions': files})

    def _publish_status(self):
        total_wp = sum(1 for e in self._events if e['type'] == 'waypoint')
        status = {
            'state': self._state,
            'name': self._mission_name,
            'events_count': len(self._events),
            'waypoints': total_wp,
        }
        if self._state == 'replaying':
            status['replay_index'] = self._replay_idx
            status['replay_total'] = len(self._waypoints) if hasattr(self, '_waypoints') else 0
            status['progress'] = round(self._replay_idx / max(1, status['replay_total']), 2)
        self.publish('mission/status', status)

    @staticmethod
    def _angle_diff(target, current):
        d = target - current
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def on_shutdown(self):
        if self._state == 'replaying':
            self._stop_driving()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = MissionNode(broker=args.broker, port=args.port,
                       robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
