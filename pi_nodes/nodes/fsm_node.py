#!/usr/bin/env python3
"""
fsm_node — Finite State Machine for autonomous ball-hunting robot.

States:
  IDLE, SEARCHING, TARGETING, APPROACHING, GRABBING, BURNING,
  CALLING, RETURNING, PATROLLING, FOLLOWING, PATH_REPLAY

Subscribes:
    samurai/{robot_id}/voice_command    — parsed voice commands
    samurai/{robot_id}/ball_detection   — JSON from YOLO: {colour, x, y, w, h, conf}
    samurai/{robot_id}/range            — {range, ts} ultrasonic
    samurai/{robot_id}/call_robot       — incoming call from second robot
    samurai/{robot_id}/gesture/command  — hand gesture commands
Publishes:
    samurai/{robot_id}/cmd_vel                  — {linear_x, angular_z}
    samurai/{robot_id}/claw/command             — "open" / "close"
    samurai/{robot_id}/laser/command            — true / false
    samurai/{robot_id}/status                   — JSON state
    samurai/{robot_id}/goal_pose                — {x, y, theta}
    samurai/{robot_id}/patrol/command           — "start" / "stop"
    samurai/{robot_id}/follow_me/command        — "start" / "stop"
    samurai/{robot_id}/path_recorder/command    — "record" / "stop" / "replay"
    samurai/{other_id}/call_robot               — outgoing call to other robot
"""

import json
import math
import os
import re
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

_MAX_CMD_LEN = 200

_P_CALLING = re.compile(r'вызови.{0,20}машин')
_P_GRAB    = re.compile(r'получи|возьми|найди')
_P_BURN    = re.compile(r'сожги|прожги|лазер')
_P_STOP    = re.compile(r'стоп|остановись|стой')
_P_HOME    = re.compile(r'домой|вернись')
_P_PATROL  = re.compile(r'патрул|обход')
_P_FOLLOW  = re.compile(r'следуй|за мной')
_P_RECORD  = re.compile(r'запиши путь|запись')
_P_REPLAY  = re.compile(r'воспроизведи|повтори путь')
_P_RESET   = re.compile(r'сбрось позицию|сброс позиции|обнули позицию|нулевая позиция')
# Движение — прямые команды cmd_vel (только в состоянии IDLE)
_P_FORWARD = re.compile(r'вперёд|вперед|прямо|едь вперёд|двигайся вперёд')
_P_BACK    = re.compile(r'назад|едь назад|двигайся назад|сдай назад')
_P_LEFT    = re.compile(r'налево|влево|поверни влево|повернись влево')
_P_RIGHT   = re.compile(r'направо|вправо|поверни вправо|повернись вправо')


class State:
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    TARGETING = 'TARGETING'
    APPROACHING = 'APPROACHING'
    GRABBING = 'GRABBING'
    BURNING = 'BURNING'
    CALLING = 'CALLING'
    RETURNING = 'RETURNING'
    PATROLLING = 'PATROLLING'
    FOLLOWING = 'FOLLOWING'
    PATH_REPLAY = 'PATH_REPLAY'


_ALL_STATES = frozenset({
    State.IDLE, State.SEARCHING, State.TARGETING,
    State.APPROACHING, State.GRABBING, State.BURNING,
    State.CALLING, State.RETURNING, State.PATROLLING,
    State.FOLLOWING, State.PATH_REPLAY,
})


class FSMNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('fsm_node', **kwargs)

        self._state = State.IDLE
        self._target_colour = ''
        self._target_action = ''
        self._ball_detection = None
        self._range_m = float('inf')
        self._approach_timeout = 0.0
        self._last_steer = 0.0
        self._lost_frames = 0
        self._search_accumulated = 0.0
        self._prev_theta = 0.0
        self._data_lock = threading.Lock()

        # Pre-allocated cmd_vel dicts — updated in-place
        self._cmd_msg = {'linear_x': 0.0, 'angular_z': 0.0}
        self._cmd_manual_msg = {'linear_x': 0.0, 'angular_z': 0.0}

        # Subscribers
        self.subscribe('voice_command', self._voice_cb, qos=1)
        self.subscribe('ball_detection', self._ball_cb)
        self.subscribe('range', self._range_cb)
        self.subscribe('call_robot', self._call_recv_cb, qos=1)
        self.subscribe('gesture/command', self._gesture_cb, qos=1)
        self.subscribe('fsm/transition', self._transition_cb, qos=1)

        # Timers
        self.create_timer(0.1, self._tick)            # 10 Hz FSM
        self.create_timer(1.0, self._publish_status)   # 1 Hz status

        self.log_info('FSM node started — state=IDLE')

    # ── Voice command parser ─────────────────────────────────
    def _voice_cb(self, topic, data):
        raw = str(data)
        if not raw or len(raw) > _MAX_CMD_LEN:
            self.log_warn('Voice command rejected: length=%d', len(raw))
            return

        text = raw.lower().strip()
        self.log_info('Voice: "%s"', text)

        if _P_CALLING.search(text):
            self._transition(State.CALLING)
            return

        if _P_GRAB.search(text):
            self._target_action = 'grab'
            self._target_colour = self._extract_colour(text)
            self.log_info('Target: grab %s ball', self._target_colour or 'any')
            self._transition(State.SEARCHING)
            return

        if _P_BURN.search(text):
            self._target_action = 'burn'
            self._target_colour = self._extract_colour(text)
            self._transition(State.SEARCHING)
            return

        if _P_STOP.search(text):
            self._transition(State.IDLE)
            self._pub_cmd_vel(0.0, 0.0)
            self._pub_cmd_vel_manual(0.0, 0.0)
            return

        # Ручное движение — публикуется в cmd_vel/manual (приоритет выше автономного).
        # Работает в ЛЮБОМ состоянии: временно перебивает автономное управление,
        # после прекращения ручных команд автономный режим возобновляется.
        if _P_FORWARD.search(text):
            self._pub_cmd_vel_manual(0.15, 0.0)
            return
        if _P_BACK.search(text):
            self._pub_cmd_vel_manual(-0.15, 0.0)
            return
        if _P_LEFT.search(text):
            self._pub_cmd_vel_manual(0.0, 0.5)
            return
        if _P_RIGHT.search(text):
            self._pub_cmd_vel_manual(0.0, -0.5)
            return

        if _P_HOME.search(text):
            self._transition(State.RETURNING)
            return

        if _P_PATROL.search(text):
            self._transition(State.PATROLLING)
            return

        if _P_FOLLOW.search(text):
            self._transition(State.FOLLOWING)
            return

        if _P_RECORD.search(text):
            self.publish('path_recorder/command', 'record', qos=1)
            self.log_info('Path recording started')
            return

        if _P_REPLAY.search(text):
            self._transition(State.PATH_REPLAY)
            return

        if _P_RESET.search(text):
            self.publish('reset_position', 'reset', qos=1)
            self.log_info('Position reset to (0, 0, 0)')
            return

    # ── Gesture command handler ──────────────────────────────
    def _gesture_cb(self, topic, data):
        gesture = str(data).strip()
        if not gesture:
            return

        if gesture == 'stop':
            self._transition(State.IDLE)
            self._pub_cmd_vel_manual(0.0, 0.0)
        elif gesture == 'forward':
            self._pub_cmd_vel_manual(0.15, 0.0)
        elif gesture == 'grab' and self._state == State.IDLE:
            self._target_action = 'grab'
            self._target_colour = ''
            self._transition(State.SEARCHING)
        elif gesture == 'follow':
            self._transition(State.FOLLOWING)
        elif gesture == 'point_left':
            self._pub_cmd_vel_manual(0.0, 0.5)
        elif gesture == 'point_right':
            self._pub_cmd_vel_manual(0.0, -0.5)

    def _transition_cb(self, topic, data):
        """Force FSM transition from admin dashboard."""
        target = str(data).strip().upper()
        if target in _ALL_STATES:
            self.log_info('Forced transition → %s', target)
            if target == State.IDLE:
                self._pub_cmd_vel(0.0, 0.0)
            self._transition(target)
        else:
            self.log_warn('Unknown transition target: %s', target)

    def _extract_colour(self, text: str) -> str:
        text_norm = text.replace('ё', 'е')
        colours = {
            'красн': 'red', 'синий': 'blue', 'синего': 'blue',
            'зелен': 'green', 'желт': 'yellow', 'бел': 'white',
            'черн': 'black', 'оранж': 'orange',
        }
        for rus, eng in colours.items():
            if rus in text_norm:
                return eng
        return ''

    # ── Sensor callbacks ─────────────────────────────────────
    def _ball_cb(self, topic, data):
        with self._data_lock:
            if isinstance(data, dict):
                self._ball_detection = data
            else:
                try:
                    self._ball_detection = json.loads(str(data))
                except (json.JSONDecodeError, TypeError):
                    self._ball_detection = None

    def _range_cb(self, topic, data):
        with self._data_lock:
            if isinstance(data, dict):
                self._range_m = float(data.get('range', float('inf')))
            else:
                try:
                    self._range_m = float(data)
                except (ValueError, TypeError):
                    pass

    def _call_recv_cb(self, topic, data):
        self.log_info('Incoming call: %s', data)
        if isinstance(data, dict):
            self._target_colour = data.get('colour', '')
            self._target_action = data.get('action', 'grab')
        self._transition(State.SEARCHING)

    # ── State machine ────────────────────────────────────────
    def _transition(self, new_state: str):
        old = self._state
        self._state = new_state
        self._approach_timeout = 0.0
        self._lost_frames = 0
        self.log_info('FSM: %s → %s', old, new_state)

        # Exit actions
        if old == State.PATROLLING:
            self.publish('patrol/command', 'stop', qos=1)
        if old == State.FOLLOWING:
            self.publish('follow_me/command', 'stop', qos=1)
        if old == State.PATH_REPLAY:
            self.publish('path_recorder/command', 'stop', qos=1)

        # Entry actions
        if new_state == State.IDLE:
            self._stop_all()
            # Stop path recording when going idle
            if old in (State.SEARCHING, State.TARGETING, State.APPROACHING,
                       State.GRABBING, State.BURNING):
                self.publish('path_recorder/command', 'stop', qos=1)
        elif new_state == State.SEARCHING:
            self._search_accumulated = 0.0
            # Auto-start path recording for return-home
            self.publish('path_recorder/command', 'record', qos=1)
        elif new_state == State.CALLING:
            self._do_call()
        elif new_state == State.PATROLLING:
            self.publish('patrol/command', 'start', qos=1)
        elif new_state == State.FOLLOWING:
            self.publish('follow_me/command', 'start', qos=1)
        elif new_state == State.PATH_REPLAY:
            self.publish('path_recorder/command', 'replay', qos=1)

    def _tick(self):
        with self._data_lock:
            det = self._ball_detection
            range_m = self._range_m

        if self._state == State.IDLE:
            pass
        elif self._state == State.SEARCHING:
            self._do_search(det)
        elif self._state == State.TARGETING:
            self._do_targeting(det)
        elif self._state == State.APPROACHING:
            self._do_approach(det, range_m)
        elif self._state == State.GRABBING:
            self._do_grab()
        elif self._state == State.BURNING:
            self._do_burn()
        elif self._state == State.RETURNING:
            self._do_return()

    # ── Behaviours ───────────────────────────────────────────
    def _do_search(self, det):
        self._search_accumulated += 0.4 * 0.1

        if self._search_accumulated > 2 * math.pi + 0.3:
            colour = self._target_colour or 'any'
            self.log_warn('Ball not found (%s) — full rotation', colour)
            self._transition(State.IDLE)
            return

        if det and self._colour_matches(det):
            self.log_info('Ball spotted: %s — targeting', det.get('colour'))
            self._pub_cmd_vel(0.0, 0.0)
            self._transition(State.TARGETING)
            return

        self._pub_cmd_vel(0.0, -0.4)

    def _do_targeting(self, det):
        self._approach_timeout += 0.1

        if det is None or not self._colour_matches(det):
            self._lost_frames += 1
            self._pub_cmd_vel(0.0, self._last_steer * 0.3)
            if self._lost_frames > 15:
                self.log_warn('Lost ball during targeting')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0
        img_cx = 320
        ball_cx = det.get('x', img_cx) + det.get('w', 0) / 2.0
        error_x = (ball_cx - img_cx) / img_cx

        if abs(error_x) < 0.15:
            self._pub_cmd_vel(0.0, 0.0)
            self.log_info('Ball centred — approaching %s', det.get('colour'))
            self._transition(State.APPROACHING)
            return

        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.4 + target_angular * 0.6
        self._last_steer = angular
        self._pub_cmd_vel(0.0, angular)

    def _do_approach(self, det, range_m):
        self._approach_timeout += 0.1

        if self._approach_timeout > 30.0:
            self.log_warn('Approach timeout — back to search')
            self._transition(State.SEARCHING)
            return

        if det is None or not self._colour_matches(det):
            self._lost_frames += 1
            self._pub_cmd_vel(0.02, self._last_steer * 0.3)
            if self._lost_frames > 20:
                self.log_warn('Ball lost — back to search')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0
        img_cx = 320
        ball_cx = det.get('x', img_cx) + det.get('w', 0) / 2.0
        error_x = (ball_cx - img_cx) / img_cx
        abs_error = abs(error_x)

        if abs_error > 0.5:
            self._pub_cmd_vel(0.0, 0.0)
            self._transition(State.TARGETING)
            return

        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.3 + target_angular * 0.7
        self._last_steer = angular

        linear = 0.12 if abs_error < 0.15 else 0.06

        if range_m < 0.10:
            self._pub_cmd_vel(0.0, 0.0)
            if self._target_action == 'burn':
                self._transition(State.BURNING)
            else:
                self._transition(State.GRABBING)
            return

        self._pub_cmd_vel(linear, angular)

    def _do_grab(self):
        self.publish('claw/command', 'open', qos=1)
        self._approach_timeout += 0.1
        if self._approach_timeout < 1.0:
            self._pub_cmd_vel(0.05, 0.0)
        elif self._approach_timeout < 2.0:
            self._pub_cmd_vel(0.0, 0.0)
        elif self._approach_timeout < 3.0:
            self.publish('claw/command', 'close', qos=1)
        else:
            self.log_info('Ball grabbed!')
            self._transition(State.RETURNING)

    def _do_burn(self):
        self._approach_timeout += 0.1
        if self._approach_timeout < 0.5:
            self.publish('laser/command', True, qos=1)
        elif self._approach_timeout < 5.0:
            pass
        else:
            self.publish('laser/command', False, qos=1)
            self.log_info('Burn complete')
            self._transition(State.SEARCHING)

    def _do_call(self):
        other_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        call_data = {
            'colour': self._target_colour,
            'action': self._target_action or 'grab',
        }
        self.publish_raw(f'samurai/{other_id}/call_robot',
                         call_data, qos=1)
        self.log_info('Called second robot')
        self._transition(State.IDLE)

    def _do_return(self):
        # Use path_recorder to replay path in reverse (same path home)
        self.publish('path_recorder/command', 'replay', qos=1)
        # Also publish goal_pose for Nav2 fallback (if laptop is connected)
        self.publish('goal_pose', {
            'x': 0.0, 'y': 0.0, 'theta': 0.0,
        }, qos=1)
        self._transition(State.PATH_REPLAY)

    # ── Helpers ──────────────────────────────────────────────
    def _colour_matches(self, det: dict) -> bool:
        if not self._target_colour:
            return True
        return det.get('colour', '') == self._target_colour

    def _stop_all(self):
        self._pub_cmd_vel(0.0, 0.0)
        self.publish('laser/command', False, qos=1)

    def _pub_cmd_vel(self, linear_x: float, angular_z: float):
        m = self._cmd_msg
        m['linear_x'] = round(linear_x, 3)
        m['angular_z'] = round(angular_z, 3)
        self.publish('cmd_vel', m)

    def _pub_cmd_vel_manual(self, linear_x: float, angular_z: float):
        """Manual override — higher priority than autonomous cmd_vel."""
        m = self._cmd_manual_msg
        m['linear_x'] = round(linear_x, 3)
        m['angular_z'] = round(angular_z, 3)
        self.publish('cmd_vel/manual', m)

    def _publish_status(self):
        self.publish('status', {
            'state': self._state,
            'target_colour': self._target_colour,
            'target_action': self._target_action,
            'range_m': round(self._range_m, 3)
                       if self._range_m != float('inf') else -1.0,
        }, qos=1, retain=True)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = FSMNode(broker=args.broker, port=args.port,
                   robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
