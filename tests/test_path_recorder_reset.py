"""Тесты `PathRecorderNode._reset_position_cb` — синхронизация с reset_position.

Контекст: пользователь жмёт «Сброс позиции» в дашборде → motor_node обнуляет
одометрию ((x,y,theta) = (0,0,0)), физическое местоположение робота
становится новым home. До этого фикса `path_recorder_node` не подписывался
на `reset_position`, поэтому его внутренний `_path` оставался в СТАРОЙ
системе координат. При последующем нажатии «Домой» (`fsm._do_return` →
`path_recorder/command replay`) робот пытался replay по неактуальным
waypoint'ам и уезжал в сторону прежнего origin'а — далеко от реального
нового home.

Тесты фиксируют что:
  * path очищается, остаётся только новая (0,0,0) — home;
  * `_home_heading` сбрасывается в 0 (выравнивание курса на финиш);
  * если шёл replay — он останавливается с cmd_vel=0;
  * circumvent-state (Bug0) сбрасывается тоже.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


@pytest.fixture
def path_recorder_factory():
    """Фабрика PathRecorderNode с замоканными MQTT и таймерами.

    publish заменён на capture-list — тесты проверяют что _stop_driving
    публикует cmd_vel(0,0).
    """
    def _factory():
        from pi_nodes.nodes import path_recorder_node as pr_module

        with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
            MockClient.return_value = MagicMock()
            with patch.object(pr_module.PathRecorderNode, 'create_timer',
                              lambda self, period, cb: None):
                with patch.object(pr_module.PathRecorderNode, 'subscribe',
                                  lambda *a, **kw: None):
                    node = pr_module.PathRecorderNode()

        node._published: list[tuple[str, object]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        return node

    return _factory


def test_reset_position_clears_stale_path(path_recorder_factory):
    """Сброс → path = [(0,0,0)]. Старые waypoint'ы из прежней системы координат
    выкидываются, иначе replay поедет к физически несуществующему месту."""
    node = path_recorder_factory()
    node._x, node._y, node._theta = 5.0, 2.0, 0.5
    node._pose_valid = True
    node._state = 'recording'
    node._path = [(0.0, 0.0, 0.0), (2.5, 1.0, 0.3), (5.0, 2.0, 0.5)]
    node._last_record_x = 5.0
    node._last_record_y = 2.0
    node._last_record_theta = 0.5
    node._home_heading = 0.0

    node._reset_position_cb('reset_position', 'reset')

    assert node._x == 0.0
    assert node._y == 0.0
    assert node._theta == 0.0
    assert node._path == [(0.0, 0.0, 0.0)]
    assert node._last_record_x == 0.0
    assert node._last_record_y == 0.0
    assert node._last_record_theta == 0.0
    assert node._home_heading == 0.0


def test_reset_position_keeps_recording_state(path_recorder_factory):
    """Если робот ехал и записывал — продолжаем recording, чтобы дальнейшее
    движение от нового home сразу попадало в path."""
    node = path_recorder_factory()
    node._x, node._y, node._theta = 5.0, 2.0, 0.5
    node._pose_valid = True
    node._state = 'recording'

    node._reset_position_cb('reset_position', 'reset')

    assert node._state == 'recording'


def test_reset_position_stops_in_flight_replay(path_recorder_factory):
    """Сброс во время replay → cmd_vel=0 и выход из replaying.

    Продолжать replay по только что обнулённому path небезопасно: робот
    физически уже в новом home, прежние waypoint'ы потеряли смысл.
    """
    node = path_recorder_factory()
    node._x, node._y, node._theta = 3.0, 1.5, 0.2
    node._pose_valid = True
    node._state = 'replaying'
    node._path = [(0.0, 0.0, 0.0), (1.5, 0.7, 0.1), (3.0, 1.5, 0.2)]
    node._replay_index = 1

    node._reset_position_cb('reset_position', 'reset')

    stop_msgs = [p for p in node._published
                 if p[0] == 'cmd_vel'
                 and isinstance(p[1], dict)
                 and p[1].get('linear_x') == 0.0
                 and p[1].get('angular_z') == 0.0]
    assert stop_msgs, 'expected cmd_vel(0,0) on reset during replay'
    assert node._state != 'replaying'
    assert node._path == [(0.0, 0.0, 0.0)]


def test_reset_position_clears_circumvent_state(path_recorder_factory):
    """Reset во время обхода препятствия → Bug0-state сбрасывается, иначе
    после reset робот продолжит turning/driving относительно старых
    self._circumvent_start_{x,y,theta} в прежней системе координат."""
    node = path_recorder_factory()
    node._x, node._y, node._theta = 2.0, 0.5, 1.0
    node._pose_valid = True
    node._state = 'replaying'
    node._circumvent_state = 'turning'
    node._circumvent_attempts = 2

    node._reset_position_cb('reset_position', 'reset')

    assert node._circumvent_state == 'none'
    assert node._circumvent_attempts == 0


def test_reset_position_subscribed_in_init(path_recorder_factory, monkeypatch):
    """Production-путь: __init__ обязан вызвать self.subscribe('reset_position', …).

    Тесты выше дёргают _reset_position_cb напрямую, но без subscribe
    реальная нода никогда не получит MQTT-сообщение.
    """
    from pi_nodes.nodes import path_recorder_node as pr_module

    captured: list[tuple] = []

    def _record_subscribe(self, suffix, cb, qos=0, **kw):
        captured.append((suffix, cb, qos))

    with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
        MockClient.return_value = MagicMock()
        with patch.object(pr_module.PathRecorderNode, 'create_timer',
                          lambda self, period, cb: None):
            with patch.object(pr_module.PathRecorderNode, 'subscribe',
                              _record_subscribe):
                pr_module.PathRecorderNode()

    suffixes = [c[0] for c in captured]
    assert 'reset_position' in suffixes, (
        f'PathRecorderNode must subscribe to reset_position; got {suffixes}')
    entry = next(c for c in captured if c[0] == 'reset_position')
    assert entry[2] == 1, 'reset_position должен идти с qos=1 (как у motor/slam)'
