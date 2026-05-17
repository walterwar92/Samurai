"""Tests for fsm_node — последовательность APPROACHING/GRABBING с новыми
позами руки (grab_ready / grab_hold) и заморозкой.

Контекст: до этого PR FSM `_do_grab` слал `claw/command "close"` и не
управлял углами 1-3. Теперь рука выезжает в `grab_ready` при входе в
APPROACHING и закрывается в `grab_hold` с заморозкой в GRABBING. Тесты
фиксируют этот контракт публикаций.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


@pytest.fixture
def fsm_node_factory():
    """Фабрика FSMNode с замоканной MQTT-связью и таймерами."""
    def _factory():
        from pi_nodes.nodes import fsm_node as fsm_module

        with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
            MockClient.return_value = MagicMock()
            with patch.object(fsm_module.FSMNode, 'create_timer',
                              lambda self, period, cb: None):
                with patch.object(fsm_module.FSMNode, 'subscribe',
                                  lambda *a, **kw: None):
                    node = fsm_module.FSMNode()

        node._published: list[tuple[str, object]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        return node

    return _factory


def test_factory_smoke(fsm_node_factory):
    """Smoke: FSMNode инстанцируется и в IDLE."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    assert node._state == State.IDLE


def test_approach_sends_grab_ready_once_on_entry(fsm_node_factory):
    """Первый _do_approach(...) при свежем входе в APPROACHING публикует
    arm/command {"command":"load_preset","name":"grab_ready"}. Повторные
    тики НЕ шлют (один раз за state).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    # Симулируем вход в APPROACHING из TARGETING (центрировались по мячу)
    node._transition(State.APPROACHING)
    node._target_colour = 'red'
    fake_det = {'colour': 'red', 'x': 300, 'y': 200, 'w': 40, 'h': 40}

    # Tick 1: должно быть load_preset grab_ready
    node._do_approach(fake_det, range_m=0.50)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1
    assert arm_pubs[0][1]['name'] == 'grab_ready'

    # Tick 2-5: НЕ шлём повторно
    for _ in range(4):
        node._do_approach(fake_det, range_m=0.40)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1   # всё ещё ровно 1


def test_approach_resends_grab_ready_after_state_exit(fsm_node_factory):
    """Если FSM ушёл в TARGETING (потерял мяч) и вернулся в APPROACHING —
    grab_ready должен послаться снова. Флаг `_approach_arm_sent`
    сбрасывается в _transition.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.APPROACHING)
    fake_det = {'colour': 'red', 'x': 300, 'y': 200, 'w': 40, 'h': 40}

    node._do_approach(fake_det, range_m=0.50)
    # Потеряли мяч → вернулись в TARGETING → снова в APPROACHING
    node._transition(State.TARGETING)
    node._transition(State.APPROACHING)
    node._do_approach(fake_det, range_m=0.50)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 2
    assert all(p[1]['name'] == 'grab_ready' for p in arm_pubs)
