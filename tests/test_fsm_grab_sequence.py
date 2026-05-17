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


def test_grab_first_tick_sends_grab_hold_preset(fsm_node_factory):
    """В первый тик _do_grab публикуется arm/command
    {"command":"load_preset","name":"grab_hold"}.
    Это разом ставит CH0/CH1/CH2 в позу захвата и CH3=180 (закрывает клешню).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1
    assert arm_pubs[0][1]['name'] == 'grab_hold'


def test_grab_does_not_publish_claw_command(fsm_node_factory):
    """Новая логика НЕ использует topic claw/command (legacy для servo_node,
    который работает с CH0=Основание — неправильный канал для клешни).
    Всё идёт через arm/command (CH3 — клешня по новой логике).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(20):    # 2 секунды эмулируем
        node._do_grab()

    claw_pubs = [p for p in node._published if p[0] == 'claw/command']
    assert claw_pubs == []


def test_grab_after_settle_sends_freeze_and_transitions_to_returning(
        fsm_node_factory):
    """Через ~1.5с после первой команды grab_hold (интерполятор успевает
    доехать) FSM шлёт arm/command {"command":"freeze"} и переходит в
    RETURNING. Рука остаётся frozen в grab_hold, корпус едет домой.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    # 15 тиков = 1.5с (tick=0.1с). Точно равно settle_s, должен запуститься
    # переход после 16-го тика (>1.5).
    for _ in range(16):
        node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)]
    freeze_pubs = [p for p in arm_pubs if p[1].get('command') == 'freeze']
    assert len(freeze_pubs) == 1
    # Переход в RETURNING
    assert node._state == State.RETURNING


def test_grab_during_settle_does_not_freeze_yet(fsm_node_factory):
    """Между t=0.1с и t=1.5с — никакие freeze/новые preset-команды не шлются.
    Только один grab_hold на старте, потом ждём.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(10):    # 1.0с
        node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)]
    # Один load_preset grab_hold и НИ ОДНОГО freeze
    preset_pubs = [p for p in arm_pubs if p[1].get('command') == 'load_preset']
    freeze_pubs = [p for p in arm_pubs if p[1].get('command') == 'freeze']
    assert len(preset_pubs) == 1
    assert len(freeze_pubs) == 0
    assert node._state == State.GRABBING    # ещё не перешли
