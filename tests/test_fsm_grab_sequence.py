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
def fsm_node_factory(monkeypatch):
    """Фабрика FSMNode с замоканной MQTT-связью и таймерами.

    Также патчит fsm_module.cfg, фиксируя servos.arm.max_speed_deg_per_sec=120
    для детерминизма grab-settle math в _do_grab. Без патча тесты
    зависят от текущего значения в config.yaml и сломаются при его
    изменении (например, понижении до 45°/с — settle становится ~3.58с
    вместо 1.5с, и проверка на 16 тиков (1.6с) переставала бы триггерить
    freeze). Этот патч изолирует тесты от runtime-конфига.
    """
    def _factory():
        from pi_nodes.nodes import fsm_node as fsm_module

        real_cfg = fsm_module.cfg

        def fake_cfg(key, default=None):
            if key == 'servos.arm.max_speed_deg_per_sec':
                return 120.0
            return real_cfg(key, default)

        monkeypatch.setattr(fsm_module, 'cfg', fake_cfg)

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


def test_grab_state_flags_initial_false(fsm_node_factory):
    """Новые поля _grab_open_sent и _grab_hold_sent инициализируются False."""
    node = fsm_node_factory()
    assert node._grab_open_sent is False
    assert node._grab_hold_sent is False


def test_grab_state_flags_reset_on_transition(fsm_node_factory):
    """После _transition в любой state флаги сбрасываются в False.
    Это гарантирует что повторный заход в GRABBING запустит все фазы заново.
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()

    node._grab_open_sent = True
    node._grab_hold_sent = True

    node._transition(State.IDLE)

    assert node._grab_open_sent is False
    assert node._grab_hold_sent is False


def test_grab_phase1_publishes_open_and_freeze_duration(fsm_node_factory):
    """Phase 1 (первый тик GRABBING): два arm/command publishes:
    1) {joint:4, angle:0} (открыть клешню)
    2) {command:freeze, joint:4, duration:20.0} (freeze клешня на 20с)
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    node._do_grab()

    arm_pubs = [p for p in node._published if p[0] == 'arm/command']
    assert {'joint': 4, 'angle': 0.0} in [p[1] for p in arm_pubs]
    assert {'command': 'freeze', 'joint': 4, 'duration': 20.0} in [p[1] for p in arm_pubs]
    assert node._grab_open_sent is True
    assert node._grab_hold_sent is False


def test_grab_phase1_only_once(fsm_node_factory):
    """Phase 1 публикуется ровно один раз за вход в GRABBING."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    for _ in range(5):
        node._do_grab()    # tick 1-5

    open_pubs = [p for p in node._published
                 if p[0] == 'arm/command'
                 and isinstance(p[1], dict)
                 and p[1].get('joint') == 4
                 and 'angle' in p[1]]
    assert len(open_pubs) == 1


def test_grab_phase2_waits_no_new_publishes(fsm_node_factory):
    """Phase 2 (t < 1.1с): после Phase 1 нет новых публикаций до t≥1.1."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)
    node._do_grab()    # Phase 1 fires
    pubs_after_phase1 = len(node._published)

    # Тики 2..10 (_grab_t = 0.2..1.0) — Phase 2 wait
    for _ in range(9):
        node._do_grab()

    assert len(node._published) == pubs_after_phase1   # никаких новых


def test_grab_phase3_publishes_grab_hold(fsm_node_factory):
    """Phase 3 (~t=1.1с): публикуется load_preset grab_hold."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    # 11 тиков → _grab_t = 1.1 (CPython float может дать 1.10000...01)
    for _ in range(11):
        node._do_grab()

    hold_pubs = [p for p in node._published
                 if p[0] == 'arm/command'
                 and isinstance(p[1], dict)
                 and p[1].get('command') == 'load_preset'
                 and p[1].get('name') == 'grab_hold']
    assert len(hold_pubs) == 1
    assert node._grab_hold_sent is True


def test_grab_phase5_publishes_grab_return_and_transitions(fsm_node_factory):
    """Phase 5 (после settle + 1с): load_preset grab_return + переход в RETURNING.

    С max_speed=120 (fixture) settle = 100/120 ≈ 0.83с.
    phase_5_t = 1.1 + 0.83 + 1.0 ≈ 2.93с → 30 тиков.
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    for _ in range(30):
        node._do_grab()

    return_pubs = [p for p in node._published
                   if p[0] == 'arm/command'
                   and isinstance(p[1], dict)
                   and p[1].get('command') == 'load_preset'
                   and p[1].get('name') == 'grab_return']
    assert len(return_pubs) == 1
    assert node._state == State.RETURNING


def test_grab_does_not_publish_old_freeze_pattern(fsm_node_factory):
    """В новой логике НЕ публикуются bare freeze / freeze joint=4 как в v1.
    Замораживание идёт через freeze joint=4 duration=20 (Phase 1) и
    load_preset _freeze_all_except_claw (Phase 3/5 неявно).
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(35):
        node._do_grab()

    arm_pubs = [p for p in node._published if p[0] == 'arm/command']
    payloads = [p[1] for p in arm_pubs if isinstance(p[1], dict)]
    # Старые паттерны должны отсутствовать
    assert {'command': 'freeze'} not in payloads
    assert {'command': 'freeze', 'joint': 4} not in payloads
