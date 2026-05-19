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
    доехать) FSM шлёт два arm/command freeze (общий + явный для клешни)
    и переходит в RETURNING. Рука остаётся frozen в grab_hold, корпус
    едет домой.

    Парный freeze нужен потому, что arm_node._freeze_all_except_claw
    исключает CH3 из общего freeze (см. UX-требование «клешня морозится
    только личной кнопкой»). FSM компенсирует это явным joint=4 freeze —
    иначе после grab PWM на клешне отключится через HOLD_TIME и мяч
    выпадет.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    # Float-арифметика: 0.1 * 15 = 1.5000000000000002 в CPython, поэтому
    # на 15-м тике уже _grab_t >= GRAB_SETTLE_S (1.5) → freeze + transition.
    # На итерации 16 _transition сбросит _grab_t в 0, и (если тест продолжит
    # вызывать _do_grab напрямую — а он это делает) Phase 1 повторит
    # load_preset grab_hold. В продакшене это не происходит, потому что
    # _tick роутится по state: после RETURNING вызывается _do_return,
    # не _do_grab. Тест ниже фиксирует контракт: freeze ровно 2 раза
    # (общий + клешня), переход состоялся.
    for _ in range(16):
        node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)]
    freeze_pubs = [p for p in arm_pubs if p[1].get('command') == 'freeze']
    assert len(freeze_pubs) == 2
    # Сначала общий freeze (без joint), затем явный для клешни (joint=4).
    assert 'joint' not in freeze_pubs[0][1]
    assert freeze_pubs[1][1].get('joint') == 4
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
