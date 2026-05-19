"""Tests for compute_node/dashboard/routers/actuators.py — фокус на
авто-unfreeze при открытии клешни.

Контекст: после захвата объекта FSM шлёт arm/command freeze. Когда
пользователь/voice/UI открывает клешню — рука должна сама размораживаться,
иначе суставы 1-3 останутся жёстко зафиксированными. Реализация — в
set_claw endpoint: при state="open" (или angle<90) дополнительно
публикуется arm/command {"command":"unfreeze"}.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

fastapi = pytest.importorskip('fastapi')

from fastapi.testclient import TestClient   # noqa: E402

from compute_node.dashboard.app import create_app   # noqa: E402
from compute_node.dashboard.state import DashboardState   # noqa: E402


@pytest.fixture
def fake_mqtt():
    m = MagicMock()
    m.connected = True
    m.publish.return_value = True
    return m


@pytest.fixture
def client(fake_mqtt):
    state = DashboardState()
    app = create_app(state, mqtt=fake_mqtt, ros2=None, enable_socketio=False)
    return TestClient(app)


# ── Tests ──────────────────────────────────────────────────────────────
def test_claw_open_publishes_joint4_and_unfreeze_claw_only(client, fake_mqtt):
    """POST /api/actuators/claw {state:open} — публикует ДВЕ команды:
    arm/command {joint:4, angle:0} (открыть клешню) и
    arm/command {command:unfreeze, joint:4} (снять заморозку ТОЛЬКО клешни,
    не CH0/1/2). До фикса bare unfreeze без joint размораживал все
    суставы — это был баг.
    """
    r = client.post('/api/v1/actuators/claw', json={'state': 'open'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    topics_and_payloads = [(call.args[0], call.args[1]) for call in pub_calls]

    # joint=4 angle=0 (открыть клешню)
    assert any(t.endswith('arm/command') and p == {'joint': 4, 'angle': 0.0}
               for t, p in topics_and_payloads)
    # unfreeze joint=4 — ТОЛЬКО клешня
    assert any(t.endswith('arm/command') and p == {'command': 'unfreeze', 'joint': 4}
               for t, p in topics_and_payloads)
    # Bare unfreeze (без joint) — НЕ должно быть, это размораживало бы CH0/1/2
    assert not any(t.endswith('arm/command') and p == {'command': 'unfreeze'}
                   for t, p in topics_and_payloads)


def test_claw_close_does_not_unfreeze(client, fake_mqtt):
    """POST {state:close} — публикуется только joint=4 angle=180.
    Никакого unfreeze (рука как раз должна оставаться frozen для удержания).
    Ни bare {command:unfreeze}, ни joint-specific {command:unfreeze, joint:4}.
    """
    r = client.post('/api/v1/actuators/claw', json={'state': 'close'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    payloads = [call.args[1] for call in pub_calls]

    assert {'joint': 4, 'angle': 180.0} in payloads
    assert {'command': 'unfreeze'} not in payloads
    assert {'command': 'unfreeze', 'joint': 4} not in payloads


def test_claw_angle_below_90_triggers_unfreeze(client, fake_mqtt):
    """POST {angle: 30} — клешня всё ещё в «открытой» зоне (логически
    меньше середины). Должно сработать как open: + unfreeze joint=4 (только клешня).
    """
    r = client.post('/api/v1/actuators/claw', json={'angle': 30.0})
    assert r.status_code == 200

    payloads = [call.args[1] for call in fake_mqtt.publish.call_args_list]
    assert {'joint': 4, 'angle': 30.0} in payloads
    assert {'command': 'unfreeze', 'joint': 4} in payloads
    assert {'command': 'unfreeze'} not in payloads


def test_claw_angle_above_90_does_not_unfreeze(client, fake_mqtt):
    """POST {angle: 150} — клешня близко к закрытой. Никакого unfreeze
    (ни bare, ни joint-specific)."""
    r = client.post('/api/v1/actuators/claw', json={'angle': 150.0})
    assert r.status_code == 200

    payloads = [call.args[1] for call in fake_mqtt.publish.call_args_list]
    assert {'joint': 4, 'angle': 150.0} in payloads
    assert {'command': 'unfreeze'} not in payloads
    assert {'command': 'unfreeze', 'joint': 4} not in payloads
