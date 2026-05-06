"""
Unit tests for compute_node.dashboard.mqtt_handlers — per-topic dispatch.

Воспроизводит баги, найденные в логах 2026-05-04:
  • [battery] Pydantic отвергает float→int (publisher шлёт `percent: 12.1`).
  • [temperature] handler падает 'float object has no attribute get'
    когда publisher шлёт голый float (`temperature_node` так и делает).
"""
from __future__ import annotations

import json

import pytest

from compute_node.dashboard.mqtt_handlers import MQTTHandlers
from compute_node.dashboard.state import DashboardState


@pytest.fixture
def handlers():
    """MQTTHandlers с фиктивным broker — start() не вызываем, только dispatch."""
    state = DashboardState()
    h = MQTTHandlers(broker='127.0.0.1', port=1883, robot_id='robot1', state=state)
    return h


# ── battery ────────────────────────────────────────────────────────────────

def test_battery_handler_accepts_float_percent(handlers):
    """battery_node.py:120 публикует round(percent, 1) — float (12.1, 8.8, ...).
    Schema BatteryStatus.percent должна это принимать без ValidationError."""
    payload = json.dumps({'voltage': 7.42, 'percent': 12.1}).encode()
    handlers._h_battery(payload)
    with handlers._state.lock:
        battery = handlers._state.sensors.battery
    assert battery.voltage == pytest.approx(7.42)
    assert battery.percent == pytest.approx(12.1)


def test_battery_handler_accepts_integer_percent(handlers):
    """Backward-compat: старые publishers могут слать int."""
    payload = json.dumps({'voltage': 8.0, 'percent': 60}).encode()
    handlers._h_battery(payload)
    with handlers._state.lock:
        battery = handlers._state.sensors.battery
    assert battery.percent == pytest.approx(60)


# ── temperature ────────────────────────────────────────────────────────────

def test_temperature_handler_accepts_scalar_float(handlers):
    """temperature_node.py:40 публикует голый float (`self.publish('temperature', 42.5)`).
    json.loads(b'42.5') возвращает float — handler должен это распознавать
    и не падать с 'float object has no attribute get'."""
    payload = b'42.5'
    handlers._h_temperature(payload)
    with handlers._state.lock:
        t = handlers._state.sensors.temperature
    assert t.value == pytest.approx(42.5)
    assert t.unit == 'C'


def test_temperature_handler_accepts_dict_payload(handlers):
    """Backward-compat: dict-форма {value, unit} тоже валидна."""
    payload = json.dumps({'value': 70.0, 'unit': 'C'}).encode()
    handlers._h_temperature(payload)
    with handlers._state.lock:
        t = handlers._state.sensors.temperature
    assert t.value == pytest.approx(70.0)
    assert t.unit == 'C'


def test_temperature_handler_accepts_integer_scalar(handlers):
    """JSON `42` парсится как int → handler должен принять."""
    payload = b'42'
    handlers._h_temperature(payload)
    with handlers._state.lock:
        t = handlers._state.sensors.temperature
    assert t.value == pytest.approx(42.0)
    assert t.unit == 'C'
