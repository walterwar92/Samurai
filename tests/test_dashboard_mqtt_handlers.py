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


# ── mps/live_state ─────────────────────────────────────────────────────
import json as _json


def test_mps_live_state_valid_payload_updates_last_and_broadcasts(handlers):
    """Корректный payload → _last_live_state записан, broadcaster вызван."""
    captured = []
    handlers.set_mps_live_state_broadcaster(lambda f: captured.append(f))

    payload = {
        'ts': 1747574400.5,
        'x': [0.0, 0.12, -0.05, 0.0, 0.0],
        'u': [0.12, 0.0],
        'scenario_active': False,
        'run_id': None,
        'schema_version': '1.0',
    }
    handlers._h_mps_live_state(_json.dumps(payload).encode())

    assert handlers._last_live_state is not None
    assert handlers._last_live_state['x'] == [0.0, 0.12, -0.05, 0.0, 0.0]
    assert len(captured) == 1
    assert captured[0]['type'] == 'live_state'
    assert captured[0]['point']['x'][1] == pytest.approx(0.12)


def test_mps_live_state_invalid_x_len_is_dropped(handlers):
    captured = []
    handlers.set_mps_live_state_broadcaster(lambda f: captured.append(f))

    bad = {'ts': 1.0, 'x': [0, 0, 0, 0], 'u': [0, 0],
           'scenario_active': False, 'run_id': None, 'schema_version': '1.0'}
    handlers._h_mps_live_state(_json.dumps(bad).encode())

    assert handlers._last_live_state is None
    assert captured == []


def test_mps_live_state_invalid_u_len_is_dropped(handlers):
    captured = []
    handlers.set_mps_live_state_broadcaster(lambda f: captured.append(f))

    bad = {'ts': 1.0, 'x': [0, 0, 0, 0, 0], 'u': [0],
           'scenario_active': False, 'run_id': None, 'schema_version': '1.0'}
    handlers._h_mps_live_state(_json.dumps(bad).encode())

    assert handlers._last_live_state is None
    assert captured == []


def test_mps_live_state_missing_keys_is_dropped(handlers):
    captured = []
    handlers.set_mps_live_state_broadcaster(lambda f: captured.append(f))

    handlers._h_mps_live_state(b'{}')

    assert handlers._last_live_state is None
    assert captured == []


def test_mps_live_state_broadcaster_exception_does_not_corrupt_buffer(handlers):
    """Падающий broadcaster (например, упавший WS) не должен ломать
    буфер _last_live_state и не должен пробрасывать исключение наверх.
    Буфер пишется ДО вызова broadcaster — это гарантирует send-on-connect
    для следующего клиента, даже если текущий упал."""
    def bad_broadcaster(_frame):
        raise RuntimeError('WS connection lost')

    handlers.set_mps_live_state_broadcaster(bad_broadcaster)
    payload = {
        'ts': 1.0, 'x': [0.0, 0.1, 0.2, 0.3, 0.4], 'u': [0.1, 0.0],
        'scenario_active': False, 'run_id': None, 'schema_version': '1.0',
    }
    # Не должно пробросить исключение.
    handlers._h_mps_live_state(_json.dumps(payload).encode())
    # Буфер записан несмотря на падение broadcaster.
    assert handlers._last_live_state is not None
    assert handlers._last_live_state['x'] == [0.0, 0.1, 0.2, 0.3, 0.4]


# ── calibration/active ─────────────────────────────────────────────────────

def test_calibration_active_saves_full_dict(handlers):
    """_h_calibration_active должен сохранять весь dict
    {profile, scale_fwd, scale_bwd, motor_trim}, а не только имя."""
    payload = json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode()
    handlers._h_calibration_active(payload)
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_incomplete_payload(handlers):
    """Неполный payload (без scale_bwd) не должен затирать существующий state."""
    # Подготовка: положим валидный state.
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    # Атака: неполный payload.
    handlers._h_calibration_active(json.dumps({
        'profile': 'broken',
        'scale_fwd': 2.0,
    }).encode())
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    # State не должен быть затёрт.
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_non_dict_payload(handlers):
    """Голый float / string / list — игнорируем, state не затираем."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    handlers._h_calibration_active(b'42.5')
    handlers._h_calibration_active(b'"justastring"')
    handlers._h_calibration_active(b'[1,2,3]')
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_garbage_json(handlers):
    """Битый JSON — не падаем, state не затираем."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    handlers._h_calibration_active(b'{not json')
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs is not None and coeffs['profile'] == 'tile'
