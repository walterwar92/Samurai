"""Tests for compute_node/dashboard/routers/mps.py.

FastAPI TestClient. MQTT — заменён фейковым стабом: REST-роутер не должен
требовать живого брокера для sim-режима и для valid/draft endpoints.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

fastapi = pytest.importorskip('fastapi')
scipy = pytest.importorskip('scipy')

from fastapi.testclient import TestClient  # noqa: E402

from compute_node.dashboard.app import create_app  # noqa: E402
from compute_node.dashboard.state import DashboardState  # noqa: E402


# ── Fixtures ───────────────────────────────────────────────────────────
@pytest.fixture
def fake_mqtt():
    """Mocked MQTTHandlers — supports .publish(topic, payload, qos) +
    .connected attribute. Does NOT touch a real broker."""
    m = MagicMock()
    m.connected = True
    m.publish.return_value = True
    return m


@pytest.fixture
def client(fake_mqtt):
    state = DashboardState()
    app = create_app(state, mqtt=fake_mqtt, ros2=None, enable_socketio=False)
    return TestClient(app)


@pytest.fixture
def matrices_payload():
    """Каноническая НЕПРЕРЫВНАЯ модель — проходит Pydantic-валидацию."""
    tau_v, tau_w = 0.15, 0.10
    return {
        'A': [
            [0.0,  1.0,        0.0,  0.0,        0.0],
            [0.0, -1.0/tau_v,  0.0,  0.0,        0.0],
            [0.0,  0.0,        0.0,  1.0,        0.0],
            [0.0,  0.0,        0.0, -1.0/tau_w,  0.0],
            [0.0,  0.0,       -1.0,  0.0,        0.0],
        ],
        'B': [
            [0.0,        0.0],
            [1.0/tau_v,  0.0],
            [0.0,        0.0],
            [0.0,        1.0/tau_w],
            [0.0,        0.0],
        ],
        'C': [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)],
        'D': [[0.0, 0.0] for _ in range(5)],
        'Q_diag': [10, 10, 5, 1, 1],
        'R_diag': [1, 1],
        'horizon_N': 10,
        'u_min': [-0.30, -2.0],
        'u_max': [0.30, 2.0],
    }


# ── Liveness ───────────────────────────────────────────────────────────
def test_root_ok(client):
    r = client.get('/api/v1/mps')
    assert r.status_code == 200
    assert r.json()['ok'] is True


# ── /matrices ──────────────────────────────────────────────────────────
def test_get_matrices_seeds_from_config(client):
    r = client.get('/api/v1/mps/matrices')
    assert r.status_code == 200
    body = r.json()
    assert body['ok'] is True
    assert body['applied'] is not None
    assert body['draft'] is None
    # 5×5 / 5×2
    assert len(body['applied']['A']) == 5
    assert len(body['applied']['A'][0]) == 5
    assert len(body['applied']['B'][0]) == 2


def test_post_matrices_saves_draft(client, matrices_payload):
    r = client.post('/api/v1/mps/matrices', json=matrices_payload)
    assert r.status_code == 200
    body = r.json()
    assert body['status'] == 'draft'
    # GET reflects draft
    r2 = client.get('/api/v1/mps/matrices')
    assert r2.json()['draft'] is not None


def test_post_matrices_invalid_shape_rejected(client, matrices_payload):
    bad = dict(matrices_payload)
    bad['A'] = [[1, 0], [0, 1]]   # 2×2 ≠ 5×5
    r = client.post('/api/v1/mps/matrices', json=bad)
    assert r.status_code == 422


def test_apply_promotes_draft_and_publishes(client, matrices_payload, fake_mqtt):
    client.post('/api/v1/mps/matrices', json=matrices_payload)
    r = client.post('/api/v1/mps/matrices/apply')
    assert r.status_code == 200
    assert r.json()['status'] == 'applied'
    # Pi receives MQTT — `mps/matrices/set` was the topic.
    topics = [c.args[0] for c in fake_mqtt.publish.call_args_list]
    assert 'mps/matrices/set' in topics


def test_apply_with_no_draft_and_no_body_400(client):
    # Reset draft just in case.
    r = client.post('/api/v1/mps/matrices/apply')
    assert r.status_code == 400


def test_reset_loads_defaults(client, matrices_payload, fake_mqtt):
    client.post('/api/v1/mps/matrices', json=matrices_payload)
    r = client.post('/api/v1/mps/matrices/reset')
    assert r.status_code == 200
    assert r.json()['status'] == 'applied'
    body = client.get('/api/v1/mps/matrices').json()
    assert body['draft'] is None


# ── /validate ──────────────────────────────────────────────────────────
def test_validate_returns_eigenvalues(client):
    # GET /matrices first to seed applied
    client.get('/api/v1/mps/matrices')
    r = client.post('/api/v1/mps/validate', json=None)
    assert r.status_code == 200
    body = r.json()
    assert len(body['eigenvalues_ad']) == 5
    assert len(body['eigenvalues_closed']) == 5
    # is_*_stable booleans present
    assert isinstance(body['is_plant_stable'], bool)
    assert isinstance(body['is_closed_loop_stable'], bool)


def test_validate_with_explicit_matrices(client, matrices_payload):
    r = client.post('/api/v1/mps/validate', json=matrices_payload)
    assert r.status_code == 200
    body = r.json()
    assert body['is_closed_loop_stable'] is True
    # step_response should have telemetry points
    assert isinstance(body['step_response'], list)


def test_validate_canonical_plant_marginal_not_unstable(client, matrices_payload):
    """Каноническая модель: 3 полюса Ad на |λ|=1 (интеграторы s,θ,e_int).
    is_plant_stable=False (строгая асимптотика), но предупреждение —
    про маргинальную устойчивость, НЕ про неустойчивость; замкнутый
    контур устойчив (модель управляема)."""
    r = client.post('/api/v1/mps/validate', json=matrices_payload)
    assert r.status_code == 200
    body = r.json()
    assert body['is_plant_stable'] is False
    assert body['is_closed_loop_stable'] is True
    joined = ' '.join(body['warnings'])
    assert 'маргинально устойчива' in joined
    assert 'неустойчива' not in joined


# ── /scenario/run (sim) ────────────────────────────────────────────────
def test_scenario_run_sim_returns_result(client):
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.5, 'v_target': 0.15, 'source': 'sim',
    })
    assert r.status_code == 200
    body = r.json()
    assert body['run_id'].startswith('sim-')
    assert body['result'] is not None
    # 'timeout_settle' добавлен в Task 4 pose-tracking refactor —
    # длинные D + дефолтный Q[s]=10 не успевают сойтись на ε_s=0.005
    # за settle_timeout=1.5 c (моторный лаг τ_v=0.15).
    assert body['result']['status'] in ('reached', 'timeout', 'timeout_settle', 'error')
    assert len(body['result']['telemetry']) > 0


def test_scenario_run_sim_writes_history(client):
    for _ in range(3):
        client.post('/api/v1/mps/scenario/run', json={
            'distance': 0.5, 'v_target': 0.10, 'source': 'sim',
        })
    r = client.get('/api/v1/mps/history')
    assert r.status_code == 200
    assert len(r.json()['history']) >= 3


def test_scenario_run_safety_cap(client):
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 50.0, 'v_target': 0.15, 'source': 'sim',
    })
    # Pydantic validator catches > 5.0 first
    assert r.status_code == 422


def test_scenario_run_robot_when_offline(client, fake_mqtt):
    fake_mqtt.connected = False
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
    })
    assert r.status_code == 503


def test_scenario_run_robot_when_online_async(client, fake_mqtt):
    fake_mqtt.connected = True
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
    })
    assert r.status_code == 200
    body = r.json()
    assert body['run_id'].startswith('robot-')
    assert body['result'] is None
    topics = [c.args[0] for c in fake_mqtt.publish.call_args_list]
    assert 'mps/scenario/run' in topics


def test_scenario_run_robot_includes_target_heading_in_mqtt(client, fake_mqtt):
    """Robot-прогон с target_heading прокидывает его в MQTT-payload
    mps/scenario/run — Pi должен знать относительный курс цели."""
    fake_mqtt.connected = True
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
        'target_heading': 0.6,
    })
    assert r.status_code == 200
    run_call = next(c for c in fake_mqtt.publish.call_args_list
                    if c.args[0] == 'mps/scenario/run')
    payload = run_call.args[1]
    assert payload['request']['target_heading'] == pytest.approx(0.6)


def test_scenario_run_includes_reference_in_mqtt_payload(client, fake_mqtt):
    """POST /api/v1/mps/scenario/run с source=robot публикует payload с reference."""
    fake_mqtt.connected = True
    resp = client.post('/api/v1/mps/scenario/run', json={
        'distance': 0.30, 'v_target': 0.15, 'target_heading': 0.0,
        'source': 'robot',
    })
    assert resp.status_code == 200
    run_call = next(c for c in fake_mqtt.publish.call_args_list
                    if c.args[0] == 'mps/scenario/run')
    payload = run_call.args[1]
    assert 'reference' in payload
    assert payload['reference']['a_max'] > 0
    assert payload['reference']['alpha_max'] > 0


def test_scenario_run_target_heading_defaults_to_zero(client):
    """Без target_heading в запросе — Pydantic дефолтит в 0.0
    (обратная совместимость, поведение «вперёд»)."""
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'sim',
    })
    assert r.status_code == 200
    assert r.json()['result']['request']['target_heading'] == 0.0


def test_scenario_run_target_heading_out_of_range_rejected(client):
    """target_heading вне [−π, π] → 422 (Pydantic ge/le)."""
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
        'target_heading': 4.0,
    })
    assert r.status_code == 422


# ── /scenario/{run_id} & abort ────────────────────────────────────────
def test_scenario_status_404_for_unknown(client):
    r = client.get('/api/v1/mps/scenario/bogus-id')
    assert r.status_code == 404


def test_scenario_status_returns_history_run(client):
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 0.5, 'v_target': 0.10, 'source': 'sim',
    })
    run_id = r.json()['run_id']
    r2 = client.get(f'/api/v1/mps/scenario/{run_id}')
    assert r2.status_code == 200
    assert r2.json()['run_id'] == run_id


def test_scenario_abort_when_no_active(client):
    r = client.post('/api/v1/mps/scenario/abort')
    assert r.status_code == 200
    assert r.json()['aborted'] is False


def test_robot_run_abort_then_rerun_does_not_409(client, fake_mqtt):
    """Регрессия: после /scenario/abort повторный /scenario/run на роботе
    не должен ловить 409 Conflict, даже если Pi не прислал mps/scenario/finished
    (mps_node лежит, MQTT временно offline, и т.п.).

    До фикса /scenario/abort только публиковал MQTT, не трогая
    state.mps.active_run → второй запуск зависал в 'running' навсегда."""
    fake_mqtt.connected = True

    r1 = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
    })
    assert r1.status_code == 200
    run_id_1 = r1.json()['run_id']

    # Abort — Pi не отвечает, mps/scenario/finished не приходит.
    ra = client.post('/api/v1/mps/scenario/abort')
    assert ra.status_code == 200
    assert ra.json()['aborted'] is True
    assert ra.json()['run_id'] == run_id_1

    # Aborted run должен быть в history со статусом 'aborted'.
    hist = client.get('/api/v1/mps/history').json()['history']
    assert any(h['run_id'] == run_id_1 and h['status'] == 'aborted' for h in hist)

    # Повторный запуск — НЕ 409.
    r2 = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.0, 'v_target': 0.10, 'source': 'robot',
    })
    assert r2.status_code == 200, f'expected 200 after abort, got {r2.status_code}: {r2.text}'
    assert r2.json()['run_id'] != run_id_1


# ── /history/{run_id}/replay ──────────────────────────────────────────
def test_history_replay_creates_new_run(client):
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 0.5, 'v_target': 0.10, 'source': 'sim',
    })
    rid = r.json()['run_id']
    r2 = client.post(f'/api/v1/mps/history/{rid}/replay')
    assert r2.status_code == 200
    new_rid = r2.json()['run_id']
    assert new_rid != rid


def test_history_replay_404_for_unknown(client):
    r = client.post('/api/v1/mps/history/bogus/replay')
    assert r.status_code == 404


# ── WebSocket /ws/mps/telemetry ───────────────────────────────────────
def test_websocket_accepts_and_streams_via_broker(client):
    """WS принимает subscribe и получает frame через broker.broadcast()."""
    from compute_node.dashboard.routers.mps import mps_broker

    with client.websocket_connect('/ws/mps/telemetry') as ws:
        ws.send_text('{"action":"subscribe"}')
        # Дать серверу подхватить subscribe (и установить broker._loop).
        # broadcast() из «другого треда» дублируется в asyncio loop —
        # для теста просто пушим напрямую в очередь подписчика.
        mps_broker.broadcast({
            'type': 'telemetry',
            'run_id': 'test',
            'point': {
                't': 0.0, 'x': [0, 0, 0, 0, 0],
                'u': [0, 0], 'y': [0, 0, 0, 0, 0], 's_remaining': 1.0,
            },
        })
        msg = ws.receive_json()
        assert msg['type'] == 'telemetry'
        assert msg['run_id'] == 'test'


# ── WebSocket /ws/mps/live_state ──────────────────────────────────────
def test_live_state_ws_replays_last_on_connect(client):
    """При подключении сервер шлёт последний известный фрейм сразу."""
    from compute_node.dashboard.routers.mps import mps_live_state_broker

    last = {
        'ts': 1747574400.0,
        'x': [0.0, 0.1, 0.0, 0.0, 0.0],
        'u': [0.1, 0.0],
        'scenario_active': False,
        'run_id': None,
        'schema_version': '1.0',
    }
    mps_live_state_broker.set_last(last)

    with client.websocket_connect('/ws/mps/live_state') as ws:
        msg = ws.receive_json()
        assert msg['type'] == 'live_state'
        assert msg['point']['x'][1] == pytest.approx(0.1)


def test_live_state_ws_broadcasts_new_frame(client):
    """Открытый WS получает новые frames через broker.broadcast()."""
    from compute_node.dashboard.routers.mps import mps_live_state_broker

    # Сбросить буфер last (предыдущий тест мог его положить).
    mps_live_state_broker.set_last(None)

    with client.websocket_connect('/ws/mps/live_state') as ws:
        mps_live_state_broker.broadcast({
            'type': 'live_state',
            'point': {
                'ts': 1747574500.0,
                'x': [1.0, 0.2, 0.05, 0.0, 0.0],
                'u': [0.2, 0.0],
                'scenario_active': True,
                'run_id': 'r-1',
                'schema_version': '1.0',
            },
        })
        msg = ws.receive_json()
        assert msg['type'] == 'live_state'
        assert msg['point']['scenario_active'] is True
        assert msg['point']['run_id'] == 'r-1'


def test_live_state_ws_no_last_no_replay(client):
    """Если _last is None — клиент не получает фрейм до broadcast."""
    from compute_node.dashboard.routers.mps import mps_live_state_broker

    mps_live_state_broker.set_last(None)

    with client.websocket_connect('/ws/mps/live_state') as ws:
        # Ожидание сразу таймаутит — мы НЕ ждём reply, пушим и проверяем.
        mps_live_state_broker.broadcast({
            'type': 'live_state',
            'point': {
                'ts': 1.0, 'x': [0, 0, 0, 0, 0], 'u': [0, 0],
                'scenario_active': False, 'run_id': None,
                'schema_version': '1.0',
            },
        })
        msg = ws.receive_json()
        assert msg['type'] == 'live_state'
