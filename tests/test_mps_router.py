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
    """Default-shaped matrices payload that passes Pydantic validation."""
    return {
        'A': [
            [1, 0, 0, 0.0425203, 0],
            [0, 1, 0.01, 0, 0.000213061],
            [0, 0, 1, 0, 0.0393469],
            [0, 0, 0, 0.716531, 0],
            [0, 0, 0, 0, 0.606531],
        ],
        'B': [
            [0.0074797, 0],
            [0, 3.69387e-05],
            [0, 0.0106531],
            [0.283469, 0],
            [0, 0.393469],
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


# ── /scenario/run (sim) ────────────────────────────────────────────────
def test_scenario_run_sim_returns_result(client):
    r = client.post('/api/v1/mps/scenario/run', json={
        'distance': 1.5, 'v_target': 0.15, 'source': 'sim',
    })
    assert r.status_code == 200
    body = r.json()
    assert body['run_id'].startswith('sim-')
    assert body['result'] is not None
    assert body['result']['status'] in ('reached', 'timeout', 'error')
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
