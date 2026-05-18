"""Тесты для POST /api/v1/system/shutdown.

Endpoint должен:
1. Опубликовать MQTT samurai/{robot_id}/system/shutdown
2. Запланировать SIGTERM самому себе через BackgroundTask
3. Вернуть 200 OK сразу (до kill)
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

fastapi = pytest.importorskip('fastapi')

from fastapi.testclient import TestClient  # noqa: E402

from compute_node.dashboard.app import create_app  # noqa: E402
from compute_node.dashboard.state import DashboardState  # noqa: E402


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


def test_shutdown_publishes_mqtt(client, fake_mqtt):
    """POST /api/v1/system/shutdown публикует system/shutdown в MQTT."""
    with patch('compute_node.dashboard.routers.system.os.kill'):
        r = client.post('/api/v1/system/shutdown')

    assert r.status_code == 200
    pub_calls = fake_mqtt.publish.call_args_list
    topics = [c.args[0] for c in pub_calls]
    assert any(t.endswith('system/shutdown') for t in topics), (
        f'Ожидался publish в system/shutdown, было: {topics}'
    )


def test_shutdown_schedules_self_kill_outside_docker(client, fake_mqtt):
    """Вне Docker (нет /.dockerenv): BackgroundTask делает os.kill(getpid(), SIGTERM)."""
    import signal as _signal

    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill, \
         patch('compute_node.dashboard.routers.system.os.path.exists',
               return_value=False) as mock_exists:
        r = client.post('/api/v1/system/shutdown')
        assert r.status_code == 200

    mock_exists.assert_any_call('/.dockerenv')
    assert mock_kill.called, 'os.kill должен быть вызван BackgroundTask-ом'
    args = mock_kill.call_args.args
    assert args[0] == os.getpid()
    assert args[1] == _signal.SIGTERM


def test_shutdown_kills_pid_1_inside_docker(client, fake_mqtt):
    """Внутри Docker (/.dockerenv существует): SIGTERM в PID 1, чтобы убить контейнер."""
    import signal as _signal

    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill, \
         patch('compute_node.dashboard.routers.system.os.path.exists',
               return_value=True):
        r = client.post('/api/v1/system/shutdown')
        assert r.status_code == 200

    assert mock_kill.called
    args = mock_kill.call_args.args
    assert args[0] == 1, f'В Docker должны убивать PID 1, не {args[0]}'
    assert args[1] == _signal.SIGTERM
