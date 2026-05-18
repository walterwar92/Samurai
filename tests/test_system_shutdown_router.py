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


def test_shutdown_schedules_self_kill(client, fake_mqtt):
    """BackgroundTask делает os.kill(getpid(), SIGTERM)."""
    import signal as _signal

    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill:
        r = client.post('/api/v1/system/shutdown')
        # TestClient ждёт BackgroundTasks → к этому моменту _shutdown_self уже отработал
        assert r.status_code == 200

    assert mock_kill.called, 'os.kill должен быть вызван BackgroundTask-ом'
    args = mock_kill.call_args.args
    assert args[0] == os.getpid()
    assert args[1] == _signal.SIGTERM
