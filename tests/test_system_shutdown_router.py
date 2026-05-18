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


def test_shutdown_outside_docker_kills_self(client, fake_mqtt):
    """Вне Docker (нет /root/Samurai mount'а): SIGTERM uvicorn'у."""
    import signal as _signal

    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill, \
         patch('compute_node.dashboard.routers.system.os.path.isdir',
               return_value=False):
        r = client.post('/api/v1/system/shutdown')
        assert r.status_code == 200

    assert mock_kill.called, 'Вне Docker должны kill self'
    args = mock_kill.call_args.args
    assert args[0] == os.getpid()
    assert args[1] == _signal.SIGTERM


def test_shutdown_inside_docker_does_not_kill_self(client, fake_mqtt):
    """В Docker: SIGTERM uvicorn'у НЕ шлётся.

    Bash-watcher на хосте увидит .shutdown_request и сделает docker stop.
    Если мы убьём FastAPI сами, ros2 launch с respawn=True перезапустит
    его раньше чем watcher отработает.
    """
    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill, \
         patch('compute_node.dashboard.routers.system._request_compute_shutdown_via_file',
               return_value=True):
        r = client.post('/api/v1/system/shutdown')
        assert r.status_code == 200

    assert not mock_kill.called, (
        'В Docker НЕ должны kill self — иначе ros2 launch перезапустит uvicorn'
    )


def test_request_compute_shutdown_writes_file_in_docker():
    """Unit test для _request_compute_shutdown_via_file."""
    from unittest.mock import mock_open as mo
    from compute_node.dashboard.routers import system as sys_mod

    m = mo()
    with patch.object(sys_mod.os.path, 'isdir', return_value=True), \
         patch('builtins.open', m):
        result = sys_mod._request_compute_shutdown_via_file()

    assert result is True
    # Файл открыт на запись по mount-пути
    call_args = m.call_args
    path_arg = call_args.args[0]
    assert path_arg.replace('\\', '/').endswith('/root/Samurai/.shutdown_request'), (
        f'Ожидался путь .../root/Samurai/.shutdown_request, получили {path_arg}'
    )
    assert call_args.args[1] == 'w'
    m().write.assert_called_once_with('dashboard\n')


def test_request_compute_shutdown_returns_false_outside_docker():
    """Если /root/Samurai не существует — возвращает False."""
    from compute_node.dashboard.routers import system as sys_mod

    with patch.object(sys_mod.os.path, 'isdir', return_value=False):
        assert sys_mod._request_compute_shutdown_via_file() is False
