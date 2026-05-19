"""Tests for compute_node/dashboard/routers/robot.py — live_state aggregator + WS.

Spec: docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md
"""
from __future__ import annotations

import math
import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

fastapi = pytest.importorskip('fastapi')

from fastapi.testclient import TestClient  # noqa: E402

from compute_node.dashboard.app import create_app  # noqa: E402
from compute_node.dashboard.state import DashboardState  # noqa: E402
from compute_node.dashboard.schemas.sensors import ImuData, ImuYpr, Vec3  # noqa: E402
from compute_node.dashboard.schemas.robot import RobotPose, VelocityDetail  # noqa: E402


# ── Fixtures ───────────────────────────────────────────────────────────
@pytest.fixture
def fake_mqtt():
    m = MagicMock()
    m.connected = True
    m.publish.return_value = True
    return m


@pytest.fixture
def state_full():
    """DashboardState с осмысленными значениями pose/velocity/imu."""
    s = DashboardState()
    with s.lock:
        s.robot.pose = RobotPose(x=1.0, y=2.0, yaw=math.pi / 2)
        s.robot.velocity_estimated = VelocityDetail(linear_x=0.1, angular_z=0.05)
        s.robot.mqtt_odom_ts = 1747574400.0
        s.robot.stationary = False
        s.sensors.imu = ImuData(
            yaw=10.0, pitch=5.0, roll=0.0,
            gyro=Vec3(x=0.01, y=0.02, z=0.21),
            accel=Vec3(x=0.05, y=0.02, z=9.81),
            ekf=ImuYpr(yaw=10.0, pitch=5.0, roll=0.0),
        )
        s.sensors.imu_ekf_bias = [0.001, -0.002, 0.003]
    return s


@pytest.fixture
def client(fake_mqtt):
    state = DashboardState()
    app = create_app(state, mqtt=fake_mqtt, ros2=None, enable_socketio=False)
    return TestClient(app)


# ── build_live_state_point ─────────────────────────────────────────────
def test_build_live_state_point_basic(state_full):
    from compute_node.dashboard.routers.robot import build_live_state_point
    point = build_live_state_point(state_full)
    assert point.pose.x == pytest.approx(1.0)
    assert point.pose.y == pytest.approx(2.0)
    assert point.pose.yaw_rad == pytest.approx(math.pi / 2)
    assert point.pose.yaw_deg == pytest.approx(90.0)
    assert point.vel.linear == pytest.approx(0.1)
    assert point.vel.angular == pytest.approx(0.05)
    assert point.imu.ypr_deg == [10.0, 5.0, 0.0]
    assert point.imu.gyro == [0.01, 0.02, 0.21]
    assert point.imu.accel == [0.05, 0.02, 9.81]
    assert point.imu.has_ekf is True
    assert point.stationary is False
    assert point.schema_version == '1.0'
    assert point.ts == pytest.approx(1747574400.0)


def test_build_live_state_point_no_ekf():
    from compute_node.dashboard.routers.robot import build_live_state_point
    s = DashboardState()
    with s.lock:
        s.sensors.imu = ImuData(
            yaw=0.0, pitch=2.0, roll=-1.5,
            gyro=Vec3(), accel=Vec3(z=9.8),
            ekf=None,
        )
    point = build_live_state_point(s)
    assert point.imu.has_ekf is False
    assert point.imu.ekf_bias_deg is None
    assert point.imu.ypr_deg == [0.0, 2.0, -1.5]  # raw fallback


def test_build_live_state_point_bias_conversion(state_full):
    """Pi публикует bias в рад/с — aggregator конвертит в °/с."""
    from compute_node.dashboard.routers.robot import build_live_state_point
    point = build_live_state_point(state_full)
    # 0.001 рад/с ≈ 0.05729° / с
    assert point.imu.ekf_bias_deg is not None
    assert point.imu.ekf_bias_deg[0] == pytest.approx(math.degrees(0.001), abs=1e-4)
    assert point.imu.ekf_bias_deg[1] == pytest.approx(math.degrees(-0.002), abs=1e-4)
    assert point.imu.ekf_bias_deg[2] == pytest.approx(math.degrees(0.003), abs=1e-4)


def test_build_live_state_point_ts_fallback_when_no_odom():
    """Если mqtt_odom_ts == 0 (холодный старт), используем time.time()."""
    from compute_node.dashboard.routers.robot import build_live_state_point
    s = DashboardState()
    # mqtt_odom_ts по умолчанию 0.0
    point = build_live_state_point(s)
    assert point.ts > 1_700_000_000  # ≥ 2023-11-15, т.е. time.time()


# ── _RobotLiveStateBroker ──────────────────────────────────────────────
def test_broker_add_remove_subscriber():
    from compute_node.dashboard.routers.robot import _RobotLiveStateBroker
    import asyncio
    b = _RobotLiveStateBroker()
    q1: asyncio.Queue = asyncio.Queue()
    q2: asyncio.Queue = asyncio.Queue()
    b.add(q1)
    b.add(q2)
    b.remove(q1)
    # broadcast должен попасть только в q2
    b.broadcast({'type': 'live_state', 'point': {'ts': 1.0}})
    assert q2.qsize() == 1
    assert q1.qsize() == 0


def test_broker_set_get_last():
    from compute_node.dashboard.routers.robot import _RobotLiveStateBroker
    b = _RobotLiveStateBroker()
    assert b.get_last() is None
    frame = {'type': 'live_state', 'point': {'ts': 2.0}}
    b.set_last(frame)
    assert b.get_last() == frame


def test_broker_broadcast_persists_last():
    """broadcast() должен также обновлять _last для replay новых клиентов."""
    from compute_node.dashboard.routers.robot import _RobotLiveStateBroker
    b = _RobotLiveStateBroker()
    frame = {'type': 'live_state', 'point': {'ts': 3.0}}
    b.broadcast(frame)
    assert b.get_last() == frame


# ── /ws/robot/live_state ───────────────────────────────────────────────
def test_robot_live_state_ws_replays_last_on_connect(client):
    """При подключении сервер шлёт последний известный фрейм сразу."""
    from compute_node.dashboard.routers.robot import robot_live_state_broker

    last = {
        'type': 'live_state',
        'point': {
            'ts': 1747574400.0,
            'pose': {'x': 0.1, 'y': 0.2, 'yaw_rad': 0.0, 'yaw_deg': 0.0},
            'vel': {'linear': 0.0, 'angular': 0.0},
            'imu': {
                'ypr_deg': [0, 0, 0], 'gyro': [0, 0, 0],
                'accel': [0, 0, 9.8], 'ekf_bias_deg': None, 'has_ekf': False,
            },
            'stationary': True,
            'schema_version': '1.0',
        },
    }
    robot_live_state_broker.set_last(last)
    try:
        with client.websocket_connect('/ws/robot/live_state') as ws:
            msg = ws.receive_json()
            assert msg['type'] == 'live_state'
            assert msg['point']['pose']['x'] == pytest.approx(0.1)
    finally:
        robot_live_state_broker.set_last(None)


def test_robot_live_state_ws_broadcasts_new_frame(client):
    """Открытый WS получает новые frames через broker.broadcast()."""
    from compute_node.dashboard.routers.robot import robot_live_state_broker
    robot_live_state_broker.set_last(None)
    try:
        with client.websocket_connect('/ws/robot/live_state') as ws:
            robot_live_state_broker.broadcast({
                'type': 'live_state',
                'point': {
                    'ts': 1747574500.0,
                    'pose': {'x': 1.5, 'y': 2.5, 'yaw_rad': 0.5, 'yaw_deg': 28.6},
                    'vel': {'linear': 0.2, 'angular': 0.1},
                    'imu': {
                        'ypr_deg': [28, 2, -1], 'gyro': [0.01, 0.02, 0.03],
                        'accel': [0.1, 0.2, 9.8],
                        'ekf_bias_deg': [0.01, -0.02, 0.03], 'has_ekf': True,
                    },
                    'stationary': False,
                    'schema_version': '1.0',
                },
            })
            msg = ws.receive_json()
            assert msg['type'] == 'live_state'
            assert msg['point']['pose']['x'] == pytest.approx(1.5)
            assert msg['point']['imu']['has_ekf'] is True
    finally:
        robot_live_state_broker.set_last(None)


def test_robot_live_state_ws_no_last_no_replay(client):
    """Если _last is None — клиент НЕ получает phantom frame до broadcast."""
    from compute_node.dashboard.routers.robot import robot_live_state_broker
    robot_live_state_broker.set_last(None)

    def _frame(ts: float) -> dict:
        return {
            'type': 'live_state',
            'point': {
                'ts': ts,
                'pose': {'x': 0, 'y': 0, 'yaw_rad': 0, 'yaw_deg': 0},
                'vel': {'linear': 0, 'angular': 0},
                'imu': {'ypr_deg': [0, 0, 0], 'gyro': [0, 0, 0],
                        'accel': [0, 0, 0], 'ekf_bias_deg': None, 'has_ekf': False},
                'stationary': True,
                'schema_version': '1.0',
            },
        }

    try:
        with client.websocket_connect('/ws/robot/live_state') as ws:
            robot_live_state_broker.broadcast(_frame(1.0))
            robot_live_state_broker.broadcast(_frame(2.0))
            msg1 = ws.receive_json()
            msg2 = ws.receive_json()
            assert msg1['point']['ts'] == pytest.approx(1.0)
            assert msg2['point']['ts'] == pytest.approx(2.0)
    finally:
        robot_live_state_broker.set_last(None)


# ── _robot_live_state_loop integration ────────────────────────────────
@pytest.mark.asyncio
async def test_robot_live_state_loop_broadcasts_when_subscriber():
    """Loop @ 10 Hz должен публиковать в broker когда есть подписчики."""
    import asyncio
    from compute_node.dashboard.app import _run_robot_live_state_tick
    from compute_node.dashboard.routers.robot import robot_live_state_broker

    s = DashboardState()
    with s.lock:
        s.sensors.imu = ImuData(
            yaw=15.0, pitch=2.0, roll=0.0,
            gyro=Vec3(), accel=Vec3(z=9.8), ekf=ImuYpr(yaw=15.0),
        )
        s.robot.mqtt_odom_ts = 1747574400.0

    # Регистрируем подписчика — иначе tick пропустит работу.
    q: asyncio.Queue[dict] = asyncio.Queue()
    robot_live_state_broker.add(q)
    try:
        await _run_robot_live_state_tick(s)
        frame = q.get_nowait()
        assert frame['type'] == 'live_state'
        assert frame['point']['pose']['x'] == pytest.approx(0.0)
        assert frame['point']['imu']['ypr_deg'][0] == pytest.approx(15.0)
    finally:
        robot_live_state_broker.remove(q)
        robot_live_state_broker.set_last(None)


@pytest.mark.asyncio
async def test_robot_live_state_loop_skips_when_no_subscribers():
    """Без подписчиков — tick не должен дёргать broker.broadcast()."""
    from unittest.mock import patch
    from compute_node.dashboard.app import _run_robot_live_state_tick
    from compute_node.dashboard.routers.robot import robot_live_state_broker
    s = DashboardState()
    robot_live_state_broker.set_last(None)
    with patch.object(robot_live_state_broker, 'broadcast') as mock:
        await _run_robot_live_state_tick(s)
        mock.assert_not_called()
