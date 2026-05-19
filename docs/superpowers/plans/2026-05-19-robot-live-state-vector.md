# Robot Live State Vector — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Добавить постоянную панель «Состояние робота» на DashboardPage,
показывающую вектор состояния (pose, velocity, полный IMU) в реальном
времени @ 10 Hz через выделенный WebSocket `/ws/robot/live_state`.

**Architecture:** Server-side aggregator @ 10 Hz читает `DashboardState`
напрямую (без новых MQTT-топиков), пушит JSON через `_RobotLiveStateBroker`
(зеркало `_MpsLiveStateBroker`) в WS-клиентов. Фронт — отдельный хук
`useRobotLiveState` с auto-reconnect и stale-таймером (2s), новый
самодостаточный компонент `RobotStateVector`.

**Tech Stack:** Python 3.11 + FastAPI (WS) + Pydantic v2 (схема),
React 18 + TypeScript + Tailwind + vitest (фронт), pytest +
`TestClient.websocket_connect` (бэк).

**Spec:** [docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md](../specs/2026-05-19-robot-live-state-vector-design.md)

---

## File Structure

**Создаются:**
- `compute_node/dashboard/schemas/robot_live_state.py` — Pydantic-контракт.
- `tests/test_robot_live_state.py` — pytest для бэкенда (aggregator + WS).
- `compute_node/frontend/src/hooks/useRobotLiveState.ts` — WS-хук.
- `compute_node/frontend/src/hooks/useRobotLiveState.test.tsx` — vitest для хука.
- `compute_node/frontend/src/components/sensors/RobotStateVector.tsx` — панель.
- `compute_node/frontend/src/components/sensors/RobotStateVector.test.tsx` — vitest для панели.

**Изменяются:**
- `compute_node/dashboard/routers/robot.py` — добавляется `ws_router`,
  `_RobotLiveStateBroker`, `build_live_state_point()`, WS-endpoint.
- `compute_node/dashboard/app.py` — регистрация `ws_router` + новый
  background-task `_robot_live_state_loop`.
- `compute_node/frontend/src/types/robot.ts` — добавляются типы
  `RobotLiveStatePoint`, `RobotLiveStateWsFrame`, константа `ROBOT_LIVE_STATE_SCHEMA`.
- `compute_node/frontend/src/pages/DashboardPage.tsx` — вставка `<RobotStateVector />`.

---

## Phase A — Backend contract + aggregator

### Task 1: Pydantic-схема `RobotLiveStatePoint`

**Files:**
- Create: `compute_node/dashboard/schemas/robot_live_state.py`

- [ ] **Step 1: Создать файл со схемой**

`compute_node/dashboard/schemas/robot_live_state.py`:

```python
"""
Pydantic-контракт для /ws/robot/live_state.

См. docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
"""
from __future__ import annotations

from typing import Literal, Optional
from pydantic import BaseModel, Field


class RobotLiveStatePose(BaseModel):
    """Поза робота в world frame."""
    x: float = Field(description='m, world frame')
    y: float = Field(description='m, world frame')
    yaw_rad: float = Field(description='heading в радианах (state.robot.pose.yaw)')
    yaw_deg: float = Field(description='heading в градусах (для удобства UI)')


class RobotLiveStateVel(BaseModel):
    """Текущая оценённая скорость."""
    linear: float = Field(description='м/с — VelocityDetail.linear_x')
    angular: float = Field(description='рад/с — VelocityDetail.angular_z')


class RobotLiveStateImu(BaseModel):
    """IMU snapshot: ориентация + сырые гироскоп/акселерометр + EKF bias."""
    ypr_deg: list[float] = Field(
        description='[yaw, pitch, roll] в °. EKF-результат если has_ekf, '
                    'иначе raw-fallback из акселя.',
    )
    gyro: list[float] = Field(description='[x, y, z] рад/с')
    accel: list[float] = Field(description='[x, y, z] м/с²')
    ekf_bias_deg: Optional[list[float]] = Field(
        default=None,
        description='[x, y, z] °/с; None если EKF выключен',
    )
    has_ekf: bool


class RobotLiveStatePoint(BaseModel):
    """Полный snapshot состояния робота для UI-панели."""
    ts: float = Field(description='Pi-clock unix-секунды')
    pose: RobotLiveStatePose
    vel: RobotLiveStateVel
    imu: RobotLiveStateImu
    stationary: bool = Field(description='ZUPT-флаг (state.robot.stationary)')
    schema_version: Literal['1.0'] = '1.0'
```

- [ ] **Step 2: Verify import works**

Run: `python -c "from compute_node.dashboard.schemas.robot_live_state import RobotLiveStatePoint; print(RobotLiveStatePoint.model_fields.keys())"`

Expected output:
```
dict_keys(['ts', 'pose', 'vel', 'imu', 'stationary', 'schema_version'])
```

- [ ] **Step 3: Commit**

```bash
git add compute_node/dashboard/schemas/robot_live_state.py
git commit -m "feat(dashboard): pydantic schema for robot live_state contract"
```

---

### Task 2: `build_live_state_point()` aggregator + тесты

**Files:**
- Modify: `compute_node/dashboard/routers/robot.py` (добавить функцию в конец файла)
- Create: `tests/test_robot_live_state.py`

- [ ] **Step 1: Написать тесты (failing)**

`tests/test_robot_live_state.py` — новый файл:

```python
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
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `pytest tests/test_robot_live_state.py -v`

Expected: 4 теста падают с `ImportError: cannot import name 'build_live_state_point'`.

- [ ] **Step 3: Реализовать `build_live_state_point` в `routers/robot.py`**

Добавить в конец файла `compute_node/dashboard/routers/robot.py` (после существующих endpoints):

```python
# ── /ws/robot/live_state ──────────────────────────────────────────────
# Постоянный канал текущего состояния робота для DashboardPage.
# Контракт: docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
import math as _math
from ..schemas.robot_live_state import (
    RobotLiveStatePoint,
    RobotLiveStatePose,
    RobotLiveStateVel,
    RobotLiveStateImu,
)


def build_live_state_point(state) -> RobotLiveStatePoint:
    """Snapshot DashboardState → RobotLiveStatePoint. Захватывает lock внутри.

    Чистая функция — юнит-тестируется без FastAPI окружения.
    Конвертирует:
      - pose.yaw (rad) → также yaw_deg для UI;
      - imu.gyro / accel (Vec3) → list[3];
      - imu_ekf_bias (rad/s) → ekf_bias_deg (°/s); None если EKF выключен.
    """
    with state.lock:
        p = state.robot.pose
        ve = state.robot.velocity_estimated
        imu = state.sensors.imu
        bias_rad = list(state.sensors.imu_ekf_bias)
        ts = state.robot.mqtt_odom_ts or time.time()
        stationary = state.robot.stationary
        has_ekf = imu.ekf is not None
        # imu.yaw/pitch/roll уже в °; gyro/accel — Vec3 → list
        ypr_deg = [imu.yaw, imu.pitch, imu.roll]
        gyro = [imu.gyro.x, imu.gyro.y, imu.gyro.z]
        accel = [imu.accel.x, imu.accel.y, imu.accel.z]
    return RobotLiveStatePoint(
        ts=ts,
        pose=RobotLiveStatePose(
            x=p.x, y=p.y, yaw_rad=p.yaw, yaw_deg=_math.degrees(p.yaw),
        ),
        vel=RobotLiveStateVel(linear=ve.linear_x, angular=ve.angular_z),
        imu=RobotLiveStateImu(
            ypr_deg=ypr_deg,
            gyro=gyro,
            accel=accel,
            ekf_bias_deg=[_math.degrees(b) for b in bias_rad] if has_ekf else None,
            has_ekf=has_ekf,
        ),
        stationary=stationary,
    )
```

- [ ] **Step 4: Запустить тесты — должны проходить**

Run: `pytest tests/test_robot_live_state.py -v -k build_live_state_point`

Expected: 4 теста проходят.

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/routers/robot.py tests/test_robot_live_state.py
git commit -m "feat(dashboard): build_live_state_point aggregator for robot WS"
```

---

### Task 3: `_RobotLiveStateBroker` (fan-out + last buffer)

Зеркало `_MpsLiveStateBroker` из `routers/mps.py` (см. lines 571-624).

**Files:**
- Modify: `compute_node/dashboard/routers/robot.py`
- Modify: `tests/test_robot_live_state.py`

- [ ] **Step 1: Написать тесты для broker**

Добавить в `tests/test_robot_live_state.py`:

```python
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
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `pytest tests/test_robot_live_state.py::test_broker_add_remove_subscriber -v`

Expected: `ImportError: cannot import name '_RobotLiveStateBroker'`.

- [ ] **Step 3: Реализовать `_RobotLiveStateBroker`**

Добавить в `compute_node/dashboard/routers/robot.py` ПЕРЕД `def build_live_state_point` (или в новый блок ниже `build_live_state_point` — порядок не важен, главное один файл):

```python
import asyncio
from threading import Lock as _Lock
from typing import Optional


def _safe_put_nowait(queue: asyncio.Queue, frame: dict) -> None:
    """Не падать при переполнении очереди — лучше потерять frame чем убить WS."""
    try:
        queue.put_nowait(frame)
    except asyncio.QueueFull:
        pass


class _RobotLiveStateBroker:
    """Fan-out + буфер последнего фрейма для replay новым подписчикам.

    Зеркало _MpsLiveStateBroker (routers/mps.py:571). Отделено от MPS,
    т.к. это другой канал данных (текущее состояние робота, а не МПС).
    """

    def __init__(self) -> None:
        self._subs: list[asyncio.Queue] = []
        self._lock = _Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._last: Optional[dict] = None

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def ensure_loop(self) -> None:
        """Idempotent: запомнить running loop, если ещё не сохранён."""
        if self._loop is None:
            self._loop = asyncio.get_event_loop()

    def add(self, queue: asyncio.Queue) -> None:
        with self._lock:
            self._subs.append(queue)

    def remove(self, queue: asyncio.Queue) -> None:
        with self._lock:
            self._subs = [q for q in self._subs if q is not queue]

    def has_subscribers(self) -> bool:
        """True если хотя бы один WS-клиент подключён (для skip в idle loop)."""
        with self._lock:
            return bool(self._subs)

    def set_last(self, frame: Optional[dict]) -> None:
        with self._lock:
            self._last = frame

    def get_last(self) -> Optional[dict]:
        with self._lock:
            return self._last

    def broadcast(self, frame: dict) -> None:
        """Called from aggregator-task or test. Persists last + fan-out."""
        loop = self._loop
        with self._lock:
            self._last = frame
            targets = list(self._subs)
        if not targets:
            return
        for q in targets:
            if loop is None or loop.is_closed():
                _safe_put_nowait(q, frame)
            else:
                loop.call_soon_threadsafe(_safe_put_nowait, q, frame)


robot_live_state_broker = _RobotLiveStateBroker()
```

- [ ] **Step 4: Запустить тесты — должны проходить**

Run: `pytest tests/test_robot_live_state.py -v -k broker`

Expected: 3 теста проходят.

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/routers/robot.py tests/test_robot_live_state.py
git commit -m "feat(dashboard): _RobotLiveStateBroker fan-out for /ws/robot/live_state"
```

---

## Phase B — WS endpoint + aggregator loop

### Task 4: WS endpoint `/ws/robot/live_state`

**Files:**
- Modify: `compute_node/dashboard/routers/robot.py`
- Modify: `tests/test_robot_live_state.py`

- [ ] **Step 1: Написать тесты для WS**

Добавить в `tests/test_robot_live_state.py`:

```python
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
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `pytest tests/test_robot_live_state.py -v -k ws_`

Expected: 3 теста падают на `client.websocket_connect('/ws/robot/live_state')` —
404 / connection rejected.

- [ ] **Step 3: Реализовать WS-endpoint**

Добавить в `compute_node/dashboard/routers/robot.py` после `robot_live_state_broker = ...`:

```python
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

ws_router = APIRouter()


@ws_router.websocket('/ws/robot/live_state')
async def robot_live_state_ws(websocket: WebSocket):
    """Без handshake. На connect — replay последнего фрейма (если есть),
    затем стрим из broker. При разрыве — клиент сам переподключается
    (см. useRobotLiveState).
    """
    await websocket.accept()
    robot_live_state_broker.ensure_loop()

    queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=64)
    robot_live_state_broker.add(queue)

    last = robot_live_state_broker.get_last()
    if last is not None:
        try:
            # last уже в обёртке {'type': 'live_state', 'point': ...} —
            # см. _RobotLiveStateBroker.broadcast() ниже.
            await websocket.send_json(
                last if last.get('type') == 'live_state'
                else {'type': 'live_state', 'point': last}
            )
        except Exception:
            pass

    try:
        while True:
            frame = await queue.get()
            await websocket.send_json(frame)
    except WebSocketDisconnect:
        pass
    finally:
        robot_live_state_broker.remove(queue)
        try:
            await websocket.close()
        except Exception:
            pass
```

- [ ] **Step 4: Зарегистрировать `ws_router` в `create_app`**

В `compute_node/dashboard/app.py`, после строки `app.include_router(mps.ws_router)`
(вокруг line 489) — добавить:

```python
    # /ws/robot/live_state — постоянный канал состояния робота для DashboardPage.
    app.include_router(robot.ws_router)
```

- [ ] **Step 5: Запустить тесты — должны проходить**

Run: `pytest tests/test_robot_live_state.py -v -k ws_`

Expected: 3 теста проходят.

- [ ] **Step 6: Полный прогон файла**

Run: `pytest tests/test_robot_live_state.py -v`

Expected: 10 тестов из 10 проходят (4 aggregator + 3 broker + 3 WS).

- [ ] **Step 7: Commit**

```bash
git add compute_node/dashboard/routers/robot.py compute_node/dashboard/app.py tests/test_robot_live_state.py
git commit -m "feat(dashboard): /ws/robot/live_state endpoint with broker fan-out"
```

---

### Task 5: Aggregator loop в `app.py`

**Files:**
- Modify: `compute_node/dashboard/app.py`
- Modify: `tests/test_robot_live_state.py`

- [ ] **Step 1: Написать тест что loop публикует во broker**

Добавить в `tests/test_robot_live_state.py`:

```python
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
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `pytest tests/test_robot_live_state.py -v -k loop`

Expected: 2 теста падают на `ImportError: cannot import name '_run_robot_live_state_tick'`.

- [ ] **Step 3: Добавить module-level `_run_robot_live_state_tick`**

В `compute_node/dashboard/app.py`, после блока импортов (до `_HAS_SENTRY = True/False` около line 49 или сразу после), добавить функцию НА УРОВНЕ МОДУЛЯ (не внутри `create_app`):

```python
async def _run_robot_live_state_tick(_state) -> None:
    """Один тик aggregator-loop'а. Вынесено на уровень модуля для
    юнит-тестируемости.

    Если нет подписчиков на /ws/robot/live_state — НЕ строим point и
    не зовём broadcast (избавляемся от лишнего lock + сериализации).
    """
    from .routers.robot import (
        build_live_state_point,
        robot_live_state_broker,
    )
    if not robot_live_state_broker.has_subscribers():
        return
    try:
        point = build_live_state_point(_state).model_dump()
    except Exception as exc:
        logging.getLogger('dashboard').exception(
            'robot live_state aggregator failed: %s', exc)
        return
    robot_live_state_broker.broadcast({'type': 'live_state', 'point': point})
```

- [ ] **Step 4: Добавить background-task внутри `create_app`**

В `create_app`, СРАЗУ ПОСЛЕ блока `_start_push_loop` (после `asyncio.create_task(_push_loop())` ~line 608), добавить:

```python
    async def _robot_live_state_loop():
        """10 Hz aggregator → /ws/robot/live_state.

        Отдельный loop от _push_loop потому что:
          - всегда тикает (нет dirty-skip);
          - изоляция: если SocketIO-broadcast тормозит, WS-канал не страдает.
        """
        while True:
            await asyncio.sleep(0.1)               # 10 Hz
            await _run_robot_live_state_tick(state)

    @app.on_event('startup')
    async def _start_robot_live_state_loop():
        from .routers.robot import robot_live_state_broker
        robot_live_state_broker.attach_loop(asyncio.get_running_loop())
        asyncio.create_task(_robot_live_state_loop())
```

- [ ] **Step 5: Запустить тесты — должны проходить**

Run: `pytest tests/test_robot_live_state.py -v -k loop`

Expected: 2 теста проходят.

- [ ] **Step 6: Полный прогон тестов бэка**

Run: `pytest tests/test_robot_live_state.py -v`

Expected: 12 тестов из 12 проходят.

- [ ] **Step 7: Smoke-тест что весь модуль импортируется**

Run: `python -c "from compute_node.dashboard.app import create_app, _run_robot_live_state_tick; print('ok')"`

Expected: `ok`.

- [ ] **Step 8: Commit**

```bash
git add compute_node/dashboard/app.py tests/test_robot_live_state.py
git commit -m "feat(dashboard): 10 Hz aggregator loop for /ws/robot/live_state"
```

---

## Phase C — Frontend types + hook

### Task 6: TS-типы в `robot.ts`

**Files:**
- Modify: `compute_node/frontend/src/types/robot.ts`

- [ ] **Step 1: Добавить типы**

В `compute_node/frontend/src/types/robot.ts`, после блока `OdometrySources` /
`RobotPoseState` (около line 175), добавить:

```ts
// ── /ws/robot/live_state ──────────────────────────────────────────────
// Постоянный поток текущего состояния робота (pose + velocity + IMU) для
// панели на DashboardPage. Контракт: docs/superpowers/specs/
// 2026-05-19-robot-live-state-vector-design.md §2.
export const ROBOT_LIVE_STATE_SCHEMA = '1.0' as const

export interface RobotLiveStatePoint {
  /** Pi-clock unix-секунды */
  ts: number
  pose: {
    /** м, world frame */
    x: number
    y: number
    /** радианы */
    yaw_rad: number
    /** градусы (для удобства UI) */
    yaw_deg: number
  }
  vel: {
    /** м/с */
    linear: number
    /** рад/с */
    angular: number
  }
  imu: {
    /** [yaw, pitch, roll] в °, активный (EKF или raw-fallback) */
    ypr_deg: [number, number, number]
    /** [x, y, z] рад/с */
    gyro: [number, number, number]
    /** [x, y, z] м/с² */
    accel: [number, number, number]
    /** [x, y, z] °/с; null если has_ekf=false */
    ekf_bias_deg: [number, number, number] | null
    has_ekf: boolean
  }
  /** ZUPT (IMU-based stationary detector) */
  stationary: boolean
  schema_version: string
}

export interface RobotLiveStateWsFrame {
  type: 'live_state'
  point: RobotLiveStatePoint
}
```

- [ ] **Step 2: Verify типы компилируются**

Run: `cd compute_node/frontend && npx tsc --noEmit`

Expected: zero errors (или существующие ошибки, не относящиеся к этому файлу).

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/types/robot.ts
git commit -m "feat(frontend): TS types for /ws/robot/live_state contract"
```

---

### Task 7: Hook `useRobotLiveState` + тесты

Зеркало `useMpsLiveState`.

**Files:**
- Create: `compute_node/frontend/src/hooks/useRobotLiveState.ts`
- Create: `compute_node/frontend/src/hooks/useRobotLiveState.test.tsx`

- [ ] **Step 1: Написать failing test**

`compute_node/frontend/src/hooks/useRobotLiveState.test.tsx`:

```tsx
/**
 * useRobotLiveState — WS-хук для постоянного состояния робота.
 * Зеркало useMpsLiveState.test.tsx (см. там для деталей FakeWebSocket).
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useRobotLiveState } from './useRobotLiveState'

interface FakeWs {
  url: string
  readyState: number
  onopen: ((ev: unknown) => void) | null
  onmessage: ((ev: { data: string }) => void) | null
  onerror: ((ev: unknown) => void) | null
  onclose: ((ev: unknown) => void) | null
  send(data: string): void
  close(): void
}

const fakes: FakeWs[] = []

class FakeWebSocket implements FakeWs {
  url: string
  readyState = 0
  onopen: ((ev: unknown) => void) | null = null
  onmessage: ((ev: { data: string }) => void) | null = null
  onerror: ((ev: unknown) => void) | null = null
  onclose: ((ev: unknown) => void) | null = null

  constructor(url: string) {
    this.url = url
    fakes.push(this)
  }
  send(_data: string) {}
  close() {
    this.readyState = 3
    this.onclose?.({})
  }
  fireOpen() {
    this.readyState = 1
    this.onopen?.({})
  }
  fireMessage(payload: unknown) {
    this.onmessage?.({ data: JSON.stringify(payload) })
  }
}

beforeEach(() => {
  fakes.length = 0
  // @ts-expect-error override global
  globalThis.WebSocket = FakeWebSocket
})

afterEach(() => {
  vi.useRealTimers()
  vi.restoreAllMocks()
})

const FRAME = {
  type: 'live_state' as const,
  point: {
    ts: 1747574400,
    pose: { x: 0.5, y: 1.2, yaw_rad: 0.1, yaw_deg: 5.7 },
    vel: { linear: 0.12, angular: 0.05 },
    imu: {
      ypr_deg: [5.7, 0.5, 0.0] as [number, number, number],
      gyro: [0.01, 0.02, 0.21] as [number, number, number],
      accel: [0.05, 0.02, 9.81] as [number, number, number],
      ekf_bias_deg: [0.06, -0.11, 0.17] as [number, number, number],
      has_ekf: true,
    },
    stationary: false,
    schema_version: '1.0',
  },
}

describe('useRobotLiveState', () => {
  it('подключается к /ws/robot/live_state при mount', () => {
    renderHook(() => useRobotLiveState())
    expect(fakes.length).toBe(1)
    expect(fakes[0].url).toMatch(/\/ws\/robot\/live_state$/)
  })

  it('обновляет point на frame', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(result.current.connected).toBe(true)
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point?.pose.x).toBeCloseTo(0.5)
    expect(result.current.point?.imu.has_ekf).toBe(true)
  })

  it('игнорирует frames с type !== "live_state"', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage({ type: 'other', point: {} }))
    expect(result.current.point).toBeNull()
  })

  it('после onclose connected=false, point сохраняется', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    act(() => ws.close())
    expect(result.current.connected).toBe(false)
    expect(result.current.point?.pose.x).toBeCloseTo(0.5)
  })

  it('после >2s без frames — stale=true', () => {
    vi.useFakeTimers()
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.stale).toBe(false)
    act(() => {
      vi.advanceTimersByTime(2500)
    })
    expect(result.current.stale).toBe(true)
    expect(result.current.ageMs).toBeGreaterThanOrEqual(2000)
  })
})
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `cd compute_node/frontend && npx vitest run src/hooks/useRobotLiveState.test.tsx`

Expected: ошибки на import `./useRobotLiveState`.

- [ ] **Step 3: Реализовать хук**

`compute_node/frontend/src/hooks/useRobotLiveState.ts`:

```ts
import { useEffect, useRef, useState } from 'react'
import type { RobotLiveStatePoint, RobotLiveStateWsFrame } from '@/types/robot'

const STALE_THRESHOLD_MS = 2_000
const RECONNECT_DELAYS_MS = [1_000, 2_000, 4_000, 8_000, 10_000] as const

export interface UseRobotLiveStateResult {
  /** Последний полученный фрейм или null если ничего ещё не пришло. */
  point: RobotLiveStatePoint | null
  /** true если WS-сокет открыт. */
  connected: boolean
  /** true если >2s без новых фреймов (даже если WS открыт). */
  stale: boolean
  /** ms с момента последнего фрейма; null если фреймов ещё не было. */
  ageMs: number | null
}

/**
 * Постоянный WS-канал /ws/robot/live_state. Без enabled/id-фильтра —
 * клиент подключается всегда; auto-reconnect с экспоненциальным backoff.
 *
 * Контракт: docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
 *
 * Структурно зеркалит useMpsLiveState — единая модель ментально для двух
 * каналов состояния (МПС-сценарий vs основной робот).
 */
export function useRobotLiveState(): UseRobotLiveStateResult {
  const [point, setPoint] = useState<RobotLiveStatePoint | null>(null)
  const [connected, setConnected] = useState(false)
  const [ageMs, setAgeMs] = useState<number | null>(null)

  const lastReceivedAtRef = useRef<number | null>(null)
  const reconnectAttemptRef = useRef(0)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  useEffect(() => {
    let cancelled = false

    function connect() {
      if (cancelled) return
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const url = `${protocol}//${window.location.host}/ws/robot/live_state`
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        if (cancelled) return
        setConnected(true)
        reconnectAttemptRef.current = 0
      }

      ws.onmessage = (ev: MessageEvent) => {
        if (cancelled) return
        let frame: RobotLiveStateWsFrame
        try {
          frame = JSON.parse(ev.data) as RobotLiveStateWsFrame
        } catch {
          return
        }
        if (frame.type !== 'live_state') return
        lastReceivedAtRef.current = Date.now()
        setPoint(frame.point)
      }

      ws.onerror = () => {
        if (cancelled) return
        setConnected(false)
      }

      ws.onclose = () => {
        if (cancelled) return
        setConnected(false)
        const idx = Math.min(
          reconnectAttemptRef.current,
          RECONNECT_DELAYS_MS.length - 1,
        )
        const delay = RECONNECT_DELAYS_MS[idx]
        reconnectAttemptRef.current += 1
        reconnectTimerRef.current = setTimeout(connect, delay)
      }
    }

    connect()

    const tick = setInterval(() => {
      if (lastReceivedAtRef.current === null) {
        setAgeMs(null)
      } else {
        setAgeMs(Date.now() - lastReceivedAtRef.current)
      }
    }, 500)

    return () => {
      cancelled = true
      clearInterval(tick)
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current)
      reconnectAttemptRef.current = 0
      try {
        wsRef.current?.close()
      } catch {
        /* ignore */
      }
      wsRef.current = null
    }
  }, [])

  const stale = ageMs !== null && ageMs > STALE_THRESHOLD_MS

  return { point, connected, stale, ageMs }
}
```

- [ ] **Step 4: Запустить тесты — должны проходить**

Run: `cd compute_node/frontend && npx vitest run src/hooks/useRobotLiveState.test.tsx`

Expected: 5 тестов проходят.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/hooks/useRobotLiveState.ts compute_node/frontend/src/hooks/useRobotLiveState.test.tsx
git commit -m "feat(frontend): useRobotLiveState hook for /ws/robot/live_state"
```

---

## Phase D — Frontend component

### Task 8: Компонент `RobotStateVector` + тесты

**Files:**
- Create: `compute_node/frontend/src/components/sensors/RobotStateVector.tsx`
- Create: `compute_node/frontend/src/components/sensors/RobotStateVector.test.tsx`

- [ ] **Step 1: Написать failing tests**

`compute_node/frontend/src/components/sensors/RobotStateVector.test.tsx`:

```tsx
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen } from '@testing-library/react'
import { RobotStateVector } from './RobotStateVector'
import type { RobotLiveStatePoint } from '@/types/robot'
import type { UseRobotLiveStateResult } from '@/hooks/useRobotLiveState'

vi.mock('@/hooks/useRobotLiveState')
import { useRobotLiveState } from '@/hooks/useRobotLiveState'
const mockHook = vi.mocked(useRobotLiveState)

const POINT: RobotLiveStatePoint = {
  ts: 1747574400,
  pose: { x: 0.234, y: 0.118, yaw_rad: -0.047, yaw_deg: -2.7 },
  vel: { linear: 0.118, angular: 0.215 },
  imu: {
    ypr_deg: [-2.7, 0.5, 0.0],
    gyro: [0.010, 0.020, 0.210],
    accel: [0.050, 0.020, 9.810],
    ekf_bias_deg: [0.06, -0.11, 0.17],
    has_ekf: true,
  },
  stationary: false,
  schema_version: '1.0',
}

function setHook(partial: Partial<UseRobotLiveStateResult>): void {
  mockHook.mockReturnValue({
    point: null,
    connected: false,
    stale: false,
    ageMs: null,
    ...partial,
  })
}

beforeEach(() => {
  mockHook.mockReset()
})

describe('RobotStateVector', () => {
  it('renders placeholder when point=null', () => {
    setHook({ point: null, connected: false })
    render(<RobotStateVector />)
    expect(screen.getByText('Состояние робота')).toBeInTheDocument()
    // Все числовые поля — «—»
    expect(screen.getAllByText('—').length).toBeGreaterThan(3)
    expect(screen.getByText(/disconnected/i)).toBeInTheDocument()
  })

  it('renders live frame with formatted values', () => {
    setHook({ point: POINT, connected: true, stale: false, ageMs: 100 })
    render(<RobotStateVector />)
    expect(screen.getByText(/\+0\.234/)).toBeInTheDocument()  // x
    expect(screen.getByText(/\+0\.118/)).toBeInTheDocument()  // y (или vel.linear)
    expect(screen.getByText(/−2\.7°/)).toBeInTheDocument()    // yaw_deg
    expect(screen.getByText(/EKF активен/)).toBeInTheDocument()
    expect(screen.getByText(/● live/)).toBeInTheDocument()
  })

  it('shows stale badge when stale=true', () => {
    setHook({ point: POINT, connected: true, stale: true, ageMs: 5300 })
    render(<RobotStateVector />)
    expect(screen.getByText(/stale/i)).toBeInTheDocument()
  })

  it('orientation header changes when has_ekf=false', () => {
    const noEkf: RobotLiveStatePoint = {
      ...POINT,
      imu: { ...POINT.imu, has_ekf: false, ekf_bias_deg: null },
    }
    setHook({ point: noEkf, connected: true })
    render(<RobotStateVector />)
    expect(screen.getByText(/raw fallback/i)).toBeInTheDocument()
  })

  it('bias row shows "—" when ekf_bias_deg=null', () => {
    const noBias: RobotLiveStatePoint = {
      ...POINT,
      imu: { ...POINT.imu, has_ekf: false, ekf_bias_deg: null },
    }
    setHook({ point: noBias, connected: true })
    render(<RobotStateVector />)
    // В секции bias должны быть прочерки.
    const biasRow = screen.getByText(/bias/i).parentElement
    expect(biasRow?.textContent).toMatch(/—/)
  })

  it('stationary=true shows STATIONARY label', () => {
    const stationary: RobotLiveStatePoint = { ...POINT, stationary: true }
    setHook({ point: stationary, connected: true })
    render(<RobotStateVector />)
    expect(screen.getByText(/STATIONARY/)).toBeInTheDocument()
  })
})
```

- [ ] **Step 2: Запустить тесты — должны падать**

Run: `cd compute_node/frontend && npx vitest run src/components/sensors/RobotStateVector.test.tsx`

Expected: ошибки на import `./RobotStateVector`.

- [ ] **Step 3: Реализовать компонент**

`compute_node/frontend/src/components/sensors/RobotStateVector.tsx`:

```tsx
import { memo } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { useRobotLiveState } from '@/hooks/useRobotLiveState'

/**
 * RobotStateVector — постоянная панель «полное состояние робота» для
 * DashboardPage. Подписывается на /ws/robot/live_state (10 Hz) и
 * показывает pose, velocity, IMU YPR + сырые гироскоп/акселерометр + bias.
 *
 * Контракт: docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
 *
 * Не требует пропсов — самодостаточен. Не пересекается с глобальным
 * useRobotStore, чтобы не зависеть от частоты SocketIO state_update и
 * не «пузыриться» по re-render'ам не относящихся слайсов state.
 */
export const RobotStateVector = memo(function RobotStateVector() {
  const { point, connected, stale, ageMs } = useRobotLiveState()

  const fmt = (v: number | null | undefined, digits = 3): string => {
    if (v === null || v === undefined || Number.isNaN(v)) return '—'
    const sign = v >= 0 ? '+' : '−'
    return `${sign}${Math.abs(v).toFixed(digits)}`
  }
  const fmtDeg = (v: number | null | undefined): string => {
    if (v === null || v === undefined || Number.isNaN(v)) return '—'
    const sign = v >= 0 ? '+' : '−'
    return `${sign}${Math.abs(v).toFixed(1)}°`
  }
  const fmtAge = (ms: number | null): string => {
    if (ms === null) return ''
    if (ms < 1000) return `${(ms / 1000).toFixed(1)}s`
    return `${Math.round(ms / 1000)}s`
  }

  const badge = (() => {
    if (!connected) return { label: 'disconnected', cls: 'text-foreground-faint' }
    if (stale) return { label: `stale · ${fmtAge(ageMs)}`, cls: 'text-amber-400' }
    return { label: '● live', cls: 'text-emerald-400' }
  })()

  const hasEkf = point?.imu.has_ekf ?? false
  const orientationHeader = hasEkf
    ? 'IMU — ориентация (EKF активен)'
    : 'IMU — ориентация (raw fallback)'

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-foreground-muted font-semibold flex items-center justify-between">
          <span>Состояние робота</span>
          <span className={`text-[10px] normal-case tracking-normal ${badge.cls}`}>
            {badge.label}
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 px-3 pb-3 font-mono tabular-nums text-[11px]">
        {/* Pose */}
        <Section title="Поза (world)">
          <Row label="x" value={`${fmt(point?.pose.x)} м`} />
          <Row label="y" value={`${fmt(point?.pose.y)} м`} />
          <Row
            label="θ"
            value={
              point
                ? `${fmtDeg(point.pose.yaw_deg)} (${fmt(point.pose.yaw_rad, 4)} рад)`
                : '—'
            }
          />
        </Section>

        {/* Velocity */}
        <Section title="Скорости">
          <Row label="v" value={`${fmt(point?.vel.linear)} м/с`} />
          <Row
            label="ω"
            value={
              point
                ? `${fmtDeg((point.vel.angular * 180) / Math.PI)}/с (${fmt(
                    point.vel.angular,
                  )} рад/с)`
                : '—'
            }
          />
        </Section>

        {/* IMU orientation */}
        <Section title={orientationHeader}>
          <div className="grid grid-cols-3 gap-2">
            <Cell label="Y" value={fmtDeg(point?.imu.ypr_deg[0])} />
            <Cell label="P" value={fmtDeg(point?.imu.ypr_deg[1])} />
            <Cell label="R" value={fmtDeg(point?.imu.ypr_deg[2])} />
          </div>
        </Section>

        {/* IMU raw */}
        <Section title="IMU — сырые">
          <Row
            label="gyro"
            value={
              point
                ? `[${fmt(point.imu.gyro[0])}, ${fmt(point.imu.gyro[1])}, ${fmt(point.imu.gyro[2])}] рад/с`
                : '—'
            }
          />
          <Row
            label="accel"
            value={
              point
                ? `[${fmt(point.imu.accel[0])}, ${fmt(point.imu.accel[1])}, ${fmt(point.imu.accel[2])}] м/с²`
                : '—'
            }
          />
          <Row
            label="bias"
            value={
              point?.imu.ekf_bias_deg
                ? `[${fmtDeg(point.imu.ekf_bias_deg[0])}, ${fmtDeg(point.imu.ekf_bias_deg[1])}, ${fmtDeg(point.imu.ekf_bias_deg[2])}]/с`
                : '—'
            }
          />
        </Section>

        {/* ZUPT + age */}
        <div className="flex items-center justify-between text-[10px] text-foreground-muted border-t border-subtle pt-2">
          <span>
            ZUPT:{' '}
            {point ? (
              point.stationary ? (
                <span className="text-emerald-400">STATIONARY</span>
              ) : (
                <span>moving</span>
              )
            ) : (
              '—'
            )}
          </span>
          <span>
            {ageMs === null ? '' : `обновлено ${fmtAge(ageMs)} назад`}
          </span>
        </div>
      </CardContent>
    </Card>
  )
})


function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="space-y-1">
      <div className="text-[10px] uppercase tracking-wider text-foreground-muted font-sans">
        {title}
      </div>
      <div className="space-y-0.5">{children}</div>
    </div>
  )
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between">
      <span className="text-foreground-faint">{label}</span>
      <span>{value}</span>
    </div>
  )
}

function Cell({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex flex-col">
      <span className="text-[9px] uppercase tracking-wider text-foreground-faint">
        {label}
      </span>
      <span>{value}</span>
    </div>
  )
}
```

- [ ] **Step 4: Запустить тесты — должны проходить**

Run: `cd compute_node/frontend && npx vitest run src/components/sensors/RobotStateVector.test.tsx`

Expected: 6 тестов проходят.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/sensors/RobotStateVector.tsx compute_node/frontend/src/components/sensors/RobotStateVector.test.tsx
git commit -m "feat(frontend): RobotStateVector panel with pose+vel+IMU display"
```

---

### Task 9: Размещение на DashboardPage

**Files:**
- Modify: `compute_node/frontend/src/pages/DashboardPage.tsx`

- [ ] **Step 1: Найти, где размещён `OdometryComparePanel`**

Run: `grep -n "OdometryComparePanel" compute_node/frontend/src/pages/DashboardPage.tsx`

Запомнить line number.

- [ ] **Step 2: Добавить import**

В верхней части `compute_node/frontend/src/pages/DashboardPage.tsx`, рядом с
`OdometryComparePanel` import:

```tsx
import { RobotStateVector } from '@/components/sensors/RobotStateVector'
```

- [ ] **Step 3: Вставить компонент над `OdometryComparePanel`**

В JSX, найти строку с `<OdometryComparePanel ...` и добавить ПЕРЕД ней:

```tsx
<RobotStateVector />
<OdometryComparePanel sources={...} pose={...} />   {/* существующая строка */}
```

- [ ] **Step 4: Запустить TypeScript check**

Run: `cd compute_node/frontend && npx tsc --noEmit`

Expected: zero errors.

- [ ] **Step 5: Запустить полный test suite фронта**

Run: `cd compute_node/frontend && npx vitest run`

Expected: все тесты проходят, новые 11 (5 hook + 6 panel) включены.

- [ ] **Step 6: Commit**

```bash
git add compute_node/frontend/src/pages/DashboardPage.tsx
git commit -m "feat(frontend): mount RobotStateVector on DashboardPage"
```

---

## Phase E — Sanity + finalisation

### Task 10: Manual sanity на симуляторе

- [ ] **Step 1: Запустить sim + dashboard**

В двух терминалах:

```bash
# T1
./samurai.sh sim
```

```bash
# T2 (если не auto-launched sim'ом)
./samurai.sh compute
```

- [ ] **Step 2: Открыть DashboardPage**

Открыть `http://localhost:5000/` в браузере.

- [ ] **Step 3: Проверить визуально**

Чеклист:
- [ ] Карточка «Состояние робота» видна над `OdometryComparePanel`.
- [ ] Бейдж `● live` (зелёный).
- [ ] `pose.x`, `pose.y`, `vel.linear`, `imu.ypr_deg` обновляются при
  движении робота в sim.
- [ ] Yaw меняется при повороте, угол согласован с MapCanvas.
- [ ] Заголовок «IMU — ориентация (EKF активен)» (предполагается дефолт EKF).
- [ ] gyro/accel — ненулевые числа.
- [ ] bias заполнен (3 значения).

- [ ] **Step 4: Тест stale-таймера**

В терминале T1 нажать Ctrl+C для остановки sim.

Через ~2 секунды:
- [ ] Бейдж становится `stale · Ns` (жёлтый).
- [ ] Числа замораживаются на последних значениях.

Перезапустить sim.
- [ ] Через ~1с бейдж возвращается в `● live` без перезагрузки страницы.

- [ ] **Step 5: Тест без EKF (опционально)**

Если есть возможность отключить EKF на Pi (в `config.yaml` `imu.ekf.enabled: false`):
- [ ] Заголовок секции «ориентация» → «(raw fallback)».
- [ ] Строка `bias` → `—`.

Если нет возможности — skip.

- [ ] **Step 6: Финальный коммит**

Если всё работает — никаких изменений не нужно. Если нашли мелкие
правки (форматирование, опечатки) — закоммитить:

```bash
git add -A
git commit -m "polish(frontend): minor RobotStateVector adjustments after sanity"
```

- [ ] **Step 7: Push в dev**

```bash
git checkout dev      # если ещё на main
git merge --no-ff main -m "merge: robot live state vector feature"
git push origin dev
```

(если работали прямо в `main` — обсудить с пользователем стратегию мержа)

---

## Summary

После всех 10 задач:

- **Бэкенд:** +1 Pydantic-схема, +1 ws_router, +1 aggregator loop, 12 pytest тестов.
- **Фронт:** +1 hook, +1 компонент, +TS-типы, 11 vitest тестов.
- **Файлы:** 6 новых, 4 модифицированных.
- **LoC:** ~600 строк production кода, ~400 строк тестов.
- **Никаких изменений** на Pi, в MQTT-топиках, в `config.yaml`, в Pi-side нодах.
