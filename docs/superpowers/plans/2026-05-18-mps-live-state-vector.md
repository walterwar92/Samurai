# MPS Live State Vector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Постоянный блок на `/mps`, показывающий `x ∈ ℝ⁵` и `u ∈ ℝ²` робота 10 Hz, независимо от того, запущен сценарий или нет.

**Architecture:** `mps_node` на Pi публикует `samurai/{robot_id}/mps/live_state` @ 10 Hz из своего `_x_meas`. `MQTTHandlers` в dashboard бриджит в новый WS `/ws/mps/live_state` через свой собственный fan-out broker (по образцу существующего `mps_broker`). Frontend хук `useMpsLiveState` всегда подключен; компонент `LiveStateVector` рендерится в левом sidebar над `OdeCard`.

**Tech Stack:** Python 3.11, paho-mqtt, FastAPI WebSockets, asyncio, React 18, TypeScript, vitest, pytest, Tailwind, shadcn/ui Card.

**Спека:** [`docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md`](../specs/2026-05-18-mps-live-state-vector-design.md)

---

## File Structure

### Создаются:
- `compute_node/frontend/src/hooks/useMpsLiveState.ts` — WS-хук, auto-reconnect, stale-таймер.
- `compute_node/frontend/src/hooks/useMpsLiveState.test.tsx` — vitest.
- `compute_node/frontend/src/components/mps/LiveStateVector.tsx` — UI-карточка.
- `compute_node/frontend/src/components/mps/LiveStateVector.test.tsx` — vitest.

### Модифицируются:
- `pi_nodes/nodes/mps_node.py` — `_last_u`, таймер 10 Hz, метод `_publish_live_state`.
- `compute_node/dashboard/mqtt_handlers.py` — handler `_h_mps_live_state`, setter `set_mps_live_state_broadcaster`, `_mps_live_state_broadcaster: Optional[Callable]`, регистрация в `_HANDLERS`, `'mps/live_state'` в `_MPS_TOPICS`.
- `compute_node/dashboard/routers/mps.py` — класс `_MpsLiveStateBroker`, singleton `mps_live_state_broker`, WS-эндпоинт `/ws/mps/live_state`.
- `compute_node/dashboard/app.py` — wire `mqtt.set_mps_live_state_broadcaster(mps.mps_live_state_broker.broadcast)`.
- `compute_node/frontend/src/types/mps.ts` — типы `MpsLiveStatePoint`, `MpsLiveStateWsFrame`, константа `MPS_LIVE_STATE_SCHEMA`.
- `compute_node/frontend/src/pages/MpsPage.tsx` — `<LiveStateVector />` в `<aside>`.
- `tests/test_mps_node.py` — 4 теста по `_publish_live_state` + `_last_u`.
- `tests/test_dashboard_mqtt_handlers.py` — 4 теста по `_h_mps_live_state`.
- `tests/test_mps_router.py` — 3 теста по `/ws/mps/live_state`.

---

## Task 1: Pi — обновлять `_last_u` в `_publish_cmd_and_telemetry`

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py`
- Test: `tests/test_mps_node.py`

Сначала фиксируем `_last_u` (нужен для p.2). Это самый изолированный кусок.

- [ ] **Step 1.1: Написать падающий тест**

Добавить в `tests/test_mps_node.py` в конец файла:

```python
# ── live_state: _last_u ────────────────────────────────────────────────
def test_last_u_updated_in_publish_cmd_and_telemetry(mps_node):
    """После публикации cmd_vel + telemetry, _last_u должен содержать u."""
    import numpy as np

    # `_RunState.__init__` требует ReferenceTrajectory; мокаем целиком —
    # `_publish_cmd_and_telemetry` использует только run_id, distance, t,
    # telemetry. См. pi_nodes/nodes/mps_node.py:74-126.
    run = MagicMock()
    run.run_id = 'r-test'
    run.distance = 1.0
    run.t = 0.0
    run.telemetry = []
    x = np.array([0.0, 0.1, 0.0, 0.0, 0.0])
    u = np.array([0.123, -0.456])
    mps_node._plant = MagicMock()
    mps_node._plant.output.return_value = x.copy()

    mps_node._publish_cmd_and_telemetry(run, x, u)

    assert mps_node._last_u[0] == pytest.approx(0.123)
    assert mps_node._last_u[1] == pytest.approx(-0.456)
```

- [ ] **Step 1.2: Запустить — должен упасть**

Run: `pytest tests/test_mps_node.py::test_last_u_updated_in_publish_cmd_and_telemetry -v`

Expected: FAIL (`AttributeError: 'MpsNode' object has no attribute '_last_u'`).

- [ ] **Step 1.3: Минимальная реализация**

В `pi_nodes/nodes/mps_node.py` найти `MpsNode.__init__`. Сразу после строки `self._x_meas = np.zeros(5)` добавить:

```python
        # Последнее опубликованное управление; используется
        # _publish_live_state когда сценарий активен.
        self._last_u = np.zeros(2)
```

В методе `_publish_cmd_and_telemetry` (около `pi_nodes/nodes/mps_node.py:722`), самым первым действием в теле:

```python
        self._last_u = u.copy()
```

- [ ] **Step 1.4: Запустить — должен пройти**

Run: `pytest tests/test_mps_node.py::test_last_u_updated_in_publish_cmd_and_telemetry -v`

Expected: PASS.

- [ ] **Step 1.5: Прогнать все mps_node тесты, не сломать существующее**

Run: `pytest tests/test_mps_node.py -v`

Expected: все existing тесты по-прежнему зелёные + новый зелёный.

- [ ] **Step 1.6: Коммит**

```bash
git add pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "feat(mps): mps_node — сохранять _last_u в _publish_cmd_and_telemetry"
```

---

## Task 2: Pi — `_publish_live_state` метод

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py`
- Test: `tests/test_mps_node.py`

- [ ] **Step 2.1: Написать падающие тесты**

Добавить в `tests/test_mps_node.py`:

```python
# ── live_state: publish ────────────────────────────────────────────────
def test_publish_live_state_idle(mps_node):
    """Без активного _run: scenario_active=False, e_int=0, u=[0,0]."""
    import numpy as np

    mps_node._x_meas = np.array([0.5, 0.12, 0.1, 0.0, 0.42])
    mps_node._last_u = np.array([0.2, -0.1])
    mps_node._run = None
    mps_node._published.clear()

    mps_node._publish_live_state()

    suffixes = [s for s, _, _ in mps_node._published]
    assert 'mps/live_state' in suffixes
    suffix, payload, qos = next(
        (s, p, q) for s, p, q in mps_node._published if s == 'mps/live_state'
    )
    assert payload['scenario_active'] is False
    assert payload['run_id'] is None
    assert payload['x'][0] == pytest.approx(0.5)
    assert payload['x'][1] == pytest.approx(0.12)
    assert payload['x'][2] == pytest.approx(0.1)
    assert payload['x'][3] == pytest.approx(0.0)
    assert payload['x'][4] == pytest.approx(0.0)  # e_int обнулён
    assert payload['u'] == [0.0, 0.0]
    assert payload['schema_version'] == '1.0'
    assert isinstance(payload['ts'], float) and payload['ts'] > 0
    assert qos == 0


def test_publish_live_state_active(mps_node):
    """С активным _run: scenario_active=True, e_int реальный, u=_last_u."""
    import numpy as np

    mps_node._x_meas = np.array([1.2, 0.18, 0.05, 0.01, 0.33])
    mps_node._last_u = np.array([0.18, 0.02])
    # Мок вместо _RunState — нужен только run_id.
    run_mock = MagicMock()
    run_mock.run_id = 'r-live-1'
    mps_node._run = run_mock
    mps_node._published.clear()

    mps_node._publish_live_state()

    payload = next(p for s, p, _ in mps_node._published if s == 'mps/live_state')
    assert payload['scenario_active'] is True
    assert payload['run_id'] == 'r-live-1'
    assert payload['x'][4] == pytest.approx(0.33)  # e_int сохранён
    assert payload['u'][0] == pytest.approx(0.18)
    assert payload['u'][1] == pytest.approx(0.02)


def test_publish_live_state_schema_version(mps_node):
    mps_node._run = None
    mps_node._published.clear()
    mps_node._publish_live_state()
    payload = next(p for s, p, _ in mps_node._published if s == 'mps/live_state')
    assert payload['schema_version'] == '1.0'
```

- [ ] **Step 2.2: Запустить — должны упасть**

Run: `pytest tests/test_mps_node.py -k live_state -v`

Expected: 3 FAIL с `AttributeError: 'MpsNode' object has no attribute '_publish_live_state'`.

- [ ] **Step 2.3: Реализовать метод**

В `pi_nodes/nodes/mps_node.py` добавить константу на уровне модуля рядом с другими (искать `_S, _V, _THETA, _OMEGA, _EINT = ...` на строке 63), сразу после неё:

```python
LIVE_STATE_RATE_HZ = 10.0
```

В `MpsNode.__init__`, после `self.create_timer(1.0, self._publish_status)` (около строки 230):

```python
        self.create_timer(1.0 / LIVE_STATE_RATE_HZ, self._publish_live_state)
```

В классе `MpsNode` добавить метод (рядом с `_publish_status`):

```python
    def _publish_live_state(self) -> None:
        """Публикует текущий вектор состояния x и управление u 10 Hz.

        Вне активного сценария: e_int=0, u=[0,0], scenario_active=False.
        Внутри сценария: x как есть из _x_meas, u=_last_u, run_id.
        Контракт см. docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.1.
        """
        x = self._x_meas.copy()
        run = self._run
        if run is None:
            x[_EINT] = 0.0
            u = [0.0, 0.0]
            scenario_active = False
            run_id = None
        else:
            u = self._last_u.tolist()
            scenario_active = True
            run_id = run.run_id
        self.publish('mps/live_state', {
            'ts': time.time(),
            'x': x.tolist(),
            'u': u,
            'scenario_active': scenario_active,
            'run_id': run_id,
            'schema_version': '1.0',
        }, qos=0)
```

- [ ] **Step 2.4: Запустить — должны пройти**

Run: `pytest tests/test_mps_node.py -k live_state -v`

Expected: 3 PASS.

- [ ] **Step 2.5: Прогнать весь файл mps_node**

Run: `pytest tests/test_mps_node.py -v`

Expected: всё зелёное.

- [ ] **Step 2.6: Коммит**

```bash
git add pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "feat(mps): mps_node — публиковать mps/live_state @ 10 Hz (x, u, scenario_active)"
```

---

## Task 3: Dashboard — handler `_h_mps_live_state`

**Files:**
- Modify: `compute_node/dashboard/mqtt_handlers.py`
- Test: `tests/test_dashboard_mqtt_handlers.py`

- [ ] **Step 3.1: Написать падающие тесты**

В конец `tests/test_dashboard_mqtt_handlers.py`:

```python
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
```

- [ ] **Step 3.2: Запустить — должны упасть**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -k mps_live_state -v`

Expected: 4 FAIL — `_h_mps_live_state` / `set_mps_live_state_broadcaster` / `_last_live_state` не существуют.

- [ ] **Step 3.3: Реализовать handler + setter + buffer**

В `compute_node/dashboard/mqtt_handlers.py`:

1. Добавить поля в `MQTTHandlers.__init__` (искать `self._lock = ...` и положить рядом, или после блока с существующими полями `_mps_*`). Поскольку `_mps_ws_broadcaster` определён как class-level (строка 642), сделаем то же для `_mps_live_state_broadcaster`. Для `_last_live_state` — instance, под lock; добавить в `__init__`:

   ```python
       # mps/live_state — последний фрейм для send-on-connect.
       self._last_live_state: Optional[dict] = None
   ```

2. Рядом с `_mps_ws_broadcaster` (строка ~642) добавить class-level:

   ```python
       # Hook для broadcast в WebSocket /ws/mps/live_state.
       _mps_live_state_broadcaster: Optional[Callable] = None
   ```

3. Рядом с `set_mps_ws_broadcaster` (строка ~644) добавить setter:

   ```python
       def set_mps_live_state_broadcaster(self, broadcaster: Optional[Callable]) -> None:
           """Plumbing: app.py регистрирует функцию `broadcast(frame: dict)`."""
           self._mps_live_state_broadcaster = broadcaster
   ```

4. После `_broadcast_mps` (около строки 655) добавить handler:

   ```python
       def _h_mps_live_state(self, payload: bytes):
           """Обновляет _last_live_state и шлёт фрейм в /ws/mps/live_state.

           Контракт: docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.1.
           Невалидные payload'ы (некорректные длины x/u, отсутствующие ключи)
           молча игнорируются — это live-канал, дроп лучше падения.
           """
           try:
               d = json.loads(payload)
           except Exception:
               return
           if not isinstance(d, dict):
               return
           try:
               x = d['x']
               u = d['u']
           except KeyError:
               return
           if not (isinstance(x, list) and len(x) == 5
                   and isinstance(u, list) and len(u) == 2):
               return
           with self._state.lock:
               self._last_live_state = d
           cb = self._mps_live_state_broadcaster
           if cb is not None:
               try:
                   cb({'type': 'live_state', 'point': d})
               except Exception as exc:
                   log.warning('mps live_state WS broadcast failed: %s', exc)
   ```

5. Зарегистрировать в маппинге `_HANDLERS` (в конце файла, рядом с `'mps/telemetry': MQTTHandlers._h_mps_telemetry`):

   ```python
       'mps/live_state': MQTTHandlers._h_mps_live_state,
   ```

- [ ] **Step 3.4: Запустить — должны пройти**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -k mps_live_state -v`

Expected: 4 PASS.

- [ ] **Step 3.5: Прогнать весь файл**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v`

Expected: всё зелёное.

- [ ] **Step 3.6: Коммит**

```bash
git add compute_node/dashboard/mqtt_handlers.py tests/test_dashboard_mqtt_handlers.py
git commit -m "feat(mps): dashboard handler для mps/live_state с буфером и broadcaster-hook"
```

---

## Task 4: Dashboard — broker `_MpsLiveStateBroker` + WS endpoint

**Files:**
- Modify: `compute_node/dashboard/routers/mps.py`
- Test: `tests/test_mps_router.py`

- [ ] **Step 4.1: Написать падающий тест**

В конец `tests/test_mps_router.py` (после существующего теста для `/ws/mps/telemetry`):

```python
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
```

- [ ] **Step 4.2: Запустить — должны упасть**

Run: `pytest tests/test_mps_router.py -k live_state -v`

Expected: 3 FAIL (`mps_live_state_broker` не существует / WS endpoint 404).

- [ ] **Step 4.3: Реализовать broker и endpoint**

В `compute_node/dashboard/routers/mps.py`, после определения `mps_broker = _MpsWsBroker()` (около строки 523), добавить:

```python
# ─────────────────────────────────────────────────────────────────────
# WebSocket /ws/mps/live_state — постоянный поток вектора состояния
# x ∈ ℝ⁵ и управления u ∈ ℝ² (10 Hz), независимо от прогона сценария.
# Контракт: docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.2.
# ─────────────────────────────────────────────────────────────────────


class _MpsLiveStateBroker:
    """Fan-out без run_id-фильтра + буфер последнего фрейма для replay."""

    def __init__(self) -> None:
        self._subs: list[asyncio.Queue] = []
        self._lock = _Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._last: Optional[dict] = None

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def add(self, queue: asyncio.Queue) -> None:
        with self._lock:
            self._subs.append(queue)

    def remove(self, queue: asyncio.Queue) -> None:
        with self._lock:
            self._subs = [q for q in self._subs if q is not queue]

    def set_last(self, frame: Optional[dict]) -> None:
        """Setter для тестов и для handler-side persist."""
        with self._lock:
            self._last = frame

    def get_last(self) -> Optional[dict]:
        with self._lock:
            return self._last

    def broadcast(self, frame: dict) -> None:
        """Called from MQTT thread or test thread. Persists last + pushes."""
        loop = self._loop
        with self._lock:
            self._last = frame
            targets = list(self._subs)
        if not targets:
            return
        for q in targets:
            if loop is None:
                # Тестовый путь: client.websocket_connect использует тот же
                # loop как и сервер — пушим напрямую.
                _safe_put_nowait(q, frame)
            else:
                loop.call_soon_threadsafe(_safe_put_nowait, q, frame)


mps_live_state_broker = _MpsLiveStateBroker()


@ws_router.websocket('/ws/mps/live_state')
async def mps_live_state_ws(websocket: WebSocket):
    """Без handshake. На connect — replay последнего фрейма, потом стрим."""
    await websocket.accept()
    if mps_live_state_broker._loop is None:
        mps_live_state_broker.attach_loop(asyncio.get_event_loop())

    queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=64)
    mps_live_state_broker.add(queue)

    last = mps_live_state_broker.get_last()
    if last is not None:
        try:
            await websocket.send_json(last if last.get('type') == 'live_state'
                                       else {'type': 'live_state', 'point': last})
        except Exception:
            pass

    try:
        while True:
            frame = await queue.get()
            await websocket.send_json(frame)
    except WebSocketDisconnect:
        pass
    finally:
        mps_live_state_broker.remove(queue)
        try:
            await websocket.close()
        except Exception:
            pass
```

Примечание: `last` в `get_last()` — это frame, который был передан в `broadcast()` (т.е. уже `{'type': 'live_state', 'point': {...}}`). Если handler положил «голый» payload через `set_last`, оборачиваем при отправке. Это покрывается двумя ветками в `send_json`.

- [ ] **Step 4.4: Запустить — должны пройти**

Run: `pytest tests/test_mps_router.py -k live_state -v`

Expected: 3 PASS.

- [ ] **Step 4.5: Прогнать весь test_mps_router.py**

Run: `pytest tests/test_mps_router.py -v`

Expected: всё зелёное (включая старый WS-тест).

- [ ] **Step 4.6: Коммит**

```bash
git add compute_node/dashboard/routers/mps.py tests/test_mps_router.py
git commit -m "feat(mps): /ws/mps/live_state — постоянный broadcast вектора состояния"
```

---

## Task 5: Dashboard — wire MQTT-handler ↔ broker в `app.py`

**Files:**
- Modify: `compute_node/dashboard/app.py`

- [ ] **Step 5.1: Найти точку врезки**

Открыть `compute_node/dashboard/app.py`, найти строку:

```python
        mqtt.set_mps_ws_broadcaster(mps.mps_broker.broadcast)
```

(была около строки 493 по последнему чтению).

- [ ] **Step 5.2: Добавить wiring для live_state**

Сразу после этой строки добавить:

```python
        mqtt.set_mps_live_state_broadcaster(mps.mps_live_state_broker.broadcast)
```

- [ ] **Step 5.3: Проверить что endpoint реально доступен**

Запустить сервер локально (можно через тестовый launcher) или проверить через тест:

```bash
pytest tests/test_mps_router.py -v
```

Expected: все WS-тесты по-прежнему зелёные.

- [ ] **Step 5.4: Smoke-test полного MQTT-пути через интеграционный тест (опционально, если есть live mosquitto)**

Не блокирующее — но если есть тестовый брокер:

```bash
mosquitto_pub -t samurai/robot1/mps/live_state -m '{"ts":1.0,"x":[0,0,0,0,0],"u":[0,0],"scenario_active":false,"run_id":null,"schema_version":"1.0"}'
```

И смотреть в dashboard логах что handler вызвался. Если mosquitto нет — пропустить.

- [ ] **Step 5.5: Коммит**

```bash
git add compute_node/dashboard/app.py
git commit -m "feat(mps): подключить mps_live_state_broker к MQTT-handler в app.py"
```

---

## Task 6: Frontend — типы

**Files:**
- Modify: `compute_node/frontend/src/types/mps.ts`

- [ ] **Step 6.1: Добавить типы в конец `types/mps.ts`**

В конец `compute_node/frontend/src/types/mps.ts`:

```ts
// ── /ws/mps/live_state ────────────────────────────────────────────────
// Постоянный поток вектора состояния x ∈ ℝ⁵ и управления u ∈ ℝ²
// (10 Hz), независимо от прогона. Контракт: docs/superpowers/specs/
// 2026-05-18-mps-live-state-vector-design.md §2.
export const MPS_LIVE_STATE_SCHEMA = '1.0' as const

export interface MpsLiveStatePoint {
  /** Pi-clock unix seconds */
  ts: number
  /** Состояние [s, v, θ, ω, e_int]; длина 5 */
  x: number[]
  /** Управление [v_cmd, ω_cmd]; длина 2 */
  u: number[]
  /** true когда mps_node в DRIVE_FORWARD_MPS */
  scenario_active: boolean
  /** UUID активного прогона, если scenario_active=true */
  run_id: string | null
  schema_version: string
}

export interface MpsLiveStateWsFrame {
  type: 'live_state'
  point: MpsLiveStatePoint
}
```

- [ ] **Step 6.2: Проверить TS-компиляцию**

Run: `cd compute_node/frontend && npm run type-check 2>&1 | tail -20`

Expected: «no errors» либо завершение без ошибок. Если script называется иначе — ищем в `package.json` (`tsc`, `vue-tsc`, `tsc --noEmit`).

- [ ] **Step 6.3: Коммит**

```bash
git add compute_node/frontend/src/types/mps.ts
git commit -m "feat(mps): TS типы для /ws/mps/live_state (MpsLiveStatePoint, ws frame)"
```

---

## Task 7: Frontend — хук `useMpsLiveState`

**Files:**
- Create: `compute_node/frontend/src/hooks/useMpsLiveState.ts`
- Create: `compute_node/frontend/src/hooks/useMpsLiveState.test.tsx`

- [ ] **Step 7.1: Написать падающие тесты**

Создать `compute_node/frontend/src/hooks/useMpsLiveState.test.tsx`:

```tsx
/**
 * useMpsLiveState — WS-хук для постоянного отображения вектора
 * состояния. Тестируем: connect → fire frame → point обновился; >2s без
 * фрейма → stale=true; onclose → connected=false, point сохраняется.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useMpsLiveState } from './useMpsLiveState'

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
    ts: 1.0,
    x: [0.5, 0.12, 0.05, 0.0, 0.0],
    u: [0.1, 0.0],
    scenario_active: false,
    run_id: null,
    schema_version: '1.0',
  },
}

describe('useMpsLiveState', () => {
  it('подключается при mount и обновляет point на frame', () => {
    const { result } = renderHook(() => useMpsLiveState())
    expect(fakes.length).toBe(1)
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(result.current.connected).toBe(true)
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('stale=true через > 2s без frame', () => {
    vi.useFakeTimers()
    // Хук считает stale через Date.now() - lastReceivedAt; явно
    // фиксируем системное время, чтобы advanceTimersByTime реально его
    // двигал (vitest fake timers могут не мокать Date по умолчанию).
    vi.setSystemTime(new Date(1_000_000))
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.stale).toBe(false)
    act(() => {
      vi.advanceTimersByTime(2500)
    })
    expect(result.current.stale).toBe(true)
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('onclose → connected=false, point сохраняется', () => {
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point).not.toBeNull()
    act(() => ws.close())
    expect(result.current.connected).toBe(false)
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('игнорирует frames не-live_state-типа', () => {
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage({ type: 'telemetry', run_id: 'x', point: {} }))
    expect(result.current.point).toBeNull()
  })
})
```

- [ ] **Step 7.2: Запустить — должны упасть**

Run: `cd compute_node/frontend && npm test -- useMpsLiveState`

Expected: FAIL — `useMpsLiveState` не существует.

- [ ] **Step 7.3: Реализовать хук**

Создать `compute_node/frontend/src/hooks/useMpsLiveState.ts`:

```ts
import { useEffect, useRef, useState } from 'react'
import type { MpsLiveStatePoint, MpsLiveStateWsFrame } from '@/types/mps'

const STALE_THRESHOLD_MS = 2_000
const RECONNECT_DELAYS_MS = [1_000, 2_000, 4_000, 8_000, 10_000] as const

export interface UseMpsLiveStateResult {
  /** Последний полученный фрейм или null если ничего ещё не пришло. */
  point: MpsLiveStatePoint | null
  /** true если WS-сокет открыт. */
  connected: boolean
  /** true если >2s без новых фреймов (даже если WS открыт). */
  stale: boolean
  /** ms с момента последнего фрейма; null если фреймов ещё не было. */
  ageMs: number | null
}

/**
 * Постоянный WS-канал /ws/mps/live_state. Без enabled/runId — клиент
 * подключается всегда; auto-reconnect с экспоненциальным backoff.
 * Контракт фреймов: docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.
 */
export function useMpsLiveState(): UseMpsLiveStateResult {
  const [point, setPoint] = useState<MpsLiveStatePoint | null>(null)
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
      const url = `${protocol}//${window.location.host}/ws/mps/live_state`
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        if (cancelled) return
        setConnected(true)
        reconnectAttemptRef.current = 0
      }

      ws.onmessage = (ev: MessageEvent) => {
        let frame: MpsLiveStateWsFrame
        try {
          frame = JSON.parse(ev.data) as MpsLiveStateWsFrame
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
        // Backoff reconnect
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

- [ ] **Step 7.4: Запустить — должны пройти**

Run: `cd compute_node/frontend && npm test -- useMpsLiveState`

Expected: 4 PASS.

- [ ] **Step 7.5: Коммит**

```bash
git add compute_node/frontend/src/hooks/useMpsLiveState.ts compute_node/frontend/src/hooks/useMpsLiveState.test.tsx
git commit -m "feat(mps): хук useMpsLiveState с auto-reconnect и stale-таймером"
```

---

## Task 8: Frontend — компонент `LiveStateVector`

**Files:**
- Create: `compute_node/frontend/src/components/mps/LiveStateVector.tsx`
- Create: `compute_node/frontend/src/components/mps/LiveStateVector.test.tsx`

- [ ] **Step 8.1: Написать падающие тесты**

Создать `compute_node/frontend/src/components/mps/LiveStateVector.test.tsx`:

```tsx
import { describe, it, expect, vi } from 'vitest'
import { render, screen } from '@testing-library/react'
import { LiveStateVector } from './LiveStateVector'
import type { UseMpsLiveStateResult } from '@/hooks/useMpsLiveState'

vi.mock('@/hooks/useMpsLiveState', () => ({
  useMpsLiveState: () => mockResult,
}))

let mockResult: UseMpsLiveStateResult = {
  point: null,
  connected: false,
  stale: false,
  ageMs: null,
}

describe('LiveStateVector', () => {
  it('point=null + disconnected: рендерит «—» и бейдж disconnected', () => {
    mockResult = { point: null, connected: false, stale: false, ageMs: null }
    render(<LiveStateVector />)
    expect(screen.getByText(/disconnected/i)).toBeInTheDocument()
    // «—» появляется в каждой строке вектора
    expect(screen.getAllByText('—').length).toBeGreaterThan(0)
  })

  it('живой фрейм: рендерит числа и бейдж live', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.0],
        u: [0.12, 0.0],
        scenario_active: false,
        run_id: null,
        schema_version: '1.0',
      },
      connected: true,
      stale: false,
      ageMs: 100,
    }
    render(<LiveStateVector />)
    expect(screen.getByText(/live/i)).toBeInTheDocument()
    // s = +0.234
    expect(screen.getByText(/\+0\.234/)).toBeInTheDocument()
    // v = +0.118
    expect(screen.getByText(/\+0\.118/)).toBeInTheDocument()
    // idle подпись
    expect(screen.getByText(/idle/i)).toBeInTheDocument()
  })

  it('stale: бейдж stale показывает возраст в секундах', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.0],
        u: [0.12, 0.0],
        scenario_active: false,
        run_id: null,
        schema_version: '1.0',
      },
      connected: true,
      stale: true,
      ageMs: 5_400,
    }
    render(<LiveStateVector />)
    expect(screen.getByText(/stale/i)).toBeInTheDocument()
    expect(screen.getByText(/5s|5\.4s/i)).toBeInTheDocument()
  })

  it('scenario_active=true: показывает префикс с обрезанным run_id', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.42],
        u: [0.12, 0.05],
        scenario_active: true,
        run_id: 'abcdef1234567890',
        schema_version: '1.0',
      },
      connected: true,
      stale: false,
      ageMs: 100,
    }
    render(<LiveStateVector />)
    expect(screen.getByText(/abcdef12/)).toBeInTheDocument()
    expect(screen.queryByText(/^idle\b/i)).not.toBeInTheDocument()
  })
})
```

- [ ] **Step 8.2: Запустить — должны упасть**

Run: `cd compute_node/frontend && npm test -- LiveStateVector`

Expected: FAIL — компонент не существует.

- [ ] **Step 8.3: Реализовать компонент**

Создать `compute_node/frontend/src/components/mps/LiveStateVector.tsx`:

```tsx
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { useMpsLiveState } from '@/hooks/useMpsLiveState'

function fmt(n: number | undefined, digits: number): string {
  if (n === undefined || Number.isNaN(n)) return '—'
  const s = n.toFixed(digits)
  return n >= 0 ? `+${s}` : s
}

function fmtAge(ms: number | null): string {
  if (ms === null) return ''
  const sec = ms / 1000
  return sec < 10 ? `${sec.toFixed(1)}s` : `${Math.round(sec)}s`
}

function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI
}

interface BadgeProps {
  connected: boolean
  stale: boolean
  ageMs: number | null
}

function StatusBadge({ connected, stale, ageMs }: BadgeProps) {
  if (!connected) {
    return (
      <span className="text-xs px-2 py-0.5 rounded bg-muted text-muted-foreground">
        disconnected
      </span>
    )
  }
  if (stale) {
    return (
      <span className="text-xs px-2 py-0.5 rounded bg-yellow-500/20 text-yellow-700">
        stale · {fmtAge(ageMs)}
      </span>
    )
  }
  return (
    <span className="text-xs px-2 py-0.5 rounded bg-green-500/20 text-green-700">
      ● live
    </span>
  )
}

export function LiveStateVector() {
  const { point, connected, stale, ageMs } = useMpsLiveState()

  const x = point?.x ?? []
  const u = point?.u ?? []
  const s = x[0]
  const v = x[1]
  const theta = x[2]
  const omega = x[3]
  const eInt = x[4]
  const vCmd = u[0]
  const omegaCmd = u[1]

  const scenarioLabel = point?.scenario_active && point.run_id
    ? `сценарий: ${point.run_id.slice(0, 8)}`
    : 'idle'
  const ageLabel = ageMs !== null ? `обновлено ${fmtAge(ageMs)} назад` : ''

  return (
    <Card>
      <CardHeader className="pb-2 flex flex-row items-center justify-between space-y-0">
        <CardTitle className="text-sm">Вектор состояния</CardTitle>
        <StatusBadge connected={connected} stale={stale} ageMs={ageMs} />
      </CardHeader>
      <CardContent className="text-xs font-mono tabular-nums space-y-2 pt-0">
        <div className="space-y-0.5">
          <div className="text-muted-foreground">x:</div>
          <Row label="s" value={fmt(s, 3)} unit="м" />
          <Row label="v" value={fmt(v, 3)} unit="м/с" />
          <Row
            label="θ"
            value={theta === undefined ? '—' : `${fmt(radToDeg(theta), 1)}°`}
            unit={theta === undefined ? '' : `(${fmt(theta, 4)} рад)`}
          />
          <Row
            label="ω"
            value={omega === undefined ? '—' : `${fmt(radToDeg(omega), 1)}°/с`}
            unit={omega === undefined ? '' : `(${fmt(omega, 4)} рад/с)`}
          />
          <Row label="e_int" value={fmt(eInt, 4)} unit="" />
        </div>
        <div className="space-y-0.5 border-t pt-2">
          <div className="text-muted-foreground">u:</div>
          <Row label="v_cmd" value={fmt(vCmd, 3)} unit="м/с" />
          <Row
            label="ω_cmd"
            value={omegaCmd === undefined ? '—' : `${fmt(radToDeg(omegaCmd), 1)}°/с`}
            unit={omegaCmd === undefined ? '' : `(${fmt(omegaCmd, 4)} рад/с)`}
          />
        </div>
        <div className="text-muted-foreground border-t pt-2">
          {scenarioLabel}
          {ageLabel ? ` · ${ageLabel}` : ''}
        </div>
      </CardContent>
    </Card>
  )
}

interface RowProps {
  label: string
  value: string
  unit: string
}

function Row({ label, value, unit }: RowProps) {
  return (
    <div className="flex items-baseline gap-2">
      <span className="w-12 text-muted-foreground">{label}</span>
      <span className="flex-1">= {value}</span>
      {unit && <span className="text-muted-foreground">{unit}</span>}
    </div>
  )
}
```

- [ ] **Step 8.4: Запустить — должны пройти**

Run: `cd compute_node/frontend && npm test -- LiveStateVector`

Expected: 4 PASS.

- [ ] **Step 8.5: Type-check**

Run: `cd compute_node/frontend && npm run type-check 2>&1 | tail -20`

Expected: no errors.

- [ ] **Step 8.6: Коммит**

```bash
git add compute_node/frontend/src/components/mps/LiveStateVector.tsx compute_node/frontend/src/components/mps/LiveStateVector.test.tsx
git commit -m "feat(mps): компонент LiveStateVector — постоянная карточка x, u"
```

---

## Task 9: Frontend — врезка в `MpsPage.tsx`

**Files:**
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx`

- [ ] **Step 9.1: Импорт + врезка**

В `compute_node/frontend/src/pages/MpsPage.tsx`, в блоке импортов с `@/components/mps/*`, добавить:

```tsx
import { LiveStateVector } from '@/components/mps/LiveStateVector'
```

Найти JSX:
```tsx
          <aside className="space-y-3 lg:sticky lg:top-16 lg:self-start lg:max-h-[calc(100vh-5rem)] lg:overflow-y-auto">
            <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
```

Заменить на:
```tsx
          <aside className="space-y-3 lg:sticky lg:top-16 lg:self-start lg:max-h-[calc(100vh-5rem)] lg:overflow-y-auto">
            <LiveStateVector />
            <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
```

- [ ] **Step 9.2: Type-check + сборка**

Run: `cd compute_node/frontend && npm run type-check && npm run build 2>&1 | tail -30`

Expected: успешная сборка без TS-ошибок.

- [ ] **Step 9.3: Smoke vitest для всей mps-секции**

Run: `cd compute_node/frontend && npm test -- mps`

Expected: всё зелёное.

- [ ] **Step 9.4: Коммит**

```bash
git add compute_node/frontend/src/pages/MpsPage.tsx
git commit -m "feat(mps): встроить LiveStateVector в левый sidebar MpsPage"
```

---

## Task 10: Manual sanity на симе

Это финальная верификация. UI можно проверить только в браузере, не в тестах.

- [ ] **Step 10.1: Поднять sim + dashboard**

Откройте 2 терминала:

```bash
./samurai.sh sim
./samurai.sh compute
```

(Если есть mps_node в sim-режиме — он должен публиковать `mps/live_state`. Если sim не имитирует mps_node, см. fallback в Step 10.5.)

- [ ] **Step 10.2: Открыть `/mps` в браузере**

Открыть `http://localhost:5000/mps` (или адрес, который выдал launcher).

Ожидаемое:
- В левом sidebar над «ОДУ» появляется карточка «Вектор состояния».
- Бейдж: `● live` (зелёный).
- Числа `s, v, θ, ω, e_int, v_cmd, ω_cmd` — заполнены, не «—».
- Внизу: `idle · обновлено <Ns> назад`.

- [ ] **Step 10.3: Остановить sim, наблюдать stale**

В терминале sim: `Ctrl+C`. В браузере, через ~2 сек:
- Бейдж меняется на жёлтый `stale · Ns`.
- Числа замораживаются на последних значениях.

- [ ] **Step 10.4: Запустить sim снова**

```bash
./samurai.sh sim
```

В браузере без перезагрузки страницы:
- Бейдж возвращается в `● live`.
- Числа снова обновляются.

- [ ] **Step 10.5: Fallback если sim не публикует mps/live_state**

Если sim-имитатор не запускает mps_node — опубликуйте payload вручную через mosquitto:

```bash
mosquitto_pub -h localhost -t samurai/robot1/mps/live_state -m '{"ts":1747574400.0,"x":[0.5,0.12,0.0,0.0,0.0],"u":[0.1,0.0],"scenario_active":false,"run_id":null,"schema_version":"1.0"}'
```

В браузере карточка должна обновиться немедленно.

- [ ] **Step 10.6: Записать результат в коммит-меседж финального деплоя (опционально)**

Если нужен «итоговый» коммит-метка после manual sanity, можно сделать пустой:

```bash
git commit --allow-empty -m "chore(mps): live state vector — manual sanity на sim OK"
```

Иначе — пропустить, история уже зелёная.

---

## Done When

- [ ] Все pytest-тесты проходят: `pytest tests/test_mps_node.py tests/test_dashboard_mqtt_handlers.py tests/test_mps_router.py -v`
- [ ] Все vitest-тесты проходят: `cd compute_node/frontend && npm test`
- [ ] `npm run type-check` + `npm run build` зелёные.
- [ ] Manual sanity на sim: live → stale → live проходят (Task 10).
- [ ] Все коммиты Task 1-9 на ветке `main` (по запросу пользователя).
