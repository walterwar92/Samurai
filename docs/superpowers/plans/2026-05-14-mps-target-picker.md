# МПС — пред-прогонный 3D-пикер цели — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** При запуске МПС-сценария на роботе показывать модальное 3D-окно, где пользователь кликом выбирает точку на окружности радиуса N (= относительный курс φ); робот разворачивается на месте к φ, затем едет прямо N метров.

**Architecture:** Двухфазный FSM на Pi (`mps_node`): TURN (разворот к φ) → DRIVE (движение N метров с удержанием курса φ). Контракт расширяется одним полем `target_heading` (дефолт 0.0 = сегодняшнее поведение «вперёд»). Фронтенд: новая модалка `MpsTargetPicker` с R3F-сценой `MpsTargetScene`, переиспользующей `RobotModel`/GLB; `MpsPageInner` ветвит запуск по `source`. Чистая математика угла вынесена в `lib/targetAngle.ts` для юнит-тестов.

**Tech Stack:** Python 3 + numpy + paho-mqtt + pytest (Pi-нода, контракт); Pydantic (схемы); React 19 + TypeScript + @react-three/fiber + @react-three/drei + three (фронтенд); vitest + @testing-library/react (фронт-тесты).

**Спека:** `docs/superpowers/specs/2026-05-14-mps-target-picker-design.md`

**Ветка:** `feat/mps-target-picker` (уже создана, от `fix/mps-heading-scenario-relative`).

---

## File Structure

### Новые файлы

| Файл | Ответственность |
|---|---|
| `compute_node/frontend/src/lib/targetAngle.ts` | Чистая геометрия пикера: точка пола ↔ угол φ, позиция маркера, подпись курса. Без Three/WebGL → юнит-тестируется. |
| `compute_node/frontend/src/lib/targetAngle.test.ts` | Юнит-тесты `targetAngle.ts`. |
| `compute_node/frontend/src/components/mps/MpsTargetScene.tsx` | R3F `<Canvas>`: модель робота, кольцо радиуса N, кликабельный пол (raycast), маркер цели. Переиспользует `RobotModel` + `targetAngle`. |
| `compute_node/frontend/src/components/mps/MpsTargetPicker.tsx` | Модалка (portal + backdrop), владеет `pickedAngle`, рендерит `MpsTargetScene`, кнопки «Старт»/✕. |
| `compute_node/frontend/src/components/mps/MpsTargetPicker.test.tsx` | Юнит-тесты модалки (сцена замокана). |

### Изменяемые файлы

| Файл | Правки |
|---|---|
| `compute_node/dashboard/schemas/mps.py` | `MpsScenarioRequest` += `target_heading: float = 0.0` (валидация `[−π, π]`). |
| `tests/test_mps_router.py` | Контракт-тесты `target_heading`. |
| `pi_nodes/nodes/mps_node.py` | `_RunState` поля turn/drive; `__init__` пороги из config; `_on_scenario_run` парсит `target_heading`; `_publish_cmd_and_telemetry` (вынос); двухфазный `_tick` + `_tick_turn` + `_tick_drive`. |
| `tests/test_mps_node.py` | Тесты двухфазного FSM. |
| `config.yaml` | `mps.scenario.turn_tolerance_rad`, `turn_timeout_s`, `omega_max_in_turn`. |
| `compute_node/frontend/src/types/mps.ts` | `MpsScenarioRequest` += `target_heading?: number`. |
| `compute_node/frontend/src/pages/MpsPage.tsx` | Состояние пикера + ветвление `handleRun` по `source`. |
| `compute_node/frontend/src/pages/MpsPage.test.tsx` | Интеграционные тесты пикера. |
| `docs/mps/api.md` | Описание `target_heading` + двухфазного robot-сценария. |

### НЕ затрагиваем

- `compute_node/mps_runner.py` — симулятор игнорирует `target_heading` (поле есть в запросе, идеальный sim его не читает).
- `compute_node/dashboard/routers/mps.py` — robot-путь уже делает `request.model_dump()` → MQTT-payload, новое поле прокидывается само.
- `compute_node/frontend/src/lib/mpsApi.ts` — `runScenario` сериализует объект запроса как есть.
- `pi_nodes/control/*` — каноническая модель переиспользуется без изменений.
- Существующие `Mps3D*` компоненты (пост-прогонное воспроизведение) — независимы от пикера.

### Соглашение о знаке угла φ (критично — единое во всех слоях)

- φ = 0 — «вперёд» (ось +X мировая; `RobotModel` yaw=0).
- φ > 0 — поворот **влево** (CCW); φ < 0 — **вправо** (CW).
- Маппинг координат (как в `Mps3DScene`): мир `(wx, wy)` ↔ Three `(wx, h, -wy)`.
- Пикер: `groundPointToAngle(threeX, threeZ) = atan2(-threeZ, threeX)`.
- Pi: `x[_THETA]` из одометрии — CCW-положительный, scenario-relative (после фикса `fix/mps-heading-scenario-relative`). Цель TURN — довести `x[_THETA]` до `φ`.

---

## Task 1: Контракт — `target_heading` в `MpsScenarioRequest`

**Files:**
- Modify: `compute_node/dashboard/schemas/mps.py` (импорт `math`; поле в `MpsScenarioRequest`, ~строки 165-183)
- Test: `tests/test_mps_router.py`

- [ ] **Step 1: Написать падающие тесты**

Добавить в `tests/test_mps_router.py` после `test_scenario_run_robot_when_online_async` (после строки ~223):

```python
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
```

- [ ] **Step 2: Запустить тесты — убедиться, что падают**

Run: `python -m pytest tests/test_mps_router.py::test_scenario_run_robot_includes_target_heading_in_mqtt tests/test_mps_router.py::test_scenario_run_target_heading_defaults_to_zero tests/test_mps_router.py::test_scenario_run_target_heading_out_of_range_rejected -v`
Expected: FAIL — `KeyError: 'target_heading'` / out-of-range возвращает 200 вместо 422.

- [ ] **Step 3: Реализовать поле**

В `compute_node/dashboard/schemas/mps.py` добавить `import math` в блок импортов (после `from datetime import datetime`):

```python
from __future__ import annotations

import math
from datetime import datetime
from typing import Literal, Optional
```

В классе `MpsScenarioRequest` добавить поле `target_heading` перед `schema_version`:

```python
class MpsScenarioRequest(BaseModel):
    """POST /api/v1/mps/scenario/run."""
    distance: float = Field(
        ...,
        gt=0,
        le=5.0,
        description='D — дистанция в метрах (safety cap 5.0)'
    )
    v_target: float = Field(
        ...,
        gt=0,
        le=0.30,
        description='Целевая продольная скорость (м/с, cap 0.30)'
    )
    source: ScenarioSource = Field(
        ...,
        description='sim — на ноуте, robot — реальный Pi через MQTT'
    )
    target_heading: float = Field(
        default=0.0,
        ge=-math.pi,
        le=math.pi,
        description='Относительный целевой курс (рад) от курса на старте '
                    'сценария. 0.0 = ехать прямо вперёд (поведение по '
                    'умолчанию). Используется только при source="robot".'
    )
    schema_version: str = MPS_SCHEMA_VERSION
```

- [ ] **Step 4: Запустить тесты — убедиться, что проходят**

Run: `python -m pytest tests/test_mps_router.py -q`
Expected: PASS — все тесты роутера, включая 3 новых.

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/schemas/mps.py tests/test_mps_router.py
git commit -m "feat(mps): MpsScenarioRequest.target_heading — относительный курс цели"
```

---

## Task 2: Pi-нода — поля turn-фазы в `_RunState` + пороги из config

**Files:**
- Modify: `config.yaml` (блок `mps.scenario`, после строки 444)
- Modify: `pi_nodes/nodes/mps_node.py` (`__init__` config-чтения; `_RunState.__slots__` + `__init__`; `_on_scenario_run`)
- Test: `tests/test_mps_node.py`

- [ ] **Step 1: Написать падающие тесты**

Добавить в `tests/test_mps_node.py` в конец файла:

```python
def test_on_scenario_run_reads_target_heading(mps_node):
    """_on_scenario_run читает target_heading из request и стартует
    в фазе 'turn'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-th',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot',
                    'target_heading': 0.6},
    })
    assert mps_node.is_running
    assert mps_node._run.target_heading == pytest.approx(0.6)
    assert mps_node._run.phase == 'turn'
    assert mps_node._run.drive_t == 0.0


def test_on_scenario_run_target_heading_defaults_zero(mps_node):
    """Без target_heading в request — дефолт 0.0 (поведение «вперёд»)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-th0',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node.is_running
    assert mps_node._run.target_heading == 0.0


def test_on_scenario_run_target_heading_out_of_range_rejected(mps_node):
    """target_heading вне [−π, π] → mps/error precondition, run не стартует."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-bad',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot',
                    'target_heading': 4.0},
    })
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err and err[0]['error_type'] == 'precondition'
    assert not mps_node.is_running


def test_mps_node_loads_turn_config(mps_node):
    """__init__ читает пороги turn-фазы из config (с дефолтами)."""
    assert isinstance(mps_node._turn_tol, float) and mps_node._turn_tol > 0
    assert isinstance(mps_node._turn_timeout, float) and mps_node._turn_timeout > 0
    assert isinstance(mps_node._omega_max_turn, float) and mps_node._omega_max_turn > 0
```

- [ ] **Step 2: Запустить тесты — убедиться, что падают**

Run: `python -m pytest tests/test_mps_node.py::test_on_scenario_run_reads_target_heading tests/test_mps_node.py::test_on_scenario_run_target_heading_defaults_zero tests/test_mps_node.py::test_on_scenario_run_target_heading_out_of_range_rejected tests/test_mps_node.py::test_mps_node_loads_turn_config -v`
Expected: FAIL — `AttributeError: '_RunState' object has no attribute 'target_heading'` / `'MpsNode' object has no attribute '_turn_tol'`.

- [ ] **Step 3a: Добавить пороги в `config.yaml`**

В `config.yaml` в блок `mps.scenario:` (после строки `default_v_target: 0.15`, строка 444) добавить:

```yaml
  scenario:
    distance_max: 5.0
    v_target_max: 0.30
    omega_max_in_forward: 0.5
    default_distance: 2.0
    default_v_target: 0.15
    turn_tolerance_rad: 0.05       # |θ−φ| < этого ⇒ фаза TURN завершена (~3°)
    turn_timeout_s: 10.0           # TURN не сошёлся за это ⇒ timeout
    omega_max_in_turn: 1.0         # кап ω в TURN (выше omega_max_in_forward)
```

- [ ] **Step 3b: Прочитать пороги в `MpsNode.__init__`**

В `pi_nodes/nodes/mps_node.py` в `__init__`, после строки `self._omega_max_fwd = float(self._cfg('mps.scenario.omega_max_in_forward', 0.5))`, добавить:

```python
        self._omega_max_fwd = float(self._cfg('mps.scenario.omega_max_in_forward', 0.5))
        self._turn_tol = float(self._cfg('mps.scenario.turn_tolerance_rad', 0.05))
        self._turn_timeout = float(self._cfg('mps.scenario.turn_timeout_s', 10.0))
        self._omega_max_turn = float(self._cfg('mps.scenario.omega_max_in_turn', 1.0))
```

- [ ] **Step 3c: Расширить `_RunState`**

В `pi_nodes/nodes/mps_node.py` заменить `__slots__` и `__init__` класса `_RunState`:

```python
    __slots__ = (
        'run_id', 'distance', 'v_target', 'started_at',
        'telemetry', 't',
        'no_odom_ticks', 's_start', 'theta_start',
        'target_heading', 'phase', 'drive_t',
    )

    def __init__(self, run_id: str, distance: float, v_target: float,
                 s_start: float = 0.0, theta_start: float = 0.0,
                 target_heading: float = 0.0):
        self.run_id = run_id
        self.distance = distance
        self.v_target = v_target
        self.started_at = datetime.now(timezone.utc)
        self.telemetry: list[dict] = []
        self.t = 0.0
        self.no_odom_ticks = 0
        # Абсолютная позиция одометрии на момент старта сценария.
        # `s_ref` стартует с 0, поэтому позицию считаем относительно неё.
        self.s_start = s_start
        # Абсолютный курс одометрии на момент старта. Сценарий «вперёд D
        # метров» — это вперёд ОТНОСИТЕЛЬНО старта, поэтому θ считаем
        # относительно θ_start (как и s). Иначе MPC трактует x_ref[θ]=0 как
        # абсолютный 0 одометрии и доворачивает робота в одну и ту же
        # сторону вместо «ехать прямо куда смотрит».
        self.theta_start = theta_start
        # Относительный целевой курс φ (рад) — куда развернуться перед
        # движением. 0.0 = ехать прямо вперёд (сегодняшнее поведение).
        self.target_heading = target_heading
        # Фаза двухфазного сценария: 'turn' (разворот к φ на месте) →
        # 'drive' (движение N метров с удержанием курса φ).
        self.phase = 'turn'
        # Часы фазы DRIVE — начинаются с 0 при переходе TURN→DRIVE.
        # Используются для ramp s_ref и drive-timeout (тайминг движения
        # считается от начала езды, а не от старта сценария). `t` при этом
        # остаётся монотонным суммарным временем (turn + drive).
        self.drive_t = 0.0
```

- [ ] **Step 3d: Парсить `target_heading` в `_on_scenario_run`**

В `pi_nodes/nodes/mps_node.py` в методе `_on_scenario_run` заменить блок парсинга request и создание `_RunState`. Найти:

```python
        run_id = str(payload.get('run_id', ''))
        request = payload.get('request') or {}
        try:
            distance = float(request['distance'])
            v_target = float(request['v_target'])
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_error('precondition', f'scenario/run: bad request: {exc}',
                                run_id=run_id)
            return

        # Pre-validate against safety caps
        if not (0 < distance <= self._distance_max):
```

заменить на:

```python
        run_id = str(payload.get('run_id', ''))
        request = payload.get('request') or {}
        try:
            distance = float(request['distance'])
            v_target = float(request['v_target'])
            target_heading = float(request.get('target_heading', 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_error('precondition', f'scenario/run: bad request: {exc}',
                                run_id=run_id)
            return

        # Pre-validate against safety caps
        if not (-math.pi - 1e-6 <= target_heading <= math.pi + 1e-6):
            self._publish_error('precondition',
                                f'target_heading {target_heading} not in [-pi, pi]',
                                run_id=run_id)
            return
        if not (0 < distance <= self._distance_max):
```

И ниже, найти создание `_RunState`:

```python
            s_start = float(self._x_meas[_S])
            theta_start = float(self._x_meas[_THETA])
            self._run = _RunState(run_id, distance, v_target,
                                  s_start, theta_start)
            self._fsm_state = 'DRIVE_FORWARD_MPS'
```

заменить на:

```python
            s_start = float(self._x_meas[_S])
            theta_start = float(self._x_meas[_THETA])
            self._run = _RunState(run_id, distance, v_target,
                                  s_start, theta_start, target_heading)
            self._fsm_state = 'DRIVE_FORWARD_MPS'
```

- [ ] **Step 4: Запустить тесты — убедиться, что проходят**

Run: `python -m pytest tests/test_mps_node.py -q`
Expected: PASS — все тесты, включая 4 новых. Существующие тесты не затронуты: `_tick` пока не меняется, `_RunState` лишь получил новые поля с дефолтами.

- [ ] **Step 5: Commit**

```bash
git add config.yaml pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "feat(mps): mps_node — поля turn-фазы в _RunState + пороги из config"
```

---

## Task 3: Pi-нода — вынести `_publish_cmd_and_telemetry` из `_tick` (рефактор)

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py` (`_tick` — вынести блок публикации в метод)
- Test: `tests/test_mps_node.py` (без нового теста — рефактор покрыт существующими)

> **Это REFACTOR-задача** (фаза REFACTOR из TDD): новой функциональности нет, поведение не меняется. Защитная сеть — существующие тесты `test_mps_node.py`, которые должны быть зелёными до и после.

- [ ] **Step 1: Зафиксировать зелёный базовый прогон**

Run: `python -m pytest tests/test_mps_node.py -q`
Expected: PASS — все тесты зелёные (базовая линия перед рефактором).

- [ ] **Step 2: Вынести метод `_publish_cmd_and_telemetry`**

В `pi_nodes/nodes/mps_node.py` в методе `_tick` найти блок публикации:

```python
        # Send cmd_vel
        self.publish('cmd_vel', {
            'linear_x': float(u[0]),
            'angular_z': float(u[1]),
        }, qos=0)

        # Output for UI
        try:
            y = self._plant.output(x, u)
        except Exception:
            y = x.copy()

        s_remaining = max(0.0, run.distance - x[_S])
        point = {
            't': round(run.t, 6),
            'x': [float(v) for v in x],
            'u': [float(v) for v in u],
            'y': [float(v) for v in y],
            's_remaining': float(s_remaining),
        }
        run.telemetry.append(point)
        self.publish('mps/telemetry', {
            'run_id': run.run_id,
            'point': point,
            'schema_version': '1.0',
        }, qos=0)
```

заменить на один вызов:

```python
        self._publish_cmd_and_telemetry(run, x, u)
```

Затем добавить новый метод **сразу после метода `_tick`** (перед `# ── Finalisation ──` / `_finish_run`):

```python
    # ── Публикация cmd_vel + телеметрии (общее для обеих фаз) ──────────
    def _publish_cmd_and_telemetry(self, run: _RunState, x: np.ndarray,
                                   u: np.ndarray) -> None:
        """Опубликовать cmd_vel и точку телеметрии. Вызывается из обеих
        фаз сценария (_tick_turn, _tick_drive). `run.t` — монотонное
        суммарное время прогона, поэтому `point['t']` строго растёт."""
        self.publish('cmd_vel', {
            'linear_x': float(u[0]),
            'angular_z': float(u[1]),
        }, qos=0)

        try:
            y = self._plant.output(x, u)
        except Exception:
            y = x.copy()

        s_remaining = max(0.0, run.distance - x[_S])
        point = {
            't': round(run.t, 6),
            'x': [float(v) for v in x],
            'u': [float(v) for v in u],
            'y': [float(v) for v in y],
            's_remaining': float(s_remaining),
        }
        run.telemetry.append(point)
        self.publish('mps/telemetry', {
            'run_id': run.run_id,
            'point': point,
            'schema_version': '1.0',
        }, qos=0)
```

- [ ] **Step 3: Запустить тесты — убедиться, что всё ещё зелёные**

Run: `python -m pytest tests/test_mps_node.py -q`
Expected: PASS — все тесты по-прежнему зелёные (поведение не изменилось).

- [ ] **Step 4: Commit**

```bash
git add pi_nodes/nodes/mps_node.py
git commit -m "refactor(mps): mps_node — вынести _publish_cmd_and_telemetry из _tick"
```

---

## Task 4: Pi-нода — двухфазный `_tick` (TURN → DRIVE)

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py` (`_tick` → оркестратор; добавить `_tick_turn`, `_tick_drive`)
- Test: `tests/test_mps_node.py`

> **Уточнение спеки §8/§11.** Спека описывала замену `run.t` на отдельные `turn_t`/`drive_t` и правку `_finish_run`. Этот план держит `run.t` как **монотонное суммарное** время (для `point['t']` и `settling_time` в `_finish_run`) и добавляет только `run.drive_t` для ramp `s_ref` и drive-timeout. Поведение то же, полей меньше, **`_finish_run` не меняется**.

- [ ] **Step 1: Написать падающие тесты**

Добавить в `tests/test_mps_node.py` в конец файла:

```python
def test_tick_turn_rotates_toward_target_heading(mps_node):
    """В фазе TURN робот крутится к target_heading: φ>0 ⇒ angular_z>0
    (CCW), ход linear_x = 0 (чистое вращение)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-turn',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run is not None and mps_node._run.phase == 'turn'
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert cmd_vel['linear_x'] == 0.0, 'в TURN ход должен быть 0 (чистое вращение)'
    assert cmd_vel['angular_z'] > 0.0, 'φ>0 ⇒ робот крутится CCW'


def test_tick_turn_transitions_to_drive_when_aligned(mps_node):
    """Когда |θ − φ| < turn_tol, фаза переключается на 'drive'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-trans',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Одометрия: курс робота уже совпал с целью φ.
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.8, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run is not None
    assert mps_node._run.phase == 'drive', 'курс совпал с φ ⇒ переход в DRIVE'


def test_tick_drive_holds_target_heading(mps_node):
    """В фазе DRIVE θ_ref = φ: если курс робота ниже φ, контроллер
    доворачивает ВВЕРХ к φ (angular_z>0), а не вниз к 0."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-hold',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Перевести в DRIVE: одометрия с курсом = φ.
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.8, 'vz': 0.0})
    mps_node._tick()
    assert mps_node._run.phase == 'drive'
    # Курс робота «сполз» ниже φ (0.6 < 0.8).
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.6, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert cmd_vel['angular_z'] > 0.0, (
        'курс 0.6 < φ=0.8 ⇒ доворот вверх к φ; '
        'если бы θ_ref был 0 — angular_z был бы < 0'
    )


def test_tick_turn_timeout(mps_node):
    """Если TURN не сходится за turn_timeout — прогон завершается timeout."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-tto',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 3.0},
    })
    max_ticks = int(mps_node._turn_timeout / mps_node._tick_dt) + 10
    for _ in range(max_ticks):
        # Робот «застрял»: курс 0, далеко от φ=3.0 — TURN не сойдётся.
        mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
        mps_node._tick()
        if not mps_node.is_running:
            break
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished and finished[-1]['status'] == 'timeout'
    assert not mps_node.is_running
```

- [ ] **Step 2: Запустить новые тесты — убедиться, что падают**

Run: `python -m pytest tests/test_mps_node.py::test_tick_turn_rotates_toward_target_heading tests/test_mps_node.py::test_tick_turn_transitions_to_drive_when_aligned tests/test_mps_node.py::test_tick_drive_holds_target_heading tests/test_mps_node.py::test_tick_turn_timeout -v`
Expected: FAIL — старый `_tick` однофазный, игнорирует `run.phase`/`target_heading` (linear_x>0 в TURN, phase остаётся 'turn', drive не держит φ, нет turn-timeout).

- [ ] **Step 3: Заменить `_tick` оркестратором + добавить `_tick_turn`, `_tick_drive`**

В `pi_nodes/nodes/mps_node.py` заменить **весь метод `_tick`** (версию после Task 3) на три метода:

```python
    # ── Tick (50 Hz во время RUNNING, no-op в IDLE) ───────────────────
    def _tick(self):
        with self._lock:
            run = self._run
            if run is None or self._fsm_state != 'DRIVE_FORWARD_MPS':
                return
            x = self._x_meas.copy()
        # Позиция и курс — относительно старта сценария (s_ref, θ_ref с 0).
        x[_S] = x[_S] - run.s_start
        x[_THETA] = _normalize_angle(x[_THETA] - run.theta_start)

        # Двухфазный сценарий: TURN (разворот к φ) → DRIVE (едем N метров).
        if run.phase == 'turn':
            if not self._tick_turn(run, x):
                return            # ещё крутимся, либо прогон завершён
            # фаза TURN завершилась этим тиком → продолжаем в DRIVE
        self._tick_drive(run, x)

    # ── Фаза TURN: разворот на месте к target_heading ─────────────────
    def _tick_turn(self, run: _RunState, x: np.ndarray) -> bool:
        """Один тик фазы разворота. `x` — уже относительный (s, θ).

        Возвращает True ровно когда поворот только что завершён — тогда
        вызывающий (`_tick`) продолжает в DRIVE тем же тиком. False —
        если ещё крутимся или прогон уже завершён (timeout / ошибка).
        """
        phi = run.target_heading

        # Курс совпал с целью → переход в DRIVE. Без публикации —
        # cmd_vel/телеметрию за этот тик опубликует _tick_drive.
        if abs(_normalize_angle(x[_THETA] - phi)) < self._turn_tol:
            run.phase = 'drive'
            run.drive_t = 0.0
            return True

        x_ref = np.array([0.0, 0.0, phi, 0.0, 0.0])
        try:
            u = self._mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            self.log_error('mps turn mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return False

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return False

        # Чистое вращение: ход — в ноль; ω — свой (более высокий) кап.
        u[0] = 0.0
        u[1] = max(-self._omega_max_turn, min(self._omega_max_turn, u[1]))

        self._publish_cmd_and_telemetry(run, x, u)

        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        # Turn timeout — по run.t (во время TURN он = времени разворота).
        if run.t > self._turn_timeout:
            self._finish_run('timeout', None)
            return False

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return False

        run.t += self._tick_dt
        return False

    # ── Фаза DRIVE: едем N метров, удерживая курс target_heading ───────
    def _tick_drive(self, run: _RunState, x: np.ndarray) -> None:
        """Один тик фазы движения. `x` — уже относительный (s, θ).

        Логика «вперёд D», но θ_ref = target_heading (удержание выбранного
        курса, НЕ доворот к 0) и тайминг ramp/timeout по run.drive_t.
        """
        phi = run.target_heading

        # Reference: ramp s_ref to D, hold v_target, hold heading φ.
        s_ref = min(run.distance, run.drive_t * run.v_target)
        x_ref = np.array([s_ref, run.v_target, phi, 0.0, 0.0])

        try:
            u = self._mpc.step(x, x_ref=x_ref)
        except Exception as exc:
            self.log_error('mps drive mpc.step failed: %s', exc)
            self._finish_run('error', f'mpc.step: {exc}')
            return

        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
            self._finish_run('error', 'NaN/Inf in u or x')
            return

        # Hard omega cap in forward scenario — guard against accidental rotation.
        u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))

        self._publish_cmd_and_telemetry(run, x, u)

        # Watchdog: нет одометрии 3 тика подряд → abort
        with self._lock:
            run.no_odom_ticks += 1
            stale = run.no_odom_ticks > _WATCHDOG_TICKS

        # Reached?
        if x[_S] >= run.distance - _REACH_EPS:
            self._finish_run('reached', None)
            return

        # Timeout?
        timeout_t = max(1.0, 3.0 * run.distance / max(run.v_target, 1e-6))
        if run.drive_t > timeout_t:
            self._finish_run('timeout', None)
            return

        if stale and time.time() - self._x_meas_ts > 5.0 * self._tick_dt:
            self._finish_run('error', 'watchdog: no odom for >3 ticks')
            return

        run.t += self._tick_dt
        run.drive_t += self._tick_dt
```

- [ ] **Step 4: Запустить новые тесты — убедиться, что проходят**

Run: `python -m pytest tests/test_mps_node.py::test_tick_turn_rotates_toward_target_heading tests/test_mps_node.py::test_tick_turn_transitions_to_drive_when_aligned tests/test_mps_node.py::test_tick_drive_holds_target_heading tests/test_mps_node.py::test_tick_turn_timeout -v`
Expected: PASS — все 4 новых теста.

- [ ] **Step 5: Запустить весь файл — регрессия φ=0**

Run: `python -m pytest tests/test_mps_node.py -q`
Expected: PASS — **все** тесты, включая существующие `test_tick_publishes_cmd_vel_and_telemetry`, `test_tick_reaches_goal`, `test_tick_position_is_scenario_relative`, `test_tick_heading_is_scenario_relative`. При `target_heading=0` (дефолт) фаза TURN завершается на первом тике (`|x[θ] − 0| < turn_tol`, робот стоит ровно) → DRIVE идентичен сегодняшнему поведению.

- [ ] **Step 6: Commit**

```bash
git add pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "feat(mps): mps_node — двухфазный сценарий TURN→DRIVE"
```

---

## Task 5: Фронтенд — `lib/targetAngle.ts` (чистая математика пикера)

**Files:**
- Create: `compute_node/frontend/src/lib/targetAngle.ts`
- Test: `compute_node/frontend/src/lib/targetAngle.test.ts`

- [ ] **Step 1: Написать падающий тест**

Создать `compute_node/frontend/src/lib/targetAngle.test.ts`:

```ts
import { describe, it, expect } from 'vitest'
import {
  groundPointToAngle,
  angleToMarkerPosition,
  formatHeadingLabel,
  groundPointRadius,
  MARKER_HEIGHT,
} from './targetAngle'

describe('groundPointToAngle', () => {
  it('точка прямо перед роботом (+X) → φ ≈ 0', () => {
    expect(groundPointToAngle(2.0, 0)).toBeCloseTo(0, 6)
  })
  it('точка слева (−Z в Three = +Y в мире) → φ = +π/2', () => {
    expect(groundPointToAngle(0, -2.0)).toBeCloseTo(Math.PI / 2, 6)
  })
  it('точка справа (+Z в Three = −Y в мире) → φ = −π/2', () => {
    expect(groundPointToAngle(0, 2.0)).toBeCloseTo(-Math.PI / 2, 6)
  })
  it('точка сзади (−X) → |φ| = π', () => {
    expect(Math.abs(groundPointToAngle(-2.0, 0))).toBeCloseTo(Math.PI, 6)
  })
})

describe('angleToMarkerPosition', () => {
  it('φ=0 → маркер на (+N, h, 0)', () => {
    const [x, h, z] = angleToMarkerPosition(0, 2.0)
    expect(x).toBeCloseTo(2.0, 6)
    expect(h).toBe(MARKER_HEIGHT)
    expect(z).toBeCloseTo(0, 6)
  })
  it('φ=+π/2 → маркер на (0, h, −N)', () => {
    const [x, h, z] = angleToMarkerPosition(Math.PI / 2, 2.0)
    expect(x).toBeCloseTo(0, 6)
    expect(h).toBe(MARKER_HEIGHT)
    expect(z).toBeCloseTo(-2.0, 6)
  })
  it('радиус сохраняется для любого угла', () => {
    const [x, , z] = angleToMarkerPosition(0.7, 3.0)
    expect(Math.hypot(x, z)).toBeCloseTo(3.0, 6)
  })
  it('round-trip: angleToMarkerPosition → groundPointToAngle', () => {
    const phi = 0.9
    const [x, , z] = angleToMarkerPosition(phi, 2.5)
    expect(groundPointToAngle(x, z)).toBeCloseTo(phi, 6)
  })
})

describe('formatHeadingLabel', () => {
  it('φ≈0 → «прямо»', () => {
    expect(formatHeadingLabel(0)).toBe('прямо')
    expect(formatHeadingLabel(0.02)).toBe('прямо')
  })
  it('φ>0 → «+N°»', () => {
    expect(formatHeadingLabel(Math.PI / 4)).toBe('+45°')
  })
  it('φ<0 → «−N°»', () => {
    expect(formatHeadingLabel(-Math.PI / 4)).toBe('-45°')
  })
})

describe('groundPointRadius', () => {
  it('расстояние от центра (origin)', () => {
    expect(groundPointRadius(3, 4)).toBeCloseTo(5, 6)
  })
})
```

- [ ] **Step 2: Запустить тест — убедиться, что падает**

Run: `cd compute_node/frontend && npm run test -- src/lib/targetAngle.test.ts`
Expected: FAIL — `Failed to resolve import './targetAngle'` (файл не существует).

- [ ] **Step 3: Реализовать `targetAngle.ts`**

Создать `compute_node/frontend/src/lib/targetAngle.ts`:

```ts
// Чистая математика 3D-пикера цели МПС. Без зависимости от Three/WebGL,
// чтобы покрываться юнит-тестами в jsdom.
//
// Соглашение о координатах (как в Mps3DScene): мир (wx, wy) ↔ Three
// (wx, h, -wy). Робот yaw=0 смотрит по +X. Угол φ — относительный курс:
// φ=0 — прямо вперёд, φ>0 — влево (CCW), φ<0 — вправо (CW).

/** Высота маркера/кольца над полом сцены (Three Y), м. */
export const MARKER_HEIGHT = 0.03

/** Зона у центра: клики ближе этой доли радиуса к роботу игнорируются
 *  (там угол скачет от микродвижений мыши). */
export const CENTER_DEADZONE_FRACTION = 0.3

/**
 * Точка клика на полу сцены (Three-координаты x, z) → относительный курс φ.
 * Возвращает угол в радианах в диапазоне (−π, π].
 */
export function groundPointToAngle(threeX: number, threeZ: number): number {
  // мир: wx = threeX, wy = -threeZ. φ = atan2(wy, wx).
  return Math.atan2(-threeZ, threeX)
}

/**
 * Угол φ + радиус N → позиция маркера в Three-координатах [x, h, z].
 * Маркер всегда на окружности радиуса N (дистанция фиксирована).
 */
export function angleToMarkerPosition(
  angle: number,
  radius: number,
): [number, number, number] {
  return [radius * Math.cos(angle), MARKER_HEIGHT, -radius * Math.sin(angle)]
}

/**
 * φ → подпись для readout. «прямо» в дедзоне ~3°, иначе «+35°» / «-40°».
 * Знак: + влево (CCW), − вправо (CW).
 */
export function formatHeadingLabel(angle: number): string {
  const deg = (angle * 180) / Math.PI
  if (Math.abs(deg) < 3) return 'прямо'
  const rounded = Math.round(deg)
  return rounded > 0 ? `+${rounded}°` : `${rounded}°`
}

/**
 * Расстояние точки пола от центра (робота), м — для дедзоны центра.
 * threeX/threeZ — Three-координаты точки на полу.
 */
export function groundPointRadius(threeX: number, threeZ: number): number {
  return Math.hypot(threeX, threeZ)
}
```

- [ ] **Step 4: Запустить тест — убедиться, что проходит**

Run: `cd compute_node/frontend && npm run test -- src/lib/targetAngle.test.ts`
Expected: PASS — все группы тестов.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/lib/targetAngle.ts compute_node/frontend/src/lib/targetAngle.test.ts
git commit -m "feat(mps): lib/targetAngle — математика 3D-пикера цели"
```

---

## Task 6: Фронтенд — `MpsTargetScene.tsx` (R3F-сцена пикера)

**Files:**
- Create: `compute_node/frontend/src/components/mps/MpsTargetScene.tsx`

> **Без юнит-теста.** R3F/WebGL не рендерится в jsdom — это согласовано с `Mps3DScene.tsx`, у которого тоже нет юнит-теста. Вся чистая математика покрыта в Task 5 (`targetAngle.test.ts`). Проверка здесь — компиляция (`npm run build`) + линт (`npm run lint`). Ручной smoke-тест — на железе (§13 спеки).

- [ ] **Step 1: Создать `MpsTargetScene.tsx`**

Создать `compute_node/frontend/src/components/mps/MpsTargetScene.tsx`:

```tsx
import { Suspense, useEffect, useRef } from 'react'
import { Canvas } from '@react-three/fiber'
import type { ThreeEvent } from '@react-three/fiber'
import { OrbitControls, Grid, Html } from '@react-three/drei'
import * as THREE from 'three'
import { RobotModel } from '@/components/3d/RobotModel'
import {
  groundPointToAngle,
  angleToMarkerPosition,
  groundPointRadius,
  CENTER_DEADZONE_FRACTION,
  MARKER_HEIGHT,
} from '@/lib/targetAngle'

interface MpsTargetSceneProps {
  /** Радиус кольца N (= дистанция сценария), м. */
  distance: number
  /** Текущий выбранный относительный курс φ, рад. */
  pickedAngle: number
  /** Колбэк выбора нового угла (клик по полу вне дедзоны центра). */
  onPick: (angle: number) => void
}

/** Линия от робота (0,0) к маркеру цели. Императивная сборка геометрии —
 *  как в Mps3DScene.AnimatedTrail (проверенный паттерн codebase). */
function TargetLine({ distance, pickedAngle }: { distance: number; pickedAngle: number }) {
  const geometryRef = useRef<THREE.BufferGeometry>(null)
  const [mx, my, mz] = angleToMarkerPosition(pickedAngle, distance)
  useEffect(() => {
    const geom = geometryRef.current
    if (!geom) return
    const positions = new Float32Array([0, MARKER_HEIGHT, 0, mx, my, mz])
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  }, [mx, my, mz])
  return (
    <line>
      <bufferGeometry ref={geometryRef} />
      <lineBasicMaterial color="#22d3ee" linewidth={2} />
    </line>
  )
}

export function MpsTargetScene({ distance, pickedAngle, onPick }: MpsTargetSceneProps) {
  const markerPos = angleToMarkerPosition(pickedAngle, distance)

  function handleGroundClick(e: ThreeEvent<MouseEvent>) {
    e.stopPropagation()
    const { x, z } = e.point
    // Дедзона у центра: слишком близкие к роботу клики игнорируем
    // (там угол скачет от микродвижений мыши).
    if (groundPointRadius(x, z) < distance * CENTER_DEADZONE_FRACTION) return
    onPick(groundPointToAngle(x, z))
  }

  // Камера наклонно-сверху, сзади робота: +X («вперёд») уходит вверх кадра.
  // Стартовые значения — тонкая подстройка под читаемость допустима.
  const camPos: [number, number, number] = [-distance * 0.7, distance * 1.9, 0]

  return (
    <div className="relative w-full h-full">
      <Canvas
        camera={{ position: camPos, fov: 50, near: 0.01, far: 100 }}
        shadows
      >
        <color attach="background" args={['#1a1a2e']} />

        <ambientLight intensity={0.9} />
        <directionalLight
          position={[2, 3, 1]}
          intensity={1.8}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <directionalLight position={[-1, 2, -1]} intensity={0.7} />
        <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]} />

        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.6}
          cellColor="#3f3f5c"
          sectionSize={0.5}
          sectionThickness={1.2}
          sectionColor="#5a5a7a"
          fadeDistance={6}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid
        />

        {/* Кликабельная плоскость пола — мишень для raycast.
            opacity=0 (не visible=false!) — прозрачная, но raycast-able. */}
        <mesh
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, 0, 0]}
          onClick={handleGroundClick}
        >
          <planeGeometry args={[40, 40]} />
          <meshBasicMaterial transparent opacity={0} />
        </mesh>

        {/* Окружность радиуса N — «куда можно выбрать точку». */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
          <ringGeometry args={[distance - 0.015, distance + 0.015, 96]} />
          <meshBasicMaterial color="#22d3ee" side={THREE.DoubleSide} />
        </mesh>

        {/* Маркер старта — серая сфера в центре (под роботом). */}
        <mesh position={[0, 0.02, 0]}>
          <sphereGeometry args={[0.025, 12, 12]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>

        {/* Линия робот → цель. */}
        <TargetLine distance={distance} pickedAngle={pickedAngle} />

        {/* Маркер выбранной цели — оранжевый конус остриём вниз. */}
        <mesh position={markerPos} rotation={[Math.PI, 0, 0]}>
          <coneGeometry args={[0.04, 0.1, 16]} />
          <meshStandardMaterial
            color="#f97316"
            emissive="#f97316"
            emissiveIntensity={0.4}
          />
        </mesh>

        <Suspense fallback={
          <Html center>
            <div className="text-zinc-300 text-sm bg-zinc-900/80 px-3 py-2 rounded border border-zinc-700 backdrop-blur whitespace-nowrap">
              Загрузка модели…
            </div>
          </Html>
        }>
          <RobotModel
            yaw={0}
            pitch={0}
            roll={0}
            posX={0}
            posY={0}
            stationary
            noSmooth
          />
        </Suspense>

        <OrbitControls
          target={[0, 0, 0]}
          maxPolarAngle={Math.PI / 2 - 0.05}
          minDistance={distance * 0.6}
          maxDistance={distance * 4}
          enableDamping
          dampingFactor={0.1}
        />
      </Canvas>
    </div>
  )
}
```

- [ ] **Step 2: Проверить компиляцию и линт**

Run: `cd compute_node/frontend && npm run build && npm run lint`
Expected: PASS — `tsc -b` без ошибок типов, `vite build` собирается, `eslint .` без ошибок.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/MpsTargetScene.tsx
git commit -m "feat(mps): MpsTargetScene — R3F-сцена пикера цели"
```

---

## Task 7: Фронтенд — `MpsTargetPicker.tsx` (модалка выбора)

**Files:**
- Create: `compute_node/frontend/src/components/mps/MpsTargetPicker.tsx`
- Test: `compute_node/frontend/src/components/mps/MpsTargetPicker.test.tsx`

- [ ] **Step 1: Написать падающий тест**

Создать `compute_node/frontend/src/components/mps/MpsTargetPicker.test.tsx`:

```tsx
import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent, act } from '@testing-library/react'
import { MpsTargetPicker } from './MpsTargetPicker'

// R3F/WebGL в jsdom не работает — подменяем сцену заглушкой, которая
// умеет вызвать onPick (имитация клика по полу под углом π/4).
vi.mock('./MpsTargetScene', () => ({
  MpsTargetScene: ({ onPick }: { onPick: (a: number) => void }) => (
    <button data-testid="scene-stub" onClick={() => onPick(Math.PI / 4)}>
      scene
    </button>
  ),
}))

describe('MpsTargetPicker', () => {
  it('рендерит шапку и кнопку «Старт»', () => {
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={vi.fn()} />,
    )
    expect(screen.getByText(/Куда ехать роботу/i)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /Старт/i })).toBeInTheDocument()
  })

  it('«Старт» с предвыбранным φ=0 зовёт onConfirm(0)', () => {
    const onConfirm = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={onConfirm} onCancel={vi.fn()} />,
    )
    act(() => { screen.getByRole('button', { name: /Старт/i }).click() })
    expect(onConfirm).toHaveBeenCalledWith(0)
  })

  it('после выбора точки «Старт» зовёт onConfirm с этим углом', () => {
    const onConfirm = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={onConfirm} onCancel={vi.fn()} />,
    )
    act(() => { screen.getByTestId('scene-stub').click() })   // onPick(π/4)
    act(() => { screen.getByRole('button', { name: /Старт/i }).click() })
    expect(onConfirm).toHaveBeenCalledWith(Math.PI / 4)
  })

  it('readout курса обновляется после выбора точки', () => {
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={vi.fn()} />,
    )
    expect(screen.getByText('прямо')).toBeInTheDocument()
    act(() => { screen.getByTestId('scene-stub').click() })   // onPick(π/4)
    expect(screen.getByText('+45°')).toBeInTheDocument()
  })

  it('✕ зовёт onCancel', () => {
    const onCancel = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={onCancel} />,
    )
    act(() => { screen.getByRole('button', { name: /Закрыть выбор цели/i }).click() })
    expect(onCancel).toHaveBeenCalled()
  })

  it('клик по backdrop зовёт onCancel', () => {
    const onCancel = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={onCancel} />,
    )
    act(() => { fireEvent.click(screen.getByTestId('mps-target-backdrop')) })
    expect(onCancel).toHaveBeenCalled()
  })
})
```

- [ ] **Step 2: Запустить тест — убедиться, что падает**

Run: `cd compute_node/frontend && npm run test -- src/components/mps/MpsTargetPicker.test.tsx`
Expected: FAIL — `Failed to resolve import './MpsTargetPicker'`.

- [ ] **Step 3: Реализовать `MpsTargetPicker.tsx`**

Создать `compute_node/frontend/src/components/mps/MpsTargetPicker.tsx`:

```tsx
import { useEffect, useMemo, useState } from 'react'
import { createPortal } from 'react-dom'
import { X } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { MpsTargetScene } from './MpsTargetScene'
import { formatHeadingLabel } from '@/lib/targetAngle'

interface MpsTargetPickerProps {
  /** Радиус окружности N (= дистанция сценария), м. */
  distance: number
  /** Целевая скорость — показывается в подвале, в прогон уходит как есть. */
  vTarget: number
  /** Подтверждение: пользователь нажал «Старт». Передаёт φ (рад). */
  onConfirm: (targetHeading: number) => void
  /** Отмена: ✕ или клик по backdrop. Прогон не запускается. */
  onCancel: () => void
}

export function MpsTargetPicker({
  distance,
  vTarget,
  onConfirm,
  onCancel,
}: MpsTargetPickerProps) {
  // φ предвыбран в 0 («прямо») — «Старт» активна сразу.
  const [pickedAngle, setPickedAngle] = useState(0)

  // Анимация появления (как в Mps3DOverlay).
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  const headingLabel = useMemo(() => formatHeadingLabel(pickedAngle), [pickedAngle])

  return createPortal(
    <div
      data-testid="mps-target-backdrop"
      onClick={onCancel}
      className={[
        'fixed inset-0 z-[70] flex items-center justify-center',
        'bg-black/70 transition-opacity duration-200',
        entered ? 'opacity-100' : 'opacity-0',
      ].join(' ')}
    >
      <div
        onClick={(e) => e.stopPropagation()}
        className={[
          'relative w-[640px] h-[520px] overflow-hidden flex flex-col',
          'rounded-lg border border-zinc-700 bg-[#1a1a2e]',
          'transition-transform duration-200 ease-out',
          entered ? 'scale-100' : 'scale-95',
        ].join(' ')}
        role="dialog"
        aria-modal="true"
        aria-label="Выбор цели для робота"
      >
        {/* Шапка */}
        <div className="flex items-center justify-between px-4 py-2 border-b border-zinc-700 bg-zinc-900/80">
          <span className="text-sm font-medium text-zinc-100">Куда ехать роботу</span>
          <button
            type="button"
            onClick={onCancel}
            className="text-zinc-400 hover:text-zinc-100 transition-colors p-1 -m-1"
            aria-label="Закрыть выбор цели"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* 3D-сцена */}
        <div className="flex-1 min-h-0">
          <MpsTargetScene
            distance={distance}
            pickedAngle={pickedAngle}
            onPick={setPickedAngle}
          />
        </div>

        {/* Подвал: курс + дистанция + «Старт» */}
        <div className="flex items-center justify-between px-4 py-3 border-t border-zinc-700 bg-zinc-900/80">
          <div className="text-xs font-mono text-zinc-300">
            Курс: <span className="text-cyan-400">{headingLabel}</span>
            {' • '}
            Дистанция: <span className="text-zinc-100">{distance.toFixed(2)} м</span>
            {' • '}
            v_target: <span className="text-zinc-100">{vTarget.toFixed(2)} м/с</span>
          </div>
          <Button size="sm" onClick={() => onConfirm(pickedAngle)}>
            ▶ Старт
          </Button>
        </div>
      </div>
    </div>,
    document.body,
  )
}
```

- [ ] **Step 4: Запустить тест — убедиться, что проходит**

Run: `cd compute_node/frontend && npm run test -- src/components/mps/MpsTargetPicker.test.tsx`
Expected: PASS — все 6 тестов.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/MpsTargetPicker.tsx compute_node/frontend/src/components/mps/MpsTargetPicker.test.tsx
git commit -m "feat(mps): MpsTargetPicker — модалка выбора точки на окружности"
```

---

## Task 8: Фронтенд — `MpsPage` через пикер + тип `target_heading`

**Files:**
- Modify: `compute_node/frontend/src/types/mps.ts` (`MpsScenarioRequest` += `target_heading?`)
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx` (состояние пикера + ветвление `handleRun`)
- Test: `compute_node/frontend/src/pages/MpsPage.test.tsx`

- [ ] **Step 1: Написать падающие тесты**

В `compute_node/frontend/src/pages/MpsPage.test.tsx`:

(а) Добавить мок `MpsTargetScene` рядом с существующим моком `Mps3DScene` (после строки `vi.mock('@/components/mps/Mps3DScene', ...)`):

```tsx
vi.mock('@/components/mps/MpsTargetScene', () => ({
  MpsTargetScene: () => <div data-testid="mps-target-scene-stub" />,
}))
```

(б) Заменить мок `useMpsRun` на стабильный спай `mockRun`. Добавить module-level переменную после `let mockRunId: string | null = null`:

```tsx
let mockRunId: string | null = null
let mockRun = vi.fn(() => Promise.resolve(null as MpsScenarioResult | null))
```

И заменить `run: vi.fn(),` в `vi.mock('@/hooks/useMpsRun', ...)` на `run: mockRun,`:

```tsx
vi.mock('@/hooks/useMpsRun', () => ({
  useMpsRun: () => ({
    running: false,
    result: mockRunResult,
    runId: mockRunId,
    error: null,
    run: mockRun,
    abort: vi.fn(),
  }),
}))
```

(в) В обоих существующих `beforeEach` (в `describe('MpsPage integration', ...)` и `describe('MpsPage — 3D toast', ...)`) добавить строку сброса спая после `mockRunId = null`:

```tsx
    mockRunId = null
    mockRun = vi.fn(() => Promise.resolve(null as MpsScenarioResult | null))
```

(г) Добавить новый describe-блок в конец файла:

```tsx
describe('MpsPage — robot target picker', () => {
  beforeEach(() => {
    mockRunResult = null
    mockRunId = null
    mockRun = vi.fn(() => Promise.resolve(null as MpsScenarioResult | null))
    if (typeof globalThis.ResizeObserver === 'undefined') {
      globalThis.ResizeObserver = class {
        observe() {}
        unobserve() {}
        disconnect() {}
      } as unknown as typeof ResizeObserver
    }
  })

  it('Run на роботе открывает пикер и НЕ запускает прогон сразу', () => {
    render(<MpsPage />)
    act(() => { fireEvent.click(screen.getByRole('button', { name: /^Robot$/i })) })
    act(() => { fireEvent.click(screen.getByRole('button', { name: /Run on Robot/i })) })
    expect(
      screen.getByRole('dialog', { name: /Выбор цели для робота/i }),
    ).toBeInTheDocument()
    expect(mockRun).not.toHaveBeenCalled()
  })

  it('Run на симуляторе запускает прогон сразу, без пикера', () => {
    render(<MpsPage />)
    // Источник по умолчанию — sim.
    act(() => { fireEvent.click(screen.getByRole('button', { name: /Run on Sim/i })) })
    expect(
      screen.queryByRole('dialog', { name: /Выбор цели для робота/i }),
    ).toBeNull()
    expect(mockRun).toHaveBeenCalledTimes(1)
  })

  it('«Старт» в пикере запускает robot-прогон с target_heading и закрывает пикер', () => {
    render(<MpsPage />)
    act(() => { fireEvent.click(screen.getByRole('button', { name: /^Robot$/i })) })
    act(() => { fireEvent.click(screen.getByRole('button', { name: /Run on Robot/i })) })
    act(() => { fireEvent.click(screen.getByRole('button', { name: /Старт/i })) })
    expect(
      screen.queryByRole('dialog', { name: /Выбор цели для робота/i }),
    ).toBeNull()
    expect(mockRun).toHaveBeenCalledTimes(1)
    const req = mockRun.mock.calls[0][0] as { source: string; target_heading?: number }
    expect(req.source).toBe('robot')
    expect(req.target_heading).toBe(0)
  })
})
```

- [ ] **Step 2: Запустить тесты — убедиться, что падают**

Run: `cd compute_node/frontend && npm run test -- src/pages/MpsPage.test.tsx`
Expected: FAIL — пикер не подключён (диалог не появляется; robot-Run сразу зовёт `mockRun`).

- [ ] **Step 3a: Добавить `target_heading` в `types/mps.ts`**

В `compute_node/frontend/src/types/mps.ts` заменить интерфейс `MpsScenarioRequest`:

```ts
export interface MpsScenarioRequest {
  /** D — дистанция в метрах (0 < D ≤ 5.0) */
  distance: number
  /** Целевая продольная скорость (0 < v ≤ 0.30) */
  v_target: number
  source: ScenarioSource
  /** Относительный целевой курс (рад, −π…π) от курса на старте сценария.
   *  0 = ехать прямо. Используется только при source='robot'. */
  target_heading?: number
  schema_version?: string
}
```

- [ ] **Step 3b: Подключить пикер в `MpsPage.tsx`**

В `compute_node/frontend/src/pages/MpsPage.tsx`:

(1) Добавить импорт рядом с другими импортами компонентов mps:

```tsx
import { MpsTargetPicker } from '@/components/mps/MpsTargetPicker'
```

(2) В `MpsPageInner` добавить состояние пикера рядом с другими `useState` (после `const [errors, setErrors] = useState...`):

```tsx
  const [picker, setPicker] = useState<{ distance: number; vTarget: number } | null>(null)
```

(3) Заменить функцию `handleRun`:

```tsx
  function handleRun(req: MpsScenarioRequest) {
    if (req.source === 'robot') {
      // На роботе — сперва выбор цели в 3D-пикере; прогон по «Старт».
      setPicker({ distance: req.distance, vTarget: req.v_target })
      return
    }
    void runHook.run(req).then((r) => {
      if (r) {
        setPrimaryResult(r)
        void historyHook.refresh()
      }
    })
  }

  function startRobotRun(targetHeading: number) {
    if (!picker) return
    const req: MpsScenarioRequest = {
      distance: picker.distance,
      v_target: picker.vTarget,
      source: 'robot',
      target_heading: targetHeading,
    }
    setPicker(null)
    void runHook.run(req).then((r) => {
      if (r) {
        setPrimaryResult(r)
        void historyHook.refresh()
      }
    })
  }
```

(4) Отрендерить пикер — добавить перед закрывающим `</div>` внешнего `<div className="min-h-screen">` (в самом конце JSX, после закрытия `<div className="p-3 ...">`):

```tsx
        </div>
      </div>
      {picker && (
        <MpsTargetPicker
          distance={picker.distance}
          vTarget={picker.vTarget}
          onConfirm={startRobotRun}
          onCancel={() => setPicker(null)}
        />
      )}
    </div>
  )
}
```

(Контекст: завершающие строки `MpsPageInner` были `</div>` (закрытие `p-3`), `</div>` (закрытие `min-h-screen`), `)`, `}` — вставка `{picker && ...}` идёт между этими двумя `</div>`.)

- [ ] **Step 4: Запустить тесты — убедиться, что проходят**

Run: `cd compute_node/frontend && npm run test -- src/pages/MpsPage.test.tsx`
Expected: PASS — существующие тесты `MpsPage` + 3 новых.

- [ ] **Step 5: Полный прогон фронт-тестов + сборка + линт**

Run: `cd compute_node/frontend && npm run test && npm run build && npm run lint`
Expected: PASS — весь vitest-сьют зелёный, `tsc -b && vite build` собирается, `eslint .` чист.

- [ ] **Step 6: Commit**

```bash
git add compute_node/frontend/src/types/mps.ts compute_node/frontend/src/pages/MpsPage.tsx compute_node/frontend/src/pages/MpsPage.test.tsx
git commit -m "feat(mps): MpsPage — robot-прогон через пикер цели"
```

---

## Task 9: Документация — `docs/mps/api.md`

**Files:**
- Modify: `docs/mps/api.md`

> Без теста — документация. Сверяет контракт с реализацией Tasks 1-8.

- [ ] **Step 1: Прочитать `docs/mps/api.md`**

Run: открыть `docs/mps/api.md`, найти раздел про `POST /api/v1/mps/scenario/run` / `MpsScenarioRequest` / robot-сценарий.

- [ ] **Step 2: Добавить описание `target_heading` и двухфазного сценария**

В разделе про `MpsScenarioRequest` добавить строку поля:

```markdown
| `target_heading` | float | Относительный целевой курс (рад, −π…π) от курса робота на старте сценария. `0.0` (дефолт) = ехать прямо вперёд. Используется только при `source="robot"`. |
```

И добавить подраздел про двухфазный robot-сценарий (рядом с описанием `mps/scenario/run`):

```markdown
### Двухфазный robot-сценарий (TURN → DRIVE)

При `source="robot"` `mps_node` выполняет сценарий в две фазы:

1. **TURN** — разворот на месте к относительному курсу `target_heading`
   (`x_ref = [0, 0, φ, 0, 0]`, ход `linear_x` зажат в 0). Завершается
   когда `|θ − φ| < mps.scenario.turn_tolerance_rad`. Если не сошёлся за
   `mps.scenario.turn_timeout_s` — прогон завершается со `status="timeout"`.
2. **DRIVE** — движение `distance` метров с удержанием курса φ
   (`x_ref = [s_ref, v_target, φ, 0, 0]`). Логика и завершение
   (`reached` / `timeout`) — как в одно­фазном «вперёд D».

При `target_heading = 0.0` фаза TURN завершается мгновенно — поведение
идентично прежнему сценарию «проехать D метров вперёд».

Симулятор (`source="sim"`) игнорирует `target_heading` — у него остаётся
одно­фазный сценарий «вперёд D».
```

(Точное место — раздел контракта `/scenario/run`; если структура `api.md` иная, разместить логически рядом с описанием запроса и MQTT-топика `mps/scenario/run`.)

- [ ] **Step 3: Commit**

```bash
git add docs/mps/api.md
git commit -m "docs(mps): api.md — target_heading + двухфазный robot-сценарий"
```

---

## Self-Review

**1. Spec coverage** — каждое требование спеки покрыто задачей:

| Спека | Задача |
|---|---|
| §6 `lib/targetAngle.ts` | Task 5 |
| §6 `MpsTargetScene.tsx` | Task 6 |
| §6 `MpsTargetPicker.tsx` | Task 7 |
| §6 `MpsPage.tsx` ветвление | Task 8 |
| §6 `types/mps.ts` | Task 8 (Step 3a) |
| §7 `MpsScenarioRequest.target_heading` | Task 1 |
| §7 `routers/mps.py` без правок | подтверждено в "НЕ затрагиваем" |
| §8 `_RunState` поля | Task 2 |
| §8 `_on_scenario_run` парсинг | Task 2 |
| §8 `__init__` config + `config.yaml` | Task 2 |
| §8 `_tick` TURN | Task 4 (`_tick_turn`) |
| §8 `_tick` DRIVE | Task 4 (`_tick_drive`) |
| §8 знак угла φ | зафиксирован в "Соглашение о знаке φ" + тесты Task 4/5 |
| §8 телеметрия без смены схемы | Task 3 (`_publish_cmd_and_telemetry` использует `run.t`) |
| §6 контракт `docs/mps/api.md` | Task 9 |
| §10 тесты (фронт/бэк/Pi) | Tasks 1, 2, 4, 5, 7, 8 |

**2. Placeholder scan** — плейсхолдеров ("TBD", "TODO", "add error handling" без кода) нет; каждый шаг с кодом содержит полный код. Камера в `MpsTargetScene` — конкретные стартовые значения с пометкой о подстройке (это инвариант спеки §12.6, не плейсхолдер). `docs/mps/api.md` (Task 9) — точная структура файла не прочитана заранее, но контент для вставки приведён полностью + указано место.

**3. Type consistency** — проверено:
- `_RunState.__init__(... target_heading=0.0)` (Task 2) ↔ вызов `_RunState(run_id, distance, v_target, s_start, theta_start, target_heading)` (Task 2) — совпадает.
- `_publish_cmd_and_telemetry(self, run, x, u)` (Task 3) ↔ вызовы в `_tick_turn`/`_tick_drive` (Task 4) — совпадает.
- `_tick_turn(run, x) -> bool`, `_tick_drive(run, x) -> None` (Task 4) ↔ вызовы в `_tick` — совпадает.
- `targetAngle.ts` экспорты `groundPointToAngle`, `angleToMarkerPosition`, `formatHeadingLabel`, `groundPointRadius`, `MARKER_HEIGHT`, `CENTER_DEADZONE_FRACTION` (Task 5) ↔ импорты в `MpsTargetScene` (Task 6) и `MpsTargetPicker` (Task 7) — совпадает.
- `MpsTargetSceneProps {distance, pickedAngle, onPick}` (Task 6) ↔ использование в `MpsTargetPicker` (Task 7) — совпадает.
- `MpsTargetPickerProps {distance, vTarget, onConfirm, onCancel}` (Task 7) ↔ рендер в `MpsPage` (Task 8) — совпадает.
- `MpsScenarioRequest.target_heading` — Python `float = 0.0` (Task 1) ↔ TS `target_heading?: number` (Task 8) — согласовано (опционально на TS, дефолт на Python).

Гэпов и несостыковок не найдено.
