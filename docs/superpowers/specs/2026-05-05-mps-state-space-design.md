# МПС — Модель Пространства Состояний — Design

**Branch:** `feat/mps`
**Date:** 2026-05-05
**Owners:** [@razdryzg-dev](https://github.com/razdryzg-dev) (backend / Pi), [@OneAstr0](https://github.com/OneAstr0) (frontend)
**Status:** approved (brainstorm 2026-05-05)
**Базируется на:** `main` (включая security-hardening и существующий state-space контроллер из коммита `8e58c98`).

---

## 1. Цель и не-цели

### Цель
Учебно-исследовательский модуль для курсовой по учебнику В.Н. Козлова: пользователь редактирует матрицы A/B/C/D + параметры регулятора (Q, R, N, ограничения), запускает сценарий «проехать D метров вперёд» в симуляторе или на реальном Samurai, видит как меняются траектории и метрики качества. Удобный UI с историей, сравнением прогонов, визуализацией собственных значений и графиками x(t)/u(t)/y(t).

### Не-цели
- Не охота за мячами, не SLAM, не повороты, не голосовые команды — только финитный сценарий «прямо вперёд».
- Не переписывание существующего MPC-контроллера (`pi_nodes/control/mpc_controller.py`) — расширяем, не заменяем.
- Не оптимизация производительности — это учебный модуль, 50 Гц на Pi достаточно.
- Не observer / Kalman-filter — C, D хранятся как параметры курсовой, в управлении не используются.
- Не auth для новых endpoints в MVP (в dev-mode, открытые); добавление security — в backlog.
- Не экспорт CSV/PDF; графики только в UI.
- Не мобильный клиент / Android.

---

## 2. Глоссарий

| Термин | Значение |
|---|---|
| **МПС** | Модель Пространства Состояний (state-space). В этой ветке — НЕ «методы принятия решений». |
| **MPC** | Model Predictive Control — регулятор с предсказанием на горизонте N. Уже реализован в `pi_nodes/control/mpc_controller.py`. |
| **Сценарий** | Финитный прогон «проехать D метров со скоростью v_target», от старта до достижения дистанции / таймаута / abort'а. |
| **Draft** | Изменённые в UI матрицы, ещё не отправленные на робота (не-applied). |
| **Applied** | Матрицы, активные сейчас в `mpc_controller`. |
| **Run** | Один прогон сценария. Имеет `run_id`, telemetry, metrics. |
| **Source** | `"sim"` (идеальный симулятор на ноуте) или `"robot"` (реальный Pi через MQTT). |
| **DRIVE_FORWARD_MPS** | Новое FSM-state в `pi_nodes/nodes/fsm_node.py`. Только в этом state контроллер == MPC. |

---

## 3. Архитектура и границы

### Что переиспользуем (НЕ переписываем)
- `pi_nodes/control/state_space_model.py` — расширяем (Cd, Dd, reload, output).
- `pi_nodes/control/mpc_controller.py` — добавляем `rebuild()`.
- `pi_nodes/control/controller_factory.py` — расширяем switch `mode`.
- `pi_nodes/filters/position_fusion.py` — источник x_meas, не трогаем.
- `compute_node/simulator.py` — существующий, не трогаем; новый `mps_runner.py` живёт рядом отдельным модулем.
- `latex_doc/control_theory/*.tex` — теория уже есть, дописываем главу про сценарий-only режим.

### Что добавляем

```
pi_nodes/
  control/
    state_space_model.py        ← +Cd, +Dd, +reload(), +output()
    mpc_controller.py           ← +rebuild()
  nodes/
    mps_node.py                 ← НОВОЕ: orchestrator + FSM + telemetry pub
    fsm_node.py                 ← +state DRIVE_FORWARD_MPS
  schemas.py                    ← +MpsMatrices, +MpsScenario, +MpsTelemetry

compute_node/
  dashboard/
    routers/mps.py              ← НОВОЕ: /api/v1/mps/*
    schemas/mps.py              ← НОВОЕ
    state.py                    ← +mps state slice
    mqtt_handlers.py            ← +mps/* topic handlers
  mps_runner.py                 ← НОВОЕ: idealный run сценария по матрицам
  frontend/src/
    pages/MpsPage.tsx           ← НОВОЕ
    components/mps/             ← 10 компонентов (см. §6)
    hooks/                      ← 5 хуков (см. §6)
    lib/api.ts                  ← +mpsApi
    types/mps.ts                ← НОВОЕ — зеркало schemas/mps.py

docs/mps/
  architecture.md               ← НОВОЕ: диаграмма flow
  api.md                        ← НОВОЕ: REST + MQTT контракт (Day 0!)
  scenario_forward.md           ← НОВОЕ: формальное описание сценария
  README.md / TODO.md           ← обновить статус

config.yaml
  mps:                          ← новая секция

tests/
  test_mps_node.py              ← НОВОЕ
  test_mps_router.py            ← НОВОЕ
  test_mps_runner.py            ← НОВОЕ
  test_state_space_extended.py  ← новые тесты к существующему
```

### Инварианты
1. MPC активен **только** в FSM-state == `DRIVE_FORWARD_MPS`. В любом другом state — старый PID/legacy.
2. Сценарий — только финитный (D метров). При завершении FSM возвращается в IDLE.
3. Матрицы применяются **между** прогонами, не во время. `Apply` отправляет MQTT, `mps_node` пересоздаёт MPCController, шлёт ack.
4. C, D — для документации курсовой. В `step()` не используются. В `output()` — для визуализации y(t) в UI.
5. Симулятор — идеальная модель `x[k+1]=Ad·x+Bd·u`. Без шумов, slip'а, motor lag'а.
6. cmd_vel во время DRIVE_FORWARD_MPS идёт **только** от `mps_node`. Voice cmds, joystick, ball detections игнорируются.

### Roles
- **@razdryzg-dev:** всё в `pi_nodes/`, `compute_node/dashboard/routers/mps.py`, `compute_node/mps_runner.py`, `compute_node/dashboard/{state,mqtt_handlers,schemas/mps}.py`, `config.yaml` секция, pytest. ~45% объёма.
- **@OneAstr0:** всё в `compute_node/frontend/src/`, vitest unit-тесты. ~55% объёма.

---

## 4. Backend (Pi + compute)

### 4.1 `pi_nodes/control/state_space_model.py`

```python
class StateSpaceModel:
    Ad: np.ndarray  # (n, n)  существует
    Bd: np.ndarray  # (n, r)  существует
    Cd: np.ndarray  # (k, n)  НОВОЕ, default = I_n (k=n=5)
    Dd: np.ndarray  # (k, r)  НОВОЕ, default = 0_{kxr}

    def reload(self, Ad=None, Bd=None, Cd=None, Dd=None) -> None:
        """Атомарная замена матриц с валидацией shape."""

    def output(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """y = Cd·x + Dd·u — для UI телеметрии, не для управления."""

    def is_stable(self) -> bool:
        """|λ(Ad)| < 1 для всех λ."""
```

### 4.2 `pi_nodes/control/mpc_controller.py`

```python
class MPCController:
    def rebuild(self, *, Ad=None, Bd=None, Q_diag=None, R_diag=None,
                N=None, u_min=None, u_max=None, Pf=None) -> None:
        """Пересборка Phi/Gamma/H/K_first при смене модели/весов/горизонта.
        Сохраняет solver mode. Если Pf не задан — solve_discrete_are."""
```
Метод дёргается через MQTT `mps/matrices/set` — **между** прогонами, не внутри `step()`.

### 4.3 `pi_nodes/nodes/mps_node.py` (новая нода)

**Subscribed:**
- `samurai/{id}/mps/matrices/set` → распарсить `MpsMatrices` → `model.reload(...)` + `mpc.rebuild(...)` → publish ack `mps/matrices/applied`.
- `samurai/{id}/mps/scenario/run` → распарсить `MpsScenarioRequest`. Pre-validate (D, v_target, u_max[1] limits). Если ОК — FSM → `DRIVE_FORWARD_MPS`, начать tick-loop. Если нет — publish error.
- `samurai/{id}/mps/scenario/abort` → emergency stop, FSM → IDLE, последние 3 cmd_vel = [0, 0].

**Published (50 Hz во время прогона):**
- `samurai/{id}/mps/telemetry` (QoS 0) → `MpsTelemetryPoint + run_id`.
- `samurai/{id}/mps/scenario/finished` (QoS 1) → `MpsScenarioResult` без telemetry, только метрики.
- `samurai/{id}/mps/error` (QoS 1) → `{run_id, error_type, message}` если NaN/instability.

**Tick logic (`tick_dt=0.02`):**
```
loop:
  x_meas = position_fusion.get_state()  # уже существует
  u = mpc.step(x_meas, x_ref=[s_ref(t), v_target, 0, 0, 0])
  motor_node.send_cmd_vel(v=u[0], omega=u[1])
  publish_telemetry(...)
  if s_meas >= D - 0.05 or t > 3*D/v_target: finish; break
  if any(NaN in u) or |x|>bound: error; break
```

### 4.4 FSM (`pi_nodes/nodes/fsm_node.py`)
- Новое state `DRIVE_FORWARD_MPS`.
- Вход — только из IDLE через MQTT `mps/scenario/run`.
- Выход — в IDLE через `mps/scenario/finished` или `mps/scenario/abort`.
- В этом state FSM игнорирует: voice intents, ball detections, joystick. Только `abort` команда переключает state.
- ControllerFactory смотрит на FSM-state: `DRIVE_FORWARD_MPS` → `mode=mpc`, иначе → `mode=off` (или legacy PID, в зависимости от config).

### 4.5 `compute_node/mps_runner.py` (новый)

```python
def run_scenario_idealized(
    matrices: MpsMatrices,
    request: MpsScenarioRequest,
    dt: float = 0.02,
    max_steps: int = 10_000,
) -> MpsScenarioResult:
    """Идеальная модель — крутит x[k+1]=A·x+B·u, никакого шума.
    Использует тот же MPCController.step что и Pi.
    Возвращает MpsScenarioResult с telemetry и metrics."""
```

Запускается синхронно в обработчике REST `/api/v1/mps/scenario/run` (source=sim) — для D=2м, dt=0.02 это ~100ms wall-clock.

### 4.6 REST роутер `compute_node/dashboard/routers/mps.py`

См. §7 — полная таблица endpoints.

### 4.7 `state.mps` (`compute_node/dashboard/state.py`)

```python
@dataclass
class MpsState:
    matrices: MpsMatrices                           # current applied
    draft: Optional[MpsMatrices] = None             # unsaved UI changes
    history: deque[MpsScenarioResult] = ...         # maxlen=20
    active_run: Optional[MpsScenarioResult] = None
    last_robot_telemetry: list[MpsTelemetryPoint] = ...  # ring N=200
```

### 4.8 `config.yaml` — секция `mps:`

```yaml
mps:
  enabled: true
  matrices:
    A: [[...], ...]    # 5x5 — берётся из matlab/main.m export
    B: [[...], ...]    # 5x2
    C: [[...], ...]    # 5x5 default I
    D: [[...], ...]    # 5x2 default 0
  weights:
    Q_diag: [10, 10, 5, 1, 1]
    R_diag: [1, 1]
  horizon_N: 10
  limits:
    u_min: [-0.30, -2.0]
    u_max: [+0.30, +2.0]
  scenario:
    distance_max: 5.0          # safety cap
    v_target_max: 0.30
    omega_max_in_forward: 0.5  # rad/s — жёсткое ограничение в forward-сценарии
    default_distance: 2.0
    default_v_target: 0.15
  history_size: 20
```

---

## 5. Сценарий «проехать D метров вперёд»

### 5.1 Состояние и управление

```
x = [s, v, θ, ω, e_int]ᵀ ∈ ℝ⁵
    s     — пройденная дистанция, м
    v     — продольная скорость, м/с
    θ     — курс, рад
    ω     — угловая скорость, рад/с
    e_int — интеграл ошибки скорости

u = [v_cmd, ω_cmd]ᵀ ∈ ℝ²
y = C·x + D·u (для UI; default y = x)
```

> **Обновление 2026-05-14:** `MpsMatrices.A/B` несут НЕПРЕРЫВНЫЕ матрицы
> `A_c/B_c` (бэкенд ZOH-дискретизирует); `ė_int` переопределён как `−θ`
> (интеграл ошибки курса) — прежнее `v_target − v` делало модель
> неуправляемой. Актуальная модель — в
> `docs/superpowers/specs/2026-05-14-mps-continuous-discretization-design.md`.

x_meas — из `pi_nodes/filters/position_fusion.py` (режим `complementary` или `ekf` по конфигу).

### 5.2 Reference

```
x_ref(k) = [s_ref(k), v_target, 0, 0, 0]ᵀ
s_ref(k) = min(D, k·dt·v_target)
```

### 5.3 Cost-функция (стандартная для MPC из §3 latex_doc)

```
J = Σ_{i=0}^{N-1} ((xᵢ−x_ref)ᵀQ(xᵢ−x_ref) + uᵢᵀRuᵢ) + (x_N−x_ref)ᵀPf(x_N−x_ref)
s.t.  x_{k+1} = Ad·xₖ + Bd·uₖ
      u_min ≤ uₖ ≤ u_max
```

Pf — solve_discrete_are(A, B, Q, R).

### 5.4 Lifecycle прогона

```
IDLE → RUNNING ─┬─ s_meas≥D−ε ──→ FINISHED(reached) → IDLE
                ├─ t>3D/v_target → FINISHED(timeout) → IDLE
                ├─ abort cmd ────→ FINISHED(aborted) → IDLE
                └─ NaN/error ────→ FINISHED(error)   → IDLE
```

При FINISHED на роботе: `motor_node` шлёт cmd_vel = [0,0] трижды (failsafe).

### 5.5 Метрики (вычисляются по концу прогона)

| Метрика | Формула | Единица |
|---|---|---|
| overshoot | max(s(t)) − D | м |
| settling_time | first t where \|s(t)−D\| < 0.02 (sustained) | с |
| control_energy | Σₖ uᵢᵀRuᵢ · dt | unitless |
| ss_error | \|s(t_end) − D\| | м |
| peak_v | max\|v(t)\| | м/с |
| peak_omega | max\|ω(t)\| | рад/с |

### 5.6 Safety

1. **Pre-validate:** `0 < D ≤ 5.0`, `0 < v_target ≤ 0.30`, `u_max[1] ≤ 0.5` рад/с. Нарушение → 400 на REST, error на MQTT.
2. **Watchdog:** если `mps_node` не получает odometry 3 тика подряд → abort.
3. **FSM lock:** в `DRIVE_FORWARD_MPS` игнорируем всё кроме abort.
4. **Boundary check:** `|x_meas − x_predicted| > threshold` → publish warning (не abort, решение через UI).
5. **u clip:** всегда после `step()`, дублируется в `motor_node.config.motor.limits`.

---

## 6. Frontend (`compute_node/frontend/src/`)

### 6.1 Стек
- React + TypeScript + Vite (как в проекте).
- **Recharts** — для x(t), u(t), y(t).
- **2D SVG** для top-down trajectory и unit-circle eigenvalues — без Three.js в MVP.
- **WebSocket** для Live mode на `/ws/mps/telemetry`.
- Existing UI primitives из `compute_node/frontend/src/components/ui/`.

### 6.2 Layout (страница `/mps`)

Две колонки. Левая (40%) = редактор + контролы. Правая (60%) = графики и визуализация.

```
Left:                              Right:
- MatrixEditor                     - ScenarioControls (D, v_target, Run, Abort, Sim/Robot toggle)
- DraftStatus                      - ResultPlots (tabs: x/u/y/s)
- Apply / Validate / Reset         - EigenvaluePanel
- HistoryPanel + Compare           - TrajectoryView (2D top-down)
                                   - TuningSliders (collapsible)
```

### 6.3 Компоненты `compute_node/frontend/src/components/mps/`

| Файл | Назначение |
|---|---|
| `MatrixEditor.tsx` | Inline edit grid для A/B/C/D + векторы Q/R/u_min/u_max + N. Валидация: число / NaN / shape. Подсветка диффа vs applied. |
| `ScenarioControls.tsx` | Поля D, v_target, кнопки Run/Abort, тумблер Sim ↔ Robot. |
| `ResultPlots.tsx` | Recharts tabbed (x/u/y/s) графики, multi-line, легенда, hover. |
| `EigenvaluePanel.tsx` | SVG: единичная окружность + точки λ(Ad) и λ(A−B·K). Цветом устойчивость. |
| `TrajectoryView.tsx` | 2D top-down SVG: трек робота по `x(t)`, target D, текущая позиция. |
| `TuningSliders.tsx` | Слайдеры Q-diag, R-diag, N → onChange = re-simulate в sim → ResultPlots обновляется. Кнопка «Promote to draft». |
| `HistoryPanel.tsx` | Список последних 20 прогонов: run_id, ts, params summary, sparkline. Compare checkbox → ResultPlots overlay. Replay. |
| `DraftStatus.tsx` | Badge: «applied» / «draft (unsaved)» / «invalid» / «unstable». |
| `ValidationBadge.tsx` | Список eigenvalues, max\|λ\|, флаг устойчивости, overshoot/settling из validate. |
| `LiveTelemetryHook.tsx` | (no-render) ws → state. |

### 6.4 Hooks

| Файл | Назначение |
|---|---|
| `useMpsMatrices.ts` | GET/POST `/matrices`, draft state, dirty-tracking. |
| `useMpsRun.ts` | run scenario, polling статуса, cancel. |
| `useMpsHistory.ts` | history + replay. |
| `useMpsValidate.ts` | вызов Validate + кэш. |
| `useMpsLiveTelemetry.ts` | WS `/ws/mps/telemetry` → буфер N=200. |

### 6.5 Поведение

- **Edit без Apply** — ResultPlots не обновляется (старый прогон), DraftStatus = «draft».
- **Validate** — POST с draft (или applied), бэк → 2-сек step-response в sim → результат + λ. EigenvaluePanel + ValidationBadge обновляются.
- **Run on Sim** — sync, ~100ms, графики + history запись.
- **Run on Robot** — async POST → run_id → live update через WS → finished в history.
- **TuningSliders** — отдельная локальная копия Q/R/N, sim re-run на каждый change. «Promote to draft» переносит в MatrixEditor.
- **Compare** — выбор 2-3 в HistoryPanel → ResultPlots overlay.

### 6.6 Тесты (Vitest)
- Unit: validation в MatrixEditor, форматирование, dirty-tracking.
- Snapshot: компоненты с моки-данными.
- Без e2e.

---

## 7. Контракты REST / MQTT / WebSocket

`docs/mps/api.md` фиксирует это в **Day 0**. Дальше @razdryzg-dev и @OneAstr0 работают параллельно.

### 7.1 Pydantic-схемы (`compute_node/dashboard/schemas/mps.py`)

```python
class MpsMatrices(BaseModel):
    A: list[list[float]]   # 5×5
    B: list[list[float]]   # 5×2
    C: list[list[float]]   # 5×5 default I
    D: list[list[float]]   # 5×2 default 0
    Q_diag: list[float]    # 5
    R_diag: list[float]    # 2
    horizon_N: int         # >= 1
    u_min: list[float]     # 2
    u_max: list[float]     # 2
    schema_version: str = "1.0"

class MpsScenarioRequest(BaseModel):
    distance: float        # 0 < D <= 5.0
    v_target: float        # 0 < v <= 0.30
    source: Literal["sim", "robot"]

class MpsTelemetryPoint(BaseModel):
    t: float
    x: list[float]         # 5
    u: list[float]         # 2
    y: list[float]         # k = rows of C
    s_remaining: float

class MpsScenarioResult(BaseModel):
    run_id: str
    started_at: datetime
    finished_at: datetime | None
    status: Literal["running", "reached", "timeout", "aborted", "error"]
    request: MpsScenarioRequest
    matrices_snapshot: MpsMatrices
    telemetry: list[MpsTelemetryPoint]
    metrics: MpsMetrics | None

class MpsMetrics(BaseModel):
    overshoot: float
    settling_time: float
    control_energy: float
    ss_error: float
    peak_v: float
    peak_omega: float

class MpsValidateResult(BaseModel):
    eigenvalues_ad: list[ComplexNumber]      # |.|<1 ⇒ stable plant
    eigenvalues_closed: list[ComplexNumber]  # A − B·K_first
    is_plant_stable: bool
    is_closed_loop_stable: bool
    step_response: list[MpsTelemetryPoint]
    warnings: list[str]
```

TS-зеркало в `compute_node/frontend/src/types/mps.ts` — синхронно по структуре.

### 7.2 REST endpoints `/api/v1/mps/*`

| Method | Path | Body | Response | Notes |
|---|---|---|---|---|
| GET | `/matrices` | — | `{applied: MpsMatrices, draft: MpsMatrices?}` | |
| POST | `/matrices` | `MpsMatrices` | `{status: "draft", matrices}` | сохраняет в `state.mps.draft` |
| POST | `/matrices/apply` | — | `{status: "applied", matrices}` | publish MQTT, сбрасывает draft |
| POST | `/matrices/reset` | — | `{status: "applied", matrices}` | загрузка дефолта из `config.yaml` |
| POST | `/validate` | `MpsMatrices?` | `MpsValidateResult` | пустой body → валидирует draft |
| POST | `/scenario/run` | `MpsScenarioRequest` | `{run_id}` | sim — sync; robot — async (см. WS) |
| GET | `/scenario/{run_id}` | — | `MpsScenarioResult` | poll для robot |
| POST | `/scenario/abort` | — | `{aborted: true, run_id}` | |
| GET | `/history` | — | `list[MpsScenarioResult]` | последние 20 |
| POST | `/history/{run_id}/replay` | — | `{run_id_new}` | new run с теми же параметрами |
| POST | `/config/save` | — | `{written: true, path}` | dump applied матриц в `config.yaml` |

Status codes: 200 ok / 400 invalid / 409 conflict (run in progress) / 503 robot offline.

### 7.3 MQTT-топики (`samurai/{robot_id}/mps/`)

| Topic | Direction | Payload | QoS |
|---|---|---|---|
| `mps/matrices/set` | compute → Pi | `MpsMatrices` | 1 |
| `mps/matrices/applied` | Pi → compute | `{matrices, applied_at}` | 1 |
| `mps/scenario/run` | compute → Pi | `MpsScenarioRequest + run_id` | 1 |
| `mps/scenario/abort` | compute → Pi | `{run_id}` | 1 |
| `mps/scenario/finished` | Pi → compute | `MpsScenarioResult` без telemetry | 1 |
| `mps/telemetry` | Pi → compute | `MpsTelemetryPoint + run_id` | 0 |
| `mps/error` | Pi → compute | `{run_id, error_type, message}` | 1 |

QoS 0 для телеметрии (50 Гц, потеря 1-2 точек ОК); QoS 1 для control commands.

### 7.4 WebSocket `/ws/mps/telemetry`

```
client → server: {action: "subscribe", run_id?: string}
server → client: {run_id, point: MpsTelemetryPoint}
server → client: {run_id, status: "finished", result: MpsScenarioResult}
```

Сервер буферизует последние 200 точек активного прогона (для подключения посреди run'а).

### 7.5 Контрактные правила

1. `schema_version` во всех payload'ах — bump при несовместимых изменениях.
2. `Idempotency-Key` для POST `/matrices/apply` и `/scenario/run` — повтор с тем же ключом = тот же `run_id` (использует существующий security-middleware).
3. MQTT publish failure → REST 503 + UI banner.

---

## 8. Тестирование

### 8.1 Backend (@razdryzg-dev)

`tests/test_state_space_extended.py`:
- `reload()` атомарно меняет матрицы; неверный shape → ValueError.
- `output()` корректен на единичных Cd, Dd.
- `is_stable()` true на дефолте, false на λ=1.5.

`tests/test_mpc_controller.py` (расширение):
- `rebuild()` пересчитывает Phi/Gamma/H/K_first; не теряет solver mode.
- После rebuild — `is_stable()` true.

`tests/test_mps_node.py`:
- testcontainers Mosquitto;
- publish `mps/matrices/set` → ack `mps/matrices/applied`;
- publish `mps/scenario/run` → телеметрия → finished status="reached";
- abort посреди прогона → status="aborted", cmd_vel=[0,0];
- voice cmd во время прогона игнорируется;
- safety-limits (D=10) → pre-validate fails.

`tests/test_mps_router.py`:
- FastAPI TestClient;
- draft/apply lifecycle;
- `/validate` возвращает eigenvalues;
- `/scenario/run` source=sim → результат с metrics;
- `/scenario/run` source=robot когда нет connection → 503;
- `/history` после 3 запусков.

`tests/test_mps_runner.py`:
- идеальный sim: `x[k+1]=Ax+Bu` совпадает с ручным расчётом для 5 шагов;
- D=2, v_target=0.15, дефолтные → `status="reached"`, `ss_error<0.05`;
- неустойчивые матрицы → `status="error"`.

Coverage target: 85% для нового кода.

### 8.2 Frontend (@OneAstr0)

`compute_node/frontend/src/__tests__/`:
- `MatrixEditor.test.tsx` — валидное/невалидное число, dirty-tracking, reset.
- `ResultPlots.test.tsx` — рендер 4 табов, переключение.
- `EigenvaluePanel.test.tsx` — точки внутри/вне круга, цвет.
- `useMpsRun.test.ts` — POST → polling → finished; abort.
- `useMpsLiveTelemetry.test.ts` — mock WS, точки в буфер.
- `mpsApi.test.ts` — mock fetch, endpoints.

Vitest + Testing Library. Без e2e.

---

## 9. План работ — порядок и зависимости

### Day 0 (совместно)
1. Заполнить `docs/mps/api.md` — JSON-schemas, REST endpoints, MQTT topics, WS — **блокер для всего**.
2. Pydantic-схемы `compute_node/dashboard/schemas/mps.py` + TS-типы `compute_node/frontend/src/types/mps.ts` — **блокер для трека B**.

### Track A — @razdryzg-dev (backend / Pi)

| # | Шаг | Зависит от |
|---|---|---|
| A1 | `state_space_model.py` — Cd, Dd, reload(), output() + тесты | Day 0 |
| A2 | `mpc_controller.py` — rebuild() + тесты | A1 |
| A3 | `compute_node/mps_runner.py` + тесты | A2 |
| A4 | `compute_node/dashboard/routers/mps.py` все endpoints + тесты (mock MQTT) | A3 |
| A5 | `pi_nodes/nodes/mps_node.py` + FSM-state + тесты с testcontainers | A2 |
| A6 | `compute_node/dashboard/state.py` + `mqtt_handlers.py` wiring | A4, A5 |
| A7 | `config.yaml` секция `mps:` с дефолтами из `matlab/main.m` | A2 |
| A8 | WebSocket `/ws/mps/telemetry` | A6 |
| A9 | `docs/mps/architecture.md` + `scenario_forward.md` | — |
| A10 | smoke-test на реальном Pi (если железо доступно) | A5 |

### Track B — @OneAstr0 (frontend)

| # | Шаг | Зависит от |
|---|---|---|
| B1 | Базовая страница `/mps` + routing + layout | Day 0 |
| B2 | `MatrixEditor.tsx` + `useMpsMatrices.ts` + тесты | B1 + (A4 или msw мок) |
| B3 | `ScenarioControls.tsx` + `useMpsRun.ts` + тесты | B2 + A4 |
| B4 | `ResultPlots.tsx` (Recharts, 4 таба) + тесты | B3 |
| B5 | `EigenvaluePanel.tsx` (SVG + λ) | B4 |
| B6 | `TrajectoryView.tsx` (2D top-down SVG) | B4 |
| B7 | `HistoryPanel.tsx` + `useMpsHistory.ts` + Compare mode | B4 |
| B8 | `TuningSliders.tsx` + sync sim | B7 |
| B9 | Live mode (WebSocket) | A8 |
| B10 | DraftStatus, ValidationBadge, polish | B9 |

**Параллельность:** до A4 фронт работает с msw-моками. После A4 — реальный бэк.

### Definition of Done для каждого PR

- [ ] CI зелёный (pytest + ruff + tsc + vitest);
- [ ] coverage не упал ниже 85% для трогнутых модулей;
- [ ] обновлён `docs/mps/TODO.md`;
- [ ] есть скриншот UI или curl-пример REST в описании PR;
- [ ] нет «.removed»-комментариев и закомменченного кода;
- [ ] SECURITY.md scope не нарушен.

### Финальный merge

1. PR @razdryzg-dev и @OneAstr0 — целятся в `feat/mps`.
2. Когда оба трека Done — squash-merge `feat/mps` → `dev` через PR с описанием всей фичи.
3. `dev` → `main` отдельным PR — стандарт проекта.
4. После merge в `main` — обновить `docs/mps/TODO.md` (Done section) + обновить memory.

---

## 10. Backlog extension'ов (вне MVP)

- 3D трек робота (react-three-fiber).
- Шумы / возмущения в симуляторе (motor lag, slip, наклон).
- Observer Луенбергера для оценки x по y (тогда C, D реально работают).
- Persistence history в SQLite.
- Export CSV / PDF отчёт.
- Bearer auth для `/api/v1/mps/*`.
- Cruise control (бесконечный сценарий с v_ref, без D).
- Playwright e2e тесты для frontend.

---

## 11. Открытые вопросы / риски

1. **5-мерный state совпадает с PositionFusion?** @razdryzg-dev верифицирует в Day 0 — если e_int не предоставляется фильтром, добавляем интегратор в `mps_node`.
2. **Hot-reload во время прогона**: текущий дизайн запрещает. Если соавторы захотят — обсуждаем после MVP.
3. **`config_save` и race conditions**: если два пользователя одновременно жмут Save — последний выиграет. Достаточно для учебного MVP.
4. **Версионирование matlab/-генерируемых матриц**: при изменении `matlab/main.m` нужно регенерить `config.yaml` секцию. README в `matlab/` это уже описывает.
