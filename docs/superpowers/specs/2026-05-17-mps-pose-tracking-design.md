# МПС — pose-tracking сценарий с feedforward-референсом

> Сценарий «доехать в `(D, 0)` локального фрейма старта и развернуться
> на финальный курс `φ` после прибытия». Один LQR/MPC + опорная
> траектория `r(t)` как функция времени; в коде нет понятия «фаза» —
> всё закрывается одним tick-обработчиком, который зовёт
> `mpc.step(x, x_ref=r(t))`.
>
> **Базируется на:** `dev` (после `feat/mps-target-picker`).
> Переиспользует существующую каноническую модель `[s, v, θ, ω, e_int]`
> и `MPCController` без переписывания математики.
>
> **Owners:** @razdryzg-dev (Pi + sim + schemas), @OneAstr0 (UI). PR-ы
> только от них. Без Co-Authored-By Claude / упоминаний AI в коммитах.
>
> **Скоуп:** `pi_nodes/control/`, `pi_nodes/nodes/mps_node.py`,
> `compute_node/mps_runner.py`, `compute_node/dashboard/{routers,schemas}/mps*`,
> `compute_node/frontend/src/components/mps/*`, `config.yaml`,
> `docs/mps/`. Без изменений в `state_space_model.py` и `mpc_controller.py`.

## 1. Контекст и проблема

Текущий сценарий (`pi_nodes/nodes/mps_node.py`) — две фазы:

1. **TURN на месте** к `target_heading` φ;
2. **DRIVE** прямо `D` метров вдоль нового курса.

`x_ref[v] = v_target` константой до самого финиша, замедления нет. Условие
«доехал» — `s_meas ≥ D − reach_tolerance_m` с дефолтом 0.02 м, поэтому
робот всегда останавливается за 2 см до целевой длины. При `D=0.30` это
0.28 м — наблюдаемое поведение в скриншоте.

Дополнительно: визуализация `TrajectoryView.tsx` проецирует точку как
`(s·cos θ, s·sin θ)`, что не является честной интеграцией дифф-привода
при меняющемся θ, и таргет нарисован в `(D, 0)` экрана независимо от φ —
получается противоречие между «куда робот должен был приехать» (по текущей
семантике это `(D·cos φ, D·sin φ)` локального фрейма) и зелёным кружком на
чарте `(D, 0)`.

Пользователь хочет: **робот всегда оказывается в `(D, 0)` локального
фрейма** (т.е. «вперёд на D» от позы старта), а `target_heading φ` — это
**финальный курс после прибытия**, не направление движения.

## 2. Решения брейншторма (2026-05-17)

| # | Вопрос | Решение |
|---|---|---|
| 1 | Где цель в пространстве? | `(D, 0)` в **локальном фрейме старта** (X-локальное = курс робота в момент `_on_scenario_run`). |
| 2 | Что значит `target_heading φ`? | **Финальный курс после прибытия** в `(D, 0)`. Робот сначала едет, затем разворачивается. |
| 3 | Чем закрываем «доехать-потом-развернуться»? | **B + feedforward-референс**. Один MPC, без `phase`-полей; всё в форме `r(t)`. |
| 4 | Почему не «pure-B» с константным `x_ref`? | LQR минимизирует сумму взвешенных ошибок параллельно — при `Q[s]≈Q[θ]` и `D=0.3, φ=π` ошибка курса доминирует (`Q·π² ≫ Q·D²`), робот начинает разворот сразу. Это противоречит «после прибытия». |
| 5 | Какой профиль скорости? | Трапец по `v` (с треугольным fallback при коротком `D`). Аналогично — трапец по `ω` для turn-сегмента. |
| 6 | Сколько ε для финиша? | По всем 4-м координатам одновременно: `s, v, θ, ω`. Дефолты `0.005 м / 0.02 м/с / 0.05 рад / 0.05 рад/с`. |

## 3. Архитектура

### 3.1 Структурное

```
┌──────────────────────────────────────────────────────┐
│ Compute (mps_runner.py)                              │
│   build_reference(...) ──┐                           │
│   mps.step(x, r(t))      │                           │
└──────────────────────────┼───────────────────────────┘
                           │  (один и тот же модуль)
┌──────────────────────────▼───────────────────────────┐
│ pi_nodes/control/mps_reference.py  (NEW)             │
│   build_reference(D, v_target, φ, a_max, α_max,      │
│                   v_max, ω_max) → ReferenceTrajectory│
│   ReferenceTrajectory.r(t) → np.ndarray[5]           │
└──────────────────────────┬───────────────────────────┘
                           │
┌──────────────────────────▼───────────────────────────┐
│ Pi (pi_nodes/nodes/mps_node.py)                      │
│   on _on_scenario_run: traj = build_reference(...)   │
│   tick: r = traj.r(run.t); u = mpc.step(x, r)        │
│   no _tick_turn / _tick_drive — один _tick_run       │
└──────────────────────────────────────────────────────┘
```

`mps_reference.py` — единый источник истины траектории. Pi и compute
делят символ; sim и робот гоняют по байт-идентичному `r(t)`.

### 3.2 Опорная траектория `r(t) = [s, v, θ, ω, e_int]`

Параметры:
- `D` — дистанция (м), `D ≥ 0`.
- `v_max = v_target` — крейс-скорость (м/с), `0 < v_max ≤ v_target_max`.
- `φ = target_heading` — финальный курс (рад), `φ ∈ [-π, π]`.
- `a_max = mps.scenario.reference.a_max` (дефолт `0.20 m/s²`).
- `α_max = mps.scenario.reference.alpha_max` (дефолт `1.0 rad/s²`).
- `ω_max = mps.scenario.omega_max_in_turn` (дефолт `1.0 rad/s`).
- `θ_start` — курс одометрии на момент `_on_scenario_run`.

#### Drive-сегмент `t ∈ [0, t_drive]`

Cruise возможен только если `v_max² / a_max ≤ D`. Иначе профиль
треугольный с `v_peak = √(D · a_max) < v_max`.

**Trapezoidal case (`D ≥ v_max² / a_max`):**
```
t_acc    = v_max / a_max
s_acc    = ½ · v_max² / a_max
s_cruise = D − 2·s_acc
t_cruise = s_cruise / v_max
t_drive  = 2·t_acc + t_cruise
```
- `t ∈ [0, t_acc]`:               `v_ref = a_max·t`,                       `s_ref = ½·a_max·t²`
- `t ∈ [t_acc, t_acc + t_cruise]`: `v_ref = v_max`,                          `s_ref = s_acc + v_max·(t − t_acc)`
- `t ∈ [t_acc + t_cruise, t_drive]`: `τ = t_drive − t`, `v_ref = a_max·τ`, `s_ref = D − ½·a_max·τ²`

**Triangular case (`D < v_max² / a_max`):**
```
v_peak   = √(D · a_max)
t_acc    = v_peak / a_max
t_drive  = 2·t_acc
```
- `t ∈ [0, t_acc]`:        `v_ref = a_max·t`,         `s_ref = ½·a_max·t²`
- `t ∈ [t_acc, t_drive]`:  `τ = t_drive − t`, `v_ref = a_max·τ`, `s_ref = D − ½·a_max·τ²`

Во всём drive-сегменте: `θ_ref = θ_start`, `ω_ref = 0`.

#### Turn-сегмент `t ∈ [t_drive, t_end]`

Δφ = `_normalize_angle(φ)` (берём кратчайший знаковый угол). Делаем
аналогичный трапец по `ω`:

**Trapezoidal (`|Δφ| ≥ ω_max² / α_max`):**
```
t_acc_ω    = ω_max / α_max
θ_acc      = ½ · ω_max² / α_max
θ_cruise   = |Δφ| − 2·θ_acc
t_cruise_ω = θ_cruise / ω_max
t_turn     = 2·t_acc_ω + t_cruise_ω
```
с `ω_ref = sign(Δφ) · profile`. `θ_ref` интегрируется из `ω_ref` относительно
`θ_start`.

**Triangular (`|Δφ| < ω_max² / α_max`):**
```
ω_peak  = √(|Δφ| · α_max)
t_acc_ω = ω_peak / α_max
t_turn  = 2·t_acc_ω
```

Во всём turn-сегменте: `s_ref = D`, `v_ref = 0`.

`t_end = t_drive + t_turn`.

`e_int_ref ≡ 0` всегда (интегратор MPC сам гонит статическую ошибку курса
в ноль).

#### Граничные случаи

- `D = 0`: `t_drive = 0`, drive пропущен. Сценарий — чистый turn.
- `φ = 0` или `|φ| < ε_θ`: `t_turn = 0`, сценарий — чистый drive.
- `D = 0, φ = 0`: тривиальный сценарий, `t_end = 0`, finish сразу
  (после первого tick'а в settling-окне).

`r(t > t_end)` возвращает финальную точку `[D, 0, θ_start + φ, 0, 0]` —
полезно для settling-окна (см. §4).

### 3.3 Tick-обработчик

Удаляется `_RunState.phase`, `_tick_turn`, `_tick_drive`. Один
`_tick_run`:

```python
def _tick_run(self, run: _RunState):
    with self._lock:
        x = self._x_meas.copy()
    r = run.traj.r(run.t)
    try:
        u = self._mpc.step(x, x_ref=r)
    except Exception as exc:
        self._finish_run('error', f'mpc.step: {exc}')
        return
    if not (np.all(np.isfinite(u)) and np.all(np.isfinite(x))):
        self._finish_run('error', 'NaN/Inf in u or x')
        return
    # Hard cap: в drive-сегменте режем |ω_cmd| ≤ omega_max_in_forward
    if abs(r[1]) > 1e-3:  # v_ref ≠ 0 → drive
        u[1] = max(-self._omega_max_fwd, min(self._omega_max_fwd, u[1]))
    # Интегратор курсовой ошибки (как сейчас, но против текущего r[2])
    self._accumulate_eint(theta_err=_normalize_angle(x[_THETA] - r[_THETA]))
    self._publish_cmd_and_telemetry(run, x, u, r=r)
    if self._check_finish(x, r, run): return
    run.t += self._tick_dt
```

`_check_finish` — см. §4. `_publish_cmd_and_telemetry` дополнительно
кладёт `r` и `x_local`/`y_local` в телеметрию (см. §5).

## 4. Финиш и edge-кейсы

### 4.1 Условие «доехал»

После `t ≥ t_end` MPC продолжает работать с финальным `r`. Каждый тик
проверяем:
```
θ_target = _normalize_angle(θ_start + φ)
θ_err    = _normalize_angle(θ_meas − θ_target)        # wrap в [-π, π]

finished = (|s_meas − D| < ε_s) AND
           (|v_meas|     < ε_v) AND
           (|θ_err|      < ε_θ) AND
           (|ω_meas|     < ε_ω)
```
Все 4 одновременно. Дефолты: `ε_s = 0.005 м`, `ε_v = 0.02 м/с`,
`ε_θ = 0.05 рад`, `ε_ω = 0.05 рад/с`. Wrap угла обязателен — иначе на
границе `φ = π` ошибка может улететь на `~2π` из-за расхождения веток
arctan'а.

### 4.2 Settling timeout

Если `t ≥ t_end + settle_timeout_s` (дефолт `1.5 с`) и
условие не сошлось — `finish('timeout_settle', detail)` где `detail`
указывает какая(ие) из 4-х координат не сошлась(ись).

### 4.3 Run-timeout

Если `t > 1.5 · t_end` ещё до выхода в settling-окно — `finish('timeout',
detail='r(t) overshoot in time domain')`. Защита от подвиса (например,
залипший watchdog).

### 4.4 Watchdog одометрии

Оставляем как сейчас: `>3 тиков без свежей одометрии → finish('error',
'watchdog: no odom for >3 ticks')`. Без изменений в логике.

### 4.5 Hard-caps в выходных командах

- `|v_cmd| ≤ v_target_max` (дефолт `0.30 м/с`).
- В drive-сегменте `r[v] > 0`: `|ω_cmd| ≤ omega_max_in_forward`
  (дефолт `0.5 rad/s`).
- В turn-сегменте `r[v] = 0`: hard-cap НЕ накладывается — `|ω_cmd|`
  ограничен только `u_max[1]` из MPC-матриц (дефолт `2.0 rad/s` в
  `mps.matrices.u_max`). Дополнительный clip до `omega_max_in_turn`
  мешает «дотяжке» в settling-окне (см. Task 5 commit `5f3c4f4` —
  deviation D4: при cap=1.0 rad/s settling по θ не успевал в 1.5с при
  D=0.30, φ=π). Профиль референса всё равно строится с
  `omega_max = omega_max_in_turn`, так что cruise-фаза turn сидит
  на ≤1 rad/s; ускоренные дотяжки случаются только когда tracking-error
  не нулевой к концу референс-профиля.

### 4.6 Деградации профиля

См. §3.2 (triangular fallback, `D=0`, `φ=0`).

### 4.7 Mpc-saturation

На первом тике `r(0) = [0, 0, θ_start, 0, 0]` — нулевая трекинг-ошибка.
Реф плавно растёт по `r(t)`, поэтому MPC всегда видит небольшую ошибку и
не упирается в hard-cap по `u`. Это ключ к стабильности схемы (vs pure-B
с константным `x_ref`, где `s_err = D` с первого тика).

## 5. Данные

### 5.1 Контракт MQTT/REST

`POST /api/mps/scenario/run` body (без изменений в полях, но
обновляется docstring `target_heading`):
```json
{
  "distance": 0.30,
  "v_target": 0.15,
  "target_heading": 3.14159,   // final heading after arrival, rad
  "source": "robot"             // или "sim"
}
```

MQTT-топик `mps/scenario/run` payload расширяется — compute обязан
передать референс-параметры, иначе Pi и compute построят разные `r(t)`
и sim-prediction разойдётся с поведением робота:
```json
{
  "run_id": "...",
  "request": {...},
  "schema_version": "...",
  "reference": {                // NEW
    "a_max": 0.20,
    "alpha_max": 1.0
  }
}
```

### 5.2 Pydantic-схемы (`compute_node/dashboard/schemas/mps.py`)

```python
class MpsTelemetryPoint(BaseModel):
    t: float
    x: list[float]                              # 5-вектор измерения
    u: list[float]                              # 2-вектор управления
    e_y: float | None = None                    # как сейчас
    theta_err: float | None = None
    delta_theta: float | None = None
    r: list[float] | None = None                # NEW — 5-вектор референса
    x_local: float | None = None                # NEW — позиция в лок. фрейме старта
    y_local: float | None = None                # NEW

class MpsScenarioResult(BaseModel):
    ...
    status: Literal['reached', 'timeout', 'timeout_settle', 'error']
                                                # timeout_settle — NEW
```

Backward compat: новые поля опциональны (None), история прогонов
рендерится по-старому (см. §6).

### 5.3 Расчёт `x_local`, `y_local` на Pi

```python
dx = x_abs - x_start_abs
dy = y_abs - y_start_abs
cs = math.cos(theta_start)
sn = math.sin(theta_start)
x_local =  dx * cs + dy * sn
y_local = -dx * sn + dy * cs
```
(стандартное обратное вращение на `−θ_start`). Эти поля идут в каждую
телеметрическую точку.

## 6. UI

### 6.1 `TrajectoryView.tsx`

- **Путь** (синий): если телеметрия содержит `x_local, y_local` —
  рисуем напрямую. Иначе fallback на текущий `(s·cos θ, s·sin θ)` для
  обратной совместимости со старыми прогонами в History.
- **План** (серый штрих, `stroke-dasharray="4 3"`): в локальном фрейме
  старта план тривиален геометрически — отрезок `(0, 0) → (D, 0)` для
  drive-сегмента и точка `(D, 0)` для turn-сегмента (позиция не
  меняется, только курс). Поэтому рисуем **одну прямую линию**
  `(0, 0) → (D, 0)` пунктиром (не point-by-point из
  `(r[s]·cos r[θ], r[s]·sin r[θ])` — эта формула на турн-сегменте
  нарисовала бы дугу, что вводит в заблуждение). Дополнительно — короткая
  стрелка в точке `(D, 0)` под углом `φ`, показывающая финальный курс.
- **Target** (зелёный кружок): остаётся в `(D, 0)` — теперь это
  физически корректная локальная цель.
- **Current** (красный): последняя точка `(x_local, y_local)`.
- **Легенда:** `план / путь / target / текущая` (4 элемента).
- **Заголовок подписи:** «Локальный фрейм старта; план показан штрихом».

### 6.2 3D-обзор (`Mps3DScene`/`Mps3DOverlay`)

Аналогично — добавляем серую линию плана из `r(t)`. Если компонент 3D
ещё не умеет рисовать пунктир — рисуем тонкой сплошной полупрозрачной
линией (детали — на усмотрение @OneAstr0; UI-визуал не блокирует
поведение робота).

### 6.3 Форма запуска

Лейбл `target_heading` в форме запуска меняется:
`Финальный курс φ (рад)`, подпись:
«Куда робот будет смотреть после прибытия в `(D, 0)`. φ=0 → не
разворачивается; φ=π → разворачивается на 180° после доезда».

## 7. Config

### 7.1 `config.yaml` — секция `mps.scenario`

```yaml
mps:
  scenario:
    # Оставляем:
    distance_max: 5.0
    v_target_max: 0.30
    omega_max_in_forward: 0.5
    omega_max_in_turn: 1.0
    odom_max_age_s: 0.5
    e_int_max: 0.5

    # Меняем дефолт (был 0.02):
    reach_tolerance_m: 0.005          # ε_s

    # Новое: остальные ε и settling
    reach:
      epsilon_v: 0.02                 # m/s
      epsilon_theta: 0.05             # rad
      epsilon_omega: 0.05             # rad/s
      settle_timeout_s: 1.5

    # Новое: параметры референса
    reference:
      a_max: 0.20                     # m/s²
      alpha_max: 1.0                  # rad/s²

    # Lateral LQR — без изменений (см. existing)
    lateral:
      enabled: true
      tau_inner: 0.10
      Q_diag: [80.0, 30.0]
      R_diag: [1.0]
      delta_theta_max: 0.20
      v_min: 0.02
```

### 7.2 Deprecated ключи

`mps.scenario.turn_tolerance_rad`, `mps.scenario.turn_timeout_s` —
больше не используются (`_tick_turn` уходит). Читаются с
deprecation-warning в `mps_node.__init__` если присутствуют в
`config.yaml`. Удаляем через 1 релиз.

## 8. Тестирование

### 8.1 pytest (Pi-side)

Новые файлы под `tests/pi/control/`:
- `test_mps_reference_drive.py` — драйв-сегмент:
  - trapezoidal: `s_ref(t_drive) == D ± 1e-9`, `v_ref(0) == v_ref(t_drive) == 0`, монотонность `s_ref`, `v_ref ≤ v_max`.
  - triangular (`D < v_max²/a_max`): `v_peak == √(D·a_max)`, `s_ref(t_drive) == D`.
  - граничный `D=0`: `t_drive == 0`.
- `test_mps_reference_turn.py` — turn-сегмент:
  - trapezoidal, triangular, `φ=0` (skip turn), `φ=±π` (граница).
  - `θ_ref(t_turn) == θ_start + φ`, `ω_ref(t_turn) == 0`.
- `test_mps_reference_continuity.py` — на стыке `t = t_drive`: значения `r`
  слева и справа совпадают (`s=D`, `v=0`, `θ=θ_start`, `ω=0`).
- `test_mps_node_pose_arrival.py` — интеграционный с фейк-MQTT и
  идеальным фейк-одометром (просто интегрирует `u`): `D=0.3, v=0.15,
  φ=π` → status='reached', `|s_final − D| < ε_s`, `|θ_final − π| < ε_θ`.
- `test_mps_node_settle_timeout.py` — фейк-одометр с замедленной
  динамикой → `timeout_settle` с указанием не-сошедшейся координаты.

### 8.2 pytest (compute-side)

- `tests/compute_node/test_mps_runner_pose.py` — sim-прогон с теми же
  параметрами что и `test_mps_node_pose_arrival.py`. Финальный
  `MpsScenarioResult.status == 'reached'`. Проверяем что `mps_runner`
  использует тот же `build_reference` (импорт-смок).

### 8.3 vitest (frontend)

`compute_node/frontend/src/components/mps/__tests__/`:
- `TrajectoryView.test.tsx`:
  - С `x_local/y_local` в телеметрии → рендерит путь из них (snapshot).
  - Без новых полей → fallback на `(s·cos θ, s·sin θ)` (snapshot).
  - С `r` в телеметрии → виден серый штриховой план (assert по
    `path[stroke-dasharray]`).
  - Легенда содержит 4 элемента.

### 8.4 Smoke на железе

Документируется в `docs/mps/scenario_pose.md` (новый файл, заменяет
`scenario_forward.md` — старый удаляем):
- Прогон `D=0.30, v=0.15, φ=π`.
- Ожидаемое: `s_final ∈ [0.295, 0.305]`, `θ_final ∈ [π − 0.05, π + 0.05]`,
  время сценария ≈ `9.5 s ± 1 s`.
- Прогон `D=0.30, v=0.15, φ=0` (без разворота) — `≈ 2.75 s`.
- Прогон `D=0, v=0.15, φ=π/2` (только разворот) — `≈ 2.6 s`.

## 9. Миграция и совместимость

- **Семантика `target_heading`** меняется: было «куда разворачиваемся
  ПЕРЕД движением», стало «куда разворачиваемся ПОСЛЕ прибытия в
  `(D, 0)`». Это behavioural break для одного потребителя (UI).
- **История прогонов:** старая телеметрия без `r/x_local/y_local`
  рендерится через fallback в `TrajectoryView`. Запись `target_heading`
  в старых прогонах остаётся валидной (поле то же), но интерпретация
  сменилась — в `History`-вьюшке добавляем бейдж «pre-2026-05-17» для
  прогонов без `r` в телеметрии.
- **CHANGELOG.md:** запись «MPS: `target_heading` теперь = финальный
  курс ПОСЛЕ прибытия (раньше = курс ПЕРЕД движением). Замена
  поведения, не контракта». Дата релиза.
- **Конфиг:** старые `turn_tolerance_rad/turn_timeout_s` — deprecation
  warning в логе, удаление через релиз. `reach_tolerance_m` дефолт
  меняется с `0.02` на `0.005`.

## 10. Порядок реализации

Грубый порядок (детальный план — в writing-plans):

1. `pi_nodes/control/mps_reference.py` — pure функция + класс
   `ReferenceTrajectory`. Тесты §8.1 первые.
2. `compute_node/dashboard/schemas/mps.py` — поля `r/x_local/y_local`,
   статус `timeout_settle`.
3. `compute_node/mps_runner.py` — импорт `build_reference`, шаг по
   `r(t)`, проверка финиша.
4. `pi_nodes/nodes/mps_node.py` — удаление `_tick_turn/_tick_drive`,
   `_RunState.phase`; новый `_tick_run`; `_check_finish`, `x_local/y_local`
   в телеметрии. Deprecation warnings для старых config-ключей.
5. `compute_node/dashboard/routers/mps.py` — payload `reference: {...}`.
6. `config.yaml` — новая структура. Обновить
   `compute_node/config.yaml` (если есть отдельный).
7. `compute_node/frontend/src/components/mps/TrajectoryView.tsx` —
   `x_local/y_local`/`r`/легенда.
8. (Опц.) `Mps3DScene/Mps3DOverlay` — линия плана.
9. Лейбл `target_heading` в форме запуска.
10. `docs/mps/scenario_pose.md` (новый, заменяет существующий
    `docs/mps/scenario_forward.md` — старый файл удаляется в этом же PR).
11. `CHANGELOG.md`.

Pi (1-4, 6) — @razdryzg-dev. Frontend (7-9) — @OneAstr0. Compute API
(2-3, 5) — @razdryzg-dev. Доки (10-11) — кто закрывает Pi.
