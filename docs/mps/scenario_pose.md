# MPS-сценарий «Pose-tracking»

> Текущий и единственный сценарий MPS-модуля. Заменил `scenario_forward.md`
> 2026-05-17 (см. [спека](../superpowers/specs/2026-05-17-mps-pose-tracking-design.md)).
> Реализация: `pi_nodes/control/mps_reference.py` (генератор `r(t)`),
> `compute_node/mps_runner.py` (sim), `pi_nodes/nodes/mps_node.py` (robot).
> Контракт payload-ов — [api.md](./api.md).

---

## 1. Что делает сценарий

Робот доезжает в точку `(D, 0)` локального фрейма старта (X-ось локального
фрейма = курс робота в момент `_on_scenario_run`), а затем разворачивается
на месте к финальному курсу `φ = target_heading`.

В коде один LQR/MPC закрывает петлю по tracking-error
$\mathbf{e}(t) = \mathbf{x}(t) - \mathbf{r}(t)$, где
$\mathbf{r}(t) = [s, v, \theta, \omega, e_\text{int}]$ — feedforward-референс.

«Фаз» в коде нет (старый `_tick_turn` / `_tick_drive` выпилены) — есть
один tick `_tick_run`, который зовёт `mpc.step(x, x_ref=traj.r(run.t))`.

---

## 2. Параметры запроса

| Поле | Тип | Диапазон | Смысл |
|---|---|---|---|
| `distance` | float, м | `(0, 5.0]` | Перемещение по X локального фрейма старта |
| `v_target` | float, м/с | `(0, 0.30]` | Крейс-скорость в drive-сегменте |
| `target_heading` | float, рад | `[-π, π]` | **Финальный** курс после прибытия в `(D, 0)` |
| `source` | enum | `sim` \| `robot` | Где гонять |

`MpsScenarioRequest` — `compute_node/dashboard/schemas/mps.py`.

---

## 3. Опорная траектория `r(t)`

Состоит из двух сегментов (если оба нетривиальны):

### Drive `t ∈ [0, t_drive]`

Трапец по `v`: разгон до `v_target` с `a_max=0.20 м/с²`, крейс, симметричный
тормоз до 0. Триангуляр при `D < v_target² / a_max`.

- `s_ref(0) = 0`, `s_ref(t_drive) = D` (точно).
- `v_ref(0) = v_ref(t_drive) = 0`, между — трапец.
- `θ_ref = 0` (относительно θ_start), `ω_ref = 0`.

### Turn `t ∈ [t_drive, t_end]`

Трапец по `ω`: разгон до `ω_max=1.0 рад/с` с `α_max=1.0 рад/с²`, крейс,
тормоз до 0. Триангуляр при `|φ| < ω_max² / α_max`.

- `s_ref = D` (фиксирован), `v_ref = 0`.
- `θ_ref(t_drive) = 0`, `θ_ref(t_end) = phi_signed` (relative).
- `ω_ref` — соответствующий трапец-профиль (signed).

### Граничные случаи

- `D = 0` → пропускаем drive (`t_drive = 0`), сценарий = чистый turn.
- `φ = 0` (или `|φ| ≤ 1e-9`) → пропускаем turn (`t_turn = 0`), сценарий = чистый drive.
- `t > t_end` → `r(t)` возвращает финальную точку `[D, 0, phi_signed, 0, 0]`.

Источник правды формул: спека §3.2. Тесты: `tests/test_mps_reference.py`.

---

## 4. Финиш по 4 координатам

После `t ≥ t_end` MPC продолжает удерживать референс. Каждый тик проверяем
**одновременно** (AND по всем):

```
|s_meas − D|             < ε_s   (default 0.005 м = 5 мм)
|v_meas|                 < ε_v   (default 0.02 м/с)
|θ_meas − phi_signed|    < ε_θ   (default 0.05 рад ≈ 2.9°)
|ω_meas|                 < ε_ω   (default 0.05 рад/с)
```

Конфиг: `mps.scenario.reach_tolerance_m` + `mps.scenario.reach.*`.

| Событие | Условие | `status` |
|---|---|---|
| Достижение цели | все 4 ε выполнены при `t ≥ t_end` | `reached` |
| Settling timeout | `t > t_end + settle_timeout_s` (default 1.5 c) | `timeout_settle` |
| Run timeout | `t > 1.5·t_end + 2.0` ещё до выхода в settling | `timeout` |
| Принудительный abort | MQTT `mps/scenario/abort` | `aborted` |
| Расходимость / NaN | `|x| > bound` или `u, x` не finite | `error` |

`timeout_settle` всегда сопровождается detail-строкой какая координата не
сошлась: `'settle timeout: |s−D|=0.012, |v|=0.001, |θ_err|=0.003, |ω|=0.001'`.

---

## 5. Состояние и управление

Не меняется относительно предыдущего сценария:

$$
\mathbf{x} = \begin{bmatrix} s \\ v \\ \theta \\ \omega \\ e_\text{int} \end{bmatrix} \in \mathbb{R}^5,
\qquad
\mathbf{u} = \begin{bmatrix} v_\text{cmd} \\ \omega_\text{cmd} \end{bmatrix} \in \mathbb{R}^2.
$$

| Компонент | Что | Источник |
|---|---|---|
| `s` | Дистанция от старта, м | `position_fusion.x_wheel` (relative) |
| `v` | Продольная скорость, м/с | `position_fusion.vx` |
| `θ` | Курс отн. старта, рад | `position_fusion.theta - theta_start` |
| `ω` | Угловая скорость, рад/с | `position_fusion.omega` |
| `e_int` | Интеграл курсовой ошибки | `mps_node._tick_run` накапливает `∫(−θ_err) dt` |

---

## 6. Smoke-тесты на железе

Прогнать на роботе и убедиться:

| `D` | `v_target` | `φ` | Ожидаемое `t_end` | `s_final` | `θ_final` |
|---|---|---|---|---|---|
| 0.30 | 0.15 | π | ≈ 9.5 c ± 1 | 0.295…0.305 | π ± 0.05 |
| 0.30 | 0.15 | 0 | ≈ 2.75 c ± 0.5 | 0.295…0.305 | 0 ± 0.05 |
| 0 | 0.15 | π/2 | ≈ 2.6 c ± 0.5 | 0 ± 0.005 | π/2 ± 0.05 |

(`s_final` целится в 5 мм; на железе с шумным dead-reckoning можно
ослабить в `config.yaml` через `mps.scenario.reach_tolerance_m: 0.02`.)

---

## 7. Что было до этого

Раньше `target_heading` значил **«куда разворачиваемся ПЕРЕД движением»**
(2-фазный сценарий `_tick_turn` → `_tick_drive`), и финиш стопил за 2 см
до D (`reach_tolerance_m = 0.02` без trapezoidal decel в референсе).

С 2026-05-17 (pose-tracking refactor):
- `target_heading` = **финальный курс ПОСЛЕ прибытия в (D, 0)**.
- Один MPC с feedforward-референсом (нет «фаз»).
- Финиш по 4 координатам, ε_s = 5 мм.
- Сим и Pi гоняют байт-идентичный `r(t)` через общий модуль
  `pi_nodes/control/mps_reference.py`.

Старая телеметрия (без полей `r/x_local/y_local`) по-прежнему рендерится
в `TrajectoryView.tsx` через fallback `(s·cosθ, s·sinθ)`. Интерпретация
`target_heading` в старых прогонах сменилась — но контракт данных тот же.
