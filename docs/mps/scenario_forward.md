# Сценарий «проехать D метров вперёд»

> Формальное описание единственного сценария в MVP. Реализация —
> `compute_node/mps_runner.py` (sim) и `pi_nodes/nodes/mps_node.py` (robot).
> Контракт payload-ов — [api.md](./api.md). Архитектура — [architecture.md](./architecture.md).

---

## 1. Постановка задачи

Робот едет прямо вперёд из текущей позиции на расстояние $D$ метров с
целевой скоростью $v_\text{target}$. Управляет MPC-регулятор по модели
пространства состояний из учебника В.Н. Козлова.

Сценарий **финитный** — заканчивается одним из четырёх событий:

| Событие | Условие | `status` |
|---|---|---|
| Достижение цели | $s \ge D - 0.05$ м | `reached` |
| Timeout | $t > 3 \cdot D / v_\text{target}$ | `timeout` |
| Принудительный abort | MQTT `mps/scenario/abort` | `aborted` |
| Расходимость / NaN | $\|x\| > $ bound или $u, x$ не finite | `error` |

---

## 2. Состояние и управление

$$
\mathbf{x} = \begin{bmatrix} s \\ v \\ \theta \\ \omega \\ e_\text{int} \end{bmatrix} \in \mathbb{R}^5,
\qquad
\mathbf{u} = \begin{bmatrix} v_\text{cmd} \\ \omega_\text{cmd} \end{bmatrix} \in \mathbb{R}^2.
$$

| Компонент | Что | Откуда |
|---|---|---|
| $s$ | Пройденная дистанция, м | `position_fusion.x_wheel` (или EKF) |
| $v$ | Продольная скорость, м/с | `position_fusion.vx` |
| $\theta$ | Курс, рад | `position_fusion.theta` |
| $\omega$ | Угловая скорость, рад/с | `position_fusion.omega` |
| $e_\text{int}$ | Интеграл ошибки скорости | накапливается в `mps_node` |

Вектор $u$ публикуется в MQTT `cmd_vel` → `motor_node`.

**Выход модели** (только для UI визуализации):
$$
\mathbf{y}(k) = C_d \mathbf{x}(k) + D_d \mathbf{u}(k).
$$
По умолчанию $C_d = I_5$, $D_d = 0$, поэтому $y \equiv x$.

---

## 3. Reference и cost-функция

```
s_ref(k) = min(D, k · dt · v_target)
x_ref(k) = [s_ref(k), v_target, 0, 0, 0]ᵀ
```

Стандартный MPC-критерий из главы 3 latex_doc:

$$
J = \sum_{i=0}^{N-1} \big( (\mathbf{x}_i - \mathbf{x}_{\text{ref}})^T Q (\mathbf{x}_i - \mathbf{x}_{\text{ref}}) + \mathbf{u}_i^T R \mathbf{u}_i \big) + (\mathbf{x}_N - \mathbf{x}_{\text{ref}})^T P_f (\mathbf{x}_N - \mathbf{x}_{\text{ref}})
$$

ограничения:

$$
\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k, \qquad
\mathbf{u}_{\min} \le \mathbf{u}_k \le \mathbf{u}_{\max}.
$$

Терминальный штраф $P_f$ — решение DARE для $(A_d, B_d, Q, R)$.

---

## 4. Lifecycle

```
        IDLE ─── mps/scenario/run ───→ RUNNING (DRIVE_FORWARD_MPS)
                                          │
                                          ├── s ≥ D − 0.05    →  reached
                                          ├── t > 3·D/v_target →  timeout
                                          ├── abort команда   →  aborted
                                          └── NaN / |x|>bound →  error
                                                  ↓
                                              FINISHED
                                                  │
                                                  └→ IDLE (cmd_vel = [0,0] × 3)
```

`motor_node` шлёт zero cmd_vel трижды как failsafe — даже если
`mps_node` упадёт в этот момент, мотор остановится по своему собственному
`cmd_vel_timeout` (см. `config.yaml: odometry.cmd_vel_timeout`).

---

## 5. Метрики

Считаются по концу прогона из `telemetry`. Все вычисления — синхронные.

| Метрика | Формула | Единица |
|---|---|---|
| `overshoot` | $\max(s(t)) - D$, обрезано $\ge 0$ | м |
| `settling_time` | первое $t$ при котором $|s(t) - D| < 0{.}02$ устойчиво до конца | с |
| `control_energy` | $\sum_k \mathbf{u}_k^T R \mathbf{u}_k \cdot \Delta t$ | unitless |
| `ss_error` | $|s(t_\text{end}) - D|$ | м |
| `peak_v` | $\max\|v(t)\|$ | м/с |
| `peak_omega` | $\max\|\omega(t)\|$ | рад/с |

---

## 6. Safety

1. **Pre-validate** (REST + Pi):
   - $0 < D \le 5{.}0$ м (`mps.scenario.distance_max`),
   - $0 < v_\text{target} \le 0{.}30$ м/с (`mps.scenario.v_target_max`),
   - $u_\text{max}[1] \le 0{.}5$ рад/с — жёсткое ограничение в forward-сценарии.
   - Нарушение → 400 на REST; на Pi → publish `mps/error`
     (`error_type="precondition"`), сценарий не стартует.

2. **Watchdog**: если `mps_node` не получает odometry > 3 тика подряд
   → status='error', `error_type="watchdog"`.

3. **FSM lock**: в `DRIVE_FORWARD_MPS` игнорируются voice cmds, ball
   detections, joystick. Только `mps/scenario/abort` переключает state.

4. **u clip** — после `mpc.step()`, дублируется в
   `motor_node.config.motor.limits`.

5. **Failsafe stop**: при `FINISHED` любого статуса → `cmd_vel = [0, 0]`
   трижды + FSM → `IDLE`.

---

## 7. Конфигурация дефолтов

См. секцию `mps:` в [`config.yaml`](../../config.yaml). Реальные
дефолты для дистанции и скорости:

```yaml
mps:
  scenario:
    default_distance: 2.0
    default_v_target: 0.15
```

UI отрисовывает эти значения в `ScenarioControls.tsx`. Reset кнопка в
MatrixEditor возвращает все матрицы к этим дефолтам через
`POST /matrices/reset`.

---

## 8. Тестовая зрелость

| Уровень | Файл тестов | Что проверяет |
|---|---|---|
| Unit (sim) | [tests/test_mps_runner.py](../../tests/test_mps_runner.py) | Идеальная propagation, reach, instability, метрики |
| Unit (router) | [tests/test_mps_router.py](../../tests/test_mps_router.py) | REST контракты, валидация, history, replay, WS |
| Unit (Pi node) | [tests/test_mps_node.py](../../tests/test_mps_node.py) | matrices/set, scenario/run, abort, watchdog |
| Расширение | [tests/test_state_space_extended.py](../../tests/test_state_space_extended.py) | Cd, Dd, reload(), output() |
| Расширение | [tests/test_mpc_controller.py](../../tests/test_mpc_controller.py) | rebuild() — горизонт, веса, plant, rollback |

Coverage таргет 85% — фактически measured отдельно (см. CI).
Smoke-test на реальном Pi (A10) выполняется руками — отдельный чек-лист.
