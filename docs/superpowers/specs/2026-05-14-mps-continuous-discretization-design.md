# МПС — Непрерывный контракт матриц + ZOH-дискретизация в бэкенде

**Branch:** `fix/mps-continuous-discretization` (от `fix/camera-flip` — текущий tip; `dev` отстал на ~50 коммитов и не содержит релевантных config/redesign-фиксов)
**Date:** 2026-05-14
**Status:** approved (brainstorm 2026-05-14)
**Связанные спеки:** `2026-05-05-mps-state-space-design.md` (главная спека модуля МПС), `2026-05-06-mps-ui-redesign-design.md` (редизайн UI, где появилась непрерывная каноническая форма)

---

## 1. Цель и не-цели

### Цель

Устранить рассогласование непрерывных/дискретных матриц в модуле МПС: фронтенд
генерирует и применяет **непрерывные** канонические матрицы `A_c/B_c`, а бэкенд
(`mps_runner`, `/validate`, `mps_node`) потребляет их как дискретные `Ad/Bd` без
ZOH-шага. Результат — расходящийся замкнутый контур при любых весах `Q/R/N`.
После фикса сценарий «проехать D метров вперёд» устойчиво работает в симуляторе
и на роботе.

### Не-цели

- Не меняем легаси-контроллеры (`lqr_controller.py`, modal) и их вектор состояния
  `[px, py, θ, v, ω]` — секция `control:` живёт как есть.
- Не переписываем `MPCController` / `StateSpaceModel` — они остаются дискретными
  по контракту (принимают `Ad/Bd`).
- Не делаем единую модель состояния на весь проект (отброшено на brainstorm).
- Не меняем REST/MQTT/WS-эндпоинты и формы payload'ов — меняется только
  **семантика** полей `A/B` (непрерывные вместо дискретных) + docstring.
- Не переписываем UI-редизайн МПС по сути — учебная идея «студент правит
  ОДУ-модель» сохраняется и становится корректной.

---

## 2. Корень проблемы (диагностика на живой системе)

Подтверждено прямым запросом к работающему дашборду (`:5000`, PID 2768).

### 2.1 `applied`-матрицы непрерывные

`GET /api/v1/mps/matrices` вернул:

```
A[1][1] = −9.0909… = −1/τ_v   (τ_v = 0.11)
A[3][3] = −10      = −1/τ_ω   (τ_ω = 0.10)
B[1][0] =  9.0909… =  1/τ_v
B[3][1] = 10       =  1/τ_ω
```

Это генератор ОДУ — непрерывная форма, сгенерированная фронтовым
`buildCanonical(τ_v, τ_ω)` (см. `2026-05-06` спека §3, §6).

### 2.2 Контракт требует дискретные

`compute_node/dashboard/schemas/mps.py`:

```python
A: ... description='5×5 — дискретная матрица состояния Ad'
B: ... description='5×2 — дискретная матрица управления Bd'
```

Главная спека §5.3: `x_{k+1} = Ad·xₖ + Bd·uₖ`.

### 2.3 ZOH-дискретизации нет нигде

Между «матрицы заданы» и «MPC/симулятор их потребляют» дискретизации нет:

- `mps_runner._build_controller` — `StateSpaceModel(Ad=A, Bd=B)`,
  `MPCController(Ad=A, Bd=B)` — сырьём.
- `mps_runner.closed_loop_eigenvalues` — `eigvals(A)`, `eigvals(A − B·K_first)` — сырьём.
- `mps_node._on_matrices_set` — `plant.reload(Ad=A, Bd=B)`,
  `mpc.rebuild(Ad=A, Bd=B)` — сырьём.
- `MPCController._build_qp_matrices` — степени `Ad^i`, `matrix_power(Ad, ·)` —
  однозначно дискретная трактовка.

### 2.4 Доказательство — `POST /validate`

- `eigenvalues_ad = [0, 0, −9.09, 0, −10]` → `is_plant_stable: false`
  (критерий `|λ|<1` применён к непрерывным полюсам; −9.09 и −10 для непрерывной
  системы устойчивы).
- `eigenvalues_closed = [−20.64, 1.337±4.85i, −1.127, ~0]`
  → `is_closed_loop_stable: false` (положительная вещественная часть).
- `step_response`: ω скачет `0 → 4.107 → −36.96` за два тика 20 мс — классика
  «непрерывную матрицу крутят как дискретное отображение».

### 2.5 Сопутствующее

- `config.yaml mps.matrices` — устаревшая ручная копия `control.matrices`
  (дискретные, для **легаси**-модели `[px, py, θ, v, ω]`), т.е. другая модель
  целиком. Комментарий «копия из control.matrices» вводит в заблуждение.
- `matlab/export_to_yaml.m` пишет только секцию `control:` — секцию `mps:`
  MATLAB не генерирует никогда.
- Рассогласование Ts: `OdeCard` показывает «Ts = 50 мс», реальный MPS-цикл —
  20 мс (`mps.tick_dt: 0.02`, `mps_runner dt=0.02`).

### 2.6 Выявлено при реализации (2026-05-14)

Два бага вскрылись при синтезе регулятора на канонической модели — план их
не предусмотрел:

1. **`MPCController.__init__` подменяет вычисленный гейн.** Конструктор строит
   QP из переданных `Ad/Bd`, считает корректный `K_first`, а в конце —
   **перетирает его** значением `control.matrices.K_mpc` из `config.yaml`
   (гейн legacy-модели `[px,py,θ,v,ω]`). Аналогично `Pf` подхватывается из
   `control.matrices.Pf`, а `_compute_terminal_penalty` читает
   `control.weights`. Любой `MPCController(Ad=другая_модель, ...)` молча
   получает чужой регулятор. Доказано: `K_first[0,:3]` после `__init__` =
   `[2.80934, 0, 0]` — ровно `control.matrices.K_mpc`. Бьёт по `mps_runner`.
2. **Каноническая `ė_int = v_target − v` делает модель неуправляемой.**
   `ė_int = −v` и `ṡ = +v` ⇒ `s + e_int = const` — сохраняющаяся
   неуправляемая мода на λ=1 (ctrb rank 4/5). `e_int` (интеграл ошибки
   скорости) математически тождественен `s_ref − s`. DARE для такой модели
   не решается. **Решение (выбор пользователя): переопределить
   `ė_int = s_ref − s`** — интеграл ошибки **позиции**, `A_c[4][0]=−1`
   вместо `A_c[4][1]=−1`. Модель становится полностью управляемой
   (rank 5/5), DARE решается, замкнутый контур строго устойчив
   (проверено: max|λ| ≈ 0.993).

---

## 3. Решения (приняты на brainstorm 2026-05-14)

| # | Развилка | Решение |
|---|---|---|
| 1 | Кто дискретизирует | **Бэкенд.** Контракт `MpsMatrices.A/B` → непрерывные `A_c/B_c`. |
| 2 | Охват | **Полный**, включая MATLAB-пайплайн. |
| 3 | MATLAB и две модели | **Генерировать обе:** legacy `[px,py,θ,v,ω]` → `control:`, каноническая `[s,v,θ,ω,e_int]` → `mps:`. |
| 4 | Критерий приёмки | **Sim + проверка на роботе.** |
| 5 | Состояние `e_int` (выявлено при реализации) | **`ė_int = s_ref − s`** (∫ ошибки позиции, `A_c[4][0]=−1`). Старое `ė_int = −v` делало модель неуправляемой. |
| 6 | `MPCController.__init__` (выявлено при реализации) | `control.matrices.K_mpc`/`Pf` из конфига — **только на legacy-пути** (когда `Ad/Bd` не переданы явно); `_compute_terminal_penalty` использует `self.Q/self.R`. |

---

## 4. Контракт и единый Ts

### 4.1 Смена контракта

`MpsMatrices.A` и `MpsMatrices.B` — **непрерывные** матрицы `A_c` (5×5),
`B_c` (5×2) канонической ОДУ-модели. `C/D` без изменений. Обновить:

- docstring полей `A/B` в `compute_node/dashboard/schemas/mps.py`;
- `docs/mps/api.md` (контракт);
- пометку в главной спеке МПС (`2026-05-05-mps-state-space-design.md` §5.3,
  §7.1) о смене семантики `A/B`.

### 4.2 Каноническая непрерывная модель

```
x = [s, v, θ, ω, e_int]ᵀ,  u = [v_cmd, ω_cmd]ᵀ

ṡ     = v
v̇     = −(1/τ_v)·v + (1/τ_v)·u_v
θ̇     = ω
ω̇     = −(1/τ_ω)·ω + (1/τ_ω)·u_ω
ė_int = s_ref − s          (s_ref — reference, в A_c/B_c не входит)
```

`A_c`: `[0][1]=1, [1][1]=−1/τ_v, [2][3]=1, [3][3]=−1/τ_ω, [4][0]=−1`, остальное 0.
`B_c`: `[1][0]=1/τ_v, [3][1]=1/τ_ω`, остальное 0.

> **Решение #5 (см. §2.6):** `ė_int = s_ref − s` (интеграл ошибки **позиции**),
> т.е. `A_c[4][0]=−1`. Прежний вариант `ė_int = v_target − v` (`A_c[4][1]=−1`)
> делал модель неуправляемой (`s + e_int = const`). С `A_c[4][0]=−1` модель
> полностью управляема, DARE решается, замкнутый контур строго устойчив.

### 4.3 Единый Ts

`Ts_mps = 0.02 с` (50 Гц) — единственный источник правды, кладётся в
`config.yaml mps.plant.Ts`. Используется: ZOH-дискретизацией, sim-loop'ом
`mps_runner`, tick-loop'ом `mps_node`, MATLAB-веткой, подписью `OdeCard`.
`mps.tick_dt` сводится к `mps.plant.Ts` или проверяется на равенство при старте
(см. Open question 1).

---

## 5. Бэкенд (Python)

### 5.1 Хелпер дискретизации

`pi_nodes/control/state_space_model.py`: `_zoh_discretize` → публичный
`zoh_discretize(A, B, Ts) -> (Ad, Bd)` (реализация уже есть — блочная матричная
экспонента). `StateSpaceModel` и `MPCController` **не меняются** — продолжают
принимать дискретные `Ad/Bd`, их существующие тесты остаются валидны.

### 5.2 Точки потребления

| Файл | Функция | Изменение |
|---|---|---|
| `compute_node/mps_runner.py` | `_build_controller` | `Ad,Bd = zoh_discretize(m.A, m.B, Ts)` перед `StateSpaceModel`/`MPCController` |
| `compute_node/mps_runner.py` | `closed_loop_eigenvalues` | дискретизировать перед `eigvals`; возвращать `λ(Ad)`, `λ(Ad−Bd·K_first)` |
| `pi_nodes/nodes/mps_node.py` | `_on_matrices_set` | `Ad,Bd = zoh_discretize(A, B, Ts)` перед `plant.reload`/`mpc.rebuild`; лог `eigvals` считать по `Ad` |

`Ts` берётся из `cfg('mps.plant.Ts', 0.02)`.

### 5.3 Критерий устойчивости в `/validate`

`compute_node/dashboard/routers/mps.py::validate`:

- `eigenvalues_ad` — теперь `λ(Ad)` (после дискретизации) → критерий `|λ|<1`
  снова корректен; единичная окружность в `EigenvaluePanel` остаётся правильной
  визуализацией, UI не трогаем.
- Нюанс: каноническая модель даёт 3 полюса `Ad` ровно на `|λ|=1` (интеграторы
  `s, θ, e_int`) — это **маргинальная устойчивость, не баг**. `is_plant_stable`
  смягчить: различать «маргинально устойчив (полюса на окружности)» и
  «неустойчив (вне)». Реальный гейт сценария — `is_closed_loop_stable`.

---

## 6. MATLAB + config.yaml (генерируем обе модели)

### 6.1 `matlab/build_canonical_mps.m` (новый)

Строит непрерывные `A_c, B_c, C, D` для `[s,v,θ,ω,e_int]` аналитически из
`τ_v, τ_w` по паттерну §4.2. Якобианы (`linearize_samurai`) не нужны.
`C = I₅`, `D = 0₅ₓ₂` (как в текущем `config.yaml mps.matrices`).

### 6.2 `matlab/samurai_params.m`

Добавить под-структуру `p.mps`: `Ts = 0.02`, `Q_mps`, `R_mps` (диагонали для
порядка `[s,v,θ,ω,e_int]` — **отличается** от легаси `[px,py,θ,v,ω]`!),
`N_mps`, `u_min/u_max`. Дефолты — текущие значения из `config.yaml mps`:
`Q_mps = diag([10,10,5,1,1])`, `R_mps = diag([1,1])`, `N_mps = 10`,
`u_min = [-0.30,-2.0]`, `u_max = [0.30,2.0]`.

### 6.3 `matlab/main.m`

После легаси-ветки — ветка MPS: `build_canonical_mps` →
`discretize_samurai(A_c, B_c, p.mps.Ts)` → `design_mpc` (проверка устойчивости
+ график переходного) → запись `mps:`.

### 6.4 `matlab/export_to_yaml.m`

Расширить на два блока. **Асимметрия (явно задокументировать в шапке функции):**

- `control.matrices.A/B` — пишем **дискретными** `Ad/Bd` (как сейчас);
- `mps.matrices.A/B` — пишем **непрерывными** `A_c/B_c` (новый контракт).

Добавить маркеры `# === mps: section auto-generated by matlab/main.m ===` /
`# === end of auto-generated mps: block ===` для идемпотентной замены (тот же
приём, что для `control:`). Для `mps:` экспортируем `matrices.{A,B,C,D}`,
`plant.{tau_v,tau_w,Ts}`, `weights`, `horizon_N`, `limits` — **без K**
(Python пересчитывает `K_first` онлайн).

### 6.5 `config.yaml`

Секция `mps:`:

- `matrices.A/B` → непрерывная каноническая форма (совпадёт с текущим `applied`
  на живом дашборде — live-состояние станет валидным без переприменения);
- добавить `mps.plant: {tau_v, tau_w, Ts: 0.02}`;
- убрать комментарий-миф «копия из control.matrices».

Можно засеять вручную аналитическими значениями сразу (для независимой проверки
Python-бэкенда), MATLAB затем перегенерирует идентично.

---

## 7. Фронтенд + доки

- **Фронт — минимально.** Уже шлёт непрерывные матрицы → теперь это контрактно
  корректно. Правки: подпись Ts в
  `compute_node/frontend/src/components/mps/OdeCard.tsx` (50 → 20 мс, лучше —
  из API/конфига, а не хардкод). Опционально — пометка «A/B непрерывные,
  бэкенд ZOH-дискретизирует».
- **Доки:** `docs/mps/api.md` (контракт `A/B`), краткое описание непрерывной
  канонической модели + ZOH в `docs/mps/`.

---

## 8. Тесты (TDD)

### Правим под новый контракт

- `tests/test_mps_runner.py` — тесты, хардкодящие дискретные `A/B`, переводим
  на непрерывные + ожидаемую дискретизацию.
- `tests/test_mps_node.py` — `_on_matrices_set` теперь дискретизирует.
- `tests/test_mps_router.py` — `/validate` возвращает `λ(Ad)`.
- `matlab/test_export_to_yaml.m` — проверка записи `mps:`-блока.

### Новые

- Корректность `zoh_discretize` (сверка с аналитикой для простых случаев).
- `/validate`: непрерывная каноническая модель → `is_closed_loop_stable: true`
  после фикса; нюанс маргинальной устойчивости открытого контура.
- Экспорт `mps:`-блока идемпотентен; `mps.matrices.A/B` записаны непрерывными.
- `build_canonical_mps` → правильная структура матриц.
- Кросс-проверка: MATLAB-MPC и Python-MPC дают одинаковый `K_first` на одной
  модели (защита от расхождения двух реализаций).

---

## 9. Верификация и критерии приёмки

### Sim (на ноутбуке)

- `POST /api/v1/mps/scenario/run` source=sim, `D=2 м`, `v_target=0.15`
  → `status: reached`.
- `POST /api/v1/mps/validate` → `is_closed_loop_stable: true`, все
  `|λ(Ad−Bd·K)| < 1`.
- Метрики: `overshoot < ~10%·D`, `ss_error < 0.05 м`, `settling_time` конечный,
  `step_response` не расходится.
- Если дефолтные `Q=[10,10,5,1,1], R=[1,1], N=10` не дают такие метрики —
  **подобрать Q/R/N** (исходный запрос «подобрать коэффициенты чтобы ехал
  вперёд»).

### Robot

- `POST /api/v1/mps/scenario/run` source=robot → прогон через MQTT на реальном
  Samurai, `status: reached`, робот физически едет вперёд.
- Зависит от живого железа + Pi + поднятого `mps_node` — отдельный шаг плана,
  допускается отложить в `physical_tests_pending.md`.

---

## 10. Файлы

### Новые

- `matlab/build_canonical_mps.m`
- `docs/superpowers/specs/2026-05-14-mps-continuous-discretization-design.md` (этот документ)
- `tests/test_zoh_discretization.py` (при необходимости — иначе расширяем существующие)

### Изменяемые

- `pi_nodes/control/state_space_model.py` — публичный `zoh_discretize`
- `pi_nodes/control/mpc_controller.py` — `__init__`: `control.matrices.K_mpc`/`Pf` только на legacy-пути; `_compute_terminal_penalty` использует `self.Q/self.R` (см. §2.6 баг 1)
- `compute_node/mps_runner.py` — `_build_controller`, `closed_loop_eigenvalues`
- `pi_nodes/nodes/mps_node.py` — `_on_matrices_set`, лог
- `compute_node/dashboard/routers/mps.py` — `/validate` критерий
- `compute_node/dashboard/schemas/mps.py` — docstring `A/B`
- `matlab/samurai_params.m` — `p.mps`
- `matlab/main.m` — ветка MPS
- `matlab/export_to_yaml.m` — запись `mps:`-блока
- `config.yaml` — секция `mps:` (непрерывная каноническая `A_c[4][0]=−1` + `mps.plant`)
- `compute_node/frontend/src/components/mps/OdeCard.tsx` — подпись Ts + уравнение `ė_int`
- `compute_node/frontend/src/lib/mps/canonical.ts` — `CANONICAL_PATTERN_A` ячейка `e_int`: col 1→0
- `compute_node/frontend/src/lib/mps/tokenMap.ts` — токен `coef_eint_v` → `coef_eint_s` (A[4][0])
- `docs/mps/api.md` — контракт
- `tests/test_mps_runner.py`, `tests/test_mps_node.py`, `tests/test_mps_router.py`,
  `tests/test_mpc_controller.py`, `matlab/test_export_to_yaml.m`

---

## 11. Риски и митигации

| Риск | Импакт | Митигация |
|---|---|---|
| MATLAB-MPC и Python-MPC расходятся в `K_first` | средний | `mps:` пишет только A/B/weights (без K), Python считает онлайн; кросс-проверочный тест |
| Существующие тесты хардкодят дискретные `A/B` | средний | TDD: правим тесты в составе работы, гоняем полный прогон |
| Рассогласование Ts между компонентами | средний | единый источник `mps.plant.Ts`; ассерт `tick_dt == plant.Ts` на старте |
| `is_plant_stable: false` для канонической модели смущает | низкий | нюансированное предупреждение «маргинально устойчив (интеграторы)» |
| Robot-верификация зависит от железа | низкий | отдельный шаг плана, допускается отложить |
| `/config/save` мог записать старые дискретные матрицы | низкий | регенерируем дефолты; документируем смену контракта |

---

## 12. Open questions (решить на implementation)

1. `mps.plant.Ts` vs `mps.tick_dt` — оставить оба (с ассертом равенства) или
   свести к одному полю? Рекомендация: `mps.plant.Ts` — параметр дискретизации,
   `tick_dt` либо удаляем, либо проверяем равенство на старте.
2. Хватает ли дефолтных `Q/R/N` для метрик приёмки, или нужен подбор — выяснится
   на первом sim-прогоне после фикса.
3. Нужна ли явная UI-пометка «непрерывные A/B» — решить при правке `OdeCard`.

---

## 13. Объём и порядок

> **Обновление 2026-05-14:** этапы 1-3 ниже выполнены (коммиты `24210ef`,
> `ddd7741`, `65e799f`, `b855747`). При реализации выявлены 2 бага (см. §2.6)
> → добавлен корректирующий этап «`e_int` → ∫ ошибки позиции + фикс
> `MPCController`». Актуальный таск-лист — в
> `docs/superpowers/plans/2026-05-14-mps-continuous-discretization.md`.

| Этап | Содержание | Зависит от |
|---|---|---|
| 1 | `zoh_discretize` публичный + тест | — |
| 2 | Бэкенд: 3 точки потребления + `/validate` + docstring схемы + тесты | 1 |
| 3 | `config.yaml mps:` — засеять непрерывную каноническую форму вручную | — (параллельно) |
| 4 | Sim-верификация: `reached` + `is_closed_loop_stable` + метрики; подбор `Q/R/N` при необходимости | 2, 3 |
| 5 | MATLAB: `build_canonical_mps` + `samurai_params` + `main` + `export_to_yaml` + тест | 3 |
| 6 | Фронтенд (`OdeCard` Ts) + доки | 2 |
| 7 | Robot-верификация | 4, 5 (+ железо) |

Реализация — ветка `fix/mps-continuous-discretization`. По завершении — merge
в актуальную рабочую ветку (`fix/camera-flip` или `dev`, в зависимости от того,
куда раньше дойдёт `fix/camera-flip`) + `git push`.
