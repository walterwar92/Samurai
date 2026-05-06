# МПС — API контракт

> Это **single source of truth** для контрактов между backend (Pi +
> compute_node/dashboard) и frontend (React UI). Любое breaking-изменение
> здесь требует bump-а `schema_version` в Pydantic-схемах
> [`compute_node/dashboard/schemas/mps.py`](../../compute_node/dashboard/schemas/mps.py)
> и в TS-зеркале [`compute_node/frontend/src/types/mps.ts`](../../compute_node/frontend/src/types/mps.ts).

**Текущая версия:** `1.0` (2026-05-05).

Полная архитектурная спека: [`docs/superpowers/specs/2026-05-05-mps-state-space-design.md`](../superpowers/specs/2026-05-05-mps-state-space-design.md).

---

## 1. Глоссарий

| Термин | Значение |
|---|---|
| **МПС** | **Модель Пространства Состояний** (state-space). В этой ветке — НЕ «методы принятия решений». |
| **MPC** | Model Predictive Control. Уже существует в `pi_nodes/control/mpc_controller.py`. |
| **Сценарий** | Финитный прогон «D метров вперёд со скоростью v_target». |
| **Draft** | Изменённые в UI матрицы, ещё не отправленные на робота. |
| **Applied** | Матрицы, активные сейчас в `mpc_controller`. |
| **Run** | Один прогон сценария (`run_id`, telemetry, metrics). |
| **Source** | `"sim"` (идеальный симулятор на ноуте) или `"robot"` (реальный Pi через MQTT). |
| **DRIVE_FORWARD_MPS** | Новое FSM-state на Pi. Только в нём контроллер == MPC. |

Размерности фиксированы: `x ∈ ℝ^5 = [s, v, θ, ω, e_int]`, `u ∈ ℝ^2 = [v_cmd, ω_cmd]`.

---

## 2. Pydantic-схемы (источник правды)

См. [`compute_node/dashboard/schemas/mps.py`](../../compute_node/dashboard/schemas/mps.py).
TS-зеркало: [`compute_node/frontend/src/types/mps.ts`](../../compute_node/frontend/src/types/mps.ts).

Ключевые модели:

- `MpsMatrices` — A/B/C/D + Q_diag/R_diag + horizon_N + u_min/u_max + schema_version.
  Валидация: shapes 5×5 / 5×2; `Q_diag ≥ 0`, `R_diag > 0`; `u_min < u_max` поэлементно.
- `MpsScenarioRequest` — distance (0..5), v_target (0..0.30), source (`"sim"`|`"robot"`).
- `MpsTelemetryPoint` — t, x[5], u[2], y[k], s_remaining.
- `MpsMetrics` — overshoot, settling_time, control_energy, ss_error, peak_v, peak_omega.
- `MpsScenarioResult` — run_id + status + telemetry[] + metrics + matrices_snapshot.
- `MpsValidateResult` — eigenvalues (Ad / closed-loop) + step_response + warnings.
- `ComplexNumber` — `{re, im}` JSON-сериализуемое комплексное число для собственных значений.

---

## 3. REST endpoints `/api/v1/mps/*`

Базовый префикс — `/api/v1/mps`. Все ответы JSON. Auth — открытые в MVP
(в backlog: Bearer-token через существующий `SAMURAI_DASHBOARD_TOKEN`).

### 3.1 Матрицы

| Method | Path | Body | Response | Status codes |
|---|---|---|---|---|
| GET | `/matrices` | — | `MpsMatricesGetResponse` (`{applied, draft?}`) | 200 |
| POST | `/matrices` | `MpsMatrices` | `MpsMatricesSetResponse` (`status="draft"`) | 200 / 400 |
| POST | `/matrices/apply` | `MpsMatrices?` (опционально — иначе apply draft) | `MpsMatricesSetResponse` (`status="applied"`) | 200 / 400 / 409 / 503 |
| POST | `/matrices/reset` | — | `MpsMatricesSetResponse` (`status="applied"` с дефолтом из `config.yaml`) | 200 |

`POST /matrices` сохраняет в `state.mps.draft` без публикации на робота.
`POST /matrices/apply` публикует MQTT `mps/matrices/set`, ждёт ack
(`mps/matrices/applied`) с timeout 2с, сбрасывает draft. Если робот offline —
**applied локально на dashboard** (для sim-режима), но возвращается
`status="applied"` с warning в логах.

### 3.2 Validate

| Method | Path | Body | Response |
|---|---|---|---|
| POST | `/validate` | `MpsMatrices?` (если null — валидируется draft, иначе applied) | `MpsValidateResult` |

Считает λ(Ad), λ(Ad − Bd·K_first), short step-response (2 сек, dt=0.02) в идеальном симуляторе. Никаких побочных эффектов.

### 3.3 Сценарий

| Method | Path | Body | Response | Status codes |
|---|---|---|---|---|
| POST | `/scenario/run` | `MpsScenarioRequest` | `MpsScenarioRunResponse` | 200 / 400 / 409 / 503 |
| GET | `/scenario/{run_id}` | — | `MpsScenarioResult` | 200 / 404 |
| POST | `/scenario/abort` | — | `MpsScenarioAbortResponse` | 200 |

- `source="sim"` → синхронно, ~100ms wall-clock для D=2м, в ответе `result` заполнен.
- `source="robot"` → асинхронно: MQTT publish `mps/scenario/run`, в ответе только `run_id`. Финальный результат приходит через MQTT `mps/scenario/finished` и WS `/ws/mps/telemetry`.
- 409 Conflict — если уже идёт `running` прогон на этом source.
- 503 — если `source="robot"` но MQTT отключён.
- Заголовок `Idempotency-Key` поддерживается middleware-ом приложения (60с TTL).

### 3.4 История и replay

| Method | Path | Response |
|---|---|---|
| GET | `/history` | `MpsHistoryResponse` (последние ≤20 прогонов) |
| POST | `/history/{run_id}/replay` | `MpsScenarioRunResponse` (новый прогон с теми же параметрами) |

### 3.5 Конфиг

| Method | Path | Response | Status codes |
|---|---|---|---|
| POST | `/config/save` | `MpsConfigSaveResponse` (`{written, path}`) | 200 / 500 |

Записывает текущий `applied` блок матриц в секцию `mps:` файла
[`config.yaml`](../../config.yaml). Race condition: последний победил
(достаточно для учебного MVP).

---

## 4. MQTT топики `samurai/{robot_id}/mps/`

Robot-side (`mps_node.py`) подписан на compute→Pi и публикует Pi→compute.
QoS 0 для high-rate телеметрии, QoS 1 для control.

| Topic | Direction | Payload | QoS | Когда |
|---|---|---|---|---|
| `mps/matrices/set` | compute → Pi | `MpsMatrices` (JSON) | 1 | `POST /matrices/apply` |
| `mps/matrices/applied` | Pi → compute | `MpsMqttApplied` | 1 | После успешного `model.reload()` + `mpc.rebuild()` |
| `mps/scenario/run` | compute → Pi | `MpsMqttScenarioRun` (`{run_id, request}`) | 1 | `POST /scenario/run` source="robot" |
| `mps/scenario/abort` | compute → Pi | `{run_id}` (JSON) | 1 | `POST /scenario/abort` |
| `mps/scenario/finished` | Pi → compute | `MpsScenarioResult` без telemetry, только metrics | 1 | По завершении прогона |
| `mps/telemetry` | Pi → compute | `MpsMqttTelemetry` (`{run_id, point}`) | 0 | Каждые `tick_dt=0.02` (50 Гц) во время `RUNNING` |
| `mps/error` | Pi → compute | `MpsMqttError` | 1 | NaN, instability, watchdog, pre-validate fail |

---

## 5. WebSocket `/ws/mps/telemetry`

Однонаправленный стрим (server → client) с минимальным управлением подпиской.

### Client → server (handshake)

```json
{ "action": "subscribe", "run_id": "<optional>" }
```

Если `run_id` опущен — подписка на любой active run. Если задан —
сервер сначала отправит буфер последних 200 точек этого run-а (если он ещё активен).

### Server → client

Три типа фреймов с дискриминантом `type`:

```json
{ "type": "telemetry", "run_id": "...", "point": MpsTelemetryPoint }
{ "type": "finished",  "run_id": "...", "result": MpsScenarioResult }
{ "type": "error",     "run_id": "...", "error_type": "nan|instability|watchdog|precondition|other", "message": "..." }
```

Сервер закрывает соединение со стороны клиента (graceful) когда отправил
`finished` или `error` если клиент подписан на конкретный `run_id`.
Бессрочные подписки (`subscribe` без `run_id`) живут до закрытия клиентом.

---

## 6. Безопасность и инварианты

1. **`schema_version`** во всех payload-ах. Bump при breaking-изменениях.
2. **MPC активен только в FSM-state `DRIVE_FORWARD_MPS`** (см. инварианты §3 спеки).
3. **Pre-validate** на стороне Pi (`mps_node.py`):
   - `0 < D ≤ 5.0`, `0 < v_target ≤ 0.30`, `u_max[1] ≤ 0.5` рад/с;
   - нарушение → `mps/error` (`error_type="precondition"`), сценарий не стартует.
4. **u clip** — после `step()`, дублируется в `motor_node.config.motor.limits`.
5. **Failsafe** — при `finished` любого типа `motor_node` шлёт `cmd_vel = [0, 0]` трижды.
6. **CORS / Bearer / RateLimit** — наследуются от существующего middleware (см. `app.py`).

---

## 7. Версионирование

`schema_version: "1.0"` — первая версия. План:
- Минорные доп. поля (опциональные) — без bump.
- Удаление поля или смена типа — bump до `2.0`.
- Backend и frontend проверяют major-часть; minor-различия — warning в консоли.

---

## 8. Примеры

### `POST /api/v1/mps/scenario/run` (sim)

```bash
curl -X POST http://localhost:5000/api/v1/mps/scenario/run \
  -H 'Content-Type: application/json' \
  -d '{"distance": 2.0, "v_target": 0.15, "source": "sim"}'
```

```json
{
  "ok": true,
  "run_id": "sim-2026-05-05T16-50-00-001",
  "result": {
    "run_id": "sim-2026-05-05T16-50-00-001",
    "status": "reached",
    "metrics": { "overshoot": 0.012, "settling_time": 12.3, "...": "..." },
    "telemetry": [{ "t": 0.0, "x": [0,0,0,0,0], "u": [0,0], "y": [0,0,0,0,0], "s_remaining": 2.0 }, "..."]
  }
}
```

### `POST /api/v1/mps/validate`

```bash
curl -X POST http://localhost:5000/api/v1/mps/validate \
  -H 'Content-Type: application/json' -d 'null'
```

```json
{
  "eigenvalues_ad": [{ "re": 0.99, "im": 0.0 }, "..."],
  "eigenvalues_closed": [{ "re": 0.5, "im": 0.0 }, "..."],
  "is_plant_stable": true,
  "is_closed_loop_stable": true,
  "step_response": ["..."],
  "warnings": []
}
```
