# МПС — архитектура модуля

> Карта компонентов и потоков данных для модуля «Модель Пространства
> Состояний». Контракт payload-ов — [`api.md`](./api.md). Сценарий —
> [`scenario_forward.md`](./scenario_forward.md).

## 1. Что входит в модуль

```
                                  Browser
                                ┌────────────────┐
                                │  React UI      │
                                │  /mps          │
                                │  (Track B)     │
                                └─────┬──────────┘
                                      │ REST + WS
                ┌─────────────────────┴────────────────────┐
                │     compute_node/dashboard (FastAPI)     │
                │  routers/mps.py        (REST + WS)       │
                │  schemas/mps.py        (Pydantic)        │
                │  mqtt_handlers.py      (MQTT subs)       │
                │  state.py  (mps slice — applied/draft/   │
                │             history/active_run)          │
                │  compute_node/mps_runner.py              │
                │             (идеальный sim source=sim)   │
                └────────────┬─────────────────────────────┘
                             │ MQTT samurai/{id}/mps/*
                             │
              ┌──────────────┴───────────────┐
              │       Pi 4 (Mosquitto)       │
              │  pi_nodes/nodes/mps_node.py  │
              │  pi_nodes/control/           │
              │      state_space_model.py    │
              │      mpc_controller.py       │
              │  fsm_node — DRIVE_FORWARD_MPS │
              │             ├ cmd_vel ┐      │
              │             │        ↓      │
              │             motor_node      │
              └──────────────────────────────┘
```

## 2. Потоки данных

### Apply matrices

```
UI (MatrixEditor)
   │ POST /api/v1/mps/matrices                   (draft only)
   ↓
state.mps.draft = matrices

UI (Apply button)
   │ POST /api/v1/mps/matrices/apply
   ↓
routers/mps.py → mqtt.publish('mps/matrices/set', QoS=1)
                 ↓
                 mps_node._on_matrices_set
                   ├ plant.reload(Ad, Bd, Cd, Dd)
                   ├ mpc.rebuild(Ad, Bd, Q, R, N, u_min, u_max)
                   └ publish('mps/matrices/applied')
                              ↓
mqtt_handlers._h_mps_matrices_applied
   └ state.mps.applied = matrices  (UI poll увидит обновление)
```

### Run scenario — sim

```
UI (Run on Sim)
   │ POST /api/v1/mps/scenario/run {source: "sim"}
   ↓
routers/mps.py → run_scenario_idealized(applied, request)
                 ↓ ~100ms wall-clock
                 MpsScenarioResult с telemetry + metrics
state.mps.history.appendleft(result)
   │
   └→ HTTP 200 result отдаётся синхронно
```

### Run scenario — robot

```
UI (Run on Robot)                      Pi
   │ POST /scenario/run                fsm_state = IDLE
   │   {source: "robot"}                │
   ↓                                    │
mqtt.publish('mps/scenario/run', QoS=1) │
state.mps.active_run = pending          │
   │                                    ↓
   │                          mps_node._on_scenario_run
   │                            ├ pre-validate (D ≤ 5, v ≤ 0.30)
   │                            ├ fsm_state → DRIVE_FORWARD_MPS
   │                            └ start tick-loop
   │                                    │
   │                          ┌─ tick (50 Гц) ─────────────┐
   │     mps/telemetry  ←──── │   x_meas ← position_fusion  │
   │     mps/cmd_vel    ←──── │   u = mpc.step(x_meas, ref) │
   │                          │   pub telemetry; pub cmd_vel│
   │                          │   if reached/timeout/error: │
   │                          │     break                   │
   │                          └─────────────────────────────┘
   │     mps/scenario/finished ←─ pub final result + status
   ↓
mqtt_handlers._h_mps_telemetry
   ├ state.mps.last_telemetry.append(point)
   └ ws_broker.broadcast({type:"telemetry", point})
                              │
                              ↓
                  WS /ws/mps/telemetry
                              │
                              ↓
                  UI (LiveTelemetryHook)
                  ResultPlots обновляется в real-time
```

### Abort

```
UI (Abort)               Pi
   │                     │
mqtt.publish('mps/scenario/abort')
   ↓                     │
                         _on_scenario_abort
                          ├ tick stops
                          ├ cmd_vel = [0,0]   ×3
                          ├ status='aborted'
                          ├ fsm → IDLE
                          └ publish 'mps/scenario/finished'
```

## 3. Граничные инварианты

1. **MPC активен только в `DRIVE_FORWARD_MPS`.** В любом другом state
   (IDLE, SEARCHING, GRABBING, ...) старый PID/cmd_vel продолжает работать.
2. **Сценарий финитный.** Достигли D, timeout, abort или error → IDLE.
3. **Apply только между прогонами.** Приходит `mps/matrices/set` во время
   running → publish `mps/error` (`error_type="precondition"`).
4. **C, D — только для UI.** В `mpc.step()` и `plant.step()` не используются.
5. **Симулятор идеальный.** `compute_node/mps_runner.py` — `x[k+1]=Ax+Bu`
   без шумов / motor lag / slip.
6. **cmd_vel в DRIVE_FORWARD_MPS только от mps_node.** Voice cmds, ball
   detections, joystick — игнорируются (см. fsm_node._voice_cb и др.).

## 4. Где живут какие схемы

| Уровень | Файл | Назначение |
|---|---|---|
| Pydantic | [`compute_node/dashboard/schemas/mps.py`](../../compute_node/dashboard/schemas/mps.py) | Источник правды для REST + MQTT. Валидация shapes, bounds. |
| TS типы | [`compute_node/frontend/src/types/mps.ts`](../../compute_node/frontend/src/types/mps.ts) | Зеркало по структуре. Синхронизируется вручную при bump `schema_version`. |
| Pi-side | `pi_nodes/nodes/mps_node.py` | Парсит `mps/*` MQTT-payload-ы как простые dict-ы (без Pydantic — Pi-нодам это слишком тяжело). |

## 5. Расширения (out of MVP)

См. §10 в [спеке](../superpowers/specs/2026-05-05-mps-state-space-design.md):
3D-трек, шумы, observer Луенбергера, SQLite persistence, экспорт CSV/PDF,
auth для `/api/v1/mps/*`, e2e Playwright. Все — отдельные PR после
merge feat/mps в dev.
