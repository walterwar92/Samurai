# МПС — TODO

Живой список задач по модулю «Модель Пространства Состояний» (`feat/mps`).
Двигаем пункты в `## Done` после мерджа PR в `feat/mps`.

## Now

- [ ] Smoke-test на реальном Pi (требует доступ к hardware).
- [ ] Frontend — параллельный PR от @OneAstr0.

## Next

- [ ] Bearer-auth для `/api/v1/mps/*`.
- [ ] Persistence истории прогонов в SQLite (сейчас in-memory).
- [ ] Дополнить `latex_doc/control_theory/` главу про сценарий-only режим.

## Backlog / parking lot

- [ ] 3D трек робота (react-three-fiber).
- [ ] Шумы / motor lag в симуляторе.
- [ ] Observer Луенбергера (использует C, D реально).
- [ ] Cruise control (бесконечный сценарий).
- [ ] Playwright e2e тесты для frontend.
- [ ] Export CSV / PDF отчёт по прогону.

## Done

### Backend — 2026-05-05

- [x] Pydantic-схемы + REST-роутер `/api/v1/mps/*` (matrices CRUD,
      validate, scenario run/abort, history, replay, config save).
- [x] WebSocket `/ws/mps/telemetry` — live-стрим с робота через
      thread-safe broker.
- [x] `pi_nodes/nodes/mps_node.py` — orchestrator + tick-loop 50 Гц
      + watchdog + pre-validate + failsafe-stop. FSM-state
      `DRIVE_FORWARD_MPS` в `fsm_node.py`.
- [x] Расширение `state_space_model.py`: Cd, Dd, `reload()`, `output()`.
- [x] `rebuild()` в `mpc_controller.py` с rollback на ошибку.
- [x] Идеальный sim на ноуте (`compute_node/mps_runner.py`).
- [x] Секция `mps:` в `config.yaml` + safety-капы.
- [x] Документация в `docs/mps/` (api, architecture, scenario_forward).
- [x] 77 unit-тестов pytest, общий suite 386 passed без регрессий.
