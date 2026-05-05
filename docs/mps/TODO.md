# МПС — TODO

Живой список задач по модулю «Модель Пространства Состояний» (`feat/mps`).
Двигаем пункты в `## Done` после мерджа PR в `feat/mps`.

## Now

- [ ] A10: smoke-test на реальном Pi (требует доступ к hardware).
- [ ] Запуск vitest + Testing Library — добавить в devDependencies
      `package.json` и описать `npm test`. Сейчас фронт-компоненты
      собираются через `tsc -b` и `vite build`; vitest пока вне CI
      (требует `npm install` обновлений; делаем отдельным PR в feat/mps).

## Next

- [ ] Bearer-auth для `/api/v1/mps/*` (см. backlog §10 спеки).
- [ ] Persistence истории прогонов в SQLite (сейчас in-memory, теряется
      при рестарте dashboard).
- [ ] Документ в `latex_doc/control_theory/` про сценарий-only режим
      (расширение существующих глав).

## Backlog / parking lot

- [ ] 3D трек робота (react-three-fiber).
- [ ] Шумы / возмущения / motor lag в симуляторе.
- [ ] Observer Луенбергера (использует C, D реально).
- [ ] Cruise control (бесконечный сценарий).
- [ ] Playwright e2e тесты для frontend.
- [ ] Export CSV / PDF отчёт по прогону.

## Done

### Day 0 + Track A (backend) — 2026-05-05

- [x] **Day 0** — `docs/mps/api.md` (REST + MQTT + WS контракт),
      `compute_node/dashboard/schemas/mps.py` (Pydantic),
      `compute_node/frontend/src/types/mps.ts` (TS-зеркало).
- [x] **A1** — `pi_nodes/control/state_space_model.py`: `Cd`, `Dd`,
      `reload()`, `output()` + 14 тестов в
      [tests/test_state_space_extended.py](../../tests/test_state_space_extended.py).
- [x] **A2** — `pi_nodes/control/mpc_controller.py`: `rebuild()` с
      rollback на ошибку + 8 тестов.
- [x] **A3** — `compute_node/mps_runner.py`: идеальный sim сценария
      «вперёд D метров» + `short_step_response`, `closed_loop_eigenvalues`
      + 8 тестов в `tests/test_mps_runner.py`.
- [x] **A4** — `compute_node/dashboard/routers/mps.py`: 11 endpoints
      `/api/v1/mps/*` + `_default_matrices_from_config()` + `Idempotency-Key`
      via existing middleware + 19 тестов в `tests/test_mps_router.py`.
- [x] **A5** — `pi_nodes/nodes/mps_node.py`: orchestrator + tick-loop
      50 Гц + watchdog + pre-validate + FSM-state `DRIVE_FORWARD_MPS` в
      `pi_nodes/nodes/fsm_node.py` + 10 тестов в
      `tests/test_mps_node.py` (без testcontainers — мокаем paho).
- [x] **A6** — `compute_node/dashboard/state.py` (`_MpsBlock`) +
      `mqtt_handlers.py` (4 новых handler'а: matrices/applied, telemetry,
      finished, error) + WS broadcaster wiring.
- [x] **A7** — секция `mps:` в `config.yaml` с дефолтами из `matlab/main.m`,
      safety-caps, `tick_dt`, `history_size`.
- [x] **A8** — WebSocket `/ws/mps/telemetry`: `_MpsWsBroker` thread-safe
      fan-out + replay буфера на subscribe + 1 unit-тест.
- [x] **A9** — `docs/mps/architecture.md` (диаграммы потоков) +
      `docs/mps/scenario_forward.md` (формальная постановка задачи).

**Backend всего: 77 unit-тестов pytest по MPS-модулям, все зелёные.
Полный suite в `tests/`: 386 passed, 1 skipped, 0 регрессий.**

### Track B (frontend) — 2026-05-05

- [x] **B1** — `compute_node/frontend/src/pages/MpsPage.tsx`, route
      `/mps` в `App.tsx` (lazy + Suspense), SPA fallback для `/mps`
      в `compute_node/dashboard/app.py`.
- [x] **B2** — `components/mps/MatrixEditor.tsx` + хук
      `hooks/useMpsMatrices.ts`: inline grid editor с валидацией
      числа/NaN/shape, подсветка дифа vs applied, dirty-tracking.
- [x] **B3** — `components/mps/ScenarioControls.tsx` + хук
      `hooks/useMpsRun.ts`: D, v_target, тумблер Sim/Robot, polling
      статуса для async robot-run.
- [x] **B4** — `components/mps/ResultPlots.tsx` (Recharts, 4 таба
      x/u/y/s, multi-line, hover, поддержка overlay для compare).
- [x] **B5** — `components/mps/EigenvaluePanel.tsx` (SVG: единичная
      окружность + λ открытой/замкнутой, цветами устойчивость).
- [x] **B6** — `components/mps/TrajectoryView.tsx` (2D top-down SVG:
      путь робота, target, текущая позиция, авто-масштаб). Без Three.js.
- [x] **B7** — `components/mps/HistoryPanel.tsx` + хук
      `hooks/useMpsHistory.ts`: список последних 20 прогонов с
      checkbox-compare (≤3) и Replay.
- [x] **B8** — `components/mps/TuningSliders.tsx`: Q/R/N слайдеры с
      debounced re-sim в симуляторе и кнопкой «Promote to draft».
- [x] **B9** — `hooks/useMpsLiveTelemetry.ts` + интеграция в `MpsPage`:
      WebSocket `/ws/mps/telemetry`, буфер 200 точек,
      авто-disconnect по finished/error.
- [x] **B10** — `components/mps/{DraftStatus,ValidationBadge}.tsx`,
      обработка ошибок в MpsPage, индикатор WS connection.

**Frontend: `tsc -b` зелёный (Vite production build готов).
Unit-тесты компонентов — в backlog (vitest setup нужен отдельным PR).**
