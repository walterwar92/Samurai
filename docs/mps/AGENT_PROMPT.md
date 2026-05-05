# Промт для запуска работы над модулем МПС

> Этот промт скармливается AI-ассистенту (Claude Code, Cursor, Copilot и т.п.) одного из соавторов в начале сессии. Все детали реализации — в [`docs/superpowers/specs/2026-05-05-mps-state-space-design.md`](../superpowers/specs/2026-05-05-mps-state-space-design.md).

---

## Главный промт (общий для обоих треков)

```
Контекст. Ты работаешь в репозитории Samurai — автономный гусеничный
робот (Raspberry Pi 4 + ноутбук + React-дэшборд). Сейчас активная ветка —
feat/mps. На ней реализуется учебный модуль «МПС — Модель Пространства
Состояний» для курсовой по учебнику В.Н. Козлова (СПбГПУ).

Внимание — терминология. В этой ветке «МПС» означает «Модель
Пространства Состояний» (state-space), НЕ «методы принятия решений».
Если встретишь старые упоминания МПС в смысле decision-making —
игнорируй, в этой ветке смысл другой.

Что уже есть в main (НЕ переписывать, переиспользовать):
- pi_nodes/control/mpc_controller.py — рабочий MPC (5 состояний,
  2 управления v/ω, clip+QP solver), коммит 8e58c98.
- pi_nodes/control/state_space_model.py — дискретная модель Ad/Bd.
- pi_nodes/control/{lqr_controller,controller_factory}.py.
- matlab/ — оффлайн-синтез матриц по учебнику Козлова.
- latex_doc/control_theory/ — теоретические главы.
- 56 unit-тестов, все зелёные. По умолчанию control.mode=off.

Что строим в feat/mps. Учебно-исследовательский UI:
- Пользователь редактирует матрицы A/B/C/D + Q/R/N/limits в дашборде.
- Запускает финитный сценарий «проехать D метров вперёд» в симуляторе
  или на реальном роботе.
- Видит графики x(t)/u(t)/y(t), собственные значения на единичном
  круге, 2D top-down траекторию, метрики (overshoot, settling,
  control_energy и т.д.).
- Умеет Validate (проверить устойчивость до Apply), History (последние
  20 прогонов), Compare (overlay 2-3 прогонов), Tuning sliders
  (live re-sim в симуляторе).

Single source of truth — спека:
docs/superpowers/specs/2026-05-05-mps-state-space-design.md

В ней детально описано:
- §3   границы и инварианты;
- §4   backend (Pi mps_node, REST router, mps_runner);
- §5   формальный сценарий «вперёд D метров», метрики, safety;
- §6   frontend (10 компонентов + 5 хуков, layout, поведение);
- §7   контракты REST/MQTT/WebSocket с Pydantic-схемами;
- §8   тесты (pytest + vitest, без e2e);
- §9   план работ с зависимостями (Day 0 → Track A / Track B).

Роли:
- @razdryzg-dev → Track A (backend / Pi). См. §9 шаги A1–A10.
- @OneAstr0    → Track B (frontend). См. §9 шаги B1–B10.

Это разные люди и разные PR. Каждый PR целится в feat/mps. Когда оба
трека Done — squash-merge feat/mps → dev одним PR с описанием всей
фичи; затем dev → main стандартом проекта.

Workflow:
1. Day 0 (совместно, ~1 день): заполнить docs/mps/api.md (REST + MQTT
   + WS контракт), Pydantic-схемы compute_node/dashboard/schemas/mps.py,
   TS-типы compute_node/frontend/src/types/mps.ts. Это блокер для всего.
2. После Day 0 — параллельно по §9 плану. До A4 (REST роутер) фронт
   работает с msw-моками контракта.
3. Каждый шаг — отдельный коммит с осмысленным сообщением. PR в feat/mps
   когда трек Done или промежуточная веха (после A4 / после B4).

Ограничения:
- Базовая ветка для PR — feat/mps. НЕ dev и НЕ main.
- Авторы PR — только @razdryzg-dev (backend) и @OneAstr0 (frontend).
- В commits и PR description НЕ добавлять подпись Co-Authored-By
  (никого) и НЕ писать "Generated with Claude Code" / упоминания
  AI-ассистентов. Сообщения пишутся как от соавтора.
- Auth для новых endpoints в MVP не делаем (dev-mode, открытые).
  Bearer-token — в backlog (§10 спеки).
- В MVP не делать: 3D трек, шумы в симуляторе, observer Луенбергера,
  SQLite persistence, экспорт CSV/PDF, Playwright e2e — всё в backlog.

Definition of Done для PR (§9 спеки):
- CI зелёный (pytest + ruff + tsc + vitest);
- coverage не упал ниже 85% для трогнутых модулей;
- обновлён docs/mps/TODO.md (галочка в Done или пункт в Next);
- скриншот UI или curl-пример REST в описании PR;
- никаких "// removed" комментариев и закомменченного кода;
- SECURITY.md scope не нарушен.

Действуй по плану из §9 спеки. Если по ходу обнаружишь противоречие
со спекой или что-то неоднозначное — сначала открой issue / напиши
сообщение, не молча отклоняйся. Если железо Pi недоступно — работай
в режиме симулятора (source="sim"), это полностью покрывает defaults
сценария.
```

---

## Sub-промт для Track A (@razdryzg-dev — backend / Pi)

После общего промта выше — добавить:

```
Твой трек — Backend / Pi. Ты делаешь шаги A1–A10 из §9 спеки:

A1. pi_nodes/control/state_space_model.py — добавить Cd, Dd, reload(),
    output() + тесты в tests/test_state_space_extended.py.
A2. pi_nodes/control/mpc_controller.py — добавить rebuild() + тесты в
    tests/test_mpc_controller.py.
A3. compute_node/mps_runner.py — идеальный sim сценария + тесты в
    tests/test_mps_runner.py.
A4. compute_node/dashboard/routers/mps.py — все REST endpoints из §7.2
    + тесты с FastAPI TestClient.
A5. pi_nodes/nodes/mps_node.py — orchestrator + FSM-state DRIVE_FORWARD_MPS
    + тесты с testcontainers Mosquitto (по примеру tests/test_mqtt_integration.py).
A6. compute_node/dashboard/state.py + mqtt_handlers.py — wiring.
A7. config.yaml — секция mps: с дефолтами из matlab/main.m.
A8. WebSocket /ws/mps/telemetry в FastAPI.
A9. docs/mps/architecture.md + docs/mps/scenario_forward.md.
A10. smoke-test на реальном Pi (если железо доступно).

НЕ трогай compute_node/frontend/. Frontend делает @OneAstr0.

Coverage 85% — обязательно. Если testcontainers недоступны — fallback на
mock MQTT через paho-mqtt тестовый клиент, но в PR description честно
обозначь.
```

---

## Sub-промт для Track B (@OneAstr0 — frontend)

После общего промта выше — добавить:

```
Твой трек — Frontend. Ты делаешь шаги B1–B10 из §9 спеки:

B1. Базовая страница compute_node/frontend/src/pages/MpsPage.tsx + routing
    в App.tsx + layout (две колонки, см. §6.2).
B2. components/mps/MatrixEditor.tsx + hooks/useMpsMatrices.ts + тесты.
B3. components/mps/ScenarioControls.tsx + hooks/useMpsRun.ts + тесты.
B4. components/mps/ResultPlots.tsx (Recharts, 4 таба x/u/y/s) + тесты.
B5. components/mps/EigenvaluePanel.tsx (SVG: единичный круг + λ).
B6. components/mps/TrajectoryView.tsx (2D top-down SVG, без Three.js).
B7. components/mps/HistoryPanel.tsx + hooks/useMpsHistory.ts + Compare mode.
B8. components/mps/TuningSliders.tsx + sync sim (re-sim на каждый change).
B9. Live mode через WebSocket /ws/mps/telemetry + hooks/useMpsLiveTelemetry.ts.
B10. components/mps/{DraftStatus,ValidationBadge}.tsx + polish.

Стек: React + TypeScript + Vite (как в проекте), Recharts для графиков,
2D SVG для unit-circle и trajectory. Без Three.js в MVP — это extension.

До A4 (REST роутер @razdryzg-dev) — работай с msw-моками контракта.
Контракт фиксирован в docs/mps/api.md и compute_node/frontend/src/types/mps.ts
(оба файла появляются в Day 0).

НЕ трогай pi_nodes/, compute_node/dashboard/, compute_node/mps_runner.py.
Backend делает @razdryzg-dev.

Тесты — Vitest + Testing Library, без Playwright e2e (в backlog).
```

---

## Как использовать

1. **@razdryzg-dev** копирует **«Главный промт»** + **«Sub-промт для Track A»** в свою AI-сессию.
2. **@OneAstr0** копирует **«Главный промт»** + **«Sub-промт для Track B»** в свою AI-сессию.
3. **Day 0** — лучше делать совместно (созвон / pair): один человек пишет `docs/mps/api.md` и Pydantic-схемы, второй ревьюит и пишет TS-типы. После этого треки расходятся.
4. Промежуточный sync — раз в 2-3 дня, через PR-ревью или комментарии в спеке.
5. Все вопросы по архитектуре — в issue или в комментарии к спеке (`docs/superpowers/specs/2026-05-05-mps-state-space-design.md`).
