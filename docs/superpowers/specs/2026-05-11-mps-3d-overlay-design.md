# МПС — 3D-оверлей траектории после симуляции

> Превращение результата симуляции МПС из набора графиков в анимированную
> 3D-сцену. После завершения расчёта сценария на странице `/mps`
> снизу-справа всплывает тост «Симуляция завершена. Показать в 3D?»; через
> 5 секунд тост сам закрывается. Если пользователь согласился — на весь
> экран открывается оверлей с 3D-сценой (стиль `/3d`), и модель `Samurai.glb`
> едет по траектории из телеметрии симуляции в реальном времени.
>
> Базируется на: ветка `feat/redesign`, существующие модули МПС
> ([`2026-05-05-mps-state-space-design.md`](./2026-05-05-mps-state-space-design.md),
> [`2026-05-06-mps-ui-redesign-design.md`](./2026-05-06-mps-ui-redesign-design.md))
> и компонент `Visualization3DPage`.
>
> Скоуп: **только frontend** (`compute_node/frontend/src/`). Backend и
> REST/WS-контракт МПС не трогаются.

## 1. Цели и не-цели

### Цели

1. **Наглядность.** Студент после клика «Запустить» видит не только графики
   `v(t)`, `ω(t)`, `s(t)`, а буквально «как машинка проехала» — модель робота
   едет по траектории `(s·cos θ, s·sin θ)` в той же 3D-сцене, что и
   страница `/3d`.
2. **Ненавязчивость.** Предложение посмотреть в 3D — это плавающий тост
   на 5 секунд, а не модальное окно поверх результатов. Хочешь — кликнул,
   не хочешь — продолжаешь работать.
3. **Возможность повтора.** Если тост закрылся, у пользователя есть кнопка
   «3D-просмотр» в шапке `TrajectoryView` для текущего `primaryResult`.
4. **Изоляция.** Оверлей рендерится через portal в `document.body`, не
   ломает sticky-колонку и z-индексы карточек. Закрытие — только по ✕.

### Не-цели

- Не трогаем backend (`compute_node/dashboard/`, `pi_nodes/`).
- Не меняем REST/WS-контракт МПС (`docs/mps/api.md`).
- Не добавляем play/pause/scrub/настройку скорости — только 1:1
  воспроизведение времени из телеметрии (вариант A на брифинге).
- Не объединяем эту сцену со страницей `/3d` — это отдельный оверлей
  с симуляционными данными, без живых SLAM/IMU/Heatmap-слоёв
  (вариант C2 на брифинге).
- Не делаем закрытие по Esc и по клику вне (явное решение B1 на брифинге).
- Не добавляем кнопку 3D-просмотра в `HistoryPanel` (R2, не R3).

## 2. UX-сценарий

```
1. Пользователь на /mps настроил матрицы и нажал «Запустить» в
   ScenarioControls.
2. Симуляция завершается:
   • source='sim'   → POST /api/mps/scenario/run отвечает синхронно с result,
                      runHook.result обновляется через ~10–50 мс.
   • source='robot' → POST возвращает run_id, через WS приходит
                      'finished'-frame, useMpsLiveTelemetry.onFinished
                      даёт result.
3. MpsPage делает setPrimaryResult(result). Это новый run_id, телеметрия
   непуста — поднимается тост.
4. Снизу-справа выезжает (slide-up + fade, 250 мс) карточка ~320×96 пкс:
   • Заголовок: «Симуляция завершена»
   • Подзаголовок: «s = {distance} м, статус: {status}»
   • Кнопка «Показать в 3D» (primary)
   • Иконка ✕ (отказ)
   • Тонкая прогресс-полоска внизу убывает за 5 с
5а. Пользователь жмёт ✕ или ждёт 5 с — тост уходит (slide-down + fade).
5б. Пользователь жмёт «Показать в 3D»:
   • Тост уходит, оверлей открывается (backdrop fade + scale 0.95→1.0,
     250 мс).
   • Тёмная панель ~90vw × 85vh поверх всего, внутри — 3D-сцена.
   • Модель Samurai стартует в (0,0), едет по телеметрии в реальном
     времени. Линия следа нарастает; цель — зелёное кольцо в (D, 0).
   • Закрытие — только ✕ в правом верхнем углу.
6. После закрытия оверлея пользователь может открыть его снова, кликнув
   на иконку «3D» в шапке TrajectoryView. Тост повторно не появляется
   для этого же run_id.
7. Replay из HistoryPanel: setPrimaryResult у виденного run_id не
   триггерит тост, но кнопка «3D» в TrajectoryView активна.
```

## 3. Архитектура и FSM

### Иерархия провайдеров на странице

```
MpsPage
└─ MpsHighlightProvider          (как сейчас)
   └─ Mps3DProvider              ← новый
      ├─ FSM-state
      ├─ <Mps3DToast/>           (portal, при state.kind === 'toasting')
      └─ <Mps3DOverlay/>         (portal, при state.kind === 'overlay')
         └─ <Mps3DScene/>        (R3F Canvas)
```

### FSM состояния

```ts
type Mps3DState =
  | { kind: 'idle' }
  | { kind: 'toasting'; result: MpsScenarioResult; startedAt: number }
  | { kind: 'overlay';  result: MpsScenarioResult }
```

### FSM-переходы

| Из | Событие | В | Побочный эффект |
|---|---|---|---|
| `idle` | `requestToast(r)` | `toasting(r)` | стартует `setTimeout(5000)` |
| `toasting(r)` | таймер истёк | `idle` | — |
| `toasting(r)` | `open()` | `overlay(r)` | `clearTimeout` |
| `toasting(r)` | `requestToast(r')` (другой run_id) | `toasting(r')` | сбрасывает таймер и стартует новый |
| `toasting(r)` | клик ✕ в тосте | `idle` | `clearTimeout` |
| `overlay(r)` | клик ✕ в оверлее | `idle` | — |
| `overlay(r)` | `requestToast(r')` | `overlay(r)` (игнор) | — |
| `idle` / `toasting` | `open(r')` из R2-кнопки | `overlay(r')` | `clearTimeout` |

**Защита от двойного триггера.** В `MpsPage` хранится
`useRef<lastSeenRunId>`; `requestToast` вызывается только когда
`primaryResult.run_id !== lastSeenRunId.current` и `telemetry.length >= 2`.

## 4. Компоненты и интерфейсы

### `Mps3DProvider.tsx` (~80 строк)

```ts
const Mps3DContext = createContext<{
  state: Mps3DState
  requestToast: (r: MpsScenarioResult) => void
  open:         (r: MpsScenarioResult) => void
  close:        () => void
} | null>(null)

export function Mps3DProvider({ children }: { children: ReactNode }) { ... }
export function useMps3D() { ... }  // throws if not in provider
```

- `useReducer` для FSM.
- `useEffect` слушает `state.kind === 'toasting'`: при входе ставит
  `setTimeout(5000)` → `dispatch({ type: 'TIMEOUT' })`. При выходе —
  `clearTimeout`.
- `requestToast` сравнивает входящий `run_id` с текущим
  `state.kind === 'toasting' ? state.result.run_id : null`;
  если совпадает — не делает ничего (избегает дребезга).

### `Mps3DToast.tsx` (~60 строк)

- `createPortal(<div .../>, document.body)`.
- Позиция: `fixed bottom-4 right-4 z-[60]`.
- Размер: `w-[320px]`, содержимое `p-3`.
- Анимация **T1** появления: Tailwind transition с начальным
  `translate-y-4 opacity-0`, переход в `translate-y-0 opacity-100`
  через 250 мс. На unmount — обратное состояние (`transition-all`
  с задержкой через ref).
- Структура:
  - Шапка: иконка `Box` (lucide) + «Симуляция завершена», справа ✕.
  - Подзаголовок: `s={r.request.distance.toFixed(2)} м • {statusText(r.status)}`,
    где `statusText` — локальный хелпер: `reached → «достигнуто»`,
    `aborted → «прервано»`, `timeout → «таймаут»`, `error → «ошибка»`,
    `running → «выполняется»`.
  - Кнопка primary «Показать в 3D» (вызывает `open(r)`).
  - Прогресс-полоска внизу: 2 пкс высоты, CSS-`animation` от 100% к 0%
    за 5 с (`@keyframes` или inline `transition: width 5s linear`).
- Стиль карточки: `bg-zinc-900/95 border border-zinc-700 rounded shadow-xl backdrop-blur`.

### `Mps3DOverlay.tsx` (~80 строк)

- `createPortal(<div .../>, document.body)`.
- Backdrop: `fixed inset-0 bg-black/70 z-[70]`. **Не** обрабатывает клик.
- Анимация **O1**: backdrop `opacity` 0→1, контейнер `scale-95 opacity-0`
  → `scale-100 opacity-100`, 250 мс.
- Контейнер: `relative w-[90vw] h-[85vh] mx-auto mt-[7.5vh] rounded-lg
  border border-zinc-700 bg-[#1a1a2e] overflow-hidden`.
- Шапка контейнера: «3D-просмотр траектории» + status-бейдж,
  справа кнопка ✕. Закрытие — `useMps3D().close()`.
- Тело: `<Mps3DScene telemetry={r.telemetry} distance={r.request.distance}
  status={r.status} />`.
- HTML-инфо-оверлей (поверх Canvas, через absolute-позиционирование внутри
  контейнера): в нижнем-левом углу `t = X.XXс • s = X.XXм • статус`.
  Обновляется через `useRef` + `requestAnimationFrame`, **не** через
  React-state (чтобы не ререндерить каждый кадр).
- **НЕ** слушаем `keydown` / `Esc` / клик по backdrop (B1).

### `Mps3DScene.tsx` (~200 строк)

Props:
```ts
interface Mps3DSceneProps {
  telemetry: MpsTelemetryPoint[]
  distance: number
  status: ScenarioStatus
  liveRef?: MutableRefObject<{ t: number; s: number } | null>  // для HTML-overlay info
}
```

#### Подготовка данных (вне рендера, useMemo)

```ts
const samples = useMemo(() => {
  return telemetry
    .filter(p => Number.isFinite(p.t) && Number.isFinite(p.x[0]) && Number.isFinite(p.x[2]))
    .map(p => ({
      t: p.t,
      s: p.x[0],
      theta: p.x[2],
      x: p.x[0] * Math.cos(p.x[2]),
      y: p.x[0] * Math.sin(p.x[2]),
    }))
}, [telemetry])
```

Если `samples.length < 2` — рендерить fallback `<p>Нет валидных данных</p>`
внутри контейнера, не запускать Canvas.

#### Сцена

```tsx
<Canvas camera={{ position: [1.0, 1.0, 1.0], fov: 50, near: 0.01, far: 100 }} shadows>
  <color attach="background" args={['#1a1a2e']}/>

  {/* освещение — то же что в Visualization3DPage */}
  <ambientLight intensity={0.9}/>
  <directionalLight position={[2,3,1]} intensity={1.8} castShadow .../>
  <directionalLight position={[-1,2,-1]} intensity={0.7}/>
  <directionalLight position={[0,1,-2]} intensity={0.4}/>
  <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]}/>

  {/* сетка и shadow-plane — те же параметры */}
  <Grid args={[10,10]} cellSize={0.1} sectionSize={0.5} ... infiniteGrid/>
  <mesh rotation={[-Math.PI/2, 0, 0]} position={[0,-0.001,0]} receiveShadow>
    <planeGeometry args={[20,20]}/>
    <shadowMaterial opacity={0.2}/>
  </mesh>
  <axesHelper args={[0.3]}/>

  {/* старт */}
  <mesh position={[0, 0.02, 0]}><sphereGeometry args={[0.02]}/><meshStandardMaterial color="#64748b"/></mesh>

  {/* цель (D на оси x, на 3D-плоскости z = -y_world) */}
  <mesh position={[distance, 0.02, 0]} rotation={[Math.PI/2, 0, 0]}>
    <torusGeometry args={[0.04, 0.005, 8, 32]}/>
    <meshStandardMaterial color="#16a34a"/>
  </mesh>

  <Suspense fallback={null}>
    <AnimatedRobot samples={samples} liveRef={liveRef}/>
  </Suspense>

  <AnimatedTrail samples={samples} liveRef={liveRef}/>

  <OrbitControls
    target={[distance/2, 0, 0]}
    maxPolarAngle={Math.PI/2 - 0.05}
    minDistance={0.3}
    maxDistance={10}
    enableDamping
  />
</Canvas>
```

**Камера.** Стартовая позиция фиксированная — `[1.0, 1.0, 1.0]`, target —
середина траектории `(distance/2, 0, 0)`. Пользователь может крутить
`OrbitControls`. Camera-follow **не** делаем (это статичный демо-режим).

**Маппинг координат.** В формуле трейла используется
`(x, y) = (s·cosθ, s·sinθ)` — это **мировые** XY. В Three-сцене мы
переводим в `(x, 0.02, -y)` (как в `Visualization3DPage`, ось Z в Three
смотрит «на нас», поэтому минус). `RobotModel` уже использует тот же
маппинг, поэтому реюзаем без изменений.

#### `AnimatedRobot` (внутренний компонент)

```ts
function AnimatedRobot({ samples, liveRef }: { samples: Sample[], liveRef?: ... }) {
  const startTimeRef = useRef<number | null>(null)
  const visibilityHandlerRef = useRef<...>()
  // визибилити-фикс: при возврате во вкладку пересчитываем startTime,
  // чтобы машинка не «прыгала» к финалу

  // currentRef хранит интерполированную позицию и угол; RobotModel рендерит из него
  // (НО RobotModel сейчас принимает props, не ref. См. ниже.)
  const [pose, setPose] = useState({ posX: 0, posY: 0, yawDeg: 0, stationary: false })

  useFrame(() => {
    if (startTimeRef.current === null) startTimeRef.current = performance.now()
    const elapsed = (performance.now() - startTimeRef.current) / 1000
    const last = samples[samples.length - 1]

    if (elapsed >= last.t) {
      setPose({ posX: last.x, posY: last.y, yawDeg: last.theta * 180/Math.PI, stationary: true })
      liveRef && (liveRef.current = { t: last.t, s: last.s })
      return
    }

    // бинарный поиск интервала
    let lo = 0, hi = samples.length - 1
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1
      if (samples[mid].t <= elapsed) lo = mid; else hi = mid
    }
    const a = samples[lo], b = samples[hi]
    const alpha = (elapsed - a.t) / (b.t - a.t)
    const x = a.x + (b.x - a.x) * alpha
    const y = a.y + (b.y - a.y) * alpha
    const theta = a.theta + (b.theta - a.theta) * alpha
    const s = a.s + (b.s - a.s) * alpha
    setPose({ posX: x, posY: y, yawDeg: theta * 180/Math.PI, stationary: false })
    liveRef && (liveRef.current = { t: elapsed, s })
  })

  return <RobotModel
    yaw={pose.yawDeg}
    pitch={0}
    roll={0}
    posX={pose.posX}
    posY={pose.posY}
    stationary={pose.stationary}
  />
}
```

**Решение о `setState` в useFrame.** `RobotModel` сейчас принимает props
и сам интерполирует через `useRef` и `useFrame`. Если мы передаём pose
через `setState` каждый кадр (~60 fps), React будет ререндерить
`<RobotModel>` 60 раз/с — лишняя работа. Но `RobotModel` после получения
новых пропсов в `useFrame()` использует их через `propsRef.current`,
который обновляется на каждом ре-рендере (строка 47-48 в `RobotModel.tsx`).
То есть основной цикл уже там; смена пропсов лишь обновляет таргет
для lerp.

Проблема в том, что мы хотим **точную** позицию (интерполированную из
телеметрии), без дополнительного сглаживания. `RobotModel`'s lerp
(`POS_LERP = 0.15`) добавит задержку.

**Решение:** добавить в `RobotModel` опциональный prop `noSmooth?: boolean`.
Когда `true` — позиция и поворот выставляются напрямую без lerp. Все
существующие вызовы `<RobotModel/>` (на `/3d`) не передают этот prop —
поведение остаётся прежним. В `<AnimatedRobot>` передаём `noSmooth={true}`.

Альтернатива (если не хочется трогать `RobotModel`): дублировать модель
в `Mps3DScene` (новый компонент `SimulationRobotModel` ~50 строк, грузит
тот же GLB). Чуть больше кода, но изоляция. **Рекомендую модификацию
`RobotModel` с `noSmooth` prop** — модель там одна и так, дублирование
хуже.

#### `AnimatedTrail` (~50 строк)

- Префиксы предвычислены в `samples` (массив `{x, y}` точек).
- Создаём `THREE.BufferGeometry` с `Float32Array(samples.length * 3)`.
- На маунте — `position.array` заполняется всеми точками сразу
  (`[x, 0.02, -y]`).
- `setDrawRange(0, 0)` изначально; на каждом `useFrame` ставим
  `setDrawRange(0, currentIndex + 1)` где `currentIndex` = тот же `lo`
  из `AnimatedRobot`. Чтобы не дублировать бинарный поиск, можно
  передать `currentIndexRef` через context или ref.
- Материал: `<lineBasicMaterial color="#2563eb" linewidth={2}/>`.
- Cleanup: `useEffect(() => () => geometry.dispose(), [])`.

#### Visibility-фикс

```ts
useEffect(() => {
  function onVisibility() {
    if (document.visibilityState === 'visible' && startTimeRef.current !== null) {
      // taken elapsed before hidden из liveRef.current.t; стартtime = now - elapsed*1000
      const lastElapsed = liveRef?.current?.t ?? 0
      startTimeRef.current = performance.now() - lastElapsed * 1000
    }
  }
  document.addEventListener('visibilitychange', onVisibility)
  return () => document.removeEventListener('visibilitychange', onVisibility)
}, [liveRef])
```

### Кнопка R2 в `TrajectoryView.tsx`

В шапке (`<CardHeader>`) рядом с `<CardTitle>` добавляется небольшая
icon-кнопка:

```tsx
<Button
  variant="outline" size="sm"
  disabled={!result || (result.telemetry?.length ?? 0) < 2}
  onClick={() => result && mps3D.open(result)}
  title="3D-просмотр траектории"
>
  <Box className="w-4 h-4" />
  <span className="ml-1">3D</span>
</Button>
```

Требует импорта `useMps3D` и `Button` (если не используется — есть в
`@/components/ui/button`). `Box` из `lucide-react`.

### Изменения `MpsPage.tsx`

1. Импортировать `Mps3DProvider`.
2. Обернуть `<MpsHighlightProvider>` → `<Mps3DProvider>` → JSX.
3. Добавить ref + effect:

```tsx
const lastSeenRunIdRef = useRef<string | null>(null)
const mps3D = useMps3D()  // НЕ работает на верхнем уровне MpsPage,
                          // потому что MpsPage сам является детьми
                          // <Mps3DProvider>. Решение: разделить MpsPage
                          // на MpsPage (внешняя обёртка) и MpsPageInner
                          // (внутренний компонент, использует useMps3D).
useEffect(() => {
  if (!primaryResult) return
  if (primaryResult.run_id === lastSeenRunIdRef.current) return
  if ((primaryResult.telemetry?.length ?? 0) < 2) return
  lastSeenRunIdRef.current = primaryResult.run_id
  mps3D.requestToast(primaryResult)
}, [primaryResult, mps3D])
```

**Структурная правка.** Поскольку `useMps3D` нельзя вызвать на том же
уровне где висит `<Mps3DProvider>`, рефакторим:

```tsx
export function MpsPage() {
  return (
    <MpsHighlightProvider>
      <Mps3DProvider>
        <MpsPageInner/>
      </Mps3DProvider>
    </MpsHighlightProvider>
  )
}

function MpsPageInner() {
  // весь существующий код MpsPage
  const mps3D = useMps3D()
  // ...
}
```

## 5. Поток данных

```
ScenarioControls.onRun(req)
  → useMpsRun.run(req)
      sim:   POST /api/mps/scenario/run → sync result
      robot: POST → run_id; WS 'finished' frame → result
  → useMpsLiveTelemetry.onFinished(r) || runHook.result
  → setPrimaryResult(r)
  → MpsPageInner useEffect:
      if r.run_id ≠ lastSeenRunIdRef.current && r.telemetry.length ≥ 2:
        lastSeenRunIdRef.current = r.run_id
        mps3D.requestToast(r)
  → Mps3DProvider FSM: idle → toasting(r), таймер 5с
  → <Mps3DToast> рендерится, slide-up
  → варианты:
      a) клик «Показать в 3D»: open(r) → overlay(r) → <Mps3DOverlay>
      b) клик ✕ или 5с истёк: → idle, тост уходит
  → <Mps3DOverlay> монтирует <Mps3DScene>
  → <AnimatedRobot> в useFrame интерполирует pose из samples
  → клик ✕ оверлея: close() → idle
```

## 6. Граничные случаи

| Случай | Поведение |
|---|---|
| `telemetry` пуст или `length < 2` | `requestToast` не вызывается. Кнопка R2 в `TrajectoryView` `disabled`. |
| `status === 'error' / 'aborted' / 'timeout'` | Тост показывается. В тосте и в шапке оверлея статус подсвечен (`text-amber-400` для warning, `text-red-400` для error). Машинка едет по тому, что есть, и останавливается. |
| `replay` из `HistoryPanel` | `setPrimaryResult` с уже-виденным `run_id` не триггерит тост (ref-защита). Кнопка R2 работает. |
| Новый запуск пока висит тост | `requestToast(r')` с другим `run_id` → state остаётся `toasting`, но с новым result; таймер перезапускается. |
| Новый запуск пока открыт оверлей | Игнор (C1). |
| Очень длинная симуляция (>30с) | 1:1, без оптимизаций — сознательный выбор A. |
| NaN/Inf в телеметрии (например, error_type='nan') | Фильтрация в `useMemo` `samples`. Если осталось < 2 точек — fallback `<p>Нет валидных данных</p>` без Canvas. |
| Вкладка скрыта во время анимации | `visibilitychange` → пересчёт `startTime` через `liveRef.current.t`. Машинка не прыгает в финал. |
| Размонтирование оверлея | `Mps3DScene` cleanup: `BufferGeometry.dispose()`, удаление `visibilitychange` listener, очистка `startTimeRef`. |
| WebGL не доступен (старый браузер) | R3F покажет `<canvas>` без рендера, видны старт и цель как HTML — приемлемая деградация. Активный пользователь использует современный браузер. |

## 7. Тестирование

Стек: vitest + @testing-library/react + vi.useFakeTimers. R3F/WebGL не
тестируем — нет jsdom-поддержки.

### `Mps3DProvider.test.tsx` (новый)

- `requestToast` переводит state в `toasting`.
- Через 5 с (`vi.advanceTimersByTime(5000)`) → `idle`.
- В `toasting` второй `requestToast` с другим `run_id` — state.result
  меняется, таймер перезапускается (проверить что через 3 с после второго
  вызова — ещё `toasting`).
- В `toasting` `open()` → `overlay`, таймер не срабатывает после.
- В `overlay` `requestToast(r')` игнорируется.
- `close()` из `overlay` → `idle`.
- `requestToast` с тем же `run_id` что текущий toasting — no-op.

### `Mps3DToast.test.tsx` (новый)

- Рендерит «Симуляция завершена», `distance`, `status` из result.
- Клик «Показать в 3D» — вызван `open(result)` контекста (мок).
- Клик ✕ — вызван `close()`.

### `Mps3DOverlay.test.tsx` (новый)

- Клик ✕ — вызван `close()`.
- Клик по backdrop — `close` **не** вызван (B1).
- `keyDown` Esc — `close` **не** вызван (B1).
- Когда `telemetry.length < 2` — рендерится fallback вместо Canvas.

### Интеграция в `MpsPage.test.tsx` (правка существующего)

- После `setPrimaryResult` с непустой telemetry — `screen.findByText(/Симуляция завершена/)` находит тост.
- Replay того же `run_id` тост не показывает (state остаётся `idle`).
- Кнопка «3D» в `TrajectoryView` `disabled` когда `result.telemetry === []`.

### Ручная проверка

- `npm run dev` → `/mps`.
- Запустить симуляцию в sim-режиме → проверить slide-up тост.
- Подождать 5 с → тост ушёл сам.
- Запустить ещё раз → нажать «Показать в 3D» → оверлей открылся,
  машинка едет по траектории, OrbitControls работает.
- Закрыть оверлей через ✕ → нажать «3D» в `TrajectoryView` → оверлей
  открылся снова с тем же result.
- Запустить симуляцию robot-режиме (если железо есть) → дождаться
  finished → проверить тост.

## 8. Список файлов

### Новые

| Файл | Назначение | Прим. размер |
|---|---|---|
| `compute_node/frontend/src/components/mps/Mps3DProvider.tsx` | FSM-контекст, тост + оверлей portal | ~110 строк |
| `compute_node/frontend/src/components/mps/Mps3DToast.tsx` | Тост-уведомление, T1 анимация, 5-с таймер | ~70 строк |
| `compute_node/frontend/src/components/mps/Mps3DOverlay.tsx` | Fullscreen modal с backdrop, O1 анимация | ~80 строк |
| `compute_node/frontend/src/components/mps/Mps3DScene.tsx` | R3F Canvas, AnimatedRobot, AnimatedTrail, освещение, маркеры | ~220 строк |
| `compute_node/frontend/src/components/mps/Mps3DProvider.test.tsx` | Unit-тесты FSM | ~120 строк |
| `compute_node/frontend/src/components/mps/Mps3DToast.test.tsx` | Unit-тесты тоста | ~60 строк |
| `compute_node/frontend/src/components/mps/Mps3DOverlay.test.tsx` | Unit-тесты оверлея + fallback | ~80 строк |

### Изменяемые

| Файл | Правки |
|---|---|
| `compute_node/frontend/src/pages/MpsPage.tsx` | Разделение на `MpsPage` (обёртка с провайдерами) + `MpsPageInner` (тело). Добавление `Mps3DProvider` и effect на `primaryResult` с ref-защитой. |
| `compute_node/frontend/src/pages/MpsPage.test.tsx` | Кейсы: тост появляется после нового result; replay не триггерит. |
| `compute_node/frontend/src/components/mps/TrajectoryView.tsx` | В шапке `<CardHeader>` — кнопка «3D» (icon + label), `disabled` если данных нет. |
| `compute_node/frontend/src/components/3d/RobotModel.tsx` | Добавить optional `noSmooth?: boolean`. По умолчанию `false` (текущее поведение). Когда `true` — позиция и поворот выставляются напрямую без lerp. |

### Не затрагиваем

- Backend (`compute_node/dashboard/`, `pi_nodes/`).
- REST/WS-контракт МПС (`docs/mps/api.md`).
- Страницу `/3d` (`Visualization3DPage.tsx`).
- Другие 3D-компоненты (`PathTrail`, `PlannedPathTrail`, `SlamMap3D`,
  `CoverageHeatmap`, `ImuVectors`, `InfoPanel`).

## 9. Допущения и риски

1. **`x[2]` в телеметрии — это θ в радианах (глобальный курс).**
   Подтверждается формулой в `TrajectoryView.tsx:17-23` (`s · cos(θ)`,
   `s · sin(θ)`). Если бы это была ошибка курса — формула была бы
   другой. Используем ту же интерпретацию.
2. **Длительность симуляции укладывается в разумное время для UX.**
   Для типичных запросов (D ≤ 5 м, v ≤ 0.3 м/с) — 15-30 с. Длиннее
   не оптимизируем.
3. **`Samurai.glb` уже загружен на странице.** Нет, `/mps` не грузит
   его. Будет загружен при первом открытии оверлея. Размер модели
   небольшой (см. `public/models/Samurai.glb`), `useGLTF.preload` в
   `RobotModel.tsx` сработает только если компонент смонтирован.
   **Решение:** оставить как есть — первый показ может иметь короткий
   suspense-fallback (~100-300 мс), это приемлемо. Если ощутимо плохо
   на практике — добавить preload-вызов в `Mps3DProvider` на маунте.
4. **OrbitControls и `makeDefault` в `Visualization3DPage`.** В `/3d`
   используется `<OrbitControls makeDefault/>`. У нас оверлей рендерится
   через portal, но Canvas изолирован — конфликта не будет. `makeDefault`
   в `Mps3DScene` **не** ставим.
5. **Performance.** `useFrame` 60 fps + бинарный поиск (O(log N)) +
   один `setState` на каждый кадр в `AnimatedRobot`. Для типичной
   телеметрии 200-600 точек — нагрузка минимальная. Замеры не нужны.
