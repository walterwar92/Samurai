# МПС 3D-Оверлей — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Добавить на страницу `/mps` тост-предложение «показать симуляцию в 3D» (снизу-справа, 5 секунд) и fullscreen-оверлей с R3F-сценой, в которой модель `Samurai.glb` едет по траектории `(s·cos θ, s·sin θ)` из телеметрии симуляции в реальном времени.

**Architecture:** Новый `Mps3DProvider` (FSM `idle / toasting / overlay`) внутри `MpsPage` рядом с существующим `MpsHighlightProvider`. Тост и оверлей рендерятся через `createPortal` в `document.body`. Сцена 3D — R3F + `useGLTF` + `useFrame` с интерполяцией pose по `elapsed` времени. Реюзаем существующий `RobotModel`, расширяем его опциональным prop'ом `noSmooth`.

**Tech Stack:** React 19 + TypeScript + Vite + Tailwind + shadcn/ui + Three.js + @react-three/fiber + @react-three/drei + lucide-react + Vitest + @testing-library/react (всё уже подключено).

**Spec:** [`docs/superpowers/specs/2026-05-11-mps-3d-overlay-design.md`](../specs/2026-05-11-mps-3d-overlay-design.md)

---

## Глобальные конвенции

- **Рабочий каталог** для npm-команд: `compute_node/frontend/`. Запускать оттуда.
- **Ветка:** `feat/redesign` (текущая активная ветка). Все коммиты в неё.
- **Коммит-стиль:** `feat(mps-3d): ...` или `test(mps-3d): ...`. Следуем конвенции `<тип>(<scope>): <описание>` на русском.
- **Импорты:** алиас `@/...` → `compute_node/frontend/src/`.
- **User preferences:** НЕ добавлять `Co-Authored-By: Claude` и «Generated with Claude Code» в коммиты.
- **TDD:** для всего что можно тестировать в jsdom (Provider, Toast, Overlay-обвязка, кнопка в TrajectoryView, интеграция в MpsPage). Для `Mps3DScene` юнит-тестов **нет** — R3F/WebGL в jsdom не работает; проверка ручная.

---

## File Structure

```
compute_node/frontend/src/
├── components/
│   ├── 3d/
│   │   └── RobotModel.tsx                ← MODIFY (добавить optional noSmooth)
│   └── mps/
│       ├── Mps3DProvider.tsx             ← NEW: FSM-провайдер + useMps3D хук + portal-рендер тоста/оверлея
│       ├── Mps3DProvider.test.tsx        ← NEW: тесты FSM
│       ├── Mps3DToast.tsx                ← NEW: тост-карточка (snowdrift snowboarder T1 slide-up, 5с)
│       ├── Mps3DToast.test.tsx           ← NEW
│       ├── Mps3DOverlay.tsx              ← NEW: backdrop+панель, только ✕ закрывает
│       ├── Mps3DOverlay.test.tsx         ← NEW
│       ├── Mps3DScene.tsx                ← NEW: R3F Canvas, AnimatedRobot, AnimatedTrail, маркеры
│       └── TrajectoryView.tsx            ← MODIFY (кнопка «3D» в CardHeader)
└── pages/
    ├── MpsPage.tsx                       ← MODIFY (разделить на MpsPage + MpsPageInner, добавить провайдер и useEffect)
    └── MpsPage.test.tsx                  ← MODIFY (кейсы тоста/replay)
```

Ответственности:
- **`Mps3DProvider`** — единственный владелец FSM-state. Управляет 5-секундным таймером тоста. Рендерит portal-ы тоста и оверлея.
- **`Mps3DToast`** — чистая презентация: заголовок, статус, прогресс-полоска, две кнопки. Через `useMps3D()` берёт `result/open/close`.
- **`Mps3DOverlay`** — backdrop + рамка модала. Внутри монтирует `Mps3DScene` или fallback при пустой телеметрии. Закрытие только ✕.
- **`Mps3DScene`** — самостоятельная R3F-сцена. Не знает про provider/state; принимает `telemetry`, `distance`, `status` через props.
- **`RobotModel`** — добавляется `noSmooth?: boolean`. Существующие вызовы остаются без изменений.
- **`TrajectoryView`** — добавляется icon-кнопка справа в шапке.
- **`MpsPage`** — разделяется на обёртку (провайдеры) и `MpsPageInner` (текущее тело), потому что `useMps3D` нельзя вызвать в том же компоненте что рендерит `<Mps3DProvider>`.

---

## Task 1: Расширить `RobotModel` опциональным `noSmooth` prop

**Цель:** Дать `Mps3DScene` возможность напрямую выставить pose без lerp-сглаживания (которое нужно для real-time робота, но даёт фазовый сдвиг для симуляции). Существующее поведение `RobotModel` на `/3d` не меняется.

**Files:**
- Modify: `compute_node/frontend/src/components/3d/RobotModel.tsx`

- [ ] **Step 1: Подтвердить текущий интерфейс**

Открыть `compute_node/frontend/src/components/3d/RobotModel.tsx`. Убедиться что interface `RobotModelProps` имеет поля `yaw, pitch, roll, posX, posY, stationary?`. Это база.

- [ ] **Step 2: Добавить `noSmooth?: boolean` в RobotModelProps**

В блок `interface RobotModelProps` (строки 8-15) добавить новое поле:

```ts
interface RobotModelProps {
  yaw: number    // degrees
  pitch: number  // degrees
  roll: number   // degrees
  posX: number   // meters
  posY: number   // meters
  stationary?: boolean
  /**
   * Если true — позиция и поворот выставляются напрямую, без lerp-сглаживания
   * и без dead-zone. Полезно для воспроизведения готовой телеметрии, где
   * сглаживание добавляет лишнюю задержку. По умолчанию false (сохраняется
   * исходное поведение для real-time робота на странице /3d).
   */
  noSmooth?: boolean
}
```

- [ ] **Step 3: Принять proп в сигнатуре**

Обновить деструктуризацию функции (строка 28):

```ts
export function RobotModel({ yaw, pitch, roll, posX, posY, stationary = false, noSmooth = false }: RobotModelProps) {
```

- [ ] **Step 4: Прокинуть `noSmooth` в `propsRef`**

Обновить инициализацию `propsRef` (строка 47):

```ts
const propsRef = useRef({ yaw, pitch, roll, posX, posY, stationary, noSmooth })
propsRef.current = { yaw, pitch, roll, posX, posY, stationary, noSmooth }
```

- [ ] **Step 5: Ветвь без сглаживания в `useFrame`**

Внутри `useFrame` (строка 56), сразу после `const p = propsRef.current` добавить early-return когда `noSmooth`:

```ts
useFrame(() => {
  if (!groupRef.current) return

  const p = propsRef.current

  // Direct mode for replay scenarios (no lerp, no deadzone)
  if (p.noSmooth) {
    smoothPos.current.set(p.posX, 0.05, -p.posY)
    smoothRot.current.set(
      p.pitch * DEG2RAD,
      -p.yaw * DEG2RAD + Math.PI / 2,
      p.roll * DEG2RAD,
      'YXZ',
    )
    groupRef.current.position.copy(smoothPos.current)
    groupRef.current.rotation.set(
      smoothRot.current.x,
      smoothRot.current.y,
      smoothRot.current.z,
      'YXZ',
    )
    return
  }

  // existing smoothing path (unchanged below this line)
  const targetX = p.posX
  // ...rest of existing code stays as-is...
})
```

Ничего ниже не меняется — существующий smoothing-код остаётся как есть.

- [ ] **Step 6: Проверить TypeScript**

Запустить из `compute_node/frontend/`:

```bash
npm run build
```

Ожидаемо: сборка проходит. Существующие вызовы `<RobotModel ...>` на `/3d` не передают `noSmooth` — компилятор не должен ругаться, т.к. поле optional.

- [ ] **Step 7: Коммит**

```bash
git add compute_node/frontend/src/components/3d/RobotModel.tsx
git commit -m "feat(mps-3d): добавить optional noSmooth prop в RobotModel для прямого режима без lerp"
```

---

## Task 2: Создать `Mps3DProvider` (FSM + portal-render обвязка) — TDD

**Цель:** Один источник правды для состояний `idle / toasting / overlay`. Управление 5-секундным таймером. Защита от двойного триггера.

**Files:**
- Create: `compute_node/frontend/src/components/mps/Mps3DProvider.tsx`
- Test:   `compute_node/frontend/src/components/mps/Mps3DProvider.test.tsx`

### Шаг A: Скелет с заглушкой портала

- [ ] **Step 1: Написать падающий тест для FSM-перехода `idle → toasting`**

Создать `compute_node/frontend/src/components/mps/Mps3DProvider.test.tsx`:

```tsx
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { act, render, renderHook } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

function makeResult(runId: string = 'r1'): MpsScenarioResult {
  return {
    run_id: runId,
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: {
      A: [], B: [], C: [], D: [],
      Q_diag: [], R_diag: [],
      horizon_N: 20, u_min: [], u_max: [],
      schema_version: '1.0',
    },
    telemetry: [
      { t: 0,   x: [0,  0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
      { t: 0.2, x: [0.2, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.8 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

const wrapper = ({ children }: { children: ReactNode }) => <Mps3DProvider>{children}</Mps3DProvider>

beforeEach(() => { vi.useFakeTimers() })
afterEach(() => { vi.useRealTimers() })

describe('Mps3DProvider — FSM', () => {
  it('requestToast переводит state в toasting', () => {
    const { result } = renderHook(() => useMps3D(), { wrapper })
    expect(result.current.state.kind).toBe('idle')
    act(() => { result.current.requestToast(makeResult('r1')) })
    expect(result.current.state.kind).toBe('toasting')
    if (result.current.state.kind === 'toasting') {
      expect(result.current.state.result.run_id).toBe('r1')
    }
  })
})
```

- [ ] **Step 2: Запустить и убедиться что падает**

```bash
cd compute_node/frontend
npx vitest run src/components/mps/Mps3DProvider.test.tsx
```

Ожидаемо: FAIL с ошибкой импорта (модуля нет).

- [ ] **Step 3: Минимальная реализация `Mps3DProvider`**

Создать `compute_node/frontend/src/components/mps/Mps3DProvider.tsx`:

```tsx
import { createContext, useCallback, useContext, useEffect, useReducer } from 'react'
import type { ReactNode } from 'react'
import { createPortal } from 'react-dom'
import type { MpsScenarioResult } from '@/types/mps'

export type Mps3DState =
  | { kind: 'idle' }
  | { kind: 'toasting'; result: MpsScenarioResult; startedAt: number }
  | { kind: 'overlay';  result: MpsScenarioResult }

type Action =
  | { type: 'REQUEST_TOAST'; result: MpsScenarioResult; now: number }
  | { type: 'OPEN'; result: MpsScenarioResult }
  | { type: 'CLOSE' }
  | { type: 'TIMEOUT' }

const TOAST_MS = 5000

function reducer(state: Mps3DState, action: Action): Mps3DState {
  switch (action.type) {
    case 'REQUEST_TOAST': {
      // Игнор пока открыт overlay
      if (state.kind === 'overlay') return state
      // No-op если уже toasting с тем же run_id
      if (state.kind === 'toasting' && state.result.run_id === action.result.run_id) return state
      return { kind: 'toasting', result: action.result, startedAt: action.now }
    }
    case 'OPEN':
      return { kind: 'overlay', result: action.result }
    case 'CLOSE':
      return { kind: 'idle' }
    case 'TIMEOUT':
      // Защита от устаревшего таймера: переход в idle только если всё ещё toasting
      return state.kind === 'toasting' ? { kind: 'idle' } : state
    default:
      return state
  }
}

interface Mps3DContextValue {
  state: Mps3DState
  requestToast: (result: MpsScenarioResult) => void
  open: (result: MpsScenarioResult) => void
  close: () => void
}

const Mps3DContext = createContext<Mps3DContextValue | null>(null)

export function Mps3DProvider({ children }: { children: ReactNode }) {
  const [state, dispatch] = useReducer(reducer, { kind: 'idle' } as Mps3DState)

  const requestToast = useCallback((result: MpsScenarioResult) => {
    dispatch({ type: 'REQUEST_TOAST', result, now: Date.now() })
  }, [])

  const open = useCallback((result: MpsScenarioResult) => {
    dispatch({ type: 'OPEN', result })
  }, [])

  const close = useCallback(() => {
    dispatch({ type: 'CLOSE' })
  }, [])

  // 5-секундный таймер: запускается на каждый вход в toasting,
  // зависимость на startedAt — при замене result (новый run_id во время
  // toasting) startedAt обновляется и таймер пересоздаётся.
  useEffect(() => {
    if (state.kind !== 'toasting') return
    const timer = setTimeout(() => dispatch({ type: 'TIMEOUT' }), TOAST_MS)
    return () => clearTimeout(timer)
  }, [state.kind, state.kind === 'toasting' ? state.startedAt : null])

  return (
    <Mps3DContext.Provider value={{ state, requestToast, open, close }}>
      {children}
      {/* Тост и оверлей будут добавлены ниже после соответствующих тасков */}
    </Mps3DContext.Provider>
  )
}

export function useMps3D(): Mps3DContextValue {
  const ctx = useContext(Mps3DContext)
  if (ctx === null) {
    throw new Error('useMps3D must be used within <Mps3DProvider>')
  }
  return ctx
}
```

- [ ] **Step 4: Запустить и убедиться что тест проходит**

```bash
npx vitest run src/components/mps/Mps3DProvider.test.tsx
```

Ожидаемо: PASS (1 test).

- [ ] **Step 5: Добавить остальные FSM-тесты**

Добавить в `Mps3DProvider.test.tsx` (внутри `describe('Mps3DProvider — FSM')`):

```tsx
it('через 5 секунд toasting сам уходит в idle', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  act(() => { result.current.requestToast(makeResult('r1')) })
  expect(result.current.state.kind).toBe('toasting')
  act(() => { vi.advanceTimersByTime(5000) })
  expect(result.current.state.kind).toBe('idle')
})

it('повторный requestToast с другим run_id заменяет result и перезапускает таймер', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  act(() => { result.current.requestToast(makeResult('r1')) })
  act(() => { vi.advanceTimersByTime(3000) })
  act(() => { result.current.requestToast(makeResult('r2')) })
  expect(result.current.state.kind).toBe('toasting')
  if (result.current.state.kind === 'toasting') {
    expect(result.current.state.result.run_id).toBe('r2')
  }
  // через ещё 3с — таймер от r1 (если бы не перезапустился) сработал бы; проверяем что нет
  act(() => { vi.advanceTimersByTime(3000) })
  expect(result.current.state.kind).toBe('toasting')
  // через ещё 2с от replacement — TOAST_MS вышел, перешли в idle
  act(() => { vi.advanceTimersByTime(2000) })
  expect(result.current.state.kind).toBe('idle')
})

it('повторный requestToast с тем же run_id no-op (таймер не перезапускается)', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  act(() => { result.current.requestToast(makeResult('r1')) })
  act(() => { vi.advanceTimersByTime(4000) })
  act(() => { result.current.requestToast(makeResult('r1')) })  // тот же id
  act(() => { vi.advanceTimersByTime(1000) })                    // суммарно 5с
  expect(result.current.state.kind).toBe('idle')
})

it('open() из toasting переводит в overlay и отменяет таймер', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  const r = makeResult('r1')
  act(() => { result.current.requestToast(r) })
  act(() => { result.current.open(r) })
  expect(result.current.state.kind).toBe('overlay')
  act(() => { vi.advanceTimersByTime(10000) })
  expect(result.current.state.kind).toBe('overlay')  // таймер не сработал
})

it('requestToast пока открыт overlay игнорируется (C1)', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  const r1 = makeResult('r1')
  const r2 = makeResult('r2')
  act(() => { result.current.open(r1) })
  act(() => { result.current.requestToast(r2) })
  expect(result.current.state.kind).toBe('overlay')
  if (result.current.state.kind === 'overlay') {
    expect(result.current.state.result.run_id).toBe('r1')  // не подменился
  }
})

it('close() из overlay возвращает в idle', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  act(() => { result.current.open(makeResult('r1')) })
  act(() => { result.current.close() })
  expect(result.current.state.kind).toBe('idle')
})

it('open(r) из idle сразу переводит в overlay (для R2-кнопки)', () => {
  const { result } = renderHook(() => useMps3D(), { wrapper })
  act(() => { result.current.open(makeResult('r1')) })
  expect(result.current.state.kind).toBe('overlay')
})

it('useMps3D вне Provider бросает', () => {
  // renderHook без wrapper
  expect(() => renderHook(() => useMps3D())).toThrow(/Mps3DProvider/)
})
```

- [ ] **Step 6: Запустить все тесты FSM**

```bash
npx vitest run src/components/mps/Mps3DProvider.test.tsx
```

Ожидаемо: PASS (8 тестов).

- [ ] **Step 7: Коммит**

```bash
git add compute_node/frontend/src/components/mps/Mps3DProvider.tsx compute_node/frontend/src/components/mps/Mps3DProvider.test.tsx
git commit -m "feat(mps-3d): Mps3DProvider с FSM idle/toasting/overlay + 5-секундный таймер"
```

---

## Task 3: Создать `Mps3DToast` — TDD

**Цель:** Карточка-уведомление снизу-справа с slide-up анимацией, прогресс-полоской и двумя действиями (Показать / ✕).

**Files:**
- Create: `compute_node/frontend/src/components/mps/Mps3DToast.tsx`
- Test:   `compute_node/frontend/src/components/mps/Mps3DToast.test.tsx`

- [ ] **Step 1: Написать падающий тест на рендер**

Создать `compute_node/frontend/src/components/mps/Mps3DToast.test.tsx`:

```tsx
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { render, screen, fireEvent, act } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

function makeResult(runId = 'r1', status: MpsScenarioResult['status'] = 'reached'): MpsScenarioResult {
  return {
    run_id: runId,
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status,
    request: { distance: 2.5, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: [
      { t: 0, x: [0, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.5 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.4 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

// Тестовый компонент, поднимающий тост через useMps3D
function ToastHarness() {
  const mps3D = useMps3D()
  return (
    <button data-testid="trigger" onClick={() => mps3D.requestToast(makeResult('r1'))}>
      trigger
    </button>
  )
}

const renderWithProvider = (children: ReactNode = <ToastHarness />) =>
  render(<Mps3DProvider>{children}</Mps3DProvider>)

beforeEach(() => { vi.useFakeTimers() })
afterEach(() => { vi.useRealTimers() })

describe('Mps3DToast', () => {
  it('появляется в DOM после requestToast', () => {
    renderWithProvider()
    expect(screen.queryByText(/Симуляция завершена/i)).toBeNull()
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
  })
})
```

- [ ] **Step 2: Запустить — должен упасть**

```bash
npx vitest run src/components/mps/Mps3DToast.test.tsx
```

Ожидаемо: FAIL (текст «Симуляция завершена» не найден — портал ещё не реализован).

- [ ] **Step 3: Создать `Mps3DToast.tsx`**

Создать `compute_node/frontend/src/components/mps/Mps3DToast.tsx`:

```tsx
import { useEffect, useState } from 'react'
import { createPortal } from 'react-dom'
import { Box, X } from 'lucide-react'
import type { MpsScenarioResult, ScenarioStatus } from '@/types/mps'
import { useMps3D } from './Mps3DProvider'

const TOAST_MS = 5000

function statusText(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'достигнуто'
    case 'aborted':  return 'прервано'
    case 'timeout':  return 'таймаут'
    case 'error':    return 'ошибка'
    case 'running':  return 'выполняется'
  }
}

function statusColor(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'text-emerald-400'
    case 'aborted':
    case 'timeout':  return 'text-amber-400'
    case 'error':    return 'text-red-400'
    case 'running':  return 'text-zinc-300'
  }
}

interface ToastCardProps {
  result: MpsScenarioResult
  onOpen: () => void
  onClose: () => void
}

function ToastCard({ result, onOpen, onClose }: ToastCardProps) {
  // Анимация появления: переход с translate-y-4 opacity-0 → translate-y-0 opacity-100.
  // Состояние entered флипается на mount → следующий тик React'а.
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  return (
    <div
      className={[
        'fixed bottom-4 right-4 z-[60] w-[320px]',
        'rounded border border-zinc-700 bg-zinc-900/95 shadow-xl backdrop-blur',
        'transition-all duration-250 ease-out',
        entered ? 'translate-y-0 opacity-100' : 'translate-y-4 opacity-0',
      ].join(' ')}
      role="dialog"
      aria-label="Симуляция завершена"
    >
      <div className="p-3">
        <div className="flex items-start gap-2">
          <Box className="w-4 h-4 mt-0.5 text-zinc-300 shrink-0" />
          <div className="flex-1 min-w-0">
            <div className="text-sm font-medium text-zinc-100">Симуляция завершена</div>
            <div className="text-xs text-zinc-400 mt-0.5">
              s = {result.request.distance.toFixed(2)} м •{' '}
              <span className={statusColor(result.status)}>{statusText(result.status)}</span>
            </div>
          </div>
          <button
            type="button"
            onClick={onClose}
            className="text-zinc-500 hover:text-zinc-200 transition-colors p-0.5 -m-0.5"
            aria-label="Закрыть"
          >
            <X className="w-4 h-4" />
          </button>
        </div>
        <button
          type="button"
          onClick={onOpen}
          className="mt-2 w-full rounded bg-blue-600 hover:bg-blue-500 transition-colors px-3 py-1.5 text-xs font-medium text-white"
        >
          Показать в 3D
        </button>
      </div>
      {/* Прогресс-полоска: убывает за TOAST_MS секунд */}
      <div className="h-0.5 bg-zinc-800 overflow-hidden rounded-b">
        <div
          className="h-full bg-blue-500/70"
          style={{
            animation: `mps3d-toast-progress ${TOAST_MS}ms linear forwards`,
            width: '100%',
          }}
        />
      </div>
      <style>{`
        @keyframes mps3d-toast-progress {
          from { width: 100%; }
          to   { width: 0%; }
        }
      `}</style>
    </div>
  )
}

export function Mps3DToast() {
  const { state, open, close } = useMps3D()
  if (state.kind !== 'toasting') return null
  return createPortal(
    <ToastCard
      result={state.result}
      onOpen={() => open(state.result)}
      onClose={close}
    />,
    document.body,
  )
}
```

- [ ] **Step 4: Подключить `Mps3DToast` в `Mps3DProvider`**

Открыть `compute_node/frontend/src/components/mps/Mps3DProvider.tsx`. В верх файла добавить импорт:

```tsx
import { Mps3DToast } from './Mps3DToast'
```

Внутри JSX `Mps3DContext.Provider` заменить комментарий-плейсхолдер на рендер тоста:

```tsx
return (
  <Mps3DContext.Provider value={{ state, requestToast, open, close }}>
    {children}
    <Mps3DToast />
  </Mps3DContext.Provider>
)
```

- [ ] **Step 5: Запустить тест — должен пройти**

```bash
npx vitest run src/components/mps/Mps3DToast.test.tsx
```

Ожидаемо: PASS.

- [ ] **Step 6: Добавить тесты на действия и таймер**

В `Mps3DToast.test.tsx` добавить:

```tsx
it('клик «Показать в 3D» переводит в overlay-state провайдера', () => {
  function StateProbe() {
    const { state } = useMps3D()
    return <div data-testid="kind">{state.kind}</div>
  }
  renderWithProvider(
    <>
      <ToastHarness />
      <StateProbe />
    </>,
  )
  act(() => { screen.getByTestId('trigger').click() })
  expect(screen.getByTestId('kind').textContent).toBe('toasting')
  act(() => { screen.getByRole('button', { name: /Показать в 3D/i }).click() })
  expect(screen.getByTestId('kind').textContent).toBe('overlay')
})

it('клик ✕ возвращает в idle', () => {
  function StateProbe() {
    const { state } = useMps3D()
    return <div data-testid="kind">{state.kind}</div>
  }
  renderWithProvider(
    <>
      <ToastHarness />
      <StateProbe />
    </>,
  )
  act(() => { screen.getByTestId('trigger').click() })
  act(() => { screen.getByRole('button', { name: /Закрыть/i }).click() })
  expect(screen.getByTestId('kind').textContent).toBe('idle')
})

it('подзаголовок содержит дистанцию и статус', () => {
  renderWithProvider()
  act(() => { screen.getByTestId('trigger').click() })
  // distance=2.5 → "s = 2.50 м"
  expect(screen.getByText(/s = 2\.50 м/)).toBeInTheDocument()
  expect(screen.getByText(/достигнуто/)).toBeInTheDocument()
})
```

- [ ] **Step 7: Запустить все тесты тоста**

```bash
npx vitest run src/components/mps/Mps3DToast.test.tsx
```

Ожидаемо: PASS (4 теста).

- [ ] **Step 8: Коммит**

```bash
git add compute_node/frontend/src/components/mps/Mps3DToast.tsx compute_node/frontend/src/components/mps/Mps3DToast.test.tsx compute_node/frontend/src/components/mps/Mps3DProvider.tsx
git commit -m "feat(mps-3d): Mps3DToast — карточка с прогресс-полоской и slide-up анимацией"
```

---

## Task 4: Создать `Mps3DOverlay` — TDD

**Цель:** Fullscreen backdrop + панель ~90vw × 85vh с заголовком и ✕. Внутри монтирует `Mps3DScene` или fallback при пустой/невалидной телеметрии. Esc и клик по backdrop **не** закрывают (строго B1).

**Files:**
- Create: `compute_node/frontend/src/components/mps/Mps3DOverlay.tsx`
- Test:   `compute_node/frontend/src/components/mps/Mps3DOverlay.test.tsx`

- [ ] **Step 1: Написать тесты (fallback + закрытие)**

Создать `compute_node/frontend/src/components/mps/Mps3DOverlay.test.tsx`:

```tsx
import { describe, it, expect, vi, beforeAll } from 'vitest'
import { render, screen, fireEvent, act } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

// Заглушка Mps3DScene: R3F/WebGL в jsdom не работает, поэтому при тестах
// заменяем сцену на пустой плейсхолдер чтобы не падать на Canvas.
vi.mock('./Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="scene-stub">scene</div>,
}))

function makeResult(opts: Partial<{ runId: string; telemetry: MpsTelemetryPoint[] }> = {}): MpsScenarioResult {
  return {
    run_id: opts.runId ?? 'r1',
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: opts.telemetry ?? [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

function OverlayHarness({ telemetry }: { telemetry?: MpsTelemetryPoint[] }) {
  const mps3D = useMps3D()
  return (
    <button data-testid="trigger" onClick={() => mps3D.open(makeResult({ telemetry }))}>
      open
    </button>
  )
}

const renderWithProvider = (node: ReactNode) =>
  render(<Mps3DProvider>{node}</Mps3DProvider>)

describe('Mps3DOverlay', () => {
  it('рендерит сцену когда есть валидная телеметрия (>= 2 точек)', () => {
    renderWithProvider(<OverlayHarness />)
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByTestId('scene-stub')).toBeInTheDocument()
    expect(screen.queryByText(/нет валидных данных/i)).toBeNull()
  })

  it('рендерит fallback когда телеметрия пуста', () => {
    renderWithProvider(<OverlayHarness telemetry={[]} />)
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.queryByTestId('scene-stub')).toBeNull()
    expect(screen.getByText(/нет валидных данных/i)).toBeInTheDocument()
  })

  it('клик ✕ закрывает (возвращает в idle)', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
    act(() => { screen.getByRole('button', { name: /Закрыть оверлей/i }).click() })
    expect(screen.getByTestId('kind').textContent).toBe('idle')
  })

  it('клик по backdrop НЕ закрывает (B1)', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    const backdrop = screen.getByTestId('mps3d-backdrop')
    act(() => { fireEvent.click(backdrop) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })

  it('Esc НЕ закрывает (B1)', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    act(() => { fireEvent.keyDown(document, { key: 'Escape' }) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })
})
```

- [ ] **Step 2: Запустить — все упадут (нет Overlay/Scene)**

```bash
npx vitest run src/components/mps/Mps3DOverlay.test.tsx
```

Ожидаемо: FAIL — модулей нет.

- [ ] **Step 3: Создать `Mps3DOverlay.tsx`**

Создать `compute_node/frontend/src/components/mps/Mps3DOverlay.tsx`:

```tsx
import { useEffect, useMemo, useState } from 'react'
import { createPortal } from 'react-dom'
import { X } from 'lucide-react'
import type { MpsScenarioResult, MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'
import { useMps3D } from './Mps3DProvider'
import { Mps3DScene } from './Mps3DScene'

function statusText(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'достигнуто'
    case 'aborted':  return 'прервано'
    case 'timeout':  return 'таймаут'
    case 'error':    return 'ошибка'
    case 'running':  return 'выполняется'
  }
}

function statusColor(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'text-emerald-400'
    case 'aborted':
    case 'timeout':  return 'text-amber-400'
    case 'error':    return 'text-red-400'
    case 'running':  return 'text-zinc-300'
  }
}

function validTelemetryCount(t: MpsTelemetryPoint[]): number {
  let count = 0
  for (const p of t) {
    if (
      Number.isFinite(p.t) &&
      Number.isFinite(p.x?.[0]) &&
      Number.isFinite(p.x?.[2])
    ) count++
  }
  return count
}

interface OverlayPanelProps {
  result: MpsScenarioResult
  onClose: () => void
}

function OverlayPanel({ result, onClose }: OverlayPanelProps) {
  // Анимация появления: backdrop fade + panel scale-95→100
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  const hasValidTelemetry = useMemo(() => validTelemetryCount(result.telemetry) >= 2, [result.telemetry])

  return (
    <div
      data-testid="mps3d-backdrop"
      className={[
        'fixed inset-0 z-[70] flex items-start justify-center',
        'bg-black/70 transition-opacity duration-250',
        entered ? 'opacity-100' : 'opacity-0',
      ].join(' ')}
      // Клик по backdrop НЕ закрывает (B1). onClick намеренно отсутствует.
    >
      <div
        className={[
          'relative mt-[7.5vh] w-[90vw] h-[85vh] overflow-hidden',
          'rounded-lg border border-zinc-700 bg-[#1a1a2e]',
          'transition-transform duration-250 ease-out',
          entered ? 'scale-100' : 'scale-95',
        ].join(' ')}
        role="dialog"
        aria-label="3D-просмотр траектории"
      >
        {/* Шапка */}
        <div className="absolute top-0 left-0 right-0 z-10 flex items-center justify-between px-4 py-2 border-b border-zinc-700 bg-zinc-900/80 backdrop-blur">
          <div className="flex items-center gap-3">
            <span className="text-sm font-medium text-zinc-100">3D-просмотр траектории</span>
            <span className="text-xs text-zinc-400">
              s = {result.request.distance.toFixed(2)} м •{' '}
              <span className={statusColor(result.status)}>{statusText(result.status)}</span>
            </span>
          </div>
          <button
            type="button"
            onClick={onClose}
            className="text-zinc-400 hover:text-zinc-100 transition-colors p-1 -m-1"
            aria-label="Закрыть оверлей"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* Содержимое: сцена или fallback */}
        <div className="absolute inset-0 pt-10">
          {hasValidTelemetry ? (
            <Mps3DScene
              telemetry={result.telemetry}
              distance={result.request.distance}
              status={result.status}
            />
          ) : (
            <div className="flex h-full items-center justify-center">
              <p className="text-sm text-zinc-400">Нет валидных данных для 3D-визуализации</p>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

export function Mps3DOverlay() {
  const { state, close } = useMps3D()
  if (state.kind !== 'overlay') return null
  return createPortal(
    <OverlayPanel result={state.result} onClose={close} />,
    document.body,
  )
}
```

- [ ] **Step 4: Создать заглушку `Mps3DScene.tsx`**

Для прохождения тестов на этом этапе создаём минимальный экспорт. Полная реализация — в Task 5.

Создать `compute_node/frontend/src/components/mps/Mps3DScene.tsx`:

```tsx
import type { MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'

interface Mps3DSceneProps {
  telemetry: MpsTelemetryPoint[]
  distance: number
  status: ScenarioStatus
}

// Заглушка — полная реализация в Task 5
export function Mps3DScene(_props: Mps3DSceneProps) {
  return <div data-scene-placeholder="true" className="w-full h-full bg-[#1a1a2e]" />
}
```

- [ ] **Step 5: Подключить `Mps3DOverlay` в `Mps3DProvider`**

Открыть `compute_node/frontend/src/components/mps/Mps3DProvider.tsx`. Добавить импорт:

```tsx
import { Mps3DOverlay } from './Mps3DOverlay'
```

И в JSX внутри Provider, после `<Mps3DToast />`:

```tsx
return (
  <Mps3DContext.Provider value={{ state, requestToast, open, close }}>
    {children}
    <Mps3DToast />
    <Mps3DOverlay />
  </Mps3DContext.Provider>
)
```

- [ ] **Step 6: Запустить тесты оверлея**

```bash
npx vitest run src/components/mps/Mps3DOverlay.test.tsx
```

Ожидаемо: PASS (5 тестов).

- [ ] **Step 7: Запустить всю тестовую папку**

```bash
npx vitest run src/components/mps/
```

Ожидаемо: PASS (Provider 8 + Toast 4 + Overlay 5 = 17 тестов).

- [ ] **Step 8: Коммит**

```bash
git add compute_node/frontend/src/components/mps/Mps3DOverlay.tsx compute_node/frontend/src/components/mps/Mps3DOverlay.test.tsx compute_node/frontend/src/components/mps/Mps3DScene.tsx compute_node/frontend/src/components/mps/Mps3DProvider.tsx
git commit -m "feat(mps-3d): Mps3DOverlay — backdrop+панель, закрытие только по ✕, fallback при пустой телеметрии"
```

---

## Task 5: Реализовать `Mps3DScene` (R3F + анимация)

**Цель:** Заменить заглушку из Task 4 на полную R3F-сцену: грид, освещение, маркеры старта/цели, `AnimatedRobot` (интерполирует pose из telemetry), `AnimatedTrail` (нарастающая линия), визибилити-фикс.

**Files:**
- Modify: `compute_node/frontend/src/components/mps/Mps3DScene.tsx`

**Без юнит-тестов** (R3F/Canvas/WebGL в jsdom не работают). Проверка ручная — в Task 8.

- [ ] **Step 1: Импорты и базовая структура**

Открыть `compute_node/frontend/src/components/mps/Mps3DScene.tsx` и полностью заменить содержимое:

```tsx
import { Suspense, useEffect, useMemo, useRef, useState } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, Grid } from '@react-three/drei'
import * as THREE from 'three'
import type { MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'
import { RobotModel } from '@/components/3d/RobotModel'

interface Mps3DSceneProps {
  telemetry: MpsTelemetryPoint[]
  distance: number
  status: ScenarioStatus
}

interface Sample {
  t: number
  s: number
  theta: number
  x: number
  y: number
}

interface LiveProgress {
  t: number
  s: number
  index: number  // последний пройденный индекс семпла (для AnimatedTrail)
}

function prepareSamples(telemetry: MpsTelemetryPoint[]): Sample[] {
  const out: Sample[] = []
  for (const p of telemetry) {
    const t = p.t
    const s = p.x?.[0]
    const theta = p.x?.[2]
    if (
      !Number.isFinite(t) ||
      !Number.isFinite(s) ||
      !Number.isFinite(theta)
    ) continue
    out.push({
      t,
      s: s as number,
      theta: theta as number,
      x: (s as number) * Math.cos(theta as number),
      y: (s as number) * Math.sin(theta as number),
    })
  }
  return out
}

interface AnimatedRobotProps {
  samples: Sample[]
  progressRef: React.MutableRefObject<LiveProgress | null>
}

function AnimatedRobot({ samples, progressRef }: AnimatedRobotProps) {
  const startTimeRef = useRef<number | null>(null)
  const [pose, setPose] = useState({
    posX: samples[0]?.x ?? 0,
    posY: samples[0]?.y ?? 0,
    yawDeg: ((samples[0]?.theta ?? 0) * 180) / Math.PI,
    stationary: false,
  })

  // Visibility-фикс: при возврате во вкладку пересчитываем startTime,
  // чтобы машинка не «прыгнула» к финалу.
  useEffect(() => {
    function onVisibility() {
      if (document.visibilityState === 'visible' && startTimeRef.current !== null) {
        const lastElapsed = progressRef.current?.t ?? 0
        startTimeRef.current = performance.now() - lastElapsed * 1000
      }
    }
    document.addEventListener('visibilitychange', onVisibility)
    return () => document.removeEventListener('visibilitychange', onVisibility)
  }, [progressRef])

  useFrame(() => {
    if (samples.length === 0) return
    if (startTimeRef.current === null) {
      startTimeRef.current = performance.now()
    }
    const elapsed = (performance.now() - startTimeRef.current) / 1000
    const last = samples[samples.length - 1]

    if (elapsed >= last.t) {
      setPose({
        posX: last.x,
        posY: last.y,
        yawDeg: (last.theta * 180) / Math.PI,
        stationary: true,
      })
      progressRef.current = { t: last.t, s: last.s, index: samples.length - 1 }
      return
    }

    // Бинарный поиск интервала [lo, hi]: samples[lo].t <= elapsed < samples[hi].t
    let lo = 0
    let hi = samples.length - 1
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1
      if (samples[mid].t <= elapsed) lo = mid
      else hi = mid
    }
    const a = samples[lo]
    const b = samples[hi]
    const span = b.t - a.t
    const alpha = span > 0 ? (elapsed - a.t) / span : 0
    const x = a.x + (b.x - a.x) * alpha
    const y = a.y + (b.y - a.y) * alpha
    const theta = a.theta + (b.theta - a.theta) * alpha
    const s = a.s + (b.s - a.s) * alpha
    setPose({
      posX: x,
      posY: y,
      yawDeg: (theta * 180) / Math.PI,
      stationary: false,
    })
    progressRef.current = { t: elapsed, s, index: lo }
  })

  return (
    <RobotModel
      yaw={pose.yawDeg}
      pitch={0}
      roll={0}
      posX={pose.posX}
      posY={pose.posY}
      stationary={pose.stationary}
      noSmooth
    />
  )
}

interface AnimatedTrailProps {
  samples: Sample[]
  progressRef: React.MutableRefObject<LiveProgress | null>
}

function AnimatedTrail({ samples, progressRef }: AnimatedTrailProps) {
  const geometryRef = useRef<THREE.BufferGeometry | null>(null)

  // Префиллим все точки на маунте; setDrawRange ограничивает рисуемую часть.
  // [x, 0.02, -y] — конвертация мир → Three (Three Z смотрит «на нас» = -y_world).
  const positions = useMemo(() => {
    const arr = new Float32Array(samples.length * 3)
    for (let i = 0; i < samples.length; i++) {
      arr[i * 3 + 0] = samples[i].x
      arr[i * 3 + 1] = 0.02
      arr[i * 3 + 2] = -samples[i].y
    }
    return arr
  }, [samples])

  useEffect(() => {
    const geom = geometryRef.current
    if (!geom) return
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geom.setDrawRange(0, 0)
    return () => {
      geom.dispose()
    }
  }, [positions])

  useFrame(() => {
    const geom = geometryRef.current
    if (!geom) return
    const idx = progressRef.current?.index ?? 0
    geom.setDrawRange(0, idx + 1)
  })

  return (
    <line>
      <bufferGeometry ref={geometryRef} />
      <lineBasicMaterial color="#2563eb" linewidth={2} />
    </line>
  )
}

interface SceneInfoOverlayProps {
  progressRef: React.MutableRefObject<LiveProgress | null>
}

function SceneInfoOverlay({ progressRef }: SceneInfoOverlayProps) {
  // Обновляем HTML каждый кадр через requestAnimationFrame, без React state.
  const tRef = useRef<HTMLSpanElement | null>(null)
  const sRef = useRef<HTMLSpanElement | null>(null)
  useEffect(() => {
    let raf = 0
    const tick = () => {
      const p = progressRef.current
      if (tRef.current && p) tRef.current.textContent = p.t.toFixed(2)
      if (sRef.current && p) sRef.current.textContent = p.s.toFixed(2)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [progressRef])

  return (
    <div className="absolute bottom-3 left-3 text-xs font-mono text-zinc-300 bg-zinc-900/70 backdrop-blur px-2 py-1 rounded border border-zinc-700">
      t = <span ref={tRef}>0.00</span>с • s = <span ref={sRef}>0.00</span>м
    </div>
  )
}

export function Mps3DScene({ telemetry, distance }: Mps3DSceneProps) {
  const samples = useMemo(() => prepareSamples(telemetry), [telemetry])
  const progressRef = useRef<LiveProgress | null>(null)

  // Защитная ветвь — реально Mps3DOverlay уже отфильтровал по validTelemetryCount,
  // но на случай прямого вызова Mps3DScene извне.
  if (samples.length < 2) {
    return (
      <div className="flex h-full items-center justify-center bg-[#1a1a2e]">
        <p className="text-sm text-zinc-400">Нет валидных данных для 3D-визуализации</p>
      </div>
    )
  }

  // Стартовая позиция камеры: смотрит сверху-сбоку на центр траектории.
  const cameraTarget: [number, number, number] = [distance / 2, 0, 0]
  const cameraPos: [number, number, number] = [distance / 2 + 1.0, 1.2, 1.2]

  return (
    <div className="relative w-full h-full">
      <Canvas
        camera={{ position: cameraPos, fov: 50, near: 0.01, far: 100 }}
        shadows
      >
        <color attach="background" args={['#1a1a2e']} />

        <ambientLight intensity={0.9} />
        <directionalLight
          position={[2, 3, 1]}
          intensity={1.8}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <directionalLight position={[-1, 2, -1]} intensity={0.7} />
        <directionalLight position={[0, 1, -2]} intensity={0.4} />
        <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]} />

        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.6}
          cellColor="#3f3f5c"
          sectionSize={0.5}
          sectionThickness={1.2}
          sectionColor="#5a5a7a"
          fadeDistance={5}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid
        />

        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.001, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <shadowMaterial opacity={0.2} />
        </mesh>

        <axesHelper args={[0.3]} />

        {/* Маркер старта: серая сфера в (0,0,0) */}
        <mesh position={[0, 0.02, 0]}>
          <sphereGeometry args={[0.02, 12, 12]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>

        {/* Маркер цели: зелёное кольцо в (D, 0, 0) */}
        <mesh position={[distance, 0.02, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.04, 0.005, 8, 32]} />
          <meshStandardMaterial color="#16a34a" emissive="#16a34a" emissiveIntensity={0.4} />
        </mesh>

        <Suspense fallback={null}>
          <AnimatedRobot samples={samples} progressRef={progressRef} />
        </Suspense>

        <AnimatedTrail samples={samples} progressRef={progressRef} />

        <OrbitControls
          target={cameraTarget}
          maxPolarAngle={Math.PI / 2 - 0.05}
          minDistance={0.3}
          maxDistance={10}
          enableDamping
          dampingFactor={0.1}
        />
      </Canvas>

      <SceneInfoOverlay progressRef={progressRef} />
    </div>
  )
}
```

- [ ] **Step 2: Проверить что TypeScript компилируется**

```bash
cd compute_node/frontend
npm run build
```

Ожидаемо: сборка проходит без ошибок типов.

- [ ] **Step 3: Запустить весь тестовый прогон по mps**

```bash
npx vitest run src/components/mps/
```

Ожидаемо: PASS (17 тестов; Mps3DScene без юнит-тестов).

- [ ] **Step 4: Коммит**

```bash
git add compute_node/frontend/src/components/mps/Mps3DScene.tsx
git commit -m "feat(mps-3d): Mps3DScene — R3F сцена с AnimatedRobot/AnimatedTrail, маркеры старта/цели, info-оверлей"
```

---

## Task 6: Кнопка «3D» в `TrajectoryView` — TDD

**Цель:** В шапке карточки `TrajectoryView` добавить icon-кнопку «3D», которая вызывает `useMps3D().open(result)`. Disabled когда `result === null` или телеметрия пуста.

**Files:**
- Modify: `compute_node/frontend/src/components/mps/TrajectoryView.tsx`
- Create: `compute_node/frontend/src/components/mps/TrajectoryView.test.tsx`

- [ ] **Step 1: Написать падающие тесты**

Создать `compute_node/frontend/src/components/mps/TrajectoryView.test.tsx`:

```tsx
import { describe, it, expect } from 'vitest'
import { render, screen, act } from '@testing-library/react'
import type { ReactNode } from 'react'
import { TrajectoryView } from './TrajectoryView'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

function makeResult(opts: Partial<MpsScenarioResult> = {}): MpsScenarioResult {
  return {
    run_id: 'r1',
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
    ...opts,
  }
}

const wrap = (node: ReactNode) => <Mps3DProvider>{node}</Mps3DProvider>

describe('TrajectoryView — кнопка 3D', () => {
  it('кнопка disabled когда result=null', () => {
    render(wrap(<TrajectoryView result={null} />))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).toBeDisabled()
  })

  it('кнопка disabled когда telemetry пуст', () => {
    render(wrap(<TrajectoryView result={makeResult({ telemetry: [] })} />))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).toBeDisabled()
  })

  it('клик переводит в overlay-state', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    render(wrap(
      <>
        <TrajectoryView result={makeResult()} />
        <StateProbe />
      </>,
    ))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).not.toBeDisabled()
    act(() => { btn.click() })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })
})
```

- [ ] **Step 2: Запустить — должны упасть**

```bash
npx vitest run src/components/mps/TrajectoryView.test.tsx
```

Ожидаемо: FAIL (кнопка ещё не добавлена).

- [ ] **Step 3: Добавить кнопку в `TrajectoryView.tsx`**

Открыть `compute_node/frontend/src/components/mps/TrajectoryView.tsx`. В верх файла добавить новые импорты:

```tsx
import { Box } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { useMps3D } from './Mps3DProvider'
```

Найти строку 58 `<CardHeader>` и заменить её и `<CardTitle>` на структуру с заголовком и кнопкой справа:

```tsx
<CardHeader>
  <div className="flex items-center justify-between gap-2">
    <CardTitle>Траектория (top-down)</CardTitle>
    <TrajectoryOpen3DButton result={result} />
  </div>
</CardHeader>
```

В конец файла (после функции `TrajectoryView`) добавить:

```tsx
function TrajectoryOpen3DButton({ result }: { result: MpsScenarioResult | null }) {
  const mps3D = useMps3D()
  const disabled = !result || (result.telemetry?.length ?? 0) < 2
  return (
    <Button
      type="button"
      variant="outline"
      size="sm"
      disabled={disabled}
      onClick={() => result && mps3D.open(result)}
      title="3D-просмотр траектории"
      aria-label="3D-просмотр траектории"
    >
      <Box className="w-4 h-4" />
      <span className="ml-1">3D</span>
    </Button>
  )
}
```

- [ ] **Step 4: Запустить тест — должен пройти**

```bash
npx vitest run src/components/mps/TrajectoryView.test.tsx
```

Ожидаемо: PASS (3 теста).

- [ ] **Step 5: Коммит**

```bash
git add compute_node/frontend/src/components/mps/TrajectoryView.tsx compute_node/frontend/src/components/mps/TrajectoryView.test.tsx
git commit -m "feat(mps-3d): кнопка 3D-просмотра в шапке TrajectoryView"
```

---

## Task 7: Интеграция в `MpsPage` — TDD

**Цель:** Разделить `MpsPage` на обёртку с провайдерами и `MpsPageInner` (текущее тело). Подключить `Mps3DProvider`. В `MpsPageInner` через `useEffect` отслеживать `primaryResult` и поднимать тост для новых `run_id` с валидной телеметрией.

**Files:**
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx`
- Modify: `compute_node/frontend/src/pages/MpsPage.test.tsx`

- [ ] **Step 1: Изучить текущий `MpsPage.test.tsx`**

```bash
cd compute_node/frontend
cat src/pages/MpsPage.test.tsx | head -120
```

Цель — увидеть какие моки уже настроены и как тестируется страница. Особенно интересно `vi.mock('@/hooks/useMpsRun', ...)` — через него мы будем подсовывать `primaryResult` в тестах.

- [ ] **Step 2: Написать падающий тест на появление тоста**

В `compute_node/frontend/src/pages/MpsPage.test.tsx` добавить (в `describe`-блок или новый):

```tsx
import { waitFor } from '@testing-library/react'
// (если ещё не импортирован)

describe('MpsPage — 3D toast', () => {
  it('после завершения симуляции появляется тост «Симуляция завершена»', async () => {
    // useMpsRun уже замокан в файле сверху — нужно убедиться что он отдаёт result.
    // Если текущий мок не отдаёт result, добавляем переменную и обновляем мок:
    // (вариант реализации зависит от существующих моков; см. Step 3 ниже)

    render(<MpsPage />)
    await waitFor(() => {
      expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
    })
  })

  it('replay того же run_id не показывает тост повторно', async () => {
    render(<MpsPage />)
    await waitFor(() => {
      expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
    })
    // Закрыть тост
    const close = screen.getByRole('button', { name: /Закрыть/i })
    act(() => { close.click() })
    expect(screen.queryByText(/Симуляция завершена/i)).toBeNull()
    // Если бы replay того же run_id триггерил тост — он бы появился; но MpsPage
    // должен запомнить run_id через ref и не звать requestToast повторно.
    // Этот сценарий проверяется через сравнение run_id'ов, и так как мок отдаёт
    // тот же result — повторного появления быть не должно.
  })
})
```

**Конкретно для этого файла** текущий мок `useMpsRun` отдаёт `result: null`, поэтому `primaryResult` всегда `null` и тост не появится. Нужно сделать мок параметризуемым.

Заменить существующий блок `vi.mock('@/hooks/useMpsRun', ...)` (строки 39-47 в `MpsPage.test.tsx`) на:

```tsx
import type { MpsScenarioResult } from '@/types/mps'

let mockRunResult: MpsScenarioResult | null = null
let mockRunId: string | null = null

vi.mock('@/hooks/useMpsRun', () => ({
  useMpsRun: () => ({
    running: false,
    result: mockRunResult,
    runId: mockRunId,
    error: null,
    run: vi.fn(),
    abort: vi.fn(),
  }),
}))
```

Добавить хелпер рядом с `makeMatrices()`:

```tsx
function makeResult(runId: string = 'r-test'): MpsScenarioResult {
  return {
    run_id: runId,
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: makeMatrices(),
    telemetry: [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}
```

В `beforeEach` внутри `describe('MpsPage integration', ...)` (строка 79) добавить сброс:

```tsx
beforeEach(() => {
  mockRunResult = null
  mockRunId = null
  // ...existing ResizeObserver setup remains...
})
```

В новых тестах перед `render` присвоить:

```tsx
mockRunResult = makeResult('r-test-1')
mockRunId = 'r-test-1'
```

**Также** Mps3DScene использует R3F Canvas — он не работает в happy-dom. Замокать его в верхней части файла:

```tsx
vi.mock('@/components/mps/Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="mps3d-scene-stub" />,
}))
```

- [ ] **Step 3: Запустить — упадёт (тост не появляется)**

```bash
npx vitest run src/pages/MpsPage.test.tsx
```

Ожидаемо: новые тесты FAIL.

- [ ] **Step 4: Рефакторить `MpsPage` (разделить на обёртку + inner)**

Открыть `compute_node/frontend/src/pages/MpsPage.tsx`. В верх файла добавить импорты:

```tsx
import { useRef } from 'react'   // если ещё нет
import { Mps3DProvider, useMps3D } from '@/components/mps/Mps3DProvider'
```

Заменить экспорт `export function MpsPage()` на следующую структуру: всё текущее тело становится `MpsPageInner` (без `export`), а `MpsPage` — обёртка:

```tsx
export function MpsPage() {
  return (
    <MpsHighlightProvider>
      <Mps3DProvider>
        <MpsPageInner />
      </Mps3DProvider>
    </MpsHighlightProvider>
  )
}

function MpsPageInner() {
  // ... весь текущий код из старого export function MpsPage() ...
  // но БЕЗ обёртки <MpsHighlightProvider> в return (она теперь снаружи)
}
```

Внутри `MpsPageInner` сразу после объявления хуков (около строки 36, где `const [primaryResult, setPrimaryResult]`) добавить:

```tsx
const mps3D = useMps3D()
const lastSeenRunIdRef = useRef<string | null>(null)

useEffect(() => {
  if (!primaryResult) return
  if (primaryResult.run_id === lastSeenRunIdRef.current) return
  if ((primaryResult.telemetry?.length ?? 0) < 2) return
  lastSeenRunIdRef.current = primaryResult.run_id
  mps3D.requestToast(primaryResult)
}, [primaryResult, mps3D])
```

И в JSX `MpsPageInner` убрать внешнюю обёртку `<MpsHighlightProvider>` (раз обёртка теперь снаружи в `MpsPage`):

```tsx
return (
  <div className="min-h-screen">
    {/* ... всё содержимое как было, без <MpsHighlightProvider> ... */}
  </div>
)
```

- [ ] **Step 5: Запустить тесты — должны пройти**

```bash
npx vitest run src/pages/MpsPage.test.tsx
```

Ожидаемо: PASS (существующие + 2 новых теста про тост).

- [ ] **Step 6: Прогнать всю тест-сюита**

```bash
npx vitest run
```

Ожидаемо: PASS, регрессий нет.

- [ ] **Step 7: Коммит**

```bash
git add compute_node/frontend/src/pages/MpsPage.tsx compute_node/frontend/src/pages/MpsPage.test.tsx
git commit -m "feat(mps-3d): подключить Mps3DProvider в MpsPage, поднимать тост на новый primaryResult"
```

---

## Task 8: Ручная проверка в браузере и финальный коммит

**Цель:** Убедиться что вся фича работает в реальном браузере (R3F, GLB-модель, анимация, OrbitControls).

**Без файловых изменений** (кроме исправлений если что-то выявится).

- [ ] **Step 1: Запустить backend (Compute stack)**

В отдельном терминале из корня репозитория:

```bash
./samurai.sh compute
```

Дождаться `Uvicorn running on http://0.0.0.0:5000`. Если бэкенд уже запущен — пропустить.

- [ ] **Step 2: Запустить dev-сервер фронтенда**

В другом терминале:

```bash
cd compute_node/frontend
npm run dev
```

Открыть `http://localhost:5173/mps` в браузере.

- [ ] **Step 3: Сценарий sim — happy path**

1. Подождать когда матрицы загрузятся (обычно <1 сек).
2. В `ScenarioControls` оставить дефолтные `distance` и `v_target`, source = `sim`.
3. Нажать «Запустить».
4. **Ожидаемо:** через <1 сек снизу-справа выезжает (slide-up) тост «Симуляция завершена».
5. Прогресс-полоска тоста плавно сокращается за 5 секунд.

- [ ] **Step 4: Сценарий sim — автозакрытие тоста**

1. Запустить симуляцию ещё раз.
2. Дождаться появления тоста.
3. Не трогать — ждать 5 секунд.
4. **Ожидаемо:** тост исчезает сам.

- [ ] **Step 5: Сценарий sim — открытие оверлея**

1. Запустить симуляцию.
2. Дождаться тоста.
3. Нажать «Показать в 3D».
4. **Ожидаемо:**
   - Тост исчезает.
   - Открывается оверлей (~250 мс анимация: backdrop fade + scale).
   - Внутри — тёмная сцена с гридом, модель Samurai в (0,0).
   - Зелёное кольцо (цель) видно на оси X на расстоянии `distance`.
   - Модель плавно едет от (0,0) к цели за время равное `t_end - t_0` телеметрии (обычно 5-15 сек).
   - Синяя линия следа нарастает по мере движения.
   - В левом нижнем углу — `t=…с • s=…м` обновляется в реальном времени.
   - OrbitControls работают: мышью можно крутить камеру, колесом — приближать/отдалять.

- [ ] **Step 6: Закрытие оверлея**

1. В открытом оверлее нажать ✕ в правом верхнем углу.
2. **Ожидаемо:** оверлей закрывается.
3. Проверить что ESC и клик по тёмному backdrop **не** закрывают: открыть оверлей снова, нажать Esc — ничего; кликнуть в чёрную область — ничего. Закрыть через ✕.

- [ ] **Step 7: R2 — повторное открытие**

1. После закрытия оверлея найти карточку «Траектория (top-down)».
2. В шапке карточки справа должна быть кнопка с иконкой коробки «3D».
3. Нажать.
4. **Ожидаемо:** оверлей открывается снова с тем же `primaryResult`. Анимация заново.

- [ ] **Step 8: Replay из истории — тост НЕ показывается**

1. В `HistoryPanel` нажать Replay на любой исторический результат.
2. **Ожидаемо:** `primaryResult` обновляется, но тост **не** появляется (это уже виденный или новый-но-не-через-Run run_id). Кнопка 3D в `TrajectoryView` остаётся активной — можно открыть оверлей через неё.

   **Замечание для агента:** если по факту replay использует *тот же* run_id что был — тост не появится из-за ref-защиты. Если replay подсовывает другой run_id — тост появится. Текущая логика replay в `historyHook.replay` использует `setPrimaryResult(r)` с историческим result. Если поведение неудобное — будущая итерация может ввести признак «это replay» в hook, но в рамках этого плана оставляем как есть.

- [ ] **Step 9: Граничный случай — невалидная телеметрия**

Если есть возможность подсунуть симуляцию с `error_type='nan'` (через настройку «плохих» матриц) или симуляцию которая упала — открыть оверлей через R2. **Ожидаемо:** видно сообщение «Нет валидных данных для 3D-визуализации» вместо Canvas.

- [ ] **Step 10: Smoke-тест других страниц**

Открыть `/3d` и убедиться что страница работает как прежде — реальный робот в сцене, IMU и SLAM-слои на месте. Это контрольная проверка что добавление `noSmooth` prop в `RobotModel` ничего не сломало.

- [ ] **Step 11: Production-сборка**

```bash
cd compute_node/frontend
npm run build
```

Ожидаемо: сборка успешна, ошибок типов нет. Артефакты появляются в `compute_node/static/assets/`.

- [ ] **Step 12: Финальный прогон всех тестов**

```bash
cd compute_node/frontend
npx vitest run
```

Ожидаемо: все тесты зелёные.

- [ ] **Step 13: Финальный коммит (если были правки по ручной проверке)**

Если на шагах 3-10 пришлось что-то поправить — закоммитить:

```bash
git add -A compute_node/frontend/src
git commit -m "fix(mps-3d): правки по ручной проверке — <конкретное описание>"
```

Если правок не было — пропустить этот шаг.

---

## Чеклист готовности

После выполнения всех 8 задач:

- [ ] `npx vitest run` — все тесты проходят (включая старые `MpsPage.test.tsx`).
- [ ] `npm run build` — production-сборка проходит.
- [ ] На `/mps` после запуска симуляции тост выезжает снизу-справа.
- [ ] Через 5 секунд тост сам исчезает.
- [ ] Клик «Показать в 3D» открывает оверлей; машинка едет 1:1 по времени.
- [ ] Закрытие только по ✕; Esc и backdrop-click игнорируются.
- [ ] Кнопка «3D» в `TrajectoryView` повторно открывает оверлей.
- [ ] Страница `/3d` работает как раньше — регрессий нет.
- [ ] `git log --oneline -10` показывает 8 коммитов в стиле `feat(mps-3d): ...` / `test(mps-3d): ...`.
