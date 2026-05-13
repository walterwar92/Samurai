import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { act, renderHook } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

// Заглушка Mps3DScene: provider всегда рендерит <Mps3DOverlay/>, а тот при
// kind='overlay' пытается смонтировать настоящую R3F-сцену → ResizeObserver
// падает в jsdom. Тот же подход уже применён в Mps3DOverlay.test.tsx.
vi.mock('./Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="scene-stub">scene</div>,
}))

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

  it('close() из toasting возвращает в idle и не даёт таймеру сработать', () => {
    const { result } = renderHook(() => useMps3D(), { wrapper })
    act(() => { result.current.requestToast(makeResult('r1')) })
    act(() => { result.current.close() })
    expect(result.current.state.kind).toBe('idle')
    act(() => { vi.advanceTimersByTime(5000) })
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
})
