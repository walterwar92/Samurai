/**
 * useMpsLiveState — WS-хук для постоянного отображения вектора
 * состояния. Тестируем: connect → fire frame → point обновился; >2s без
 * фрейма → stale=true; onclose → connected=false, point сохраняется.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useMpsLiveState } from './useMpsLiveState'

interface FakeWs {
  url: string
  readyState: number
  onopen: ((ev: unknown) => void) | null
  onmessage: ((ev: { data: string }) => void) | null
  onerror: ((ev: unknown) => void) | null
  onclose: ((ev: unknown) => void) | null
  send(data: string): void
  close(): void
}

const fakes: FakeWs[] = []

class FakeWebSocket implements FakeWs {
  url: string
  readyState = 0
  onopen: ((ev: unknown) => void) | null = null
  onmessage: ((ev: { data: string }) => void) | null = null
  onerror: ((ev: unknown) => void) | null = null
  onclose: ((ev: unknown) => void) | null = null

  constructor(url: string) {
    this.url = url
    fakes.push(this)
  }
  send(_data: string) {}
  close() {
    this.readyState = 3
    this.onclose?.({})
  }
  fireOpen() {
    this.readyState = 1
    this.onopen?.({})
  }
  fireMessage(payload: unknown) {
    this.onmessage?.({ data: JSON.stringify(payload) })
  }
}

beforeEach(() => {
  fakes.length = 0
  // @ts-expect-error override global
  globalThis.WebSocket = FakeWebSocket
})

afterEach(() => {
  vi.useRealTimers()
  vi.restoreAllMocks()
})

const FRAME = {
  type: 'live_state' as const,
  point: {
    ts: 1.0,
    x: [0.5, 0.12, 0.05, 0.0, 0.0],
    u: [0.1, 0.0],
    scenario_active: false,
    run_id: null,
    schema_version: '1.0',
  },
}

describe('useMpsLiveState', () => {
  it('подключается при mount и обновляет point на frame', () => {
    const { result } = renderHook(() => useMpsLiveState())
    expect(fakes.length).toBe(1)
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(result.current.connected).toBe(true)
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('stale=true через > 2s без frame', () => {
    vi.useFakeTimers()
    // Хук считает stale через Date.now() - lastReceivedAt; явно
    // фиксируем системное время, чтобы advanceTimersByTime реально его
    // двигал (vitest fake timers могут не мокать Date по умолчанию).
    vi.setSystemTime(new Date(1_000_000))
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.stale).toBe(false)
    act(() => {
      vi.advanceTimersByTime(2500)
    })
    expect(result.current.stale).toBe(true)
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('onclose → connected=false, point сохраняется', () => {
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point).not.toBeNull()
    act(() => ws.close())
    expect(result.current.connected).toBe(false)
    expect(result.current.point?.x[1]).toBeCloseTo(0.12)
  })

  it('игнорирует frames не-live_state-типа', () => {
    const { result } = renderHook(() => useMpsLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage({ type: 'telemetry', run_id: 'x', point: {} }))
    expect(result.current.point).toBeNull()
  })
})
