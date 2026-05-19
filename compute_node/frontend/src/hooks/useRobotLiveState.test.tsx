/**
 * useRobotLiveState — WS-хук для постоянного состояния робота.
 * Зеркало useMpsLiveState.test.tsx (см. там для деталей FakeWebSocket).
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useRobotLiveState } from './useRobotLiveState'

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
    ts: 1747574400,
    pose: { x: 0.5, y: 1.2, yaw_rad: 0.1, yaw_deg: 5.7 },
    vel: { linear: 0.12, angular: 0.05 },
    imu: {
      ypr_deg: [5.7, 0.5, 0.0] as [number, number, number],
      gyro: [0.01, 0.02, 0.21] as [number, number, number],
      accel: [0.05, 0.02, 9.81] as [number, number, number],
      ekf_bias_deg: [0.06, -0.11, 0.17] as [number, number, number],
      has_ekf: true,
    },
    stationary: false,
    schema_version: '1.0',
  },
}

describe('useRobotLiveState', () => {
  it('подключается к /ws/robot/live_state при mount', () => {
    renderHook(() => useRobotLiveState())
    expect(fakes.length).toBe(1)
    expect(fakes[0].url).toMatch(/\/ws\/robot\/live_state$/)
  })

  it('обновляет point на frame', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(result.current.connected).toBe(true)
    act(() => ws.fireMessage(FRAME))
    expect(result.current.point?.pose.x).toBeCloseTo(0.5)
    expect(result.current.point?.imu.has_ekf).toBe(true)
  })

  it('игнорирует frames с type !== "live_state"', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage({ type: 'other', point: {} }))
    expect(result.current.point).toBeNull()
  })

  it('после onclose connected=false, point сохраняется', () => {
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    act(() => ws.close())
    expect(result.current.connected).toBe(false)
    expect(result.current.point?.pose.x).toBeCloseTo(0.5)
  })

  it('после >2s без frames — stale=true', () => {
    vi.useFakeTimers()
    const { result } = renderHook(() => useRobotLiveState())
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => ws.fireMessage(FRAME))
    expect(result.current.stale).toBe(false)
    act(() => {
      vi.advanceTimersByTime(2500)
    })
    expect(result.current.stale).toBe(true)
    expect(result.current.ageMs).toBeGreaterThanOrEqual(2000)
  })
})
