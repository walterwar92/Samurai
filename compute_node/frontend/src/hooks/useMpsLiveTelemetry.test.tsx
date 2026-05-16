/**
 * useMpsLiveTelemetry — проверка что live точки приходят и не сбрасываются.
 *
 * Bug 2026-05-16: после двухфазного сценария TURN→DRIVE + outer LQR робот
 * шлёт телеметрию, но UI рисует пустые оси. Тест ловит regression на
 * первом тике — что хук вообще получает point и кладёт его в state.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useMpsLiveTelemetry } from './useMpsLiveTelemetry'

interface FakeWs {
  url: string
  readyState: number
  sent: string[]
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
  sent: string[] = []
  onopen: ((ev: unknown) => void) | null = null
  onmessage: ((ev: { data: string }) => void) | null = null
  onerror: ((ev: unknown) => void) | null = null
  onclose: ((ev: unknown) => void) | null = null

  constructor(url: string) {
    this.url = url
    fakes.push(this)
  }

  send(data: string) {
    this.sent.push(data)
  }

  close() {
    this.readyState = 3
    this.onclose?.({})
  }

  // Helpers for the test
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
  // @ts-expect-error — overriding global для теста
  globalThis.WebSocket = FakeWebSocket
})

afterEach(() => {
  vi.restoreAllMocks()
})

describe('useMpsLiveTelemetry — pipeline WS → points', () => {
  it('после открытия WS клиент шлёт subscribe c run_id', () => {
    renderHook(() => useMpsLiveTelemetry({ enabled: true, runId: 'r1' }))
    expect(fakes.length).toBe(1)
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(ws.sent.length).toBe(1)
    expect(JSON.parse(ws.sent[0])).toEqual({ action: 'subscribe', run_id: 'r1' })
  })

  it('frame type=telemetry → точка добавлена в points', () => {
    const { result } = renderHook(() =>
      useMpsLiveTelemetry({ enabled: true, runId: 'r1' }),
    )
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() =>
      ws.fireMessage({
        type: 'telemetry',
        run_id: 'r1',
        point: { t: 0.1, x: [0.5, 0.2, 0, 0, 0], u: [0.2, 0], y: [], s_remaining: 1.5 },
      }),
    )
    expect(result.current.points).toHaveLength(1)
    expect(result.current.points[0].x[0]).toBeCloseTo(0.5)
  })

  it('frame schema 1.1 (с e_y/theta_err/delta_theta) валидно кладётся в points', () => {
    const { result } = renderHook(() =>
      useMpsLiveTelemetry({ enabled: true, runId: 'r1' }),
    )
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() =>
      ws.fireMessage({
        type: 'telemetry',
        run_id: 'r1',
        point: {
          t: 0.1, x: [0.5, 0.2, 0.1, 0, 0], u: [0.2, 0.05], y: [],
          s_remaining: 1.5,
          e_y: 0.02, theta_err: 0.1, delta_theta: -0.05,
        },
      }),
    )
    expect(result.current.points).toHaveLength(1)
    expect(result.current.points[0].e_y).toBeCloseTo(0.02)
  })

  it('20 frames подряд → все 20 в points', () => {
    const { result } = renderHook(() =>
      useMpsLiveTelemetry({ enabled: true, runId: 'r1' }),
    )
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    act(() => {
      for (let i = 0; i < 20; i++) {
        ws.fireMessage({
          type: 'telemetry',
          run_id: 'r1',
          point: { t: i * 0.02, x: [i * 0.1, 0.1, 0, 0, 0], u: [0.1, 0], y: [], s_remaining: 1 },
        })
      }
    })
    expect(result.current.points).toHaveLength(20)
  })

  it('enabled=true, runId=undefined: WS НЕ открывается (race-guard)', () => {
    // Bug 2026-05-16: подписка с runId=undefined ловила race —
    // первый useEffect открывал WS до того как runHook возвращал runId,
    // backend накапливал frames, а cleanup → reopen терял их.
    // Теперь WS открывается только когда runId известен.
    renderHook(() => useMpsLiveTelemetry({ enabled: true, runId: undefined }))
    expect(fakes.length).toBe(0)
  })

  it('runId меняется undefined → "r1": WS открывается с правильным фильтром', () => {
    const { rerender } = renderHook(
      ({ runId }) => useMpsLiveTelemetry({ enabled: true, runId }),
      { initialProps: { runId: undefined as string | undefined } },
    )
    expect(fakes.length).toBe(0)
    rerender({ runId: 'r1' })
    expect(fakes.length).toBe(1)
    const ws = fakes[0] as unknown as FakeWebSocket
    act(() => ws.fireOpen())
    expect(JSON.parse(ws.sent[0])).toEqual({ action: 'subscribe', run_id: 'r1' })
  })
})
