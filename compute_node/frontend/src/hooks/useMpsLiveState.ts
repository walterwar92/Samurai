import { useEffect, useRef, useState } from 'react'
import type { MpsLiveStatePoint, MpsLiveStateWsFrame } from '@/types/mps'

const STALE_THRESHOLD_MS = 2_000
const RECONNECT_DELAYS_MS = [1_000, 2_000, 4_000, 8_000, 10_000] as const

export interface UseMpsLiveStateResult {
  /** Последний полученный фрейм или null если ничего ещё не пришло. */
  point: MpsLiveStatePoint | null
  /** true если WS-сокет открыт. */
  connected: boolean
  /** true если >2s без новых фреймов (даже если WS открыт). */
  stale: boolean
  /** ms с момента последнего фрейма; null если фреймов ещё не было. */
  ageMs: number | null
}

/**
 * Постоянный WS-канал /ws/mps/live_state. Без enabled/runId — клиент
 * подключается всегда; auto-reconnect с экспоненциальным backoff.
 * Контракт фреймов: docs/superpowers/specs/2026-05-18-mps-live-state-vector-design.md §2.
 */
export function useMpsLiveState(): UseMpsLiveStateResult {
  const [point, setPoint] = useState<MpsLiveStatePoint | null>(null)
  const [connected, setConnected] = useState(false)
  const [ageMs, setAgeMs] = useState<number | null>(null)

  const lastReceivedAtRef = useRef<number | null>(null)
  const reconnectAttemptRef = useRef(0)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  useEffect(() => {
    let cancelled = false

    function connect() {
      if (cancelled) return
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const url = `${protocol}//${window.location.host}/ws/mps/live_state`
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        if (cancelled) return
        setConnected(true)
        reconnectAttemptRef.current = 0
      }

      ws.onmessage = (ev: MessageEvent) => {
        if (cancelled) return
        let frame: MpsLiveStateWsFrame
        try {
          frame = JSON.parse(ev.data) as MpsLiveStateWsFrame
        } catch {
          return
        }
        if (frame.type !== 'live_state') return
        lastReceivedAtRef.current = Date.now()
        setPoint(frame.point)
      }

      ws.onerror = () => {
        if (cancelled) return
        setConnected(false)
      }

      ws.onclose = () => {
        if (cancelled) return
        setConnected(false)
        // Backoff reconnect
        const idx = Math.min(
          reconnectAttemptRef.current,
          RECONNECT_DELAYS_MS.length - 1,
        )
        const delay = RECONNECT_DELAYS_MS[idx]
        reconnectAttemptRef.current += 1
        reconnectTimerRef.current = setTimeout(connect, delay)
      }
    }

    connect()

    const tick = setInterval(() => {
      if (lastReceivedAtRef.current === null) {
        setAgeMs(null)
      } else {
        setAgeMs(Date.now() - lastReceivedAtRef.current)
      }
    }, 500)

    return () => {
      cancelled = true
      clearInterval(tick)
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current)
      reconnectAttemptRef.current = 0
      try {
        wsRef.current?.close()
      } catch {
        /* ignore */
      }
      wsRef.current = null
    }
  }, [])

  const stale = ageMs !== null && ageMs > STALE_THRESHOLD_MS

  return { point, connected, stale, ageMs }
}
