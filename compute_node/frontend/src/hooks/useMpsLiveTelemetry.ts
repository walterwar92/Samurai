import { useEffect, useRef, useState } from 'react'
import type {
  MpsScenarioResult,
  MpsTelemetryPoint,
  MpsWsFrame,
} from '@/types/mps'

const BUFFER_SIZE = 200

interface UseMpsLiveTelemetryParams {
  enabled: boolean
  runId?: string
  onFinished?: (result: MpsScenarioResult) => void
  onError?: (msg: string, errorType: string) => void
}

interface UseMpsLiveTelemetryResult {
  points: MpsTelemetryPoint[]
  /** True если WebSocket был открыт хотя бы раз и сейчас живой. */
  connected: boolean
}

/** WebSocket /ws/mps/telemetry — буфер до 200 точек активного прогона.
 *  Подписывается на запрошенный runId или (если не задан) — на все frames. */
export function useMpsLiveTelemetry({
  enabled,
  runId,
  onFinished,
  onError,
}: UseMpsLiveTelemetryParams): UseMpsLiveTelemetryResult {
  const [points, setPoints] = useState<MpsTelemetryPoint[]>([])
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)

  // Stable refs так, чтобы reconnect не пересоздавался при каждом re-render
  const finishedRef = useRef(onFinished)
  const errorRef = useRef(onError)
  finishedRef.current = onFinished
  errorRef.current = onError

  useEffect(() => {
    if (!enabled) {
      setPoints([])
      return
    }

    setPoints([])
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const url = `${protocol}//${window.location.host}/ws/mps/telemetry`
    const ws = new WebSocket(url)
    wsRef.current = ws

    ws.onopen = () => {
      setConnected(true)
      ws.send(JSON.stringify({ action: 'subscribe', run_id: runId }))
    }

    ws.onmessage = (ev: MessageEvent) => {
      let frame: MpsWsFrame
      try {
        frame = JSON.parse(ev.data) as MpsWsFrame
      } catch {
        return
      }
      switch (frame.type) {
        case 'telemetry':
          setPoints((prev) => {
            const next = prev.length >= BUFFER_SIZE
              ? prev.slice(prev.length - BUFFER_SIZE + 1)
              : prev.slice()
            next.push(frame.point)
            return next
          })
          break
        case 'finished':
          finishedRef.current?.(frame.result)
          break
        case 'error':
          errorRef.current?.(frame.message, frame.error_type)
          break
      }
    }

    ws.onerror = () => setConnected(false)
    ws.onclose = () => setConnected(false)

    return () => {
      try {
        ws.close()
      } catch {
        /* ignore */
      }
      wsRef.current = null
    }
  }, [enabled, runId])

  return { points, connected }
}
