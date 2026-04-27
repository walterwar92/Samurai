import { useEffect, useRef, useState } from 'react'

export interface SamcanTelemetry {
  th?: number      // theta deg
  om?: number      // omega filt deg/s
  d?: number       // distance cm
  L?: number       // left PWM
  R?: number       // right PWM
  m?: string       // mode: IDLE/FWD/LEFT/RIGHT
  ob?: number      // obstacle stop flag
  a?: number       // arm angle
  b?: number       // base angle
  c?: number       // claw angle
}

export interface SamcanState {
  connected: boolean
  port: string | null
  telemetry_fresh: boolean
  telemetry: SamcanTelemetry
  scenarios: string[]
  // диагностика
  last_error?: string
  attempted_port?: string | null
  rx_count?: number
  age_sec?: number | null
}

export interface SamcanDiag {
  connected: boolean
  port: string | null
  attempted_port: string | null
  last_error: string
  last_attempt_ts: number
  last_seen_ts: number
  rx_count: number
  available_ports: Array<{
    device: string
    description: string
    hwid?: string
    manufacturer?: string
    vid?: number | null
    pid?: number | null
  }>
}

export interface SamcanPresets {
  park?:    { base?: number; arm?: number; claw?: number }
  forward?: { base?: number; arm?: number; claw?: number }
  claw_open?:   number
  claw_closed?: number
  settle_ms?:   number
  hold_ms?:     number
  [key: string]: unknown
}

export interface SamcanApi {
  state: SamcanState | null
  send: (cmd: string, arg?: number) => Promise<void>
  scenario: (name: string) => Promise<void>
  log: () => Promise<string[]>
  diag: () => Promise<SamcanDiag | null>
  getPresets: () => Promise<SamcanPresets>
  savePreset: (
    name: 'park' | 'forward' | 'claw_open' | 'claw_closed',
    fields: { base?: number; arm?: number; claw?: number },
  ) => Promise<void>
  applyPreset: (name: 'park' | 'forward' | 'grab') => Promise<void>
}

// Polling fallback — used only when the SSE stream is unavailable
// (older proxy / bridge without /api/samcan/stream support).
const POLL_OK_MS = 1000
const POLL_MAX_MS = 5000

export function useSamcan(): SamcanApi {
  const [state, setState] = useState<SamcanState | null>(null)
  const aliveRef = useRef(true)

  useEffect(() => {
    aliveRef.current = true
    let pollTimer: ReturnType<typeof setTimeout> | null = null
    let eventSource: EventSource | null = null
    let useSSE = true   // start with SSE; flip on first error

    // ── SSE path (preferred, #29) ───────────────────────────────────
    // EventSource handles auto-reconnect. We just listen and parse.
    const startSSE = () => {
      try {
        eventSource = new EventSource('/api/samcan/stream')
        eventSource.onmessage = (ev) => {
          if (!aliveRef.current) return
          try {
            const data = JSON.parse(ev.data)
            setState(data)
          } catch {
            /* malformed frame — drop */
          }
        }
        eventSource.onerror = () => {
          // Browser will auto-retry; mark stale meanwhile. If this fails
          // repeatedly, fall back to polling.
          if (!aliveRef.current) return
          setState((s) => (s ? { ...s, connected: false, telemetry_fresh: false } : null))
          // After 3 retries with no successful message, switch to polling.
          // EventSource's `readyState === CLOSED` is the signal it gave up.
          if (eventSource && eventSource.readyState === EventSource.CLOSED) {
            useSSE = false
            eventSource.close()
            eventSource = null
            startPolling()
          }
        }
      } catch {
        useSSE = false
        startPolling()
      }
    }

    // ── Polling fallback ───────────────────────────────────────────
    const startPolling = () => {
      let backoff = POLL_OK_MS
      const schedule = (ms: number) => {
        if (!aliveRef.current) return
        pollTimer = setTimeout(tick, ms)
      }
      const tick = async () => {
        let ok = false
        try {
          const ctrl = new AbortController()
          const t = setTimeout(() => ctrl.abort(), 1500)
          const res = await fetch('/api/samcan/state', { cache: 'no-store', signal: ctrl.signal })
          clearTimeout(t)
          if (res.ok) {
            const data = await res.json()
            if (aliveRef.current) setState(data)
            ok = true
          }
        } catch {
          /* network/timeout/abort */
        }
        if (!ok && aliveRef.current) {
          setState((s) => (s ? { ...s, connected: false, telemetry_fresh: false } : null))
        }
        backoff = ok ? POLL_OK_MS : Math.min(POLL_MAX_MS, Math.max(POLL_OK_MS * 2, backoff * 2))
        schedule(backoff)
      }
      tick()
    }

    if (useSSE && typeof EventSource !== 'undefined') {
      startSSE()
    } else {
      startPolling()
    }

    return () => {
      aliveRef.current = false
      if (pollTimer) clearTimeout(pollTimer)
      if (eventSource) eventSource.close()
    }
  }, [])

  const send = async (cmd: string, arg?: number) => {
    await fetch('/api/samcan/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd, arg }),
    })
  }

  const scenario = async (name: string) => {
    await fetch('/api/samcan/scenario', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    })
  }

  const log = async (): Promise<string[]> => {
    try {
      const res = await fetch('/api/samcan/log?lines=80')
      const data = await res.json()
      return data.lines || []
    } catch {
      return []
    }
  }

  const diag = async (): Promise<SamcanDiag | null> => {
    try {
      const res = await fetch('/api/samcan/diag', { cache: 'no-store' })
      if (!res.ok) return null
      return (await res.json()) as SamcanDiag
    } catch {
      return null
    }
  }

  const getPresets = async (): Promise<SamcanPresets> => {
    try {
      const res = await fetch('/api/samcan/presets')
      const data = await res.json()
      return (data.presets || {}) as SamcanPresets
    } catch {
      return {}
    }
  }

  const savePreset: SamcanApi['savePreset'] = async (name, fields) => {
    await fetch('/api/samcan/preset/save', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, ...fields }),
    })
  }

  const applyPreset: SamcanApi['applyPreset'] = async (name) => {
    await fetch('/api/samcan/preset/apply', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    })
  }

  return { state, send, scenario, log, diag, getPresets, savePreset, applyPreset }
}
