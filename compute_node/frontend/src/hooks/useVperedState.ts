import { useEffect, useRef, useState } from 'react'

export interface VperedTelemetry {
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

export interface VperedState {
  connected: boolean
  port: string | null
  telemetry_fresh: boolean
  telemetry: VperedTelemetry
  scenarios: string[]
}

export interface VperedApi {
  state: VperedState | null
  send: (cmd: string, arg?: number) => Promise<void>
  scenario: (name: string) => Promise<void>
  log: () => Promise<string[]>
}

const POLL_MS = 250

export function useVpered(): VperedApi {
  const [state, setState] = useState<VperedState | null>(null)
  const aliveRef = useRef(true)

  useEffect(() => {
    aliveRef.current = true
    const tick = async () => {
      try {
        const res = await fetch('/api/vpered/state', { cache: 'no-store' })
        if (res.ok) {
          const data = await res.json()
          if (aliveRef.current) setState(data)
        } else if (aliveRef.current) {
          setState(s => s ? { ...s, connected: false, telemetry_fresh: false } : null)
        }
      } catch {
        if (aliveRef.current) {
          setState(s => s ? { ...s, connected: false, telemetry_fresh: false } : null)
        }
      }
    }
    tick()
    const id = setInterval(tick, POLL_MS)
    return () => { aliveRef.current = false; clearInterval(id) }
  }, [])

  const send = async (cmd: string, arg?: number) => {
    await fetch('/api/vpered/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd, arg }),
    })
  }

  const scenario = async (name: string) => {
    await fetch('/api/vpered/scenario', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    })
  }

  const log = async (): Promise<string[]> => {
    try {
      const res = await fetch('/api/vpered/log?lines=80')
      const data = await res.json()
      return data.lines || []
    } catch {
      return []
    }
  }

  return { state, send, scenario, log }
}
