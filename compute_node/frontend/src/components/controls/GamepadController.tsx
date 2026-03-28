import { useEffect, useRef, useState, useCallback } from 'react'
import { api } from '@/lib/api'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

const DEAD_ZONE = 0.12
const SEND_HZ = 10       // команд в секунду
const MAX_LINEAR = 0.25   // м/с
const MAX_ANGULAR = 1.5   // рад/с

function applyDeadzone(v: number): number {
  if (Math.abs(v) < DEAD_ZONE) return 0
  const sign = v > 0 ? 1 : -1
  return sign * (Math.abs(v) - DEAD_ZONE) / (1 - DEAD_ZONE)
}

export function GamepadController() {
  const [connected, setConnected] = useState(false)
  const [gamepadName, setGamepadName] = useState('')
  const [axes, setAxes] = useState({ linear: 0, angular: 0 })
  const [enabled, setEnabled] = useState(false)
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const lastSentRef = useRef({ linear: 0, angular: 0 })

  const pollGamepad = useCallback(() => {
    const gamepads = navigator.getGamepads()
    let gp: Gamepad | null = null

    for (const g of gamepads) {
      if (g?.connected) {
        gp = g
        break
      }
    }

    if (!gp) {
      if (connected) {
        setConnected(false)
        setGamepadName('')
      }
      return
    }

    if (!connected) {
      setConnected(true)
      setGamepadName(gp.id.substring(0, 40))
    }

    if (!enabled) return

    // Left stick: Y axis = forward/back, X axis = rotation
    // Invert Y (push forward = negative in browser API)
    const rawLinear = -(gp.axes[1] ?? 0)
    const rawAngular = -(gp.axes[0] ?? 0)  // left stick X → angular

    // Right stick alternative: axes[3] = Y (forward), axes[2] = X (rotate)
    const rawLinear2 = -(gp.axes[3] ?? 0)

    // Use whichever stick has larger input
    const linear = Math.abs(rawLinear) > Math.abs(rawLinear2) ? rawLinear : rawLinear2
    const angular = rawAngular

    const lin = applyDeadzone(linear) * MAX_LINEAR
    const ang = applyDeadzone(angular) * MAX_ANGULAR

    setAxes({ linear: lin, angular: ang })

    // Send if changed significantly
    const dl = Math.abs(lin - lastSentRef.current.linear)
    const da = Math.abs(ang - lastSentRef.current.angular)
    if (dl > 0.01 || da > 0.05) {
      api.sendVelocity(lin, ang)
      lastSentRef.current = { linear: lin, angular: ang }
    }

    // Buttons
    // A (0) = stop, B (1) = claw toggle, X (2) = laser toggle
    // Y (3) = reset position, LB (4) = slow, RB (5) = fast
    if (gp.buttons[0]?.pressed) {
      api.emergencyStop()
      lastSentRef.current = { linear: 0, angular: 0 }
    }
  }, [connected, enabled])

  useEffect(() => {
    if (intervalRef.current) clearInterval(intervalRef.current)
    intervalRef.current = setInterval(pollGamepad, 1000 / SEND_HZ)
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current)
    }
  }, [pollGamepad])

  // Listen for gamepad connect/disconnect
  useEffect(() => {
    const onConnect = () => setConnected(true)
    const onDisconnect = () => {
      setConnected(false)
      setGamepadName('')
    }
    window.addEventListener('gamepadconnected', onConnect)
    window.addEventListener('gamepaddisconnected', onDisconnect)
    return () => {
      window.removeEventListener('gamepadconnected', onConnect)
      window.removeEventListener('gamepaddisconnected', onDisconnect)
    }
  }, [])

  // Send stop when disabled
  useEffect(() => {
    if (!enabled) {
      api.sendVelocity(0, 0)
      lastSentRef.current = { linear: 0, angular: 0 }
      setAxes({ linear: 0, angular: 0 })
    }
  }, [enabled])

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Геймпад
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="flex items-center gap-2 mb-2">
          <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-zinc-600'}`} />
          <span className="text-xs text-zinc-400">
            {connected ? gamepadName : 'Нет геймпада'}
          </span>
        </div>

        <div className="flex items-center gap-2">
          <button
            onClick={() => setEnabled(!enabled)}
            disabled={!connected}
            className={`px-3 py-1.5 text-xs rounded border transition-colors font-medium ${
              enabled
                ? 'bg-emerald-900/80 border-emerald-600 text-emerald-300'
                : 'bg-zinc-800 border-zinc-600 text-zinc-400'
            } ${!connected ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            {enabled ? 'Управление ВКЛ' : 'Управление ВЫКЛ'}
          </button>

          {enabled && (
            <div className="flex gap-3 text-xs font-mono text-zinc-300">
              <span>Lin: {axes.linear.toFixed(2)}</span>
              <span>Ang: {axes.angular.toFixed(2)}</span>
            </div>
          )}
        </div>

        {enabled && (
          <div className="mt-2 text-[10px] text-zinc-500">
            Левый стик: движение | A: стоп | Отключи для передачи управления
          </div>
        )}
      </CardContent>
    </Card>
  )
}
