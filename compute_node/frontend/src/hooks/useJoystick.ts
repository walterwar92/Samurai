import { useState, useRef, useCallback, useEffect } from 'react'
import { api } from '@/lib/api'

interface JoystickState {
  knobX: number // percent 0-100
  knobY: number // percent 0-100
  linear: number
  angular: number
  active: boolean
}

export function useJoystick(maxLinear = 0.3, maxAngular = 2.0) {
  const containerRef = useRef<HTMLDivElement>(null)
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const velocityRef = useRef({ linear: 0, angular: 0 })

  const [state, setState] = useState<JoystickState>({
    knobX: 50,
    knobY: 50,
    linear: 0,
    angular: 0,
    active: false,
  })

  const getDisplacement = useCallback(
    (clientX: number, clientY: number) => {
      const el = containerRef.current
      if (!el) return { dx: 0, dy: 0 }
      const rect = el.getBoundingClientRect()
      const centerX = rect.left + rect.width / 2
      const centerY = rect.top + rect.height / 2
      const maxR = rect.width / 2 - 22

      let dx = clientX - centerX
      let dy = clientY - centerY
      const dist = Math.sqrt(dx * dx + dy * dy)
      if (dist > maxR) {
        dx = (dx / dist) * maxR
        dy = (dy / dist) * maxR
      }
      return { dx, dy, maxR }
    },
    []
  )

  const updateFromPointer = useCallback(
    (clientX: number, clientY: number) => {
      const { dx, dy, maxR } = getDisplacement(clientX, clientY)
      if (maxR === undefined) return
      const el = containerRef.current
      if (!el) return

      const rect = el.getBoundingClientRect()
      const pctX = ((dx + rect.width / 2) / rect.width) * 100
      const pctY = ((dy + rect.height / 2) / rect.height) * 100

      const linear = (-dy / maxR) * maxLinear
      const angular = (-dx / maxR) * maxAngular

      velocityRef.current = { linear, angular }
      setState({ knobX: pctX, knobY: pctY, linear, angular, active: true })
    },
    [getDisplacement, maxLinear, maxAngular]
  )

  const startJoy = useCallback(
    (e: React.PointerEvent) => {
      e.preventDefault()
      ;(e.target as HTMLElement).setPointerCapture(e.pointerId)
      updateFromPointer(e.clientX, e.clientY)

      intervalRef.current = setInterval(() => {
        const v = velocityRef.current
        api.sendVelocity(v.linear, v.angular)
      }, 100)
    },
    [updateFromPointer]
  )

  const moveJoy = useCallback(
    (e: React.PointerEvent) => {
      if (!state.active) return
      updateFromPointer(e.clientX, e.clientY)
    },
    [state.active, updateFromPointer]
  )

  const endJoy = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current)
      intervalRef.current = null
    }
    velocityRef.current = { linear: 0, angular: 0 }
    api.sendVelocity(0, 0)
    setState({ knobX: 50, knobY: 50, linear: 0, angular: 0, active: false })
  }, [])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current)
    }
  }, [])

  return {
    containerRef,
    state,
    startJoy,
    moveJoy,
    endJoy,
  }
}
