import { useCallback, useRef, useState } from 'react'
import { api } from '@/lib/api'

/**
 * Shared FSM force-transition logic for buttons that trigger
 * `POST /api/fsm/transition`.
 *
 * Bare `api.forceTransition()` calls in components had three latent issues:
 *   - no error handling: a 5xx silently disappeared,
 *   - no loading state: user couldn't tell whether the click registered,
 *   - no double-click guard: rapid taps queued duplicate requests during
 *     network slowness.
 *
 * This hook returns a single `transitionTo(state)` callback that:
 *   - is idempotent against rapid double-clicks (in-flight ref),
 *   - reports `pending` for UI affordance (button disable/spinner),
 *   - surfaces the most recent error to the caller.
 */
export function useFsmTransition() {
  const [pending, setPending] = useState(false)
  const [error, setError] = useState<unknown>(null)
  const inFlight = useRef(false)

  const transitionTo = useCallback(async (state: string) => {
    if (inFlight.current) return false
    inFlight.current = true
    setPending(true)
    setError(null)
    try {
      const res = await api.forceTransition(state)
      if (!res.ok) {
        throw new Error(`HTTP ${res.status}`)
      }
      return true
    } catch (err) {
      setError(err)
      return false
    } finally {
      inFlight.current = false
      setPending(false)
    }
  }, [])

  return { transitionTo, pending, error }
}
