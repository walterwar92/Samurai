import { createContext, useCallback, useContext, useEffect, useReducer } from 'react'
import type { ReactNode } from 'react'
import type { MpsScenarioResult } from '@/types/mps'

export type Mps3DState =
  | { kind: 'idle' }
  | { kind: 'toasting'; result: MpsScenarioResult; startedAt: number }
  | { kind: 'overlay';  result: MpsScenarioResult }

type Action =
  | { type: 'REQUEST_TOAST'; result: MpsScenarioResult; now: number }
  | { type: 'OPEN'; result: MpsScenarioResult }
  | { type: 'CLOSE' }
  | { type: 'TIMEOUT' }

const TOAST_MS = 5000

function reducer(state: Mps3DState, action: Action): Mps3DState {
  switch (action.type) {
    case 'REQUEST_TOAST': {
      // Игнор пока открыт overlay
      if (state.kind === 'overlay') return state
      // No-op если уже toasting с тем же run_id
      if (state.kind === 'toasting' && state.result.run_id === action.result.run_id) return state
      return { kind: 'toasting', result: action.result, startedAt: action.now }
    }
    case 'OPEN':
      return { kind: 'overlay', result: action.result }
    case 'CLOSE':
      return { kind: 'idle' }
    case 'TIMEOUT':
      // Защита от устаревшего таймера: переход в idle только если всё ещё toasting
      return state.kind === 'toasting' ? { kind: 'idle' } : state
    default:
      return state
  }
}

interface Mps3DContextValue {
  state: Mps3DState
  requestToast: (result: MpsScenarioResult) => void
  open: (result: MpsScenarioResult) => void
  close: () => void
}

const Mps3DContext = createContext<Mps3DContextValue | null>(null)

export function Mps3DProvider({ children }: { children: ReactNode }) {
  const [state, dispatch] = useReducer(reducer, { kind: 'idle' } as Mps3DState)

  const requestToast = useCallback((result: MpsScenarioResult) => {
    dispatch({ type: 'REQUEST_TOAST', result, now: Date.now() })
  }, [])

  const open = useCallback((result: MpsScenarioResult) => {
    dispatch({ type: 'OPEN', result })
  }, [])

  const close = useCallback(() => {
    dispatch({ type: 'CLOSE' })
  }, [])

  // 5-секундный таймер: запускается на каждый вход в toasting,
  // зависимость на startedAt — при замене result (новый run_id во время
  // toasting) startedAt обновляется и таймер пересоздаётся.
  const startedAt = state.kind === 'toasting' ? state.startedAt : null
  useEffect(() => {
    if (state.kind !== 'toasting') return
    const timer = setTimeout(() => dispatch({ type: 'TIMEOUT' }), TOAST_MS)
    return () => clearTimeout(timer)
  }, [state.kind, startedAt])

  return (
    <Mps3DContext.Provider value={{ state, requestToast, open, close }}>
      {children}
      {/* Тост и оверлей будут добавлены ниже после соответствующих тасков */}
    </Mps3DContext.Provider>
  )
}

export function useMps3D(): Mps3DContextValue {
  const ctx = useContext(Mps3DContext)
  if (ctx === null) {
    throw new Error('useMps3D must be used within <Mps3DProvider>')
  }
  return ctx
}
