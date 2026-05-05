import { useEffect, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { mpsApi } from '@/lib/mpsApi'
import type { MpsMatrices, MpsScenarioResult } from '@/types/mps'

interface TuningSlidersProps {
  applied: MpsMatrices | null
  /** sim run на каждый change для live re-evaluation. Throttle через useEffect timer. */
  onSimResult: (r: MpsScenarioResult | null) => void
  /** Перенос настроек в MatrixEditor draft. */
  onPromote: (m: MpsMatrices) => void
}

const DEBOUNCE_MS = 300

interface TuningState {
  q0: number
  q1: number
  q2: number
  q3: number
  q4: number
  r0: number
  r1: number
  N: number
}

function buildMatrices(base: MpsMatrices, t: TuningState): MpsMatrices {
  return {
    ...base,
    Q_diag: [t.q0, t.q1, t.q2, t.q3, t.q4],
    R_diag: [t.r0, t.r1],
    horizon_N: Math.max(1, Math.round(t.N)),
  }
}

function toState(m: MpsMatrices): TuningState {
  return {
    q0: m.Q_diag[0] ?? 10,
    q1: m.Q_diag[1] ?? 10,
    q2: m.Q_diag[2] ?? 5,
    q3: m.Q_diag[3] ?? 1,
    q4: m.Q_diag[4] ?? 1,
    r0: m.R_diag[0] ?? 1,
    r1: m.R_diag[1] ?? 1,
    N: m.horizon_N,
  }
}

interface SliderRowProps {
  label: string
  value: number
  min: number
  max: number
  step: number
  onChange: (v: number) => void
}

function SliderRow({ label, value, min, max, step, onChange }: SliderRowProps) {
  return (
    <div className="flex items-center gap-2 text-xs">
      <span className="font-mono w-20">{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="flex-1"
      />
      <span className="font-mono w-16 text-right">{value.toFixed(2)}</span>
    </div>
  )
}

export function TuningSliders({ applied, onSimResult, onPromote }: TuningSlidersProps) {
  const [collapsed, setCollapsed] = useState(true)
  const [state, setState] = useState<TuningState | null>(
    applied ? toState(applied) : null,
  )
  const [pending, setPending] = useState(false)

  useEffect(() => {
    if (applied) setState(toState(applied))
  }, [applied])

  // Sim re-run on any change (debounced). NB: backend применяет sim к
  // currently applied матрицам — для true preview надо сначала Promote +
  // Apply tuned. UI это допускает: после первого Apply слайдеры сходятся
  // быстрее (короткий горизонт re-sim).
  useEffect(() => {
    if (!applied || !state || collapsed) return
    setPending(true)
    const timer = setTimeout(async () => {
      try {
        const r = await mpsApi.runScenario({
          distance: 2.0,
          v_target: 0.15,
          source: 'sim',
        })
        onSimResult(r.result)
      } catch (err) {
        console.warn('tuning sim failed:', err)
      } finally {
        setPending(false)
      }
    }, DEBOUNCE_MS)
    return () => clearTimeout(timer)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state?.q0, state?.q1, state?.q2, state?.q3, state?.q4,
      state?.r0, state?.r1, state?.N, collapsed])

  if (!applied || !state) return null

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between">
          Tuning sliders
          <Button size="sm" variant="ghost"
                  onClick={() => setCollapsed((c) => !c)}>
            {collapsed ? 'Развернуть' : 'Свернуть'}
          </Button>
        </CardTitle>
      </CardHeader>
      {!collapsed && (
        <CardContent className="space-y-1.5">
          <SliderRow label="Q[s]"   value={state.q0} min={0}   max={100} step={0.5}
                     onChange={(v) => setState({ ...state, q0: v })} />
          <SliderRow label="Q[v]"   value={state.q1} min={0}   max={100} step={0.5}
                     onChange={(v) => setState({ ...state, q1: v })} />
          <SliderRow label="Q[θ]"   value={state.q2} min={0}   max={100} step={0.5}
                     onChange={(v) => setState({ ...state, q2: v })} />
          <SliderRow label="Q[ω]"   value={state.q3} min={0}   max={100} step={0.5}
                     onChange={(v) => setState({ ...state, q3: v })} />
          <SliderRow label="Q[e]"   value={state.q4} min={0}   max={100} step={0.5}
                     onChange={(v) => setState({ ...state, q4: v })} />
          <SliderRow label="R[v]"   value={state.r0} min={0.1} max={20}  step={0.1}
                     onChange={(v) => setState({ ...state, r0: v })} />
          <SliderRow label="R[ω]"   value={state.r1} min={0.1} max={20}  step={0.1}
                     onChange={(v) => setState({ ...state, r1: v })} />
          <SliderRow label="N"      value={state.N}  min={1}   max={50}  step={1}
                     onChange={(v) => setState({ ...state, N: v })} />

          <div className="flex items-center gap-2 pt-2">
            <Button size="sm" onClick={() => onPromote(buildMatrices(applied, state))}>
              Promote to draft
            </Button>
            {pending && (
              <span className="text-xs text-muted-foreground">re-sim…</span>
            )}
          </div>
        </CardContent>
      )}
    </Card>
  )
}
