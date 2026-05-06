import { useEffect, useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { MatrixGrid } from './MatrixGrid'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { detectPhysics } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

type Tab = 'AB' | 'QRN' | 'CD'

const STATE_LABELS = ['s', 'v', 'θ', 'ω', 'e_int']
const STATE_DOTS = ['ṡ', 'v̇', 'θ̇', 'ω̇', 'ė_int']
const Y_LABELS = ['y₁', 'y₂', 'y₃', 'y₄', 'y₅']
const CONTROL_LABELS = ['u_v', 'u_ω']

interface MatrixEditorProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onChange: (m: MpsMatrices) => void
  onApply: () => void
  onValidate: () => void
  onReset: () => void
  saving?: boolean
  validationStatus?: 'unknown' | 'stable' | 'unstable'
}

const TABS: { id: Tab; label: string }[] = [
  { id: 'AB',  label: 'Динамика A·B' },
  { id: 'QRN', label: 'Веса Q·R·N' },
  { id: 'CD',  label: 'Выход C·D' },
]

function makeMatrix(rows: number, cols: number, fill = 0): number[][] {
  return Array.from({ length: rows }, () => Array(cols).fill(fill))
}

function makeIdentity(n: number): number[][] {
  return Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)),
  )
}

export function MatrixEditor({
  applied,
  draft,
  onChange,
  onApply,
  onValidate,
  onReset,
  saving,
  validationStatus = 'unknown',
}: MatrixEditorProps) {
  const initial = draft ?? applied
  const [local, setLocal] = useState<MpsMatrices | null>(initial)
  const [tab, setTab] = useState<Tab>('AB')

  useEffect(() => {
    setLocal(draft ?? applied)
  }, [applied, draft])

  const valid = useMemo(() => {
    if (!local) return false
    const flat = [
      ...local.A.flat(),
      ...local.B.flat(),
      ...local.C.flat(),
      ...local.D.flat(),
      ...local.Q_diag,
      ...local.R_diag,
      ...local.u_min,
      ...local.u_max,
    ]
    if (flat.some((v) => !Number.isFinite(v))) return false
    if (local.horizon_N < 1 || local.horizon_N > 200) return false
    if (local.u_min.some((lo, i) => lo >= local.u_max[i])) return false
    if (local.R_diag.some((r) => r <= 0)) return false
    if (local.Q_diag.some((q) => q < 0)) return false
    return true
  }, [local])

  const dirty = useMemo(() => {
    if (!local || !applied) return Boolean(local)
    return JSON.stringify(local) !== JSON.stringify(applied)
  }, [local, applied])

  const dirtyCount = useMemo(() => {
    if (!local || !applied) return 0
    let count = 0
    const compareMatrix = (a: number[][], b: number[][]) => {
      for (let i = 0; i < a.length; i++) {
        for (let j = 0; j < a[i].length; j++) {
          if (!b[i] || b[i][j] !== a[i][j]) count++
        }
      }
    }
    compareMatrix(local.A, applied.A)
    compareMatrix(local.B, applied.B)
    compareMatrix(local.C, applied.C)
    compareMatrix(local.D, applied.D)
    if (local.horizon_N !== applied.horizon_N) count++
    for (let i = 0; i < 5; i++) if (local.Q_diag[i] !== applied.Q_diag[i]) count++
    for (let i = 0; i < 2; i++) if (local.R_diag[i] !== applied.R_diag[i]) count++
    for (let i = 0; i < 2; i++) {
      if (local.u_min[i] !== applied.u_min[i]) count++
      if (local.u_max[i] !== applied.u_max[i]) count++
    }
    return count
  }, [local, applied])

  const physicsStatus = useMemo(() => (local ? detectPhysics(local).status : null), [local])

  if (!local) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Матрицы</CardTitle>
        </CardHeader>
        <CardContent className="text-sm text-muted-foreground">Загрузка матриц…</CardContent>
      </Card>
    )
  }

  function updateMatrix(key: 'A' | 'B' | 'C' | 'D', ri: number, ci: number, raw: string) {
    if (!local) return
    const parsed = Number(raw)
    const value = Number.isFinite(parsed) ? parsed : (raw as unknown as number)
    const m = {
      ...local,
      [key]: local[key].map((row, r) =>
        r === ri ? row.map((c, k) => (k === ci ? (value as number) : c)) : row,
      ),
    }
    setLocal(m)
    onChange(m)
  }

  function updateVector(key: 'Q_diag' | 'R_diag' | 'u_min' | 'u_max', i: number, raw: string) {
    if (!local) return
    const parsed = Number(raw)
    const value = Number.isFinite(parsed) ? parsed : (raw as unknown as number)
    const v = local[key].map((c, k) => (k === i ? (value as number) : c))
    const m = { ...local, [key]: v }
    setLocal(m)
    onChange(m)
  }

  function updateHorizon(raw: string) {
    if (!local) return
    const parsed = Number(raw)
    if (!Number.isFinite(parsed)) return
    const m = { ...local, horizon_N: Math.round(parsed) }
    setLocal(m)
    onChange(m)
  }

  function resetCToIdentity() {
    if (!local) return
    const m = { ...local, C: makeIdentity(5) }
    setLocal(m)
    onChange(m)
  }
  function resetDToZero() {
    if (!local) return
    const m = { ...local, D: makeMatrix(5, 2, 0) }
    setLocal(m)
    onChange(m)
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between gap-2 flex-wrap">
          <span>Матрицы и веса</span>
          <div className="flex gap-2">
            <Button size="sm" variant="secondary" onClick={onValidate} disabled={!valid}>
              Validate
            </Button>
            <Button size="sm" disabled={!valid || saving} onClick={onApply}>
              {saving ? 'Применяем…' : dirty ? `Apply (${dirtyCount})` : 'Apply'}
            </Button>
          </div>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div role="tablist" className="flex gap-1 border-b">
          {TABS.map((t) => (
            <button
              key={t.id}
              role="tab"
              aria-selected={tab === t.id}
              type="button"
              onClick={() => setTab(t.id)}
              className={[
                'px-3 py-1.5 text-sm border-b-2 transition-colors',
                tab === t.id
                  ? 'border-primary text-primary font-medium'
                  : 'border-transparent text-muted-foreground hover:text-foreground',
              ].join(' ')}
            >
              {t.label}
            </button>
          ))}
        </div>

        {tab === 'AB' && (
          <div className="space-y-4">
            <section>
              <div className="text-xs font-mono text-muted-foreground mb-1">
                A — матрица состояния (5×5)
              </div>
              <MatrixGrid
                matrix="A"
                values={local.A}
                applied={applied?.A}
                rowLabels={STATE_DOTS}
                colLabels={STATE_LABELS}
                onCell={(r, c, v) => updateMatrix('A', r, c, v)}
              />
            </section>
            <section>
              <div className="text-xs font-mono text-muted-foreground mb-1">
                B — матрица управления (5×2)
              </div>
              <MatrixGrid
                matrix="B"
                values={local.B}
                applied={applied?.B}
                rowLabels={STATE_DOTS}
                colLabels={CONTROL_LABELS}
                onCell={(r, c, v) => updateMatrix('B', r, c, v)}
              />
            </section>
          </div>
        )}

        {tab === 'QRN' && (
          <QRNSection
            local={local}
            updateVector={updateVector}
            updateHorizon={updateHorizon}
          />
        )}

        {tab === 'CD' && (
          <div className="space-y-4">
            <section>
              <div className="flex items-center justify-between mb-1">
                <div className="text-xs font-mono text-muted-foreground">
                  C — матрица выхода (5×5, default I)
                </div>
                <Button size="sm" variant="ghost" onClick={resetCToIdentity}>
                  Сбросить к I₅
                </Button>
              </div>
              <MatrixGrid
                matrix="C"
                values={local.C}
                applied={applied?.C}
                rowLabels={Y_LABELS}
                colLabels={STATE_LABELS}
                onCell={(r, c, v) => updateMatrix('C', r, c, v)}
              />
            </section>
            <section>
              <div className="flex items-center justify-between mb-1">
                <div className="text-xs font-mono text-muted-foreground">
                  D — прямая связь (5×2, default 0)
                </div>
                <Button size="sm" variant="ghost" onClick={resetDToZero}>
                  Сбросить к 0
                </Button>
              </div>
              <MatrixGrid
                matrix="D"
                values={local.D}
                applied={applied?.D}
                rowLabels={Y_LABELS}
                colLabels={CONTROL_LABELS}
                onCell={(r, c, v) => updateMatrix('D', r, c, v)}
              />
              <div className="text-xs text-muted-foreground mt-2">
                ⓘ y = C·x + D·u — для UI-визуализации, в управлении не используется.
              </div>
            </section>
          </div>
        )}

        <div className="flex items-center justify-between gap-2 pt-2 border-t flex-wrap">
          <div className="text-xs text-muted-foreground space-x-2">
            {dirty ? (
              <span>
                Изменено: <strong>{dirtyCount}</strong>
              </span>
            ) : (
              <span>Без изменений</span>
            )}
            <span>·</span>
            <span>
              Канонически:{' '}
              <strong
                className={
                  physicsStatus === 'canonical'
                    ? 'text-green-700'
                    : physicsStatus === 'incoherent'
                      ? 'text-amber-700'
                      : 'text-orange-700'
                }
              >
                {physicsStatus === 'canonical'
                  ? '✓'
                  : physicsStatus === 'incoherent'
                    ? 'неоднозначно'
                    : 'отклонения'}
              </strong>
            </span>
            <span>·</span>
            <span>
              λ:{' '}
              <strong
                className={
                  validationStatus === 'stable'
                    ? 'text-green-700'
                    : validationStatus === 'unstable'
                      ? 'text-red-700'
                      : 'text-muted-foreground'
                }
              >
                {validationStatus === 'stable'
                  ? 'stable'
                  : validationStatus === 'unstable'
                    ? 'unstable'
                    : '?'}
              </strong>
            </span>
          </div>
          <Button size="sm" variant="outline" onClick={onReset}>
            Reset draft
          </Button>
        </div>

        {!valid && (
          <div className="text-xs text-red-500" role="alert">
            Есть невалидные значения — Apply заблокирован.
          </div>
        )}
      </CardContent>
    </Card>
  )
}

interface QRNSectionProps {
  local: MpsMatrices
  updateVector: (
    key: 'Q_diag' | 'R_diag' | 'u_min' | 'u_max',
    i: number,
    raw: string,
  ) => void
  updateHorizon: (raw: string) => void
}

function QRNSection({ local, updateVector, updateHorizon }: QRNSectionProps) {
  const Q_LABELS = ['Q[s]', 'Q[v]', 'Q[θ]', 'Q[ω]', 'Q[eᵢ]']
  const Q_DESC = ['состояние s', 'состояние v', 'состояние θ', 'состояние ω', 'состояние e_int']
  const R_LABELS = ['R[u_v]', 'R[u_ω]']
  const R_DESC = ['v_cmd', 'ω_cmd']
  const { setVector } = useMpsHighlight()

  return (
    <div className="space-y-4">
      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">
          Q — веса ошибки состояния (диагональ)
        </div>
        <div className="space-y-1.5">
          {local.Q_diag.map((v, i) => (
            <div
              key={`q-${i}`}
              className="flex items-center gap-2 text-xs"
              onMouseEnter={() => setVector({ name: 'Q', index: i })}
              onMouseLeave={() => setVector(null)}
            >
              <span className="font-mono w-12">{Q_LABELS[i]}</span>
              <span className="text-muted-foreground w-32">{Q_DESC[i]}</span>
              <input
                type="range"
                min={0}
                max={100}
                step={0.5}
                value={v}
                onChange={(e) => updateVector('Q_diag', i, e.target.value)}
                className="flex-1"
                aria-label={Q_LABELS[i]}
              />
              <span className="font-mono w-12 text-right">{v.toFixed(2)}</span>
            </div>
          ))}
        </div>
        <div className="text-xs text-muted-foreground mt-2">
          ⓘ Большее Qᵢ = MPC сильнее штрафует отклонение i-го состояния от reference.
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">
          R — веса управления (диагональ)
        </div>
        <div className="space-y-1.5">
          {local.R_diag.map((v, i) => (
            <div key={`r-${i}`} className="flex items-center gap-2 text-xs">
              <span className="font-mono w-12">{R_LABELS[i]}</span>
              <span className="text-muted-foreground w-32">{R_DESC[i]}</span>
              <input
                type="range"
                min={0.1}
                max={20}
                step={0.1}
                value={v}
                onChange={(e) => updateVector('R_diag', i, e.target.value)}
                className="flex-1"
                aria-label={R_LABELS[i]}
              />
              <span className="font-mono w-12 text-right">{v.toFixed(2)}</span>
            </div>
          ))}
        </div>
        <div className="text-xs text-muted-foreground mt-2">
          ⓘ Большие R ⇒ экономия управления, плавнее езда.
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">N — горизонт прогноза</div>
        <div className="flex items-center gap-2 text-xs">
          <span className="font-mono w-12">N steps</span>
          <input
            type="range"
            min={1}
            max={50}
            step={1}
            value={local.horizon_N}
            onChange={(e) => updateHorizon(e.target.value)}
            className="flex-1"
            aria-label="horizon_N"
          />
          <span className="font-mono w-24 text-right">
            {local.horizon_N} ({(local.horizon_N * 0.05).toFixed(2)} c)
          </span>
        </div>
        <div className="text-xs text-muted-foreground mt-1">
          ⓘ Больше N — точнее план, дороже вычисления (Ts = 50 мс).
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">Ограничения управления</div>
        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="flex items-center gap-2">
            <span className="font-mono w-12">u_min</span>
            {local.u_min.map((v, i) => (
              <Input
                key={`umin-${i}`}
                type="number"
                step="0.01"
                value={String(v)}
                onChange={(e) => updateVector('u_min', i, e.target.value)}
                className="h-7 text-xs font-mono"
                aria-label={`u_min[${i}]`}
              />
            ))}
          </div>
          <div className="flex items-center gap-2">
            <span className="font-mono w-12">u_max</span>
            {local.u_max.map((v, i) => (
              <Input
                key={`umax-${i}`}
                type="number"
                step="0.01"
                value={String(v)}
                onChange={(e) => updateVector('u_max', i, e.target.value)}
                className="h-7 text-xs font-mono"
                aria-label={`u_max[${i}]`}
              />
            ))}
          </div>
        </div>
        <div className="text-xs text-muted-foreground mt-1">
          ⓘ MPC clip-ит u в эти рамки на каждом шаге.
        </div>
      </section>
    </div>
  )
}
