import { useEffect, useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { N_CONTROLS, N_STATES } from '@/types/mps'
import type { MpsMatrices } from '@/types/mps'

interface MatrixEditorProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onChange: (m: MpsMatrices) => void
  onApply: () => void
  onValidate: () => void
  onReset: () => void
  saving?: boolean
}

type MatrixKey = 'A' | 'B' | 'C' | 'D'
type VectorKey = 'Q_diag' | 'R_diag' | 'u_min' | 'u_max'

const MATRIX_LABELS: Record<MatrixKey, string> = {
  A: 'A — матрица состояния (5×5)',
  B: 'B — матрица управления (5×2)',
  C: 'C — матрица выхода (k×5, default I)',
  D: 'D — матрица прямой связи (k×2, default 0)',
}
const VECTOR_LABELS: Record<VectorKey, string> = {
  Q_diag: 'Q (диагональ, 5)',
  R_diag: 'R (диагональ, 2)',
  u_min: 'u_min (2)',
  u_max: 'u_max (2)',
}

function isValidNumberCell(value: string): boolean {
  if (value.trim() === '') return false
  const n = Number(value)
  return Number.isFinite(n)
}

interface MatrixGridProps {
  label: string
  matrix: number[][]
  appliedMatrix: number[][] | undefined
  cols: number
  onCell: (row: number, col: number, value: string) => void
}

function MatrixGrid({ label, matrix, appliedMatrix, cols, onCell }: MatrixGridProps) {
  return (
    <div className="space-y-1">
      <div className="text-xs font-mono text-muted-foreground">{label}</div>
      <div
        className="grid gap-1"
        style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` }}
      >
        {matrix.flatMap((row, ri) =>
          row.map((cell, ci) => {
            const appliedCell = appliedMatrix?.[ri]?.[ci]
            const dirty = appliedCell !== undefined && appliedCell !== cell
            const text = String(cell)
            const valid = isValidNumberCell(text)
            return (
              <Input
                key={`${label}-${ri}-${ci}`}
                value={text}
                onChange={(e) => onCell(ri, ci, e.target.value)}
                className={[
                  'h-7 text-xs font-mono',
                  !valid ? 'border-red-500' : '',
                  dirty ? 'bg-amber-500/10' : '',
                ].join(' ')}
                aria-invalid={!valid}
                aria-label={`${label} [${ri},${ci}]`}
              />
            )
          }),
        )}
      </div>
    </div>
  )
}

interface VectorGridProps {
  label: string
  values: number[]
  appliedValues: number[] | undefined
  onCell: (i: number, value: string) => void
}

function VectorGrid({ label, values, appliedValues, onCell }: VectorGridProps) {
  return (
    <div className="space-y-1">
      <div className="text-xs font-mono text-muted-foreground">{label}</div>
      <div
        className="grid gap-1"
        style={{ gridTemplateColumns: `repeat(${values.length}, minmax(0, 1fr))` }}
      >
        {values.map((v, i) => {
          const appliedV = appliedValues?.[i]
          const dirty = appliedV !== undefined && appliedV !== v
          const text = String(v)
          const valid = isValidNumberCell(text)
          return (
            <Input
              key={`${label}-${i}`}
              value={text}
              onChange={(e) => onCell(i, e.target.value)}
              className={[
                'h-7 text-xs font-mono',
                !valid ? 'border-red-500' : '',
                dirty ? 'bg-amber-500/10' : '',
              ].join(' ')}
              aria-label={`${label} [${i}]`}
            />
          )
        })}
      </div>
    </div>
  )
}

/** Inline grid editor для MPS-матриц A/B/C/D + векторов Q/R/u_min/u_max + N. */
export function MatrixEditor({
  applied,
  draft,
  onChange,
  onApply,
  onValidate,
  onReset,
  saving,
}: MatrixEditorProps) {
  const initial = draft ?? applied
  const [local, setLocal] = useState<MpsMatrices | null>(initial)

  useEffect(() => {
    // Re-sync если applied/draft пришли позже (refresh, reset).
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

  if (!local) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Матрицы</CardTitle>
        </CardHeader>
        <CardContent className="text-sm text-muted-foreground">
          Загрузка матриц…
        </CardContent>
      </Card>
    )
  }

  function update(key: MatrixKey, ri: number, ci: number, raw: string) {
    if (!local) return
    const parsed = Number(raw)
    const value = Number.isFinite(parsed) ? parsed : (raw as unknown as number)
    const m = { ...local, [key]: local[key].map((row, r) =>
      r === ri ? row.map((c, k) => (k === ci ? (value as number) : c)) : row,
    ) }
    setLocal(m)
    onChange(m)
  }

  function updateVector(key: VectorKey, i: number, raw: string) {
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

  return (
    <Card>
      <CardHeader>
        <CardTitle>Матрицы</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 text-xs">
        <MatrixGrid label={MATRIX_LABELS.A} matrix={local.A}
                    appliedMatrix={applied?.A} cols={N_STATES}
                    onCell={(r, c, v) => update('A', r, c, v)} />
        <MatrixGrid label={MATRIX_LABELS.B} matrix={local.B}
                    appliedMatrix={applied?.B} cols={N_CONTROLS}
                    onCell={(r, c, v) => update('B', r, c, v)} />
        <MatrixGrid label={MATRIX_LABELS.C} matrix={local.C}
                    appliedMatrix={applied?.C} cols={N_STATES}
                    onCell={(r, c, v) => update('C', r, c, v)} />
        <MatrixGrid label={MATRIX_LABELS.D} matrix={local.D}
                    appliedMatrix={applied?.D} cols={N_CONTROLS}
                    onCell={(r, c, v) => update('D', r, c, v)} />

        <VectorGrid label={VECTOR_LABELS.Q_diag} values={local.Q_diag}
                    appliedValues={applied?.Q_diag}
                    onCell={(i, v) => updateVector('Q_diag', i, v)} />
        <VectorGrid label={VECTOR_LABELS.R_diag} values={local.R_diag}
                    appliedValues={applied?.R_diag}
                    onCell={(i, v) => updateVector('R_diag', i, v)} />
        <VectorGrid label={VECTOR_LABELS.u_min} values={local.u_min}
                    appliedValues={applied?.u_min}
                    onCell={(i, v) => updateVector('u_min', i, v)} />
        <VectorGrid label={VECTOR_LABELS.u_max} values={local.u_max}
                    appliedValues={applied?.u_max}
                    onCell={(i, v) => updateVector('u_max', i, v)} />

        <div className="flex items-center gap-2">
          <span className="text-xs font-mono text-muted-foreground">N (горизонт):</span>
          <Input
            value={String(local.horizon_N)}
            onChange={(e) => updateHorizon(e.target.value)}
            className="h-7 text-xs font-mono w-20"
            aria-label="horizon_N"
          />
        </div>

        <div className="flex flex-wrap gap-2 pt-2">
          <Button size="sm" disabled={!valid || saving} onClick={onApply}>
            {saving ? 'Применяем…' : dirty ? 'Apply (есть изменения)' : 'Apply'}
          </Button>
          <Button size="sm" variant="secondary" onClick={onValidate} disabled={!valid}>
            Validate
          </Button>
          <Button size="sm" variant="outline" onClick={onReset}>
            Reset
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
