import { useMemo } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { buildCanonical, detectPhysics } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

interface PhysicsParamsProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onPatch: (next: MpsMatrices) => void
  defaults: { tau_v: number; tau_omega: number }
}

const TAU_V_MIN = 0.05
const TAU_V_MAX = 0.50
const TAU_OMEGA_MIN = 0.05
const TAU_OMEGA_MAX = 0.50
const TAU_STEP = 0.005

interface SliderRowProps {
  label: string
  ariaLabel: string
  value: number | null
  min: number
  max: number
  step: number
  disabled?: boolean
  unit: string
  onChange: (next: number) => void
  onReset: () => void
  resetLabel: string
  cellsInfo: string
  incoherent?: boolean
}

function SliderRow(props: SliderRowProps) {
  const {
    label,
    ariaLabel,
    value,
    min,
    max,
    step,
    disabled,
    unit,
    onChange,
    onReset,
    resetLabel,
    cellsInfo,
    incoherent,
  } = props
  const displayValue = value === null ? min : value
  const valueText = incoherent
    ? 'неоднозначно'
    : value === null
      ? 'N/A'
      : `${value.toFixed(3)} ${unit}`

  return (
    <div className="space-y-1">
      <div className="flex items-center justify-between text-xs">
        <span className="font-mono">{label}</span>
        <span className="font-mono text-muted-foreground">{valueText}</span>
      </div>
      <div className="flex items-center gap-2">
        <input
          type="range"
          aria-label={ariaLabel}
          min={min}
          max={max}
          step={step}
          value={displayValue}
          disabled={disabled}
          onChange={(e) => onChange(Number(e.target.value))}
          className="flex-1"
        />
        <Button
          size="icon"
          variant="ghost"
          aria-label={resetLabel}
          onClick={onReset}
          disabled={disabled}
          className="h-7 w-7 text-xs"
          title={resetLabel}
        >
          ↻
        </Button>
      </div>
      <div className="text-xs text-muted-foreground font-mono">{cellsInfo}</div>
    </div>
  )
}

function patchCanonicalCells(
  base: MpsMatrices,
  tauV: number,
  tauOmega: number,
): MpsMatrices {
  const { A, B } = buildCanonical(tauV, tauOmega)
  const A_next = base.A.map((row) => row.slice())
  const B_next = base.B.map((row) => row.slice())
  A_next[0][1] = A[0][1]
  A_next[1][1] = A[1][1]
  A_next[2][3] = A[2][3]
  A_next[3][3] = A[3][3]
  A_next[4][1] = A[4][1]
  B_next[1][0] = B[1][0]
  B_next[3][1] = B[3][1]
  return { ...base, A: A_next, B: B_next }
}

function buildFromScratch(
  base: MpsMatrices,
  tauV: number,
  tauOmega: number,
): MpsMatrices {
  const { A, B } = buildCanonical(tauV, tauOmega)
  return { ...base, A, B }
}

export function PhysicsParams({ applied, draft, onPatch, defaults }: PhysicsParamsProps) {
  const current = draft ?? applied
  const physics = useMemo(() => (current ? detectPhysics(current) : null), [current])

  const tauV = physics?.tau_v ?? null
  const tauOmega = physics?.tau_omega ?? null
  const isNonCanonical = physics?.status === 'non_canonical'
  const isIncoherent = physics?.status === 'incoherent'
  const deviationCount = physics?.deviations.length ?? 0

  function handleTauVChange(next: number) {
    if (!current) return
    onPatch(patchCanonicalCells(current, next, tauOmega ?? defaults.tau_omega))
  }
  function handleTauOmegaChange(next: number) {
    if (!current) return
    onPatch(patchCanonicalCells(current, tauV ?? defaults.tau_v, next))
  }
  function handleResetTauV() {
    if (!current) return
    onPatch(patchCanonicalCells(current, defaults.tau_v, tauOmega ?? defaults.tau_omega))
  }
  function handleResetTauOmega() {
    if (!current) return
    onPatch(patchCanonicalCells(current, tauV ?? defaults.tau_v, defaults.tau_omega))
  }
  function handleRestoreCanonical() {
    if (!current) return
    onPatch(buildFromScratch(current, tauV ?? defaults.tau_v, tauOmega ?? defaults.tau_omega))
  }

  if (!current) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Физические параметры</CardTitle>
        </CardHeader>
        <CardContent className="text-sm text-muted-foreground">Загрузка матриц…</CardContent>
      </Card>
    )
  }

  const tauVCellsInfo =
    tauV === null
      ? `A[1][1] = ${current.A[1][1].toFixed(2)}, B[1][0] = ${current.B[1][0].toFixed(2)}`
      : `A[1][1] = ${(-1 / tauV).toFixed(2)}, B[1][0] = ${(1 / tauV).toFixed(2)}`
  const tauOmegaCellsInfo =
    tauOmega === null
      ? `A[3][3] = ${current.A[3][3].toFixed(2)}, B[3][1] = ${current.B[3][1].toFixed(2)}`
      : `A[3][3] = ${(-1 / tauOmega).toFixed(2)}, B[3][1] = ${(1 / tauOmega).toFixed(2)}`

  return (
    <Card>
      <CardHeader>
        <CardTitle>Физические параметры</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <SliderRow
          label="τ_v (постоянная времени v)"
          ariaLabel="τ_v"
          value={tauV}
          min={TAU_V_MIN}
          max={TAU_V_MAX}
          step={TAU_STEP}
          disabled={isNonCanonical}
          unit="c"
          onChange={handleTauVChange}
          onReset={handleResetTauV}
          resetLabel="Сбросить τ_v"
          cellsInfo={tauVCellsInfo}
          incoherent={isIncoherent && tauV === null}
        />
        <SliderRow
          label="τ_ω (постоянная времени ω)"
          ariaLabel="τ_ω"
          value={tauOmega}
          min={TAU_OMEGA_MIN}
          max={TAU_OMEGA_MAX}
          step={TAU_STEP}
          disabled={isNonCanonical}
          unit="c"
          onChange={handleTauOmegaChange}
          onReset={handleResetTauOmega}
          resetLabel="Сбросить τ_ω"
          cellsInfo={tauOmegaCellsInfo}
          incoherent={isIncoherent && tauOmega === null}
        />

        {physics?.status === 'canonical' && (
          <div className="rounded border border-green-300 bg-green-500/10 p-2 text-xs text-green-800">
            ✓ Каноническая форма
            <div className="text-muted-foreground mt-1">
              Все «крутящиеся» ячейки A, B соответствуют τ_v={tauV?.toFixed(3)}, τ_ω=
              {tauOmega?.toFixed(3)}.
            </div>
          </div>
        )}

        {isIncoherent && (
          <div className="rounded border border-amber-300 bg-amber-500/10 p-2 text-xs text-amber-800">
            ⚠ Несогласованность: значения A и B дают разные τ. Слайдеры показывают «N/A».
          </div>
        )}

        {isNonCanonical && (
          <div className="rounded border border-orange-300 bg-orange-500/10 p-2 text-xs text-orange-800">
            ⚠ Не-каноническая форма: {deviationCount} ненулевых ячеек вне паттерна.
            Слайдеры заблокированы.
          </div>
        )}

        <Button
          size="sm"
          variant={isNonCanonical ? 'default' : 'outline'}
          onClick={handleRestoreCanonical}
          className="w-full"
        >
          Восстановить каноническую форму
        </Button>
      </CardContent>
    </Card>
  )
}
