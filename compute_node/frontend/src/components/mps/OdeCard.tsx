import { useMemo, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { KatexFormula } from './KatexFormula'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { detectPhysics, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'
import type { EquationIndex } from './HighlightContext'

interface OdeCardProps {
  matrices: MpsMatrices | null
  showNumeric?: boolean
}

interface EquationDef {
  index: EquationIndex
  symbolic: string
  numericFormula: (tauV: number, tauOmega: number) => string
  description: string
}

const EQUATIONS: EquationDef[] = [
  {
    index: 0,
    symbolic: '\\dot{s} = v',
    numericFormula: () => '\\dot{s} = v',
    description: 'уравнение для ṡ',
  },
  {
    index: 1,
    symbolic: '\\dot{v} = -\\frac{1}{\\tau_v}\\,v + \\frac{1}{\\tau_v}\\,u_v',
    numericFormula: (tauV) =>
      `\\dot{v} = ${(-1 / tauV).toFixed(2)}\\,v + ${(1 / tauV).toFixed(2)}\\,u_v`,
    description: 'уравнение для v̇',
  },
  {
    index: 2,
    symbolic: '\\dot{\\theta} = \\omega',
    numericFormula: () => '\\dot{\\theta} = \\omega',
    description: 'уравнение для θ̇',
  },
  {
    index: 3,
    symbolic:
      '\\dot{\\omega} = -\\frac{1}{\\tau_\\omega}\\,\\omega + \\frac{1}{\\tau_\\omega}\\,u_\\omega',
    numericFormula: (_, tauOmega) =>
      `\\dot{\\omega} = ${(-1 / tauOmega).toFixed(2)}\\,\\omega + ${(1 / tauOmega).toFixed(2)}\\,u_\\omega`,
    description: 'уравнение для ω̇',
  },
  {
    index: 4,
    symbolic: '\\dot{e}_{int} = v_{target} - v',
    numericFormula: () => '\\dot{e}_{int} = v_{target} - v',
    description: 'уравнение для ė_int',
  },
]

export function OdeCard({ matrices, showNumeric = false }: OdeCardProps) {
  const { hovered, setEquation } = useMpsHighlight()
  const [localShowNumeric, setLocalShowNumeric] = useState(showNumeric)

  const physics = useMemo(() => (matrices ? detectPhysics(matrices) : null), [matrices])

  const tauV = physics?.tau_v ?? DEFAULT_TAU_V
  const tauOmega = physics?.tau_omega ?? DEFAULT_TAU_OMEGA
  const deviationCount = physics?.deviations.length ?? 0

  const cellHighlightedEqRow: number | null = (() => {
    if (hovered.equation !== null) return hovered.equation
    if (hovered.cell && (hovered.cell.matrix === 'A' || hovered.cell.matrix === 'B')) {
      return hovered.cell.row
    }
    if (hovered.vector && hovered.vector.name === 'Q') return hovered.vector.index
    return null
  })()

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between">
          <span>ОДУ-модель робота</span>
          <button
            type="button"
            className="text-xs text-muted-foreground hover:text-foreground transition-colors"
            onClick={() => setLocalShowNumeric((s) => !s)}
            aria-label="toggle numeric display"
          >
            {localShowNumeric ? '∑ символьно' : '№ численно'}
          </button>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 text-sm">
        <div className="rounded border bg-muted/30 p-3 space-y-1">
          {EQUATIONS.map((eq) => {
            const isHl = cellHighlightedEqRow === eq.index
            const formula = localShowNumeric
              ? eq.numericFormula(tauV, tauOmega)
              : eq.symbolic
            return (
              <button
                key={eq.index}
                type="button"
                aria-label={eq.description}
                onMouseEnter={() => setEquation(eq.index)}
                onMouseLeave={() => setEquation(null)}
                className={[
                  'block w-full text-left px-2 py-1 rounded transition-colors',
                  isHl
                    ? 'bg-primary/10 ring-1 ring-primary/40'
                    : 'hover:bg-muted',
                ].join(' ')}
              >
                <KatexFormula formula={formula} inline />
              </button>
            )
          })}
        </div>

        <div className="text-xs text-muted-foreground space-y-1">
          <div>
            Состояние: <span className="font-mono">x = [s, v, θ, ω, e_int]ᵀ</span>
          </div>
          <div>
            Управление: <span className="font-mono">u = [v_cmd, ω_cmd]ᵀ</span>
          </div>
        </div>

        <div className="rounded border p-2 text-xs space-y-1 bg-muted/20">
          <div className="text-muted-foreground">Дискретизация ZOH (Ts = 50 мс):</div>
          <div className="font-mono">x[k+1] = A·x[k] + B·u[k]</div>
          <div className="font-mono">y[k]   = C·x[k] + D·u[k]</div>
        </div>

        {deviationCount > 0 && (
          <div className="rounded border border-orange-300 bg-orange-500/10 p-2 text-xs">
            <span className="font-medium text-orange-700">
              ⚠ нестандартные члены ({deviationCount})
            </span>{' '}
            <span className="text-muted-foreground">
              — A или B содержат коэффициенты вне канонической линеаризации.
              Подсвечены оранжевой пунктирной рамкой в матрицах справа.
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
