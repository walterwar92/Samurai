import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { ComplexNumber } from '@/types/mps'

interface EigenvaluePanelProps {
  open: ComplexNumber[]
  closed: ComplexNumber[]
  isPlantStable: boolean
  isClosedLoopStable: boolean
  warnings?: string[]
  lastValidatedAt?: string | null
}

const SIZE = 240
const RADIUS = 95
const CENTER = SIZE / 2

function isFiniteComplex(z: ComplexNumber): boolean {
  // Pydantic v2 в режиме JSON эмитит NaN/Inf как null (см. mps_runner →
  // closed_loop_eigenvalues: при падении MPCController возвращает
  // [complex('nan')] × N). Без этой проверки z.re.toFixed падает.
  return Number.isFinite(z.re) && Number.isFinite(z.im)
}

function projectToSvg(z: ComplexNumber): { x: number; y: number } | null {
  if (!isFiniteComplex(z)) return null
  return { x: CENTER + z.re * RADIUS, y: CENTER - z.im * RADIUS }
}

function isStablePoint(z: ComplexNumber): boolean {
  if (!isFiniteComplex(z)) return false
  return Math.hypot(z.re, z.im) < 1
}

function formatComplex(z: ComplexNumber): string {
  if (!isFiniteComplex(z)) return 'не вычислено (NaN)'
  const sign = z.im >= 0 ? '+' : '−'
  return `${z.re.toFixed(3)} ${sign} ${Math.abs(z.im).toFixed(3)}i`
}

function ChecklistRow({ ok, label, hint }: { ok: boolean; label: string; hint?: string }) {
  return (
    <div className="flex items-start gap-2 text-xs">
      <span className={ok ? 'text-green-600' : 'text-red-600'}>{ok ? '✓' : '✗'}</span>
      <div>
        <div className={ok ? 'text-foreground' : 'text-red-700 font-medium'}>{label}</div>
        {hint && <div className="text-muted-foreground">{hint}</div>}
      </div>
    </div>
  )
}

export function EigenvaluePanel({
  open,
  closed,
  isPlantStable,
  isClosedLoopStable,
  warnings = [],
  lastValidatedAt,
}: EigenvaluePanelProps) {
  const hasPoles = open.length > 0 || closed.length > 0

  return (
    <Card>
      <CardHeader>
        <CardTitle>Анализ устойчивости</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="grid grid-cols-1 md:grid-cols-[auto_1fr] gap-4 items-start">
          <svg width={SIZE} height={SIZE} role="img" aria-label="полюса в единичной окружности">
            <rect width={SIZE} height={SIZE} fill="transparent" />
            <line x1={0} y1={CENTER} x2={SIZE} y2={CENTER} stroke="#94a3b8" strokeWidth={0.5} />
            <line x1={CENTER} y1={0} x2={CENTER} y2={SIZE} stroke="#94a3b8" strokeWidth={0.5} />
            <circle
              cx={CENTER}
              cy={CENTER}
              r={RADIUS}
              fill="none"
              stroke="#64748b"
              strokeWidth={1.5}
              strokeDasharray="4 4"
            />
            {open.map((z, i) => {
              const p = projectToSvg(z)
              if (p === null) return null
              return (
                <circle
                  key={`o-${i}`}
                  cx={p.x}
                  cy={p.y}
                  r={5}
                  fill={isStablePoint(z) ? '#16a34a' : '#dc2626'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
            {closed.map((z, i) => {
              const p = projectToSvg(z)
              if (p === null) return null
              return (
                <rect
                  key={`c-${i}`}
                  x={p.x - 4}
                  y={p.y - 4}
                  width={8}
                  height={8}
                  fill={isStablePoint(z) ? '#0ea5e9' : '#f97316'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
            <text x={CENTER + RADIUS - 8} y={CENTER - 6} fontSize={10} fill="#64748b">
              Re
            </text>
            <text x={CENTER + 6} y={14} fontSize={10} fill="#64748b">
              Im
            </text>
          </svg>

          <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 text-xs font-mono">
            <div>
              <div className="font-sans text-muted-foreground mb-1 flex items-center gap-1">
                <span className="inline-block w-3 h-3 rounded-full bg-green-600" />
                λ(A) — открытый контур
              </div>
              {open.length === 0 ? (
                <div className="text-muted-foreground italic">нет данных</div>
              ) : (
                open.map((z, i) => (
                  <div key={i} className={isStablePoint(z) ? '' : 'text-red-600'}>
                    {formatComplex(z)} {isStablePoint(z) ? '◯' : '⚠'}
                  </div>
                ))
              )}
            </div>
            <div>
              <div className="font-sans text-muted-foreground mb-1 flex items-center gap-1">
                <span className="inline-block w-3 h-3 bg-sky-500" />
                λ(A − B·K) — замкнутый
              </div>
              {closed.length === 0 ? (
                <div className="text-muted-foreground italic">нет данных</div>
              ) : (
                closed.map((z, i) => (
                  <div key={i} className={isStablePoint(z) ? '' : 'text-orange-600'}>
                    {formatComplex(z)} {isStablePoint(z) ? '◻' : '⚠'}
                  </div>
                ))
              )}
            </div>
          </div>
        </div>

        <div className="rounded border bg-muted/20 p-3 space-y-2">
          <ChecklistRow
            ok={isPlantStable}
            label={isPlantStable ? 'Объект (A) устойчив' : 'Объект НЕ устойчив'}
            hint="все |λ(A)| < 1"
          />
          <ChecklistRow
            ok={isClosedLoopStable}
            label={
              isClosedLoopStable
                ? 'Замкнутая система (A − B·K) устойчива'
                : 'Замкнутая система НЕ устойчива'
            }
            hint="все |λ(A − B·K)| < 1 — MPC стабилизирует объект"
          />
          {warnings.length > 0 && (
            <div className="border-t pt-2 space-y-1">
              {warnings.map((w, i) => (
                <div key={i} className="flex items-start gap-2 text-xs text-amber-700">
                  <span>⚠</span>
                  <span>{w}</span>
                </div>
              ))}
            </div>
          )}
        </div>

        {lastValidatedAt && (
          <div className="text-xs text-muted-foreground">Последняя проверка: {lastValidatedAt}</div>
        )}

        {!hasPoles && (
          <div className="text-xs text-muted-foreground italic">
            Нажмите Validate в редакторе матриц, чтобы вычислить полюса.
          </div>
        )}
      </CardContent>
    </Card>
  )
}
