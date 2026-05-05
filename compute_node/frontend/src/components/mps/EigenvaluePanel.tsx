import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { ComplexNumber } from '@/types/mps'

interface EigenvaluePanelProps {
  open: ComplexNumber[]
  closed: ComplexNumber[]
  isPlantStable: boolean
  isClosedLoopStable: boolean
}

const SIZE = 220
const RADIUS = 90
const CENTER = SIZE / 2

function projectToSvg(z: ComplexNumber): { x: number; y: number } {
  // Map complex disk |z|≤2 → SVG circle. Стабильность — внутри RADIUS.
  const x = CENTER + z.re * RADIUS
  const y = CENTER - z.im * RADIUS  // Y инвертирован
  return { x, y }
}

export function EigenvaluePanel({
  open,
  closed,
  isPlantStable,
  isClosedLoopStable,
}: EigenvaluePanelProps) {
  return (
    <Card>
      <CardHeader>
        <CardTitle>Собственные значения</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex flex-wrap gap-4 items-start">
          <svg width={SIZE} height={SIZE} role="img" aria-label="unit circle eigenvalues">
            <rect width={SIZE} height={SIZE} fill="transparent" />
            {/* axes */}
            <line x1={0} y1={CENTER} x2={SIZE} y2={CENTER} stroke="#94a3b8" strokeWidth={0.5} />
            <line x1={CENTER} y1={0} x2={CENTER} y2={SIZE} stroke="#94a3b8" strokeWidth={0.5} />
            {/* unit circle */}
            <circle
              cx={CENTER}
              cy={CENTER}
              r={RADIUS}
              fill="none"
              stroke="#64748b"
              strokeWidth={1.5}
              strokeDasharray="3 3"
            />
            {/* eigenvalues — open */}
            {open.map((z, i) => {
              const { x, y } = projectToSvg(z)
              const stable = Math.hypot(z.re, z.im) < 1
              return (
                <circle
                  key={`o-${i}`}
                  cx={x}
                  cy={y}
                  r={5}
                  fill={stable ? '#16a34a' : '#dc2626'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
            {/* eigenvalues — closed */}
            {closed.map((z, i) => {
              const { x, y } = projectToSvg(z)
              const stable = Math.hypot(z.re, z.im) < 1
              return (
                <rect
                  key={`c-${i}`}
                  x={x - 4}
                  y={y - 4}
                  width={8}
                  height={8}
                  fill={stable ? '#0ea5e9' : '#f97316'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
          </svg>

          <div className="text-xs space-y-1">
            <div>
              <span className="inline-block w-3 h-3 rounded-full mr-2 align-middle"
                    style={{ background: '#16a34a' }} />
              λ(A) — устойчивость объекта:{' '}
              <strong className={isPlantStable ? 'text-green-600' : 'text-red-600'}>
                {isPlantStable ? 'OK' : 'UNSTABLE'}
              </strong>
            </div>
            <div>
              <span className="inline-block w-3 h-3 mr-2 align-middle"
                    style={{ background: '#0ea5e9' }} />
              λ(A − B·K) — замкнутая система:{' '}
              <strong className={isClosedLoopStable ? 'text-green-600' : 'text-red-600'}>
                {isClosedLoopStable ? 'OK' : 'UNSTABLE'}
              </strong>
            </div>
            <div className="text-muted-foreground">
              Внутри пунктирной окружности — |λ| &lt; 1.
            </div>
          </div>
        </div>

        {(open.length > 0 || closed.length > 0) && (
          <details>
            <summary className="text-xs text-muted-foreground cursor-pointer">
              Численные значения
            </summary>
            <div className="text-xs font-mono mt-2 grid grid-cols-2 gap-4">
              <div>
                <div className="text-muted-foreground">λ(A)</div>
                {open.map((z, i) => (
                  <div key={i}>
                    {z.re.toFixed(3)}
                    {z.im >= 0 ? ' + ' : ' − '}
                    {Math.abs(z.im).toFixed(3)}i
                  </div>
                ))}
              </div>
              <div>
                <div className="text-muted-foreground">λ(A − B·K)</div>
                {closed.map((z, i) => (
                  <div key={i}>
                    {z.re.toFixed(3)}
                    {z.im >= 0 ? ' + ' : ' − '}
                    {Math.abs(z.im).toFixed(3)}i
                  </div>
                ))}
              </div>
            </div>
          </details>
        )}
      </CardContent>
    </Card>
  )
}
