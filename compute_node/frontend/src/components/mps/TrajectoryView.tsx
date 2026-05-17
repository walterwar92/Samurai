import { Box } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'
import { useMps3D } from './Mps3DProvider'

interface TrajectoryViewProps {
  result: MpsScenarioResult | null
  liveTelemetry?: MpsTelemetryPoint[]
}

const W = 520
const H = 220
const PADDING = 24

/** Маппинг телеметрии → 2D top-down координаты в локальном фрейме старта.
 *  Schema 1.2+: используем честные x_local/y_local. Fallback для pre-1.2
 *  прогонов из History — (s·cosθ, s·sinθ). */
function points2d(telemetry: MpsTelemetryPoint[]): { x: number; y: number }[] {
  return telemetry.map((p) => {
    // Schema 1.2+: телеметрия содержит честные локальные координаты
    if (p.x_local !== undefined && p.y_local !== undefined) {
      return { x: p.x_local, y: p.y_local }
    }
    // Fallback для pre-1.2 прогонов из History (s·cosθ, s·sinθ)
    return {
      x: (p.x[0] ?? 0) * Math.cos(p.x[2] ?? 0),
      y: (p.x[0] ?? 0) * Math.sin(p.x[2] ?? 0),
    }
  })
}

export function TrajectoryView({ result, liveTelemetry }: TrajectoryViewProps) {
  const telemetry = liveTelemetry?.length
    ? liveTelemetry
    : result?.telemetry ?? []
  const distance = result?.request.distance ?? 2.0
  const pts = points2d(telemetry)

  // Auto-scale
  const maxAbs = Math.max(distance + 0.5, ...pts.map((p) => Math.max(Math.abs(p.x), Math.abs(p.y))))
  const scale = (W - 2 * PADDING) / (2 * maxAbs)
  const centerX = W / 2
  const centerY = H / 2

  const project = (x: number, y: number): [number, number] => [
    centerX + x * scale,
    centerY - y * scale,
  ]

  const pathD = pts.length > 0
    ? pts
        .map((p, i) => {
          const [px, py] = project(p.x, p.y)
          return `${i === 0 ? 'M' : 'L'}${px.toFixed(2)},${py.toFixed(2)}`
        })
        .join(' ')
    : ''

  const last = pts.length > 0 ? pts[pts.length - 1] : null
  const target = project(distance, 0)
  const start = project(0, 0)

  const phi = result?.request.target_heading ?? 0
  // Пунктирный план: прямая (0,0)→(D,0) в локальном фрейме старта
  const planStart = project(0, 0)
  const planEnd = project(distance, 0)
  // Короткая стрелка финального курса в (D, 0)
  const arrowLen = Math.max(0.05, distance * 0.15)
  const arrowTip = project(distance + arrowLen * Math.cos(phi),
                            arrowLen * Math.sin(phi))

  return (
    <Card>
      <CardHeader right={<TrajectoryOpen3DButton result={result} />}>
        <CardTitle>Траектория (top-down)</CardTitle>
      </CardHeader>
      <CardContent>
        <svg width="100%" height={H} viewBox={`0 0 ${W} ${H}`} role="img"
             aria-label="trajectory top-down">
          <rect width={W} height={H} fill="transparent" />
          {/* defs: arrowhead marker (один раз для стрелки финального курса) */}
          <defs>
            <marker id="arrowhead" markerWidth="6" markerHeight="6"
                    refX="5" refY="3" orient="auto">
              <path d="M0,0 L6,3 L0,6 z" fill="#16a34a" />
            </marker>
          </defs>
          {/* axes */}
          <line x1={PADDING} y1={centerY} x2={W - PADDING} y2={centerY}
                stroke="#94a3b8" strokeWidth={0.5} />
          <line x1={centerX} y1={PADDING} x2={centerX} y2={H - PADDING}
                stroke="#94a3b8" strokeWidth={0.5} />
          {/* план: пунктир (0,0)→(D,0) */}
          <line x1={planStart[0]} y1={planStart[1]}
                x2={planEnd[0]} y2={planEnd[1]}
                stroke="#94a3b8" strokeWidth={1.5}
                strokeDasharray="4 3" />
          {/* стрелка финального курса в (D, 0) */}
          <line data-testid="target-heading-arrow"
                x1={target[0]} y1={target[1]}
                x2={arrowTip[0]} y2={arrowTip[1]}
                stroke="#16a34a" strokeWidth={1.5}
                markerEnd="url(#arrowhead)" />
          {/* target */}
          <circle cx={target[0]} cy={target[1]} r={6} fill="none"
                  stroke="#16a34a" strokeWidth={1.5} />
          <text x={target[0] + 8} y={target[1] - 6}
                fontSize={10} fill="#16a34a">
            target s={distance.toFixed(2)} м
          </text>
          {/* start */}
          <circle cx={start[0]} cy={start[1]} r={4} fill="#64748b" />
          <text x={start[0] + 6} y={start[1] - 4}
                fontSize={10} fill="#64748b">
            старт
          </text>
          {/* path */}
          {pathD && (
            <path d={pathD} fill="none" stroke="#2563eb" strokeWidth={1.5} />
          )}
          {/* current */}
          {last && (() => {
            const [px, py] = project(last.x, last.y)
            return (
              <>
                <circle cx={px} cy={py} r={5} fill="#dc2626" />
                <text x={px + 6} y={py + 12} fontSize={10} fill="#dc2626">
                  s={Math.hypot(last.x, last.y).toFixed(2)} м
                </text>
              </>
            )
          })()}
          {/* legend (top-left) */}
          <g transform={`translate(${PADDING + 4}, 14)`} fontSize={10} fontFamily="monospace">
            <line x1={0} y1={0} x2={20} y2={0} stroke="#94a3b8" strokeWidth={1.5} strokeDasharray="4 3" />
            <text x={26} y={3} fill="#94a3b8">план</text>
            <line x1={70} y1={0} x2={90} y2={0} stroke="#2563eb" strokeWidth={1.5} />
            <text x={96} y={3} fill="#2563eb">путь</text>
            <circle cx={140} cy={0} r={4} fill="none" stroke="#16a34a" strokeWidth={1.5} />
            <text x={150} y={3} fill="#16a34a">target</text>
            <circle cx={200} cy={0} r={3} fill="#dc2626" />
            <text x={208} y={3} fill="#dc2626">текущая</text>
          </g>
          {/* scale bar (bottom-left): 1 метр */}
          {(() => {
            const scaleBarPx = scale
            const x0 = PADDING + 4
            const y0 = H - 12
            return (
              <g fontSize={10} fontFamily="monospace" fill="#475569">
                <line x1={x0} y1={y0} x2={x0 + scaleBarPx} y2={y0} stroke="#475569" strokeWidth={1.5} />
                <line x1={x0} y1={y0 - 3} x2={x0} y2={y0 + 3} stroke="#475569" />
                <line x1={x0 + scaleBarPx} y1={y0 - 3} x2={x0 + scaleBarPx} y2={y0 + 3} stroke="#475569" />
                <text x={x0 + scaleBarPx / 2} y={y0 + 14} textAnchor="middle">1 м</text>
              </g>
            )
          })()}
        </svg>
        <div className="mt-1 text-xs text-muted-foreground">
          Локальный фрейм старта; план показан штрихом, стрелка — финальный курс.
        </div>
      </CardContent>
    </Card>
  )
}

function TrajectoryOpen3DButton({ result }: { result: MpsScenarioResult | null }) {
  const mps3D = useMps3D()
  const disabled = !result || (result.telemetry?.length ?? 0) < 2
  return (
    <Button
      type="button"
      variant="outline"
      size="sm"
      disabled={disabled}
      onClick={() => result && mps3D.open(result)}
      title="3D-просмотр траектории"
      aria-label="3D-просмотр траектории"
    >
      <Box className="w-4 h-4" />
      <span className="ml-1">3D</span>
    </Button>
  )
}
