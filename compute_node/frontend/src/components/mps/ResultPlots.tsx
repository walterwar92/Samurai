import { useMemo, useState } from 'react'
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

type Tab = 'x' | 'u' | 'y' | 's'

interface ResultPlotsProps {
  primary: MpsScenarioResult | null
  /** Опциональные overlays для Compare-mode. */
  overlays?: MpsScenarioResult[]
  /** Live-mode: точки приходят отдельно (через WebSocket). Если задано —
   *  используем их вместо primary.telemetry. */
  liveTelemetry?: MpsTelemetryPoint[]
}

const STATE_LABELS = ['s', 'v', 'θ', 'ω', 'e_int']
const CONTROL_LABELS = ['v_cmd', 'ω_cmd']
const Y_LABELS = ['y₁', 'y₂', 'y₃', 'y₄', 'y₅']

const STATE_COLORS = ['#2563eb', '#16a34a', '#dc2626', '#a855f7', '#ea580c']
const CONTROL_COLORS = ['#0ea5e9', '#f59e0b']

interface ChartRow {
  t: number
  [k: string]: number
}

function buildRows(points: MpsTelemetryPoint[], kind: Tab, prefix: string = ''): ChartRow[] {
  return points.map((p) => {
    const row: ChartRow = { t: p.t }
    if (kind === 'x') {
      p.x.forEach((v, i) => {
        row[`${prefix}${STATE_LABELS[i] ?? `x${i}`}`] = v
      })
    } else if (kind === 'u') {
      p.u.forEach((v, i) => {
        row[`${prefix}${CONTROL_LABELS[i] ?? `u${i}`}`] = v
      })
    } else if (kind === 'y') {
      p.y.forEach((v, i) => {
        row[`${prefix}${Y_LABELS[i] ?? `y${i}`}`] = v
      })
    } else if (kind === 's') {
      row[`${prefix}s`] = p.x[0] ?? 0
      row[`${prefix}s_remaining`] = p.s_remaining
    }
    return row
  })
}

function mergeOverlays(
  base: ChartRow[],
  overlays: { rows: ChartRow[]; suffix: string }[],
): ChartRow[] {
  if (overlays.length === 0) return base
  // Простое объединение по t (округлённому до 3 знаков).
  const map = new Map<number, ChartRow>()
  base.forEach((row) => {
    map.set(Math.round(row.t * 1000), { ...row })
  })
  overlays.forEach(({ rows }) => {
    rows.forEach((row) => {
      const key = Math.round(row.t * 1000)
      const existing = map.get(key) ?? { t: row.t }
      map.set(key, { ...existing, ...row })
    })
  })
  return Array.from(map.values()).sort((a, b) => a.t - b.t)
}

const TABS: { id: Tab; label: string }[] = [
  { id: 'x', label: 'x(t) состояние' },
  { id: 'u', label: 'u(t) управление' },
  { id: 'y', label: 'y(t) выход' },
  { id: 's', label: 's(t) дистанция' },
]

export function ResultPlots({ primary, overlays = [], liveTelemetry }: ResultPlotsProps) {
  const [tab, setTab] = useState<Tab>('s')

  const baseTelemetry = liveTelemetry?.length
    ? liveTelemetry
    : primary?.telemetry ?? []

  const data = useMemo(() => {
    const base = buildRows(baseTelemetry, tab)
    const overlayRows = overlays.map((o, idx) => ({
      rows: buildRows(o.telemetry, tab, `[${idx + 2}]`),
      suffix: `[${idx + 2}]`,
    }))
    return mergeOverlays(base, overlayRows)
  }, [baseTelemetry, overlays, tab])

  const seriesKeys = data.length > 0
    ? Object.keys(data[0]).filter((k) => k !== 't')
    : []

  return (
    <Card>
      <CardHeader>
        <CardTitle>Графики прогона</CardTitle>
      </CardHeader>
      <CardContent className="space-y-2">
        <div role="tablist" className="flex gap-1 border-b">
          {TABS.map((t) => (
            <button
              key={t.id}
              role="tab"
              aria-selected={tab === t.id}
              type="button"
              onClick={() => setTab(t.id)}
              className={[
                'px-3 py-1 text-sm border-b-2',
                tab === t.id
                  ? 'border-primary text-primary'
                  : 'border-transparent text-muted-foreground',
              ].join(' ')}
            >
              {t.label}
            </button>
          ))}
        </div>

        {data.length === 0 ? (
          <div className="h-72 flex items-center justify-center text-sm text-muted-foreground">
            Нет данных — запусти сценарий или подожди телеметрию.
          </div>
        ) : (
          <div className="h-72">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={data}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="t" tickFormatter={(v: number) => v.toFixed(2)}
                       label={{ value: 't, с', position: 'insideBottom', offset: -5 }} />
                <YAxis />
                <Tooltip />
                <Legend />
                {seriesKeys.map((k, i) => {
                  const color =
                    tab === 'u'
                      ? CONTROL_COLORS[i % CONTROL_COLORS.length]
                      : STATE_COLORS[i % STATE_COLORS.length]
                  return (
                    <Line
                      key={k}
                      type="monotone"
                      dataKey={k}
                      stroke={color}
                      dot={false}
                      isAnimationActive={false}
                      strokeWidth={1.5}
                    />
                  )
                })}
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
