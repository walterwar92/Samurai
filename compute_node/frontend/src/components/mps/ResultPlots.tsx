import { useMemo, useState } from 'react'
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

type Tab = 's' | 'v' | 'theta_omega' | 'u' | 'y' | 'all'

interface ResultPlotsProps {
  primary: MpsScenarioResult | null
  overlays?: MpsScenarioResult[]
  liveTelemetry?: MpsTelemetryPoint[]
}

const STATE_LABELS = ['s', 'v', 'θ', 'ω', 'e_int']
const CONTROL_LABELS = ['v_cmd', 'ω_cmd']
const Y_LABELS = ['y₁', 'y₂', 'y₃', 'y₄', 'y₅']

const STATE_COLORS = ['#2563eb', '#16a34a', '#dc2626', '#a855f7', '#ea580c']
const CONTROL_COLORS = ['#0ea5e9', '#f59e0b']
const OVERLAY_COLORS = ['#94a3b8', '#fbbf24', '#a78bfa']

const TABS: { id: Tab; label: string }[] = [
  { id: 's',           label: 's(t)' },
  { id: 'v',           label: 'v(t)' },
  { id: 'theta_omega', label: 'θ,ω(t)' },
  { id: 'u',           label: 'u(t)' },
  { id: 'y',           label: 'y(t)' },
  { id: 'all',         label: 'Всё вместе' },
]

interface ChartRow {
  t: number
  [k: string]: number
}

function buildRows(points: MpsTelemetryPoint[], tab: Tab, prefix = ''): ChartRow[] {
  return points.map((p) => {
    const row: ChartRow = { t: p.t }
    if (tab === 's') {
      row[`${prefix}s`] = p.x[0] ?? 0
    } else if (tab === 'v') {
      row[`${prefix}v`] = p.x[1] ?? 0
    } else if (tab === 'theta_omega') {
      row[`${prefix}θ`] = p.x[2] ?? 0
      row[`${prefix}ω`] = p.x[3] ?? 0
    } else if (tab === 'u') {
      p.u.forEach((v, i) => {
        row[`${prefix}${CONTROL_LABELS[i] ?? `u${i}`}`] = v
      })
    } else if (tab === 'y') {
      p.y.forEach((v, i) => {
        row[`${prefix}${Y_LABELS[i] ?? `y${i}`}`] = v
      })
    } else if (tab === 'all') {
      p.x.forEach((v, i) => {
        row[`${prefix}${STATE_LABELS[i] ?? `x${i}`}`] = v
      })
      p.u.forEach((v, i) => {
        row[`${prefix}${CONTROL_LABELS[i] ?? `u${i}`}`] = v
      })
    }
    return row
  })
}

function mergeOverlays(
  base: ChartRow[],
  overlayPacks: { rows: ChartRow[] }[],
): ChartRow[] {
  if (overlayPacks.length === 0) return base
  const map = new Map<number, ChartRow>()
  base.forEach((row) => {
    map.set(Math.round(row.t * 1000), { ...row })
  })
  overlayPacks.forEach(({ rows }) => {
    rows.forEach((row) => {
      const key = Math.round(row.t * 1000)
      const existing = map.get(key) ?? { t: row.t }
      map.set(key, { ...existing, ...row })
    })
  })
  return Array.from(map.values()).sort((a, b) => a.t - b.t)
}

function shortRunId(id: string): string {
  return id.length > 8 ? id.slice(0, 8) : id
}

export function ResultPlots({ primary, overlays = [], liveTelemetry }: ResultPlotsProps) {
  const [tab, setTab] = useState<Tab>('s')

  const referenceD = primary?.request.distance ?? null
  const referenceVTarget = primary?.request.v_target ?? null

  const data = useMemo(() => {
    const baseTelemetry = liveTelemetry?.length ? liveTelemetry : (primary?.telemetry ?? [])
    const base = buildRows(baseTelemetry, tab)
    const overlayPacks = overlays.map((o, idx) => ({
      rows: buildRows(o.telemetry, tab, `[${idx + 2}] `),
    }))
    return mergeOverlays(base, overlayPacks)
  }, [liveTelemetry, primary, overlays, tab])

  const seriesKeys = data.length > 0 ? Object.keys(data[0]).filter((k) => k !== 't') : []

  function colorForSeries(key: string, idx: number): string {
    if (key.startsWith('[')) {
      const ovIdx = parseInt(key.slice(1, key.indexOf(']'))) - 2
      return OVERLAY_COLORS[ovIdx % OVERLAY_COLORS.length]
    }
    if (tab === 'u') return CONTROL_COLORS[idx % CONTROL_COLORS.length]
    return STATE_COLORS[idx % STATE_COLORS.length]
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>Графики прогона</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div role="tablist" className="flex gap-1 border-b overflow-x-auto">
          {TABS.map((t) => (
            <button
              key={t.id}
              role="tab"
              aria-selected={tab === t.id}
              type="button"
              onClick={() => setTab(t.id)}
              className={[
                'px-3 py-1 text-sm border-b-2 whitespace-nowrap transition-colors',
                tab === t.id
                  ? 'border-primary text-primary font-medium'
                  : 'border-transparent text-muted-foreground hover:text-foreground',
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
              <LineChart data={data} margin={{ top: 10, right: 24, left: 8, bottom: 16 }}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis
                  dataKey="t"
                  tickFormatter={(v: number) => v.toFixed(2)}
                  label={{ value: 't, с', position: 'insideBottom', offset: -8 }}
                />
                <YAxis />
                <Tooltip />
                <Legend />
                {tab === 's' && referenceD !== null && (
                  <ReferenceLine
                    y={referenceD}
                    stroke="#6b7280"
                    strokeDasharray="4 4"
                    label={{ value: `D = ${referenceD}`, position: 'right', fontSize: 11 }}
                  />
                )}
                {tab === 'v' && referenceVTarget !== null && (
                  <ReferenceLine
                    y={referenceVTarget}
                    stroke="#6b7280"
                    strokeDasharray="4 4"
                    label={{ value: `v* = ${referenceVTarget}`, position: 'right', fontSize: 11 }}
                  />
                )}
                {seriesKeys.map((k, i) => (
                  <Line
                    key={k}
                    type="monotone"
                    dataKey={k}
                    stroke={colorForSeries(k, i)}
                    dot={false}
                    isAnimationActive={false}
                    strokeWidth={k.startsWith('[') ? 1 : 1.5}
                    strokeDasharray={k.startsWith('[') ? '4 2' : undefined}
                    strokeOpacity={k.startsWith('[') ? 0.7 : 1}
                  />
                ))}
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}

        {primary?.metrics && (
          <div className="grid grid-cols-2 gap-x-6 gap-y-1 text-xs font-mono border rounded p-3 bg-muted/20">
            <div className="flex justify-between">
              <span className="text-muted-foreground">overshoot:</span>
              <span>{primary.metrics.overshoot.toFixed(3)} м</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">settling:</span>
              <span>{primary.metrics.settling_time.toFixed(2)} c</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">ss_error:</span>
              <span>{primary.metrics.ss_error.toFixed(3)} м</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">peak v:</span>
              <span>{primary.metrics.peak_v.toFixed(3)} м/с</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">control E:</span>
              <span>{primary.metrics.control_energy.toFixed(3)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">peak ω:</span>
              <span>{primary.metrics.peak_omega.toFixed(3)} рад/с</span>
            </div>
          </div>
        )}

        {primary && (
          <div className="text-xs text-muted-foreground font-mono flex flex-wrap items-center gap-x-3 gap-y-1">
            <span>
              Run: <strong>{shortRunId(primary.run_id)}</strong>
            </span>
            <span>·</span>
            <span>{new Date(primary.started_at).toLocaleString()}</span>
            <span>·</span>
            <span>
              status: <strong>{primary.status}</strong>
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
