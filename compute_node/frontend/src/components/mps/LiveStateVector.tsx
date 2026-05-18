import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { useMpsLiveState } from '@/hooks/useMpsLiveState'

function fmt(n: number | undefined, digits: number): string {
  if (n === undefined || Number.isNaN(n)) return '—'
  const s = n.toFixed(digits)
  return n >= 0 ? `+${s}` : s
}

function fmtAge(ms: number): string {
  const sec = ms / 1000
  return sec < 10 ? `${sec.toFixed(1)}s` : `${Math.round(sec)}s`
}

function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI
}

interface BadgeProps {
  connected: boolean
  stale: boolean
  ageMs: number | null
}

function StatusBadge({ connected, stale, ageMs }: BadgeProps) {
  if (!connected) {
    return (
      <span
        data-testid="status-badge"
        className="text-xs px-2 py-0.5 rounded bg-muted text-muted-foreground"
      >
        disconnected
      </span>
    )
  }
  if (stale) {
    return (
      <span
        data-testid="status-badge"
        className="text-xs px-2 py-0.5 rounded bg-yellow-500/20 text-yellow-700"
      >
        stale{ageMs !== null && ` · ${fmtAge(ageMs)}`}
      </span>
    )
  }
  return (
    <span
      data-testid="status-badge"
      className="text-xs px-2 py-0.5 rounded bg-green-500/20 text-green-700"
    >
      ● live
    </span>
  )
}

interface RowProps {
  label: string
  value: string
  unit: string
}

function Row({ label, value, unit }: RowProps) {
  return (
    <div className="flex items-baseline gap-2">
      <span className="w-12 text-muted-foreground">{label}</span>
      <span className="flex-1">
        <span className="text-muted-foreground">= </span>
        <span>{value}</span>
      </span>
      {unit && <span className="text-muted-foreground">{unit}</span>}
    </div>
  )
}

export function LiveStateVector() {
  const { point, connected, stale, ageMs } = useMpsLiveState()

  const x = point?.x ?? []
  const u = point?.u ?? []
  const s = x[0]
  const v = x[1]
  const theta = x[2]
  const omega = x[3]
  const eInt = x[4]
  const vCmd = u[0]
  const omegaCmd = u[1]

  const scenarioLabel =
    point?.scenario_active && point.run_id
      ? `сценарий: ${point.run_id.slice(0, 8)}`
      : 'idle'

  return (
    <Card>
      <CardHeader right={<StatusBadge connected={connected} stale={stale} ageMs={ageMs} />}>
        <CardTitle className="text-sm">Вектор состояния</CardTitle>
      </CardHeader>
      <CardContent className="text-xs font-mono tabular-nums space-y-2 pt-0">
        <div className="space-y-0.5">
          <div className="text-muted-foreground">x:</div>
          <Row label="s" value={fmt(s, 3)} unit="м" />
          <Row label="v" value={fmt(v, 3)} unit="м/с" />
          <Row
            label="θ"
            value={theta === undefined ? '—' : `${fmt(radToDeg(theta), 1)}°`}
            unit={theta === undefined ? '' : `(${fmt(theta, 4)} рад)`}
          />
          <Row
            label="ω"
            value={omega === undefined ? '—' : `${fmt(radToDeg(omega), 1)}°/с`}
            unit={omega === undefined ? '' : `(${fmt(omega, 4)} рад/с)`}
          />
          <Row label="e_int" value={fmt(eInt, 4)} unit="" />
        </div>
        <div className="space-y-0.5 border-t pt-2">
          <div className="text-muted-foreground">u:</div>
          <Row label="v_cmd" value={fmt(vCmd, 3)} unit="м/с" />
          <Row
            label="ω_cmd"
            value={omegaCmd === undefined ? '—' : `${fmt(radToDeg(omegaCmd), 1)}°/с`}
            unit={omegaCmd === undefined ? '' : `(${fmt(omegaCmd, 4)} рад/с)`}
          />
        </div>
        <div
          data-testid="scenario-caption"
          className="text-muted-foreground border-t pt-2"
        >
          {scenarioLabel}
          {ageMs !== null && ` · обновлено ${fmtAge(ageMs)} назад`}
        </div>
      </CardContent>
    </Card>
  )
}
