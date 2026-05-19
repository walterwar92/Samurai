import { memo } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { useRobotLiveState } from '@/hooks/useRobotLiveState'

/**
 * RobotStateVector — постоянная панель «полное состояние робота» для
 * DashboardPage. Подписывается на /ws/robot/live_state (10 Hz) и
 * показывает pose, velocity, IMU YPR + сырые гироскоп/акселерометр + bias.
 *
 * Контракт: docs/superpowers/specs/2026-05-19-robot-live-state-vector-design.md §2.
 *
 * Не требует пропсов — самодостаточен. Не пересекается с глобальным
 * useRobotStore, чтобы не зависеть от частоты SocketIO state_update и
 * не «пузыриться» по re-render'ам не относящихся слайсов state.
 */
export const RobotStateVector = memo(function RobotStateVector() {
  const { point, connected, stale, ageMs } = useRobotLiveState()

  const fmt = (v: number | null | undefined, digits = 3): string => {
    if (v === null || v === undefined || Number.isNaN(v)) return '—'
    const sign = v >= 0 ? '+' : '−'
    return `${sign}${Math.abs(v).toFixed(digits)}`
  }
  const fmtDeg = (v: number | null | undefined): string => {
    if (v === null || v === undefined || Number.isNaN(v)) return '—'
    const sign = v >= 0 ? '+' : '−'
    return `${sign}${Math.abs(v).toFixed(1)}°`
  }
  const fmtAge = (ms: number | null): string => {
    if (ms === null) return ''
    if (ms < 1000) return `${(ms / 1000).toFixed(1)}s`
    return `${Math.round(ms / 1000)}s`
  }

  const badge = (() => {
    if (!connected) return { label: 'disconnected', cls: 'text-foreground-faint' }
    if (stale) return { label: `stale · ${fmtAge(ageMs)}`, cls: 'text-amber-400' }
    return { label: '● live', cls: 'text-emerald-400' }
  })()

  const hasEkf = point?.imu.has_ekf ?? false
  const orientationHeader = hasEkf
    ? 'IMU — ориентация (EKF активен)'
    : 'IMU — ориентация (raw fallback)'

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-foreground-muted font-semibold flex items-center justify-between">
          <span>Состояние робота</span>
          <span className={`text-[10px] normal-case tracking-normal ${badge.cls}`}>
            {badge.label}
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 px-3 pb-3 font-mono tabular-nums text-[11px]">
        {/* Pose */}
        <Section title="Поза (world)">
          <Row label="x" value={`${fmt(point?.pose.x)} м`} />
          <Row label="y" value={`${fmt(point?.pose.y)} м`} />
          <Row
            label="θ"
            value={
              point
                ? `${fmtDeg(point.pose.yaw_deg)} (${fmt(point.pose.yaw_rad, 4)} рад)`
                : '—'
            }
          />
        </Section>

        {/* Velocity */}
        <Section title="Скорости">
          <Row label="v" value={`${fmt(point?.vel.linear)} м/с`} />
          <Row
            label="ω"
            value={
              point
                ? `${fmtDeg((point.vel.angular * 180) / Math.PI)}/с (${fmt(
                    point.vel.angular,
                  )} рад/с)`
                : '—'
            }
          />
        </Section>

        {/* IMU orientation */}
        <Section title={orientationHeader}>
          <div className="grid grid-cols-3 gap-2">
            <Cell label="Y" value={fmtDeg(point?.imu.ypr_deg[0])} />
            <Cell label="P" value={fmtDeg(point?.imu.ypr_deg[1])} />
            <Cell label="R" value={fmtDeg(point?.imu.ypr_deg[2])} />
          </div>
        </Section>

        {/* IMU raw */}
        <Section title="IMU — сырые">
          <Row
            label="gyro"
            value={
              point
                ? `[${fmt(point.imu.gyro[0])}, ${fmt(point.imu.gyro[1])}, ${fmt(point.imu.gyro[2])}] рад/с`
                : '—'
            }
          />
          <Row
            label="accel"
            value={
              point
                ? `[${fmt(point.imu.accel[0])}, ${fmt(point.imu.accel[1])}, ${fmt(point.imu.accel[2])}] м/с²`
                : '—'
            }
          />
          <Row
            label="bias"
            value={
              point?.imu.ekf_bias_deg
                ? `[${fmtDeg(point.imu.ekf_bias_deg[0])}, ${fmtDeg(point.imu.ekf_bias_deg[1])}, ${fmtDeg(point.imu.ekf_bias_deg[2])}]/с`
                : '—'
            }
          />
        </Section>

        {/* ZUPT + age */}
        <div className="flex items-center justify-between text-[10px] text-foreground-muted border-t border-subtle pt-2">
          <span>
            ZUPT:{' '}
            {point ? (
              point.stationary ? (
                <span className="text-emerald-400">STATIONARY</span>
              ) : (
                <span>moving</span>
              )
            ) : (
              '—'
            )}
          </span>
          <span>
            {ageMs === null ? '' : `обновлено ${fmtAge(ageMs)} назад`}
          </span>
        </div>
      </CardContent>
    </Card>
  )
})


function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="space-y-1">
      <div className="text-[10px] uppercase tracking-wider text-foreground-muted font-sans">
        {title}
      </div>
      <div className="space-y-0.5">{children}</div>
    </div>
  )
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between">
      <span className="text-foreground-faint">{label}</span>
      <span>{value}</span>
    </div>
  )
}

function Cell({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex flex-col">
      <span className="text-[9px] uppercase tracking-wider text-foreground-faint">
        {label}
      </span>
      <span>{value}</span>
    </div>
  )
}
