import { Crosshair, Battery, Gauge, Square, Thermometer } from 'lucide-react'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { FSM_COLORS, COLOUR_CSS, COLOUR_RU, ACTION_RU } from '@/lib/constants'
import { api } from '@/lib/api'
import { cn } from '@/lib/utils'
import type { RobotState } from '@/types/robot'

interface StatusBannerProps {
  state: RobotState | null
}

/**
 * Крупный sticky-banner статуса робота Samurai.
 * Показывает FSM-режим, цель, pose, батарею, CPU-temp + кнопку экстренного стопа.
 * Используется в верху всех страниц Samurai (Dashboard, Admin, 3D, Hardware).
 */
export function SamuraiStatusBanner({ state }: StatusBannerProps) {
  const st    = state?.status
  const pose  = state?.pose
  const vel   = state?.velocity
  const fsm   = (st?.state ?? 'IDLE') as keyof typeof FSM_COLORS
  const fsmColor = FSM_COLORS[fsm] || '#37474f'

  const targetColour = st?.target_colour || ''
  const targetAction = st?.target_action || ''
  const bVolt = state?.battery_voltage
  const bPct  = state?.battery_percent
  const cpu   = state?.cpu_temp
  const detectionOn = state?.detection_enabled ?? true
  const avoidanceOn = state?.obstacle_avoidance_enabled ?? true
  const patrol      = state?.patrol?.active
  const followMe    = state?.follow_me?.active

  const speed = vel?.speed ?? 0

  return (
    <div className="max-w-[1920px] mx-auto px-2.5 pt-2.5">
      <div className="flex flex-wrap items-center gap-2 rounded-lg border border-border bg-gradient-to-r from-zinc-950/80 via-zinc-900/60 to-zinc-950/80 backdrop-blur-sm p-2.5">
        {/* FSM state (большой badge) */}
        <div
          className="px-3 py-1.5 rounded-md border text-[11px] font-bold tracking-wider uppercase"
          style={{
            color: fsmColor,
            borderColor: fsmColor + '66',
            backgroundColor: fsmColor + '22',
          }}
        >
          {fsm}
        </div>

        {/* Target colour + action */}
        {targetColour && (
          <div className="flex items-center gap-1.5 text-[11px]">
            <Crosshair className="w-3 h-3 text-muted-foreground" />
            <span
              className="font-bold"
              style={{ color: COLOUR_CSS[targetColour] || undefined }}
            >
              {COLOUR_RU[targetColour] || targetColour}
            </span>
            {targetAction && (
              <span className="text-muted-foreground">· {ACTION_RU[targetAction] || targetAction}</span>
            )}
          </div>
        )}

        {/* Mode pills */}
        <div className="flex gap-1 text-[9px]">
          {patrol && <Pill label="PATROL" color="rgb(56 189 248)" />}
          {followMe && <Pill label="FOLLOW" color="rgb(132 204 22)" />}
          {!detectionOn && <Pill label="DETECT OFF" color="rgb(239 68 68)" />}
          {!avoidanceOn && <Pill label="NO AVOID" color="rgb(234 179 8)" />}
        </div>

        {/* Divider */}
        <div className="flex-1 min-w-0" />

        {/* Speed */}
        <Metric
          icon={<Gauge className="w-3 h-3" />}
          label="speed"
          value={`${speed.toFixed(2)} м/с`}
        />

        {/* Pose */}
        {pose && (
          <Metric
            icon={null}
            label="pose"
            value={`${pose.x.toFixed(1)},${pose.y.toFixed(1)} · ${(pose.yaw_deg ?? 0).toFixed(0)}°`}
            mono
          />
        )}

        {/* Battery */}
        {bVolt !== undefined && (
          <BatteryMetric volt={bVolt} pct={bPct} />
        )}

        {/* CPU temp */}
        {cpu !== undefined && (
          <Metric
            icon={<Thermometer className={cn('w-3 h-3', cpu > 70 ? 'text-red-400' : cpu > 60 ? 'text-amber-400' : 'text-muted-foreground')} />}
            label="cpu"
            value={`${cpu.toFixed(0)}°C`}
            valueColor={cpu > 70 ? 'rgb(248 113 113)' : cpu > 60 ? 'rgb(251 191 36)' : undefined}
          />
        )}

        {/* Emergency stop */}
        <Button
          size="sm"
          variant="destructive"
          className="h-7 text-xs px-3 font-bold"
          onClick={() => api.emergencyStop()}
        >
          <Square className="w-3 h-3 mr-1" /> STOP
        </Button>
      </div>
    </div>
  )
}

/* ── helpers ── */

function Pill({ label, color }: { label: string; color: string }) {
  return (
    <span
      className="px-1.5 py-0.5 rounded border text-[9px] font-bold tracking-wider"
      style={{
        color,
        borderColor: color + '66',
        backgroundColor: color + '18',
      }}
    >
      {label}
    </span>
  )
}

function Metric({
  icon, label, value, mono, valueColor,
}: {
  icon: React.ReactNode
  label: string
  value: string
  mono?: boolean
  valueColor?: string
}) {
  return (
    <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-zinc-900/40 border border-border/30">
      {icon}
      <span className="text-[9px] uppercase tracking-wider text-muted-foreground">{label}</span>
      <span
        className={cn('text-[11px] font-semibold', mono && 'font-mono')}
        style={{ color: valueColor }}
      >
        {value}
      </span>
    </div>
  )
}

function BatteryMetric({ volt, pct }: { volt: number; pct?: number }) {
  const lvl = pct ?? (volt > 0 ? Math.min(100, Math.max(0, ((volt - 6.5) / 1.9) * 100)) : 0)
  const color = lvl > 50 ? 'rgb(34 197 94)' : lvl > 20 ? 'rgb(234 179 8)' : 'rgb(239 68 68)'
  return (
    <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-zinc-900/40 border border-border/30">
      <Battery className="w-3 h-3" style={{ color }} />
      <span className="text-[9px] uppercase tracking-wider text-muted-foreground">bat</span>
      <div className="relative w-14 h-2 rounded-sm bg-zinc-800 border border-border/60 overflow-hidden">
        <div
          className="absolute inset-y-0 left-0"
          style={{ width: `${lvl}%`, background: color, transition: 'width 200ms' }}
        />
      </div>
      <span className="text-[11px] font-mono font-semibold" style={{ color }}>
        {volt.toFixed(1)}В
      </span>
    </div>
  )
}

/* Также экспортируем отдельный компактный status-pill для других мест */
export function FsmPill({ state }: { state: string }) {
  const fsm = state as keyof typeof FSM_COLORS
  const color = FSM_COLORS[fsm] || '#37474f'
  return (
    <Badge
      className="text-[10px] font-bold tracking-wider border-0"
      style={{ backgroundColor: color, color: '#fff' }}
    >
      {state}
    </Badge>
  )
}
