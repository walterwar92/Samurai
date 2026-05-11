import { memo } from 'react'
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { SensorSample } from '@/hooks/useSensorHistory'

interface SensorChartsProps {
  data: SensorSample[]
}

const CHART_HEIGHT = 140

// Цвета серий — из палитры редизайна (см. PROMPT.md §6.12).
// Используем CSS-переменные через hsl(var()) — авто-адаптация к light/dark.
const STROKE_ACCENT = 'hsl(var(--accent))'
const STROKE_WARNING = 'hsl(var(--warning))'
const STROKE_INFO = 'hsl(var(--info))'
const STROKE_DANGER = 'hsl(var(--danger))'

// Дополнительные цвета (приглушённые из ball/fsm палитр) для серий, где
// нужно 3-4 различимых линии (IMU YPR, accel XYZ, odom XY).
const STROKE_LAVENDER = '#B894C9'   // fsm.grabbing
const STROKE_PINK = '#C97B7B'       // danger / ball.red
const STROKE_TEAL = '#7DA88A'       // success / ball.green
const STROKE_BLUE_MUTED = '#7DA1C9' // info / fsm.searching

const axisProps = {
  tick: { fontSize: 10, fill: 'hsl(var(--foreground-faint))', fontFamily: 'JetBrains Mono' },
  stroke: 'hsl(var(--surface-3))',
  tickLine: { stroke: 'hsl(var(--surface-3))' },
} as const

const gridStyle = {
  stroke: 'hsl(var(--surface-3))',
  strokeDasharray: '0',
  vertical: false,
} as const

const tooltipStyle = {
  contentStyle: {
    background: 'hsl(var(--surface-3))',
    border: '1px solid hsl(var(--border-subtle))',
    borderRadius: '8px',
    fontFamily: 'JetBrains Mono',
    fontSize: '12px',
    color: 'hsl(var(--foreground))',
  },
  labelStyle: { color: 'hsl(var(--foreground-muted))' },
  itemStyle: { color: 'hsl(var(--foreground))' },
} as const

const legendStyle = {
  fontSize: 11,
  color: 'hsl(var(--foreground-muted))',
  fontFamily: 'JetBrains Mono',
} as const

function TimeXAxis() {
  return (
    <XAxis
      dataKey="t"
      {...axisProps}
      tickFormatter={(v: number) => `${v.toFixed(0)}с`}
    />
  )
}

function SectionLabel({ children }: { children: React.ReactNode }) {
  return (
    <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
      {children}
    </span>
  )
}

/**
 * Memoised: Recharts re-renders are expensive (SVG rebuild for 7 charts).
 * `SensorSample[]` is owned by `useSensorHistory`, which only produces a new
 * array when a sample is appended — so referential equality is the right
 * short-circuit here.
 */
export const SensorCharts = memo(function SensorCharts({ data }: SensorChartsProps) {
  if (data.length < 2) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Графики сенсоров</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-body text-foreground-muted italic">Накопление данных…</p>
        </CardContent>
      </Card>
    )
  }

  const tooltipLabelFormatter = (v: number) => `${v.toFixed(1)}с`

  return (
    <Card>
      <CardHeader>
        <CardTitle>Графики сенсоров</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* Linear Speed */}
        <div className="space-y-1">
          <SectionLabel>Линейная скорость (м/с)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Legend wrapperStyle={legendStyle} />
              <Line
                type="monotone"
                dataKey="fusedSpeed"
                stroke={STROKE_ACCENT}
                strokeWidth={1.5}
                dot={false}
                activeDot={{ r: 3, fill: STROKE_ACCENT }}
                name="Скорость (EKF)"
              />
              <Line
                type="monotone"
                dataKey="linearSpeed"
                stroke={STROKE_WARNING}
                strokeWidth={1.5}
                dot={false}
                name="Лин. (vx)"
                strokeDasharray="4 2"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Angular Speed */}
        <div className="space-y-1">
          <SectionLabel>Угловая скорость (рад/с)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Line
                type="monotone"
                dataKey="angularSpeed"
                stroke={STROKE_LAVENDER}
                strokeWidth={1.5}
                dot={false}
                name="Угловая (рад/с)"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Acceleration */}
        <div className="space-y-1">
          <SectionLabel>Ускорение (без гравитации, g)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Legend wrapperStyle={legendStyle} />
              <Line type="monotone" dataKey="accelX" stroke={STROKE_PINK} strokeWidth={1.5} dot={false} name="Accel X" />
              <Line type="monotone" dataKey="accelY" stroke={STROKE_INFO} strokeWidth={1.5} dot={false} name="Accel Y" />
              <Line type="monotone" dataKey="accelZ" stroke={STROKE_TEAL} strokeWidth={1} dot={false} name="Accel Z" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Battery & Temperature */}
        <div className="space-y-1">
          <SectionLabel>Батарея (%) / Температура (°C)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} domain={[0, 100]} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Line
                type="monotone"
                dataKey="battery"
                stroke={STROKE_ACCENT}
                strokeWidth={1.5}
                dot={false}
                name="Батарея %"
              />
              <Line
                type="monotone"
                dataKey="cpuTemp"
                stroke={STROKE_DANGER}
                strokeWidth={1.5}
                dot={false}
                name="Температура °C"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* IMU YPR */}
        <div className="space-y-1">
          <SectionLabel>IMU (Yaw / Pitch / Roll)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} domain={[-180, 180]} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Line type="monotone" dataKey="imuYaw" stroke={STROKE_INFO} strokeWidth={1.5} dot={false} name="Yaw" />
              <Line type="monotone" dataKey="imuPitch" stroke={STROKE_LAVENDER} strokeWidth={1.5} dot={false} name="Pitch" />
              <Line type="monotone" dataKey="imuRoll" stroke={STROKE_WARNING} strokeWidth={1.5} dot={false} name="Roll" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Distance */}
        <div className="space-y-1">
          <SectionLabel>Дистанция (м)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} domain={[0, 'auto']} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Line type="monotone" dataKey="range" stroke={STROKE_BLUE_MUTED} strokeWidth={1.5} dot={false} name="Дистанция" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Odometry */}
        <div className="space-y-1">
          <SectionLabel>Одометрия (X / Y)</SectionLabel>
          <ResponsiveContainer width="100%" height={CHART_HEIGHT}>
            <LineChart data={data}>
              <CartesianGrid {...gridStyle} />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip {...tooltipStyle} labelFormatter={tooltipLabelFormatter} />
              <Line type="monotone" dataKey="odomX" stroke={STROKE_ACCENT} strokeWidth={1.5} dot={false} name="X" />
              <Line type="monotone" dataKey="odomY" stroke={STROKE_TEAL} strokeWidth={1.5} dot={false} name="Y" />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  )
})
