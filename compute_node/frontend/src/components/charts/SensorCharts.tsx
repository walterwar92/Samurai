import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { SensorSample } from '@/hooks/useSensorHistory'

interface SensorChartsProps {
  data: SensorSample[]
}

const chartHeight = 140

export function SensorCharts({ data }: SensorChartsProps) {
  if (data.length < 2) {
    return (
      <Card>
        <CardHeader className="py-2 px-3">
          <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
            Графики сенсоров
          </CardTitle>
        </CardHeader>
        <CardContent className="p-3">
          <p className="text-xs text-muted-foreground italic">
            Накопление данных...
          </p>
        </CardContent>
      </Card>
    )
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Графики сенсоров
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        {/* Battery & Temperature */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Батарея (%) / Температура (°C)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="t"
                tick={{ fontSize: 9, fill: '#888' }}
                tickFormatter={(v: number) => `${v.toFixed(0)}с`}
              />
              <YAxis tick={{ fontSize: 9, fill: '#888' }} domain={[0, 100]} />
              <Tooltip
                contentStyle={{ background: '#1a1a1a', border: '1px solid #333', fontSize: 11 }}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Line
                type="monotone"
                dataKey="battery"
                stroke="#22c55e"
                strokeWidth={1.5}
                dot={false}
                name="Батарея %"
              />
              <Line
                type="monotone"
                dataKey="cpuTemp"
                stroke="#ef4444"
                strokeWidth={1.5}
                dot={false}
                name="Температура °C"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* IMU */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            IMU (Yaw / Pitch / Roll)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="t"
                tick={{ fontSize: 9, fill: '#888' }}
                tickFormatter={(v: number) => `${v.toFixed(0)}с`}
              />
              <YAxis tick={{ fontSize: 9, fill: '#888' }} domain={[-180, 180]} />
              <Tooltip
                contentStyle={{ background: '#1a1a1a', border: '1px solid #333', fontSize: 11 }}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Line type="monotone" dataKey="imuYaw" stroke="#3b82f6" strokeWidth={1.5} dot={false} name="Yaw" />
              <Line type="monotone" dataKey="imuPitch" stroke="#a855f7" strokeWidth={1.5} dot={false} name="Pitch" />
              <Line type="monotone" dataKey="imuRoll" stroke="#f59e0b" strokeWidth={1.5} dot={false} name="Roll" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Distance */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Дистанция (м)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="t"
                tick={{ fontSize: 9, fill: '#888' }}
                tickFormatter={(v: number) => `${v.toFixed(0)}с`}
              />
              <YAxis tick={{ fontSize: 9, fill: '#888' }} domain={[0, 'auto']} />
              <Tooltip
                contentStyle={{ background: '#1a1a1a', border: '1px solid #333', fontSize: 11 }}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Line type="monotone" dataKey="range" stroke="#06b6d4" strokeWidth={1.5} dot={false} name="Дистанция" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Odometry */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Одометрия (X / Y)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="t"
                tick={{ fontSize: 9, fill: '#888' }}
                tickFormatter={(v: number) => `${v.toFixed(0)}с`}
              />
              <YAxis tick={{ fontSize: 9, fill: '#888' }} />
              <Tooltip
                contentStyle={{ background: '#1a1a1a', border: '1px solid #333', fontSize: 11 }}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Line type="monotone" dataKey="odomX" stroke="#ec4899" strokeWidth={1.5} dot={false} name="X" />
              <Line type="monotone" dataKey="odomY" stroke="#14b8a6" strokeWidth={1.5} dot={false} name="Y" />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  )
}
