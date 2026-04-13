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

const chartHeight = 140

const axisProps = {
  tick: { fontSize: 9, fill: '#888' },
}

const tooltipStyle = {
  contentStyle: { background: '#1a1a1a', border: '1px solid #333', fontSize: 11 },
}

function TimeXAxis() {
  return (
    <XAxis
      dataKey="t"
      {...axisProps}
      tickFormatter={(v: number) => `${v.toFixed(0)}с`}
    />
  )
}

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
        {/* Linear Speed Chart */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Линейная скорость (м/с)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip
                {...tooltipStyle}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Legend wrapperStyle={{ fontSize: 10 }} />
              <Line
                type="monotone"
                dataKey="fusedSpeed"
                stroke="#22c55e"
                strokeWidth={2}
                dot={false}
                name="Скорость (EKF)"
              />
              <Line
                type="monotone"
                dataKey="linearSpeed"
                stroke="#f59e0b"
                strokeWidth={1.5}
                dot={false}
                name="Лин. (vx)"
                strokeDasharray="4 2"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Angular Speed Chart */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Угловая скорость (рад/с)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip
                {...tooltipStyle}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Line
                type="monotone"
                dataKey="angularSpeed"
                stroke="#a855f7"
                strokeWidth={1.5}
                dot={false}
                name="Угловая (рад/с)"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Acceleration (gravity-compensated) */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Ускорение (без гравитации, g)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip
                {...tooltipStyle}
                labelFormatter={(v: number) => `${v.toFixed(1)}с`}
              />
              <Legend wrapperStyle={{ fontSize: 10 }} />
              <Line type="monotone" dataKey="accelX" stroke="#ef4444" strokeWidth={1.5} dot={false} name="Accel X" />
              <Line type="monotone" dataKey="accelY" stroke="#3b82f6" strokeWidth={1.5} dot={false} name="Accel Y" />
              <Line type="monotone" dataKey="accelZ" stroke="#10b981" strokeWidth={1} dot={false} name="Accel Z" />
            </LineChart>
          </ResponsiveContainer>
        </div>

        {/* Battery & Temperature */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Батарея (%) / Температура (°C)
          </span>
          <ResponsiveContainer width="100%" height={chartHeight}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <TimeXAxis />
              <YAxis {...axisProps} domain={[0, 100]} />
              <Tooltip
                {...tooltipStyle}
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
              <TimeXAxis />
              <YAxis {...axisProps} domain={[-180, 180]} />
              <Tooltip
                {...tooltipStyle}
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
              <TimeXAxis />
              <YAxis {...axisProps} domain={[0, 'auto']} />
              <Tooltip
                {...tooltipStyle}
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
              <TimeXAxis />
              <YAxis {...axisProps} />
              <Tooltip
                {...tooltipStyle}
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
