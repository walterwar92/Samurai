import { useEffect, useRef, useState } from 'react'
import { Area, AreaChart, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts'

interface Sample {
  t: number       // ms since first sample
  th: number
  d: number
  L: number
  R: number
}

interface TelemetryHistoryProps {
  th?: number
  d?: number
  L?: number
  R?: number
  /** ms между точками (поллинг ~250 мс достаточно) */
  windowSec?: number
}

/**
 * Накапливает поток телеметрии и рисует две sparkline:
 *   - heading θ (deg) — должен быть около 0
 *   - distance d (см) — обстакл-датчик
 */
export function TelemetryHistory({ th, d, L, R, windowSec = 30 }: TelemetryHistoryProps) {
  const [data, setData] = useState<Sample[]>([])
  const startRef = useRef<number>(performance.now())

  useEffect(() => {
    if (th === undefined && d === undefined) return
    const t = performance.now() - startRef.current
    setData(prev => {
      const next = [...prev, {
        t,
        th: th ?? prev[prev.length - 1]?.th ?? 0,
        d: d ?? prev[prev.length - 1]?.d ?? 0,
        L: L ?? prev[prev.length - 1]?.L ?? 0,
        R: R ?? prev[prev.length - 1]?.R ?? 0,
      }]
      const cutoff = t - windowSec * 1000
      return next.filter(s => s.t > cutoff)
    })
  }, [th, d, L, R, windowSec])

  const tickFmt = (t: number) => `-${((data[data.length - 1]?.t ?? 0 - t) / 1000).toFixed(0)}s`

  return (
    <div className="space-y-1.5">
      <ChartCard title="Курс θ (°)" color="rgb(56 189 248)">
        <ResponsiveContainer width="100%" height={70}>
          <AreaChart data={data} margin={{ top: 4, bottom: 0, left: 0, right: 0 }}>
            <defs>
              <linearGradient id="thGrad" x1="0" y1="0" x2="0" y2="1">
                <stop offset="0%" stopColor="rgb(56 189 248)" stopOpacity={0.5} />
                <stop offset="100%" stopColor="rgb(56 189 248)" stopOpacity={0} />
              </linearGradient>
            </defs>
            <XAxis dataKey="t" hide />
            <YAxis domain={[-30, 30]} hide />
            <ReferenceLine y={0} stroke="rgb(34 197 94)" strokeDasharray="3 3" opacity={0.6} />
            <Tooltip
              contentStyle={{ background: 'rgb(15 23 42)', border: '1px solid rgb(51 65 85)', fontSize: 11 }}
              labelFormatter={tickFmt}
              formatter={(v: number) => [v.toFixed(2) + '°', 'θ']}
            />
            <Area
              type="monotone"
              dataKey="th"
              stroke="rgb(56 189 248)"
              strokeWidth={2}
              fill="url(#thGrad)"
              isAnimationActive={false}
            />
          </AreaChart>
        </ResponsiveContainer>
      </ChartCard>

      <ChartCard title="Расстояние (см)" color="rgb(132 204 22)">
        <ResponsiveContainer width="100%" height={70}>
          <AreaChart data={data} margin={{ top: 4, bottom: 0, left: 0, right: 0 }}>
            <defs>
              <linearGradient id="dGrad" x1="0" y1="0" x2="0" y2="1">
                <stop offset="0%" stopColor="rgb(132 204 22)" stopOpacity={0.5} />
                <stop offset="100%" stopColor="rgb(132 204 22)" stopOpacity={0} />
              </linearGradient>
            </defs>
            <XAxis dataKey="t" hide />
            <YAxis domain={[0, 200]} hide />
            <ReferenceLine y={10} stroke="rgb(239 68 68)" strokeDasharray="3 3" opacity={0.6} />
            <Tooltip
              contentStyle={{ background: 'rgb(15 23 42)', border: '1px solid rgb(51 65 85)', fontSize: 11 }}
              labelFormatter={tickFmt}
              formatter={(v: number) => [v.toFixed(1) + ' cm', 'd']}
            />
            <Area
              type="monotone"
              dataKey="d"
              stroke="rgb(132 204 22)"
              strokeWidth={2}
              fill="url(#dGrad)"
              isAnimationActive={false}
            />
          </AreaChart>
        </ResponsiveContainer>
      </ChartCard>
    </div>
  )
}

function ChartCard({
  title,
  color,
  children,
}: {
  title: string
  color: string
  children: React.ReactNode
}) {
  return (
    <div className="bg-zinc-900/60 border border-border rounded-md p-2">
      <div className="flex items-center gap-1.5 mb-1">
        <div className="w-1.5 h-1.5 rounded-full" style={{ background: color }} />
        <span className="text-[10px] uppercase tracking-wider text-muted-foreground font-semibold">
          {title}
        </span>
      </div>
      {children}
    </div>
  )
}
