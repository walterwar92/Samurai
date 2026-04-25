import { useEffect, useState } from 'react'

interface HeadingCompassProps {
  /** Текущий угол робота, градусы (0 = идти прямо, +вправо/-влево) */
  theta?: number
  /** Текущая угловая скорость, °/с */
  omega?: number
  /** Размер квадрата SVG в px */
  size?: number
}

/**
 * Большой SVG-компас. Стрелка показывает текущее θ относительно цели (0).
 * При |θ|→0 кольцо становится зелёным; при больших отклонениях — красным.
 */
export function HeadingCompass({ theta = 0, omega = 0, size = 280 }: HeadingCompassProps) {
  // Сглаживаем стрелку чтобы не дёргалась на каждый кадр телеметрии
  const [smooth, setSmooth] = useState(theta)
  useEffect(() => {
    const id = requestAnimationFrame(() => setSmooth(t => t + (theta - t) * 0.25))
    return () => cancelAnimationFrame(id)
  }, [theta])

  const r = size / 2 - 14
  const cx = size / 2
  const cy = size / 2

  const abs = Math.abs(smooth)
  const ringHue = abs < 3 ? 'rgb(34 197 94)' : abs < 12 ? 'rgb(234 179 8)' : 'rgb(239 68 68)'
  const ticks = Array.from({ length: 36 }, (_, i) => i * 10)

  return (
    <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`} className="block">
      {/* outer ring */}
      <circle cx={cx} cy={cy} r={r + 8} fill="rgb(15 23 42)" />
      <circle
        cx={cx}
        cy={cy}
        r={r}
        fill="none"
        stroke={ringHue}
        strokeWidth={2}
        opacity={0.55}
      />
      {/* tick marks */}
      {ticks.map(deg => {
        const big = deg % 30 === 0
        const a = ((deg - 90) * Math.PI) / 180
        const x1 = cx + Math.cos(a) * (r - (big ? 14 : 6))
        const y1 = cy + Math.sin(a) * (r - (big ? 14 : 6))
        const x2 = cx + Math.cos(a) * r
        const y2 = cy + Math.sin(a) * r
        return (
          <line
            key={deg}
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            stroke="rgb(100 116 139)"
            strokeWidth={big ? 2 : 1}
            opacity={big ? 0.8 : 0.4}
          />
        )
      })}
      {/* labels (cardinal) */}
      {[
        { deg: 0,   l: 'CRS' },
        { deg: 90,  l: '+90' },
        { deg: 180, l: '180' },
        { deg: 270, l: '−90' },
      ].map(({ deg, l }) => {
        const a = ((deg - 90) * Math.PI) / 180
        const x = cx + Math.cos(a) * (r - 28)
        const y = cy + Math.sin(a) * (r - 28)
        return (
          <text
            key={deg}
            x={x}
            y={y}
            textAnchor="middle"
            dominantBaseline="middle"
            fontSize={10}
            fill="rgb(148 163 184)"
            fontFamily="monospace"
          >
            {l}
          </text>
        )
      })}
      {/* target tick (0) */}
      <line
        x1={cx}
        y1={cy - r + 2}
        x2={cx}
        y2={cy - r + 14}
        stroke="rgb(34 197 94)"
        strokeWidth={3}
      />
      {/* arrow */}
      <g transform={`rotate(${smooth} ${cx} ${cy})`}>
        <line
          x1={cx}
          y1={cy}
          x2={cx}
          y2={cy - r + 18}
          stroke="rgb(56 189 248)"
          strokeWidth={3}
          strokeLinecap="round"
        />
        <polygon
          points={`${cx},${cy - r + 14} ${cx - 6},${cy - r + 26} ${cx + 6},${cy - r + 26}`}
          fill="rgb(56 189 248)"
        />
      </g>
      {/* hub */}
      <circle cx={cx} cy={cy} r={8} fill="rgb(30 41 59)" stroke="rgb(56 189 248)" strokeWidth={2} />
      {/* center text */}
      <text
        x={cx}
        y={cy + 38}
        textAnchor="middle"
        fontSize={28}
        fontWeight={700}
        fill="white"
        fontFamily="monospace"
      >
        {smooth.toFixed(1)}°
      </text>
      <text
        x={cx}
        y={cy + 56}
        textAnchor="middle"
        fontSize={10}
        fill="rgb(148 163 184)"
        fontFamily="monospace"
      >
        ω {omega.toFixed(1)}°/s
      </text>
    </svg>
  )
}
