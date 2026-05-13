interface CompassHUDProps {
  /** Yaw в градусах. Конвенция робота: 0° = направление +X сцены («север» в нашей карте). */
  yaw: number
}

/**
 * HUD-компас в правом нижнем углу. SVG, не Three.js — рендерится поверх Canvas
 * как абсолютно позиционированный div. Стрелка показывает направление носа
 * робота, тики N/E/S/W вращаются вместе со стрелкой.
 *
 * Конвенция yaw: 0° = +X сцены (выходит из origin вправо), растёт против
 * часовой (CCW), как в ROS/2D-карте. На SVG North = вверх, поэтому SVG-угол
 * `θ_svg = 90° - yaw`. При yaw=0 N (+X сцены) смотрит вверх — это совпадает
 * с привычной 2D-картой.
 */
export function CompassHUD({ yaw }: CompassHUDProps) {
  // Конвертация yaw (мат. CCW от +X) → SVG-rotate (CW от +Y_up).
  // Стрелку рисуем вверх (yaw=0) и поворачиваем на -(yaw) (минус потому что
  // SVG-rotate идёт по часовой). Но т.к. N в SVG = вверх, итого: rotate(-yaw).
  const yawDeg = ((yaw % 360) + 360) % 360
  const arrowRotate = -yawDeg

  const size = 110
  const cx = size / 2
  const cy = size / 2
  const r = size / 2 - 8

  // Кардинальные точки в координатах робота: N=+X (yaw=0), E=-Y, S=-X, W=+Y.
  // На SVG (Y вниз) N = вверх. После rotate(-yaw) метки вращаются вместе со стрелкой.
  const cardinals: { label: string; angle: number; color: string }[] = [
    { label: 'N', angle: 0, color: '#ef4444' },   // нос робота, красный
    { label: 'E', angle: 90, color: '#a1a1aa' },
    { label: 'S', angle: 180, color: '#a1a1aa' },
    { label: 'W', angle: 270, color: '#a1a1aa' },
  ]

  return (
    <div
      className="absolute bottom-3 right-3 z-10 select-none pointer-events-none"
      style={{
        background: 'rgba(20, 20, 30, 0.85)',
        border: '1px solid #3f3f46',
        borderRadius: 10,
        padding: 8,
        backdropFilter: 'blur(4px)',
        WebkitBackdropFilter: 'blur(4px)',
      }}
    >
      <svg width={size} height={size}>
        {/* фоновый круг */}
        <circle cx={cx} cy={cy} r={r} fill="rgba(40,40,55,0.6)" stroke="#52525b" strokeWidth={1} />

        {/* тики каждые 30° */}
        {Array.from({ length: 12 }, (_, i) => {
          const a = (i * 30 * Math.PI) / 180
          const r1 = r - 3
          const r2 = i % 3 === 0 ? r - 9 : r - 6
          const x1 = cx + Math.sin(a) * r1
          const y1 = cy - Math.cos(a) * r1
          const x2 = cx + Math.sin(a) * r2
          const y2 = cy - Math.cos(a) * r2
          return (
            <line
              key={i}
              x1={x1}
              y1={y1}
              x2={x2}
              y2={y2}
              stroke="#71717a"
              strokeWidth={i % 3 === 0 ? 1.5 : 0.8}
            />
          )
        })}

        {/* группа: метки сторон + стрелка вращаются вместе на -yaw */}
        <g transform={`rotate(${arrowRotate} ${cx} ${cy})`}>
          {cardinals.map((c) => {
            const a = (c.angle * Math.PI) / 180
            const lr = r - 18
            const x = cx + Math.sin(a) * lr
            const y = cy - Math.cos(a) * lr
            return (
              <text
                key={c.label}
                x={x}
                y={y}
                fontSize={10}
                fontFamily="ui-monospace, monospace"
                fontWeight={600}
                fill={c.color}
                textAnchor="middle"
                dominantBaseline="middle"
              >
                {c.label}
              </text>
            )
          })}

          {/* стрелка-нос робота */}
          <polygon
            points={`${cx},${cy - r + 12} ${cx - 6},${cy + 4} ${cx + 6},${cy + 4}`}
            fill="#ef4444"
            stroke="#fca5a5"
            strokeWidth={0.6}
          />
          {/* «хвост» стрелки */}
          <polygon
            points={`${cx - 4},${cy + 4} ${cx + 4},${cy + 4} ${cx},${cy + r - 12}`}
            fill="#71717a"
            opacity={0.8}
          />
        </g>

        {/* центральная точка */}
        <circle cx={cx} cy={cy} r={2} fill="#fbbf24" />
      </svg>

      <div
        style={{
          marginTop: 4,
          textAlign: 'center',
          fontFamily: 'ui-monospace, monospace',
          fontSize: 11,
          color: '#e4e4e7',
        }}
      >
        yaw: <span style={{ color: '#fbbf24' }}>{yaw.toFixed(1)}°</span>
      </div>
    </div>
  )
}
