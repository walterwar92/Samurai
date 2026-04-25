interface DistanceRadarProps {
  /** Расстояние до препятствия, см. 999 = нет эха. */
  distance?: number
  /** Порог автостопа, см. */
  stopDistance?: number
  /** Срабатывание стопа */
  obstacle?: boolean
}

/**
 * Полу-круговой радар. Сектор вперёд от робота, заполнение
 * показывает близость препятствия. Цвет переходит зелёный→жёлтый→красный.
 */
export function DistanceRadar({ distance = 999, stopDistance = 10, obstacle }: DistanceRadarProps) {
  const cm = Math.min(distance, 200)
  const ratio = Math.max(0, Math.min(1, 1 - cm / 100))   // 0..1, 1 = очень близко
  const color =
    cm <= stopDistance ? 'rgb(239 68 68)' :
    cm <= 30           ? 'rgb(234 179 8)' :
    cm <= 60           ? 'rgb(132 204 22)' :
                          'rgb(56 189 248)'

  // 5 концентрических дуг: ближняя самая яркая
  const arcs = [10, 25, 50, 80, 120]

  return (
    <div className="flex flex-col items-center">
      <svg width={260} height={150} viewBox="0 0 260 150" className="block">
        <defs>
          <radialGradient id="radarGlow" cx="50%" cy="100%" r="100%">
            <stop offset="0%" stopColor={color} stopOpacity={0.5} />
            <stop offset="100%" stopColor={color} stopOpacity={0} />
          </radialGradient>
        </defs>
        {/* активная подсветка ближайшей зоны */}
        <circle cx={130} cy={140} r={130 * (ratio * 0.9 + 0.1)} fill="url(#radarGlow)" />

        {/* концентрические дуги-метки */}
        {arcs.map((d, i) => {
          const r = (d / 200) * 130
          return (
            <path
              key={d}
              d={`M ${130 - r} 140 A ${r} ${r} 0 0 1 ${130 + r} 140`}
              fill="none"
              stroke="rgb(71 85 105)"
              strokeWidth={1}
              strokeDasharray="2 3"
              opacity={0.6 - i * 0.08}
            />
          )
        })}

        {/* центральная риска (робот) */}
        <circle cx={130} cy={140} r={6} fill={color} />
        <line x1={130} y1={140} x2={130} y2={120} stroke={color} strokeWidth={2} />

        {/* заштрихованная "зона стопа" */}
        <path
          d={`M ${130 - (stopDistance / 200) * 130} 140 A ${(stopDistance / 200) * 130} ${(stopDistance / 200) * 130} 0 0 1 ${130 + (stopDistance / 200) * 130} 140 Z`}
          fill="rgb(239 68 68)"
          opacity={0.18}
        />

        {/* labels */}
        {arcs.map(d => {
          const r = (d / 200) * 130
          return (
            <text
              key={`l-${d}`}
              x={130 + r + 4}
              y={138}
              fontSize={9}
              fill="rgb(148 163 184)"
              fontFamily="monospace"
            >
              {d}
            </text>
          )
        })}
      </svg>
      <div className="flex items-baseline gap-2 mt-1">
        <span className="text-[26px] font-bold font-mono" style={{ color }}>
          {cm >= 999 ? '—' : cm.toFixed(0)}
        </span>
        <span className="text-xs text-muted-foreground">cm</span>
        {obstacle && (
          <span className="text-[10px] uppercase tracking-wider text-red-400 font-semibold ml-2">
            ⚠ STOP
          </span>
        )}
      </div>
    </div>
  )
}
