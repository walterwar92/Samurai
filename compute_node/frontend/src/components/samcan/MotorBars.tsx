interface MotorBarsProps {
  left?: number     // 0..255
  right?: number
  basePwm?: number  // визуальная отметка крейсера
}

export function MotorBars({ left = 0, right = 0, basePwm = 180 }: MotorBarsProps) {
  return (
    <div className="flex items-end gap-4 h-[180px] px-2">
      <Bar label="L" value={left} basePwm={basePwm} />
      <Bar label="R" value={right} basePwm={basePwm} />
    </div>
  )
}

function Bar({ label, value, basePwm }: { label: string; value: number; basePwm: number }) {
  const pct = Math.max(0, Math.min(1, value / 255))
  const baseY = (1 - basePwm / 255) * 100
  const color =
    value === 0          ? 'rgb(71 85 105)' :
    value > basePwm + 10 ? 'rgb(34 197 94)' :
    value < basePwm - 10 ? 'rgb(234 179 8)' :
                            'rgb(56 189 248)'

  return (
    <div className="flex flex-col items-center gap-1 flex-1">
      <div className="text-[10px] uppercase tracking-wider text-muted-foreground font-semibold">
        {label}
      </div>
      <div className="relative w-full h-[150px] bg-zinc-900 rounded overflow-hidden border border-border">
        {/* base PWM marker */}
        <div
          className="absolute left-0 right-0 border-t border-dashed border-zinc-600 pointer-events-none"
          style={{ top: `${baseY}%` }}
        >
          <span className="absolute -top-3 right-1 text-[8px] text-zinc-500 font-mono">
            {basePwm}
          </span>
        </div>
        {/* fill */}
        <div
          className="absolute left-0 right-0 bottom-0 transition-all duration-150"
          style={{
            height: `${pct * 100}%`,
            backgroundColor: color,
            boxShadow: value > 0 ? `0 0 12px ${color}55` : 'none',
          }}
        />
      </div>
      <div className="text-[12px] font-mono font-bold tabular-nums" style={{ color }}>
        {value}
      </div>
    </div>
  )
}
