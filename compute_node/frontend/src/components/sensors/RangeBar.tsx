import { cn } from '@/lib/utils'

interface RangeBarProps {
  value: number
  max?: number
}

export function RangeBar({ value, max = 2 }: RangeBarProps) {
  const pct = Math.min((value / max) * 100, 100)
  const color =
    value < 0.15 ? 'bg-samurai-red' : value < 0.5 ? 'bg-samurai-orange' : 'bg-primary'

  return (
    <div className="flex items-center gap-2">
      <span className="text-xs font-mono w-14 text-right">{value.toFixed(2)} м</span>
      <div className="flex-1 h-2 rounded-full bg-secondary overflow-hidden">
        <div
          className={cn('h-full rounded-full transition-all duration-200', color)}
          style={{ width: `${pct}%` }}
        />
      </div>
    </div>
  )
}
