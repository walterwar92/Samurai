import { Battery, BatteryLow, BatteryMedium, BatteryFull } from 'lucide-react'

interface BatteryIndicatorProps {
  voltage: number
  percent: number
}

export function BatteryIndicator({ voltage, percent }: BatteryIndicatorProps) {
  const color = percent > 50 ? '#66bb6a' : percent > 20 ? '#ffa726' : '#ef5350'
  const Icon = percent > 60 ? BatteryFull : percent > 20 ? BatteryMedium : BatteryLow

  return (
    <div className="flex items-center gap-1.5 text-xs">
      <Icon size={16} style={{ color }} />
      <span style={{ color }}>{voltage.toFixed(1)}V</span>
      <span className="text-muted-foreground">{percent.toFixed(0)}%</span>
    </div>
  )
}
