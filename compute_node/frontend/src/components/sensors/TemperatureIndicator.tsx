import { Thermometer } from 'lucide-react'

interface TemperatureIndicatorProps {
  temp: number
}

export function TemperatureIndicator({ temp }: TemperatureIndicatorProps) {
  const color = temp < 60 ? '#66bb6a' : temp < 75 ? '#ffa726' : '#ef5350'

  return (
    <div className="flex items-center gap-1.5 text-xs">
      <Thermometer size={14} style={{ color }} />
      <span style={{ color }}>{temp.toFixed(1)}°C</span>
    </div>
  )
}
