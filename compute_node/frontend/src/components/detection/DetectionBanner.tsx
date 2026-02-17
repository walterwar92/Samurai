import { COLOUR_CSS, COLOUR_RU } from '@/lib/constants'
import type { Detection } from '@/types/robot'

interface DetectionBannerProps {
  detection: Detection | null
}

export function DetectionBanner({ detection }: DetectionBannerProps) {
  if (!detection || !detection.colour) return null

  return (
    <div className="flex items-center gap-3 px-4 py-2 rounded-lg bg-secondary/50 border border-border text-sm">
      <div
        className="w-3 h-3 rounded-full shrink-0"
        style={{ backgroundColor: COLOUR_CSS[detection.colour] }}
      />
      <span>
        Обнаружен: <strong>{COLOUR_RU[detection.colour] || detection.colour}</strong> мяч
        — {detection.distance?.toFixed(2)} м
        <span className="text-muted-foreground ml-1">
          (уверенность: {(detection.conf * 100).toFixed(0)}%)
        </span>
      </span>
    </div>
  )
}
