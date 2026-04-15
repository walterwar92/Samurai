import { Button } from '@/components/ui/button'
import type { Platform } from '@/types/hardware'

const PLATFORMS: { id: Platform; label: string }[] = [
  { id: 'raspberry_pi_pca9685', label: 'RPi + PCA9685' },
  { id: 'arduino', label: 'Arduino' },
  { id: 'esp32', label: 'ESP32' },
  { id: 'custom', label: 'Custom' },
]

interface PlatformSelectorProps {
  value: Platform
  onChange: (p: Platform) => void
}

export function PlatformSelector({ value, onChange }: PlatformSelectorProps) {
  return (
    <div className="flex gap-1.5">
      {PLATFORMS.map((p) => (
        <Button
          key={p.id}
          size="sm"
          variant={value === p.id ? 'default' : 'outline'}
          className="text-xs h-7 px-3 flex-1"
          onClick={() => onChange(p.id)}
        >
          {p.label}
        </Button>
      ))}
    </div>
  )
}
