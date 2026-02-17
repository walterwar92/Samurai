import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import { cn } from '@/lib/utils'
import type { ZoneMode } from '@/hooks/useZoneDrawing'

interface MapToolbarProps {
  mode: ZoneMode
  setMode: (mode: ZoneMode) => void
}

export function MapToolbar({ mode, setMode }: MapToolbarProps) {
  return (
    <div className="flex gap-1">
      <Button
        variant="outline"
        size="sm"
        className={cn(
          'text-[10px] h-6 px-2',
          mode === 'draw' && 'bg-destructive/20 border-destructive text-destructive'
        )}
        onClick={() => setMode(mode === 'draw' ? 'none' : 'draw')}
      >
        + Зона
      </Button>
      <Button
        variant="outline"
        size="sm"
        className={cn(
          'text-[10px] h-6 px-2',
          mode === 'delete' && 'bg-destructive/20 border-destructive text-destructive'
        )}
        onClick={() => setMode(mode === 'delete' ? 'none' : 'delete')}
      >
        - Зона
      </Button>
      <Button
        variant="outline"
        size="sm"
        className="text-[10px] h-6 px-2"
        onClick={() => api.clearZones()}
      >
        Очистить
      </Button>
    </div>
  )
}
