import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import {
  AlertIcon,
  GridIcon,
  LayersIcon,
  PlusIcon,
  MinusIcon,
} from '@/components/icons'
import type { ZoneMode } from '@/hooks/useZoneDrawing'

export interface MapToolbarProps {
  /** Legacy: режим рисования/удаления запрещённых зон. */
  mode: ZoneMode
  setMode: (mode: ZoneMode) => void
  /** Новое (Q15D): toggle для grid overlay. */
  showGrid?: boolean
  onToggleGrid?: () => void
  /** Новое (Q15D): toggle для coverage heatmap. */
  showCoverage?: boolean
  onToggleCoverage?: () => void
  /** Новое (Q15D): zoom controls. */
  zoom?: number
  onZoomIn?: () => void
  onZoomOut?: () => void
}

/**
 * MapToolbar — действия над картой:
 *   • Draw / Delete forbidden zone (legacy, mode-based)
 *   • Clear all zones (api.clearZones)
 *   • Grid overlay toggle (новое)
 *   • Coverage heatmap toggle (новое)
 *   • Zoom in/out (новое, опционально)
 */
export function MapToolbar({
  mode,
  setMode,
  showGrid = false,
  onToggleGrid,
  showCoverage = false,
  onToggleCoverage,
  zoom,
  onZoomIn,
  onZoomOut,
}: MapToolbarProps) {
  return (
    <div className="flex flex-wrap items-center gap-1.5">
      <Button
        size="sm"
        variant={mode === 'draw' ? 'default' : 'secondary'}
        onClick={() => setMode(mode === 'draw' ? 'none' : 'draw')}
      >
        <PlusIcon className="h-3.5 w-3.5" />Зона
      </Button>
      <Button
        size="sm"
        variant={mode === 'delete' ? 'destructive' : 'secondary'}
        onClick={() => setMode(mode === 'delete' ? 'none' : 'delete')}
      >
        <MinusIcon className="h-3.5 w-3.5" />Зона
      </Button>
      <Button size="sm" variant="ghost" onClick={() => api.clearZones()}>
        <AlertIcon className="h-3.5 w-3.5" />Очистить
      </Button>

      {onToggleGrid && (
        <Button
          size="sm"
          variant={showGrid ? 'default' : 'secondary'}
          onClick={onToggleGrid}
        >
          <GridIcon className="h-3.5 w-3.5" />Grid
        </Button>
      )}
      {onToggleCoverage && (
        <Button
          size="sm"
          variant={showCoverage ? 'default' : 'secondary'}
          onClick={onToggleCoverage}
        >
          <LayersIcon className="h-3.5 w-3.5" />Coverage
        </Button>
      )}

      {(onZoomIn || onZoomOut) && (
        <span className="ml-auto inline-flex items-center gap-1.5">
          {onZoomIn && (
            <Button size="sm" variant="ghost" onClick={onZoomIn} aria-label="Zoom in">
              <PlusIcon className="h-3.5 w-3.5" />
            </Button>
          )}
          {zoom !== undefined && (
            <span className="font-mono text-micro text-foreground-faint tabular-nums">
              {zoom}%
            </span>
          )}
          {onZoomOut && (
            <Button size="sm" variant="ghost" onClick={onZoomOut} aria-label="Zoom out">
              <MinusIcon className="h-3.5 w-3.5" />
            </Button>
          )}
        </span>
      )}
    </div>
  )
}
