import { useRef, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { MapToolbar } from './MapToolbar'
import { useMapCanvas } from '@/hooks/useMapCanvas'
import { useZoneDrawing } from '@/hooks/useZoneDrawing'
import type { RobotState } from '@/types/robot'

interface MapCanvasProps {
  state: RobotState | null
}

export function MapCanvas({ state }: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)

  const { transformRef } = useMapCanvas(canvasRef, {
    mapInfo: state?.map_info ?? null,
    pose: state?.pose ?? null,
    scanPoints: state?.scan_points ?? [],
    plannedPath: state?.planned_path ?? [],
    zones: state?.zones ?? [],
    yaw: state?.pose?.yaw ?? 0,
    drawPreview: null, // Will be set from zone drawing
  })

  const {
    mode,
    setMode,
    drawPreview,
    handleMouseDown,
    handleMouseMove,
    handleMouseUp,
  } = useZoneDrawing(canvasRef, transformRef)

  // Re-draw with zone preview
  useMapCanvas(canvasRef, {
    mapInfo: state?.map_info ?? null,
    pose: state?.pose ?? null,
    scanPoints: state?.scan_points ?? [],
    plannedPath: state?.planned_path ?? [],
    zones: state?.zones ?? [],
    yaw: state?.pose?.yaw ?? 0,
    drawPreview,
  })

  // Resize canvas to container
  useEffect(() => {
    const resize = () => {
      const container = containerRef.current
      const canvas = canvasRef.current
      if (!container || !canvas) return
      canvas.width = container.clientWidth
      canvas.height = container.clientWidth * 0.75
    }
    resize()
    window.addEventListener('resize', resize)
    return () => window.removeEventListener('resize', resize)
  }, [])

  return (
    <Card>
      <CardHeader className="py-2 px-3 flex-row items-center justify-between">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Карта
        </CardTitle>
        <div className="flex items-center gap-2">
          <MapToolbar mode={mode} setMode={setMode} />
          <span className="text-[10px] text-muted-foreground">
            Зон: {state?.zones?.length ?? 0}
          </span>
        </div>
      </CardHeader>
      <CardContent className="p-0">
        <div ref={containerRef} className="relative w-full bg-[#1e2130]">
          <canvas
            ref={canvasRef}
            className="w-full block"
            style={{ cursor: mode === 'draw' ? 'crosshair' : mode === 'delete' ? 'pointer' : 'default' }}
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
          />
        </div>
      </CardContent>
    </Card>
  )
}
