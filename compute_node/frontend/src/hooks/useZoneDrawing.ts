import { useState, useCallback, type MouseEvent } from 'react'
import { canvasToWorld, type MapTransform } from '@/lib/mapMath'
import { api } from '@/lib/api'

export type ZoneMode = 'none' | 'draw' | 'delete'

export function useZoneDrawing(
  canvasRef: React.RefObject<HTMLCanvasElement | null>,
  transformRef: React.RefObject<MapTransform | null>
) {
  const [mode, setMode] = useState<ZoneMode>('none')
  const [drawStart, setDrawStart] = useState<{ x: number; y: number } | null>(null)
  const [drawCurrent, setDrawCurrent] = useState<{ x: number; y: number } | null>(null)

  const getCanvasPos = useCallback(
    (e: MouseEvent) => {
      const canvas = canvasRef.current
      if (!canvas) return null
      const rect = canvas.getBoundingClientRect()
      const scaleX = canvas.width / rect.width
      const scaleY = canvas.height / rect.height
      return {
        x: (e.clientX - rect.left) * scaleX,
        y: (e.clientY - rect.top) * scaleY,
      }
    },
    [canvasRef]
  )

  const handleMouseDown = useCallback(
    (e: MouseEvent) => {
      if (mode === 'none') return
      const pos = getCanvasPos(e)
      if (!pos) return

      if (mode === 'draw') {
        setDrawStart(pos)
        setDrawCurrent(pos)
      }

      if (mode === 'delete' && transformRef.current) {
        // Find clicked zone and delete
        const world = canvasToWorld(pos.x, pos.y, transformRef.current)
        // We emit a custom event to let the parent handle deletion
        const event = new CustomEvent('zone-click', { detail: world })
        canvasRef.current?.dispatchEvent(event)
      }
    },
    [mode, getCanvasPos, transformRef, canvasRef]
  )

  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (mode !== 'draw' || !drawStart) return
      const pos = getCanvasPos(e)
      if (pos) setDrawCurrent(pos)
    },
    [mode, drawStart, getCanvasPos]
  )

  const handleMouseUp = useCallback(async () => {
    if (mode !== 'draw' || !drawStart || !drawCurrent || !transformRef.current) {
      setDrawStart(null)
      setDrawCurrent(null)
      return
    }

    const w1 = canvasToWorld(drawStart.x, drawStart.y, transformRef.current)
    const w2 = canvasToWorld(drawCurrent.x, drawCurrent.y, transformRef.current)

    const dx = Math.abs(w2.wx - w1.wx)
    const dy = Math.abs(w2.wy - w1.wy)

    if (dx > 0.05 && dy > 0.05) {
      await api.createZone(
        Math.min(w1.wx, w2.wx),
        Math.min(w1.wy, w2.wy),
        Math.max(w1.wx, w2.wx),
        Math.max(w1.wy, w2.wy)
      )
    }

    setDrawStart(null)
    setDrawCurrent(null)
  }, [mode, drawStart, drawCurrent, transformRef])

  const drawPreview =
    drawStart && drawCurrent
      ? { x1: drawStart.x, y1: drawStart.y, x2: drawCurrent.x, y2: drawCurrent.y }
      : null

  return {
    mode,
    setMode,
    drawPreview,
    handleMouseDown,
    handleMouseMove,
    handleMouseUp,
  }
}
