import { useEffect, useRef, useCallback } from 'react'
import type { MapInfo, RobotPose, ForbiddenZone } from '@/types/robot'
import type { MapTransform } from '@/lib/mapMath'

interface MapDrawData {
  mapInfo: MapInfo | null
  pose: RobotPose | null
  scanPoints: [number, number][]
  plannedPath: [number, number][]
  zones: ForbiddenZone[]
  yaw: number
  drawPreview: { x1: number; y1: number; x2: number; y2: number } | null
}

export function useMapCanvas(
  canvasRef: React.RefObject<HTMLCanvasElement | null>,
  data: MapDrawData
) {
  const mapImgRef = useRef(new Image())
  const transformRef = useRef<MapTransform | null>(null)

  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const { mapInfo, pose, scanPoints, plannedPath, zones, yaw, drawPreview } = data
    if (!mapInfo) return

    const img = mapImgRef.current
    const cw = canvas.width
    const ch = canvas.height

    ctx.clearRect(0, 0, cw, ch)
    ctx.fillStyle = '#1e2130'
    ctx.fillRect(0, 0, cw, ch)

    if (img.naturalWidth === 0) return

    // Calculate transform to fit map in canvas
    const scaleX = cw / img.naturalWidth
    const scaleY = ch / img.naturalHeight
    const scale = Math.min(scaleX, scaleY)
    const offsetX = (cw - img.naturalWidth * scale) / 2
    const offsetY = (ch - img.naturalHeight * scale) / 2

    transformRef.current = { offsetX, offsetY, scale, info: mapInfo }

    // Draw map image
    ctx.drawImage(img, offsetX, offsetY, img.naturalWidth * scale, img.naturalHeight * scale)

    // Helper: world -> canvas
    const w2c = (wx: number, wy: number) => {
      const mx = (wx - mapInfo.origin_x) / mapInfo.resolution
      const my = mapInfo.height - (wy - mapInfo.origin_y) / mapInfo.resolution
      return { cx: mx * scale + offsetX, cy: my * scale + offsetY }
    }

    // Draw scan points
    if (scanPoints.length > 0 && pose) {
      ctx.fillStyle = 'rgba(100, 180, 255, 0.5)'
      const cosY = Math.cos(yaw)
      const sinY = Math.sin(yaw)
      for (const [lx, ly] of scanPoints) {
        const gx = pose.x + lx * cosY - ly * sinY
        const gy = pose.y + lx * sinY + ly * cosY
        const { cx, cy } = w2c(gx, gy)
        ctx.fillRect(cx - 1, cy - 1, 2, 2)
      }
    }

    // Draw planned path
    if (plannedPath.length > 1) {
      ctx.strokeStyle = '#66bb6a'
      ctx.lineWidth = 2
      ctx.setLineDash([6, 4])
      ctx.beginPath()
      const first = w2c(plannedPath[0][0], plannedPath[0][1])
      ctx.moveTo(first.cx, first.cy)
      for (let i = 1; i < plannedPath.length; i++) {
        const p = w2c(plannedPath[i][0], plannedPath[i][1])
        ctx.lineTo(p.cx, p.cy)
      }
      ctx.stroke()
      ctx.setLineDash([])

      // Waypoint dots
      ctx.fillStyle = '#66bb6a'
      for (const [wx, wy] of plannedPath) {
        const { cx, cy } = w2c(wx, wy)
        ctx.beginPath()
        ctx.arc(cx, cy, 3, 0, Math.PI * 2)
        ctx.fill()
      }

      // Goal marker
      const goal = plannedPath[plannedPath.length - 1]
      const g = w2c(goal[0], goal[1])
      ctx.strokeStyle = '#66bb6a'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.arc(g.cx, g.cy, 8, 0, Math.PI * 2)
      ctx.stroke()
      ctx.beginPath()
      ctx.moveTo(g.cx - 6, g.cy)
      ctx.lineTo(g.cx + 6, g.cy)
      ctx.moveTo(g.cx, g.cy - 6)
      ctx.lineTo(g.cx, g.cy + 6)
      ctx.stroke()
    }

    // Draw zones
    ctx.strokeStyle = 'rgba(239, 83, 80, 0.7)'
    ctx.lineWidth = 2
    ctx.setLineDash([6, 3])
    for (const z of zones) {
      const tl = w2c(z.x1, z.y2)
      const br = w2c(z.x2, z.y1)
      ctx.strokeRect(tl.cx, tl.cy, br.cx - tl.cx, br.cy - tl.cy)
    }
    ctx.setLineDash([])

    // Draw zone preview
    if (drawPreview) {
      ctx.strokeStyle = 'rgba(239, 83, 80, 0.9)'
      ctx.lineWidth = 2
      ctx.setLineDash([4, 4])
      const x = Math.min(drawPreview.x1, drawPreview.x2)
      const y = Math.min(drawPreview.y1, drawPreview.y2)
      const w = Math.abs(drawPreview.x2 - drawPreview.x1)
      const h = Math.abs(drawPreview.y2 - drawPreview.y1)
      ctx.strokeRect(x, y, w, h)
      ctx.setLineDash([])
    }

    // Draw robot
    if (pose) {
      const r = w2c(pose.x, pose.y)
      ctx.fillStyle = '#4fc3f7'
      ctx.beginPath()
      ctx.arc(r.cx, r.cy, 6, 0, Math.PI * 2)
      ctx.fill()

      // Direction arrow
      const arrowLen = 14
      const angle = -yaw // canvas Y is inverted
      ctx.strokeStyle = '#4fc3f7'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(r.cx, r.cy)
      ctx.lineTo(r.cx + Math.cos(angle) * arrowLen, r.cy - Math.sin(angle) * arrowLen)
      ctx.stroke()
    }
  }, [canvasRef, data])

  // Load map image and redraw
  useEffect(() => {
    const img = mapImgRef.current
    img.onload = draw
    img.src = '/map.png?t=' + Date.now()
  }, [data.mapInfo, data.pose, draw])

  return { transformRef }
}
