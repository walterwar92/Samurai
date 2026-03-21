import { useRef, useEffect } from 'react'
import * as THREE from 'three'

interface PathTrailProps {
  posX: number
  posY: number
  stationary?: boolean
  maxPoints?: number
  clearSignal?: number
}

export function PathTrail({ posX, posY, stationary = false, maxPoints = 500, clearSignal = 0 }: PathTrailProps) {
  const lineRef = useRef<THREE.Line>(null)
  const pointsRef = useRef<THREE.Vector3[]>([])
  const lastClear = useRef(0)

  useEffect(() => {
    if (clearSignal !== lastClear.current) {
      pointsRef.current = []
      lastClear.current = clearSignal
      if (lineRef.current) {
        lineRef.current.geometry.setFromPoints([])
      }
    }
  }, [clearSignal])

  useEffect(() => {
    // Don't add trail points when robot is stationary — prevents phantom trail
    if (stationary) return

    const pts = pointsRef.current
    const newPoint = new THREE.Vector3(posX, 0.005, -posY)

    // Minimum distance 1cm (was 5mm) to avoid dense clusters from noise
    if (pts.length === 0 || newPoint.distanceTo(pts[pts.length - 1]) > 0.01) {
      pts.push(newPoint)
      if (pts.length > maxPoints) pts.shift()

      if (lineRef.current) {
        lineRef.current.geometry.setFromPoints(pts)
      }
    }
  }, [posX, posY, stationary, maxPoints])

  return (
    <line ref={lineRef as any}>
      <bufferGeometry />
      <lineBasicMaterial color="#f59e0b" linewidth={2} transparent opacity={0.8} />
    </line>
  )
}
