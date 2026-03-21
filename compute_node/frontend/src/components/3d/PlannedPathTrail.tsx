import { useRef, useEffect, useMemo } from 'react'
import * as THREE from 'three'

interface PlannedPathTrailProps {
  /** Recorded path waypoints [[x, y], ...] from path_recorder */
  path: [number, number][] | null
  /** Whether the robot is currently replaying (returning home) */
  replaying?: boolean
}

/**
 * Renders the recorded return-home path as a dashed line in the 3D scene.
 * Cyan when replaying, dim gray otherwise.
 */
export function PlannedPathTrail({ path, replaying = false }: PlannedPathTrailProps) {
  const lineRef = useRef<THREE.Line>(null)

  const points = useMemo(() => {
    if (!path || path.length < 2) return []
    return path.map(([x, y]) => new THREE.Vector3(x, 0.008, -y))
  }, [path])

  useEffect(() => {
    if (!lineRef.current) return
    if (points.length < 2) {
      lineRef.current.geometry.setFromPoints([])
      return
    }
    lineRef.current.geometry.setFromPoints(points)
    lineRef.current.computeLineDistances()  // required for dashed material
    lineRef.current.geometry.computeBoundingSphere()
  }, [points])

  // Home marker position (first waypoint = home)
  const homePos = points.length > 0 ? points[0] : null

  const color = replaying ? '#22d3ee' : '#6b7280'  // cyan when active, gray when idle
  const opacity = replaying ? 0.9 : 0.4

  if (!path || path.length < 2) return null

  return (
    <group>
      {/* Path line */}
      <line ref={lineRef as any}>
        <bufferGeometry />
        <lineDashedMaterial
          color={color}
          linewidth={1}
          dashSize={0.02}
          gapSize={0.01}
          transparent
          opacity={opacity}
        />
      </line>

      {/* Home marker — small ring at the start point */}
      {homePos && (
        <mesh position={homePos} rotation={[-Math.PI / 2, 0, 0]}>
          <ringGeometry args={[0.015, 0.025, 16]} />
          <meshBasicMaterial
            color={replaying ? '#22d3ee' : '#9ca3af'}
            transparent
            opacity={replaying ? 1.0 : 0.5}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}
    </group>
  )
}
