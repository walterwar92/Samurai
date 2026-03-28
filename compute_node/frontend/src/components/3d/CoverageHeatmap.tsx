import { useMemo } from 'react'
import * as THREE from 'three'

interface CoverageHeatmapProps {
  /** Trail points [[x,y], ...] from slam_map */
  trail: [number, number][] | null
  /** Cell size in meters */
  cellSize?: number
}

/**
 * Renders a 2D coverage heatmap on the ground plane.
 * Each visited cell gets colored by visit count (green→yellow→red).
 */
export function CoverageHeatmap({ trail, cellSize = 0.08 }: CoverageHeatmapProps) {
  const heatmapData = useMemo(() => {
    if (!trail?.length) return null

    // Count visits per grid cell
    const cells = new Map<string, number>()
    for (const [x, y] of trail) {
      const ci = Math.floor(x / cellSize)
      const cj = Math.floor(y / cellSize)
      const key = `${ci},${cj}`
      cells.set(key, (cells.get(key) ?? 0) + 1)
    }

    if (cells.size === 0) return null

    const maxCount = Math.max(...cells.values())
    const count = Math.min(cells.size, 3000) // performance cap

    const matrices = new Float32Array(count * 16)
    const colors = new Float32Array(count * 3)
    const matrix = new THREE.Matrix4()
    const color = new THREE.Color()

    let i = 0
    for (const [key, visits] of cells) {
      if (i >= count) break
      const [ci, cj] = key.split(',').map(Number)
      const wx = (ci + 0.5) * cellSize
      const wy = (cj + 0.5) * cellSize

      matrix.makeTranslation(wx, 0.002, -wy)
      matrix.toArray(matrices, i * 16)

      // Color: green(cold) → yellow → red(hot)
      const t = Math.min(visits / Math.max(maxCount, 1), 1.0)
      if (t < 0.5) {
        color.setRGB(t * 2, 0.8, 0.2 * (1 - t * 2))
      } else {
        color.setRGB(1.0, 1.6 * (1 - t), 0.0)
      }
      colors[i * 3] = color.r
      colors[i * 3 + 1] = color.g
      colors[i * 3 + 2] = color.b
      i++
    }

    return { count: i, matrices, colors }
  }, [trail, cellSize])

  if (!heatmapData || heatmapData.count === 0) return null

  return (
    <instancedMesh
      args={[undefined, undefined, heatmapData.count]}
      frustumCulled={false}
    >
      <planeGeometry args={[cellSize * 0.9, cellSize * 0.9]} />
      <meshStandardMaterial
        transparent
        opacity={0.35}
        side={THREE.DoubleSide}
        depthWrite={false}
      />
      {/* Apply instance matrices and colors via ref */}
      <HeatmapUpdater data={heatmapData} />
    </instancedMesh>
  )
}

/** Helper to update instanced mesh data */
function HeatmapUpdater({ data }: { data: { count: number; matrices: Float32Array; colors: Float32Array } }) {
  const meshRef = useMemo(() => {
    // Create a dummy to trigger the effect via key
    return { key: Math.random() }
  }, [data])

  return (
    <group
      key={meshRef.key}
      ref={(group) => {
        if (!group?.parent) return
        const mesh = group.parent as unknown as THREE.InstancedMesh
        if (!mesh?.isInstancedMesh) return

        const matrix = new THREE.Matrix4()
        const color = new THREE.Color()

        for (let i = 0; i < data.count; i++) {
          matrix.fromArray(data.matrices, i * 16)
          mesh.setMatrixAt(i, matrix)
          color.setRGB(
            data.colors[i * 3],
            data.colors[i * 3 + 1],
            data.colors[i * 3 + 2]
          )
          mesh.setColorAt(i, color)
        }
        mesh.instanceMatrix.needsUpdate = true
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
        mesh.rotation.x = -Math.PI / 2
      }}
    />
  )
}
