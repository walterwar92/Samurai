import { useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { SlamMapData } from '@/types/robot'

// Color lookup for detected objects
const COLOUR_HEX: Record<string, string> = {
  red: '#ef5350',
  orange: '#ff9800',
  yellow: '#ffee58',
  green: '#66bb6a',
  blue: '#42a5f5',
  white: '#e0e0e0',
  black: '#424242',
  unknown: '#9e9e9e',
}

interface SlamMap3DProps {
  slamMap: SlamMapData | null
}

export function SlamMap3D({ slamMap }: SlamMap3DProps) {
  const obstaclesRef = useRef<THREE.InstancedMesh>(null)
  const objectsRef = useRef<THREE.Group>(null)

  // Memoize obstacle positions into instanced mesh
  const obstacleData = useMemo(() => {
    if (!slamMap?.obstacles?.length) return null

    const obstacles = slamMap.obstacles
    const count = Math.min(obstacles.length, 5000) // limit for performance
    const matrices = new Float32Array(count * 16)
    const colors = new Float32Array(count * 3)
    const matrix = new THREE.Matrix4()

    for (let i = 0; i < count; i++) {
      const [wx, wy] = obstacles[i]
      // In Three.js: X=world_x, Y=height, Z=-world_y
      matrix.makeTranslation(wx, 0.015, -wy)
      matrix.toArray(matrices, i * 16)
      // Dark gray obstacles
      colors[i * 3] = 0.35
      colors[i * 3 + 1] = 0.15
      colors[i * 3 + 2] = 0.15
    }

    return { count, matrices, colors }
  }, [slamMap?.obstacles])

  // Update instanced mesh when data changes
  useFrame(() => {
    if (!obstaclesRef.current || !obstacleData) return

    const mesh = obstaclesRef.current
    const matrix = new THREE.Matrix4()

    for (let i = 0; i < obstacleData.count; i++) {
      matrix.fromArray(obstacleData.matrices, i * 16)
      mesh.setMatrixAt(i, matrix)
      mesh.setColorAt(
        i,
        new THREE.Color(
          obstacleData.colors[i * 3],
          obstacleData.colors[i * 3 + 1],
          obstacleData.colors[i * 3 + 2]
        )
      )
    }
    mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
  })

  // Detected objects (spheres with color)
  const detectedObjects = useMemo(() => {
    if (!slamMap?.detected_objects?.length) return []
    return slamMap.detected_objects.map((obj) => ({
      key: obj.id,
      x: obj.x,
      y: obj.y,
      colour: obj.colour,
      label: `${obj.colour} ${obj.class}`,
      conf: obj.conf,
    }))
  }, [slamMap?.detected_objects])

  const obstacleCount = obstacleData?.count ?? 0

  return (
    <group>
      {/* Obstacles as instanced mesh (efficient for thousands of small cubes) */}
      {obstacleCount > 0 && (
        <instancedMesh
          ref={obstaclesRef}
          args={[undefined, undefined, obstacleCount]}
          frustumCulled={false}
        >
          <boxGeometry args={[0.05, 0.03, 0.05]} />
          <meshStandardMaterial
            color="#8b2020"
            transparent
            opacity={0.7}
            roughness={0.8}
          />
        </instancedMesh>
      )}

      {/* Detected objects as colored spheres */}
      <group ref={objectsRef}>
        {detectedObjects.map((obj) => (
          <group key={obj.key} position={[obj.x, 0.04, -obj.y]}>
            {/* Object sphere */}
            <mesh>
              <sphereGeometry args={[0.025, 12, 12]} />
              <meshStandardMaterial
                color={COLOUR_HEX[obj.colour] ?? COLOUR_HEX.unknown}
                emissive={COLOUR_HEX[obj.colour] ?? COLOUR_HEX.unknown}
                emissiveIntensity={0.4}
                roughness={0.3}
              />
            </mesh>
            {/* Vertical marker line */}
            <mesh position={[0, 0.04, 0]}>
              <cylinderGeometry args={[0.003, 0.003, 0.08, 4]} />
              <meshStandardMaterial
                color={COLOUR_HEX[obj.colour] ?? COLOUR_HEX.unknown}
                transparent
                opacity={0.6}
              />
            </mesh>
          </group>
        ))}
      </group>

      {/* Home marker (origin crosshair) */}
      <group position={[0, 0.001, 0]}>
        <mesh rotation={[-Math.PI / 2, 0, 0]}>
          <ringGeometry args={[0.03, 0.04, 16]} />
          <meshStandardMaterial
            color="#fbbf24"
            emissive="#fbbf24"
            emissiveIntensity={0.5}
            transparent
            opacity={0.8}
            side={THREE.DoubleSide}
          />
        </mesh>
      </group>
    </group>
  )
}
