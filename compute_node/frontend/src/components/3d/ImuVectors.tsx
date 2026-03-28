import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'

interface ImuVectorsProps {
  posX: number
  posY: number
  yaw: number
  accel: [number, number, number]  // [ax, ay, az] m/s²
  gyro: [number, number, number]   // [gx, gy, gz] rad/s
  showAccel?: boolean
  showGyro?: boolean
}

const DEG2RAD = Math.PI / 180
const ACCEL_SCALE = 0.02   // visual scale for accel vector
const GYRO_SCALE = 0.05    // visual scale for gyro rings

export function ImuVectors({
  posX, posY, yaw,
  accel, gyro,
  showAccel = true, showGyro = true,
}: ImuVectorsProps) {
  const accelRef = useRef<THREE.ArrowHelper>(null)
  const gyroGroupRef = useRef<THREE.Group>(null)

  // Use ref to always have the latest props in useFrame (prevents stale closure)
  const propsRef = useRef({ posX, posY, accel, gyro })
  propsRef.current = { posX, posY, accel, gyro }

  useFrame(() => {
    const p = propsRef.current
    const origin = new THREE.Vector3(p.posX, 0.12, -p.posY)

    // Accel vector arrow
    if (accelRef.current && showAccel) {
      const [ax, ay, az] = p.accel
      const dir = new THREE.Vector3(ax, az, -ay).normalize()
      const len = Math.sqrt(ax * ax + ay * ay + az * az) * ACCEL_SCALE
      accelRef.current.position.copy(origin)
      accelRef.current.setDirection(dir)
      accelRef.current.setLength(Math.min(len, 0.4), 0.03, 0.015)
      accelRef.current.visible = true
    }

    // Gyro rings
    if (gyroGroupRef.current && showGyro) {
      gyroGroupRef.current.position.copy(origin)
      const children = gyroGroupRef.current.children as THREE.Mesh[]
      const [gx, gy, gz] = p.gyro

      // X-axis ring (pitch rate) — red
      const scaleX = Math.min(Math.abs(gx) * GYRO_SCALE + 0.01, 0.15)
      children[0].scale.set(scaleX, scaleX, scaleX)
      children[0].material = children[0].material as THREE.MeshBasicMaterial
      ;(children[0].material as THREE.MeshBasicMaterial).opacity = Math.min(Math.abs(gx) * 0.5, 1)

      // Y-axis ring (yaw rate) — green
      const scaleY = Math.min(Math.abs(gy) * GYRO_SCALE + 0.01, 0.15)
      children[1].scale.set(scaleY, scaleY, scaleY)
      ;(children[1].material as THREE.MeshBasicMaterial).opacity = Math.min(Math.abs(gy) * 0.5, 1)

      // Z-axis ring (roll rate) — blue
      const scaleZ = Math.min(Math.abs(gz) * GYRO_SCALE + 0.01, 0.15)
      children[2].scale.set(scaleZ, scaleZ, scaleZ)
      ;(children[2].material as THREE.MeshBasicMaterial).opacity = Math.min(Math.abs(gz) * 0.5, 1)
    }
  })

  return (
    <>
      {showAccel && (
        <arrowHelper
          ref={accelRef}
          args={[
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(0, 0, 0),
            0.2,
            0xef4444,
            0.03,
            0.015,
          ]}
        />
      )}

      {showGyro && (
        <group ref={gyroGroupRef}>
          {/* X-axis ring (pitch) — red */}
          <mesh rotation={[0, 0, Math.PI / 2]}>
            <torusGeometry args={[1, 0.15, 8, 24]} />
            <meshBasicMaterial color="#ef4444" transparent opacity={0.3} wireframe />
          </mesh>
          {/* Y-axis ring (yaw) — green */}
          <mesh rotation={[0, 0, 0]}>
            <torusGeometry args={[1, 0.15, 8, 24]} />
            <meshBasicMaterial color="#22c55e" transparent opacity={0.3} wireframe />
          </mesh>
          {/* Z-axis ring (roll) — blue */}
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <torusGeometry args={[1, 0.15, 8, 24]} />
            <meshBasicMaterial color="#3b82f6" transparent opacity={0.3} wireframe />
          </mesh>
        </group>
      )}
    </>
  )
}
