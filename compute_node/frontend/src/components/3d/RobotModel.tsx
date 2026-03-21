import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'

interface RobotModelProps {
  yaw: number    // degrees
  pitch: number  // degrees
  roll: number   // degrees
  posX: number   // meters
  posY: number   // meters
  stationary?: boolean
}

const DEG2RAD = Math.PI / 180

// Lerp factor: 0.15 = smooth but responsive (higher = snappier)
const POS_LERP = 0.15
const ROT_LERP = 0.2

// Dead zone: ignore changes smaller than these when stationary
// Prevents phantom movement from sensor noise
const POS_DEADZONE = 0.001   // 1 mm
const ROT_DEADZONE = 0.001   // ~0.06°

export function RobotModel({ yaw, pitch, roll, posX, posY, stationary = false }: RobotModelProps) {
  const groupRef = useRef<THREE.Group>(null)
  // Smoothed position/rotation to avoid jitter from sensor noise
  const smoothPos = useRef(new THREE.Vector3(posX, 0.05, -posY))
  const smoothRot = useRef(new THREE.Euler(
    pitch * DEG2RAD, -yaw * DEG2RAD, roll * DEG2RAD, 'YXZ'
  ))
  // Frozen targets — locked when stationary to prevent phantom drift
  const frozenPos = useRef({ x: posX, z: -posY })
  const frozenRot = useRef({ x: pitch * DEG2RAD, y: -yaw * DEG2RAD, z: roll * DEG2RAD })
  const wasStationary = useRef(stationary)

  useFrame(() => {
    if (!groupRef.current) return

    const targetX = posX
    const targetZ = -posY
    const targetPitch = pitch * DEG2RAD
    const targetYaw = -yaw * DEG2RAD
    const targetRoll = roll * DEG2RAD

    if (stationary) {
      // When stationary: freeze targets on entry, snap position
      if (!wasStationary.current) {
        // Just became stationary — freeze current targets
        frozenPos.current = { x: targetX, z: targetZ }
        frozenRot.current = { x: targetPitch, y: targetYaw, z: targetRoll }
      }
      wasStationary.current = true

      // Use frozen targets — ignore sensor noise
      const fx = frozenPos.current.x
      const fz = frozenPos.current.z

      // Quick snap to frozen position (faster lerp to settle)
      smoothPos.current.x += (fx - smoothPos.current.x) * 0.3
      smoothPos.current.z += (fz - smoothPos.current.z) * 0.3
      smoothPos.current.y = 0.05

      smoothRot.current.x += (frozenRot.current.x - smoothRot.current.x) * 0.3
      smoothRot.current.y += (frozenRot.current.y - smoothRot.current.y) * 0.3
      smoothRot.current.z += (frozenRot.current.z - smoothRot.current.z) * 0.3
    } else {
      wasStationary.current = false

      // When moving: normal lerp with dead zone
      const dxPos = targetX - smoothPos.current.x
      const dzPos = targetZ - smoothPos.current.z

      if (Math.abs(dxPos) > POS_DEADZONE || Math.abs(dzPos) > POS_DEADZONE) {
        smoothPos.current.x += dxPos * POS_LERP
        smoothPos.current.z += dzPos * POS_LERP
      }
      smoothPos.current.y = 0.05

      const dxRot = targetPitch - smoothRot.current.x
      const dyRot = targetYaw - smoothRot.current.y
      const dzRot = targetRoll - smoothRot.current.z

      if (Math.abs(dxRot) > ROT_DEADZONE || Math.abs(dyRot) > ROT_DEADZONE || Math.abs(dzRot) > ROT_DEADZONE) {
        smoothRot.current.x += dxRot * ROT_LERP
        smoothRot.current.y += dyRot * ROT_LERP
        smoothRot.current.z += dzRot * ROT_LERP
      }
    }

    groupRef.current.position.copy(smoothPos.current)
    groupRef.current.rotation.set(
      smoothRot.current.x,
      smoothRot.current.y,
      smoothRot.current.z,
      'YXZ'
    )
  })

  const bodyW = 0.17, bodyH = 0.08, bodyD = 0.12
  const trackW = 0.02, trackH = 0.04, trackD = 0.14
  const wheelR = 0.02, wheelH = 0.02

  return (
    <group ref={groupRef}>
      {/* Body */}
      <mesh castShadow>
        <boxGeometry args={[bodyW, bodyH, bodyD]} />
        <meshStandardMaterial color="#4a5568" metalness={0.5} roughness={0.35} />
      </mesh>

      {/* Top plate */}
      <mesh position={[0, bodyH / 2 + 0.005, 0]} castShadow>
        <boxGeometry args={[bodyW * 0.9, 0.01, bodyD * 0.8]} />
        <meshStandardMaterial color="#5a6a7e" metalness={0.4} roughness={0.3} />
      </mesh>

      {/* Direction arrow (front indicator) */}
      <mesh position={[0, bodyH / 2 + 0.015, -bodyD / 2 + 0.02]} rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.015, 0.03, 4]} />
        <meshStandardMaterial color="#22d3ee" emissive="#22d3ee" emissiveIntensity={0.8} />
      </mesh>

      {/* Left track */}
      <mesh position={[-(bodyW / 2 + trackW / 2), -bodyH / 2 + trackH / 2, 0]} castShadow>
        <boxGeometry args={[trackW, trackH, trackD]} />
        <meshStandardMaterial color="#2d3a4d" roughness={0.7} />
      </mesh>

      {/* Right track */}
      <mesh position={[(bodyW / 2 + trackW / 2), -bodyH / 2 + trackH / 2, 0]} castShadow>
        <boxGeometry args={[trackW, trackH, trackD]} />
        <meshStandardMaterial color="#2d3a4d" roughness={0.7} />
      </mesh>

      {/* Wheels (decorative) */}
      {[-1, 1].map(side =>
        [-1, 0, 1].map(pos => (
          <mesh
            key={`${side}-${pos}`}
            position={[
              side * (bodyW / 2 + trackW + wheelH / 2),
              -bodyH / 2 + trackH / 2,
              pos * (trackD / 2 - wheelR)
            ]}
            rotation={[0, 0, Math.PI / 2]}
          >
            <cylinderGeometry args={[wheelR, wheelR, wheelH, 8]} />
            <meshStandardMaterial color="#1e2a3d" />
          </mesh>
        ))
      )}

      {/* IMU sensor (small box on top) */}
      <mesh position={[0, bodyH / 2 + 0.02, 0.01]}>
        <boxGeometry args={[0.02, 0.01, 0.02]} />
        <meshStandardMaterial color="#16a34a" emissive="#16a34a" emissiveIntensity={0.6} />
      </mesh>
    </group>
  )
}
