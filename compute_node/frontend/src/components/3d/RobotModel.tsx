import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'

interface RobotModelProps {
  yaw: number    // degrees
  pitch: number  // degrees
  roll: number   // degrees
  posX: number   // meters
  posY: number   // meters
}

const DEG2RAD = Math.PI / 180

// Lerp factor: 0.15 = smooth but responsive (higher = snappier)
const POS_LERP = 0.15
const ROT_LERP = 0.2

export function RobotModel({ yaw, pitch, roll, posX, posY }: RobotModelProps) {
  const groupRef = useRef<THREE.Group>(null)
  // Smoothed position/rotation to avoid jitter from sensor noise
  const smoothPos = useRef(new THREE.Vector3(posX, 0.05, -posY))
  const smoothRot = useRef(new THREE.Euler(
    pitch * DEG2RAD, -yaw * DEG2RAD, roll * DEG2RAD, 'YXZ'
  ))

  useFrame(() => {
    if (!groupRef.current) return

    const targetX = posX
    const targetZ = -posY

    // Lerp position — smooths out micro-jitter from sensor noise
    smoothPos.current.x += (targetX - smoothPos.current.x) * POS_LERP
    smoothPos.current.z += (targetZ - smoothPos.current.z) * POS_LERP
    smoothPos.current.y = 0.05

    groupRef.current.position.copy(smoothPos.current)

    // Lerp rotation
    const targetPitch = pitch * DEG2RAD
    const targetYaw = -yaw * DEG2RAD
    const targetRoll = roll * DEG2RAD

    smoothRot.current.x += (targetPitch - smoothRot.current.x) * ROT_LERP
    smoothRot.current.y += (targetYaw - smoothRot.current.y) * ROT_LERP
    smoothRot.current.z += (targetRoll - smoothRot.current.z) * ROT_LERP

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
