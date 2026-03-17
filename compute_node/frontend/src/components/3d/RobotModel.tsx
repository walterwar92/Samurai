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

export function RobotModel({ yaw, pitch, roll, posX, posY }: RobotModelProps) {
  const groupRef = useRef<THREE.Group>(null)

  useFrame(() => {
    if (!groupRef.current) return
    groupRef.current.position.set(posX, 0.05, -posY)
    groupRef.current.rotation.set(
      pitch * DEG2RAD,
      -yaw * DEG2RAD,
      roll * DEG2RAD,
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
        <meshStandardMaterial color="#334155" metalness={0.6} roughness={0.4} />
      </mesh>

      {/* Top plate */}
      <mesh position={[0, bodyH / 2 + 0.005, 0]} castShadow>
        <boxGeometry args={[bodyW * 0.9, 0.01, bodyD * 0.8]} />
        <meshStandardMaterial color="#475569" metalness={0.5} roughness={0.3} />
      </mesh>

      {/* Direction arrow (front indicator) */}
      <mesh position={[0, bodyH / 2 + 0.015, -bodyD / 2 + 0.02]} rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.015, 0.03, 4]} />
        <meshStandardMaterial color="#22d3ee" emissive="#22d3ee" emissiveIntensity={0.5} />
      </mesh>

      {/* Left track */}
      <mesh position={[-(bodyW / 2 + trackW / 2), -bodyH / 2 + trackH / 2, 0]} castShadow>
        <boxGeometry args={[trackW, trackH, trackD]} />
        <meshStandardMaterial color="#1e293b" roughness={0.8} />
      </mesh>

      {/* Right track */}
      <mesh position={[(bodyW / 2 + trackW / 2), -bodyH / 2 + trackH / 2, 0]} castShadow>
        <boxGeometry args={[trackW, trackH, trackD]} />
        <meshStandardMaterial color="#1e293b" roughness={0.8} />
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
            <meshStandardMaterial color="#0f172a" />
          </mesh>
        ))
      )}

      {/* IMU sensor (small box on top) */}
      <mesh position={[0, bodyH / 2 + 0.02, 0.01]}>
        <boxGeometry args={[0.02, 0.01, 0.02]} />
        <meshStandardMaterial color="#16a34a" emissive="#16a34a" emissiveIntensity={0.3} />
      </mesh>
    </group>
  )
}
