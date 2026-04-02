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

  // ── Use ref to always have the latest props in useFrame ──
  // This prevents stale closure issues with React Three Fiber's reconciler
  const propsRef = useRef({ yaw, pitch, roll, posX, posY, stationary })
  propsRef.current = { yaw, pitch, roll, posX, posY, stationary }

  // Smoothed position/rotation to avoid jitter from sensor noise
  // +PI/2 offset: model front is -Z, but robot yaw=0 faces +X in world coords
  const smoothPos = useRef(new THREE.Vector3(posX, 0.05, -posY))
  const smoothRot = useRef(new THREE.Euler(
    pitch * DEG2RAD, -yaw * DEG2RAD + Math.PI / 2, roll * DEG2RAD, 'YXZ'
  ))
  useFrame(() => {
    if (!groupRef.current) return

    // Read latest props from ref (not closure) to avoid stale values
    const p = propsRef.current
    const targetX = p.posX
    const targetZ = -p.posY
    const targetPitch = p.pitch * DEG2RAD
    const targetYaw = -p.yaw * DEG2RAD + Math.PI / 2  // +90°: align model -Z front with robot +X forward
    const targetRoll = p.roll * DEG2RAD
    const isStationary = p.stationary

    // Always lerp towards target — use slower lerp when stationary to filter noise
    const lerpPos = isStationary ? 0.08 : POS_LERP
    const lerpRot = isStationary ? 0.1 : ROT_LERP
    const deadPos = isStationary ? POS_DEADZONE : POS_DEADZONE
    const deadRot = isStationary ? ROT_DEADZONE : ROT_DEADZONE

    const dxPos = targetX - smoothPos.current.x
    const dzPos = targetZ - smoothPos.current.z

    if (Math.abs(dxPos) > deadPos || Math.abs(dzPos) > deadPos) {
      smoothPos.current.x += dxPos * lerpPos
      smoothPos.current.z += dzPos * lerpPos
    }
    smoothPos.current.y = 0.05

    const dxRot = targetPitch - smoothRot.current.x
    const dyRot = targetYaw - smoothRot.current.y
    const dzRot = targetRoll - smoothRot.current.z

    if (Math.abs(dxRot) > deadRot || Math.abs(dyRot) > deadRot || Math.abs(dzRot) > deadRot) {
      smoothRot.current.x += dxRot * lerpRot
      smoothRot.current.y += dyRot * lerpRot
      smoothRot.current.z += dzRot * lerpRot
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
