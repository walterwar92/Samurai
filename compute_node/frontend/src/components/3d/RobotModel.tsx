import { useRef, useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import { useGLTF } from '@react-three/drei'
import * as THREE from 'three'

const MODEL_URL = '/models/Samurai.glb'

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
  const { scene } = useGLTF(MODEL_URL)

  // Clone so multiple instances don't share material state, and enable shadows
  const modelScene = useMemo(() => {
    const cloned = scene.clone(true)
    cloned.traverse((obj) => {
      const mesh = obj as THREE.Mesh
      if (mesh.isMesh) {
        mesh.castShadow = true
        mesh.receiveShadow = true
      }
    })
    return cloned
  }, [scene])

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

  return (
    <group ref={groupRef}>
      <primitive object={modelScene} scale={0.001} />
    </group>
  )
}

useGLTF.preload(MODEL_URL)
