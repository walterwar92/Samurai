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
  /**
   * Если true — позиция и поворот выставляются напрямую, без lerp-сглаживания
   * и без dead-zone. Полезно для воспроизведения готовой телеметрии, где
   * сглаживание добавляет лишнюю задержку. По умолчанию false (сохраняется
   * исходное поведение для real-time робота на странице /3d).
   */
  noSmooth?: boolean
}

/**
 * Базовое смещение yaw, выравнивающее «нос» GLB-модели Samurai.glb с
 * направлением робота yaw=0 → +X в world-координатах сцены. Подобрано
 * вручную под текущий экспорт GLB; если модель пересобрана и нос смотрит
 * в другую сторону — крути это число на ±π/2.
 */
const YAW_MOUNT_OFFSET = 0  // нос Samurai.glb смотрит по +X

const DEG2RAD = Math.PI / 180

// Lerp 0.3 — единый коэф. без специального «stationary slow mode».
// Раньше при stationary=true коэф. падал до 0.08 — это давало заметное
// «опаздывание» модели при старте движения после остановки. Дедзоны
// 1 мм / ~0.06° сами по себе глушат сенсорный шум.
const POS_LERP = 0.3
const ROT_LERP = 0.3

const POS_DEADZONE = 0.001   // 1 mm
const ROT_DEADZONE = 0.001   // ~0.06°

export function RobotModel({ yaw, pitch, roll, posX, posY, stationary = false, noSmooth = false }: RobotModelProps) {
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
  const propsRef = useRef({ yaw, pitch, roll, posX, posY, stationary, noSmooth })
  propsRef.current = { yaw, pitch, roll, posX, posY, stationary, noSmooth }

  // Smoothed position/rotation to avoid jitter from sensor noise
  const smoothPos = useRef(new THREE.Vector3(posX, 0.05, -posY))
  const smoothRot = useRef(new THREE.Euler(
    pitch * DEG2RAD, -yaw * DEG2RAD + YAW_MOUNT_OFFSET, roll * DEG2RAD, 'YXZ'
  ))
  useFrame(() => {
    if (!groupRef.current) return

    // Read latest props from ref (not closure) to avoid stale values
    const p = propsRef.current

    // Direct mode for replay scenarios (no lerp, no deadzone)
    if (p.noSmooth) {
      smoothPos.current.set(p.posX, 0.05, -p.posY)
      smoothRot.current.set(
        p.pitch * DEG2RAD,
        -p.yaw * DEG2RAD + YAW_MOUNT_OFFSET,
        p.roll * DEG2RAD,
        'YXZ',
      )
      groupRef.current.position.copy(smoothPos.current)
      groupRef.current.rotation.set(
        smoothRot.current.x,
        smoothRot.current.y,
        smoothRot.current.z,
        'YXZ',
      )
      return
    }

    // existing smoothing path (unchanged below this line)
    const targetX = p.posX
    const targetZ = -p.posY
    const targetPitch = p.pitch * DEG2RAD
    const targetYaw = -p.yaw * DEG2RAD + YAW_MOUNT_OFFSET  // align model nose with robot +X forward
    const targetRoll = p.roll * DEG2RAD

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
