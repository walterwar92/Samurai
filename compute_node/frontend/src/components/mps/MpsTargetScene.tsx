import { Suspense, useEffect, useRef } from 'react'
import { Canvas } from '@react-three/fiber'
import type { ThreeEvent } from '@react-three/fiber'
import { OrbitControls, Grid, Html } from '@react-three/drei'
import * as THREE from 'three'
import { RobotModel } from '@/components/3d/RobotModel'
import {
  groundPointToAngle,
  angleToMarkerPosition,
  groundPointRadius,
  CENTER_DEADZONE_FRACTION,
  MARKER_HEIGHT,
} from '@/lib/targetAngle'

interface MpsTargetSceneProps {
  /** Радиус кольца N (= дистанция сценария), м. */
  distance: number
  /** Текущий выбранный относительный курс φ, рад. */
  pickedAngle: number
  /** Колбэк выбора нового угла (клик по полу вне дедзоны центра). */
  onPick: (angle: number) => void
}

/** Линия от робота (0,0) к маркеру цели. Императивная сборка геометрии —
 *  как в Mps3DScene.AnimatedTrail (проверенный паттерн codebase). */
function TargetLine({ distance, pickedAngle }: { distance: number; pickedAngle: number }) {
  const geometryRef = useRef<THREE.BufferGeometry>(null)
  const [mx, my, mz] = angleToMarkerPosition(pickedAngle, distance)
  useEffect(() => {
    const geom = geometryRef.current
    if (!geom) return
    const positions = new Float32Array([0, MARKER_HEIGHT, 0, mx, my, mz])
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  }, [mx, my, mz])
  return (
    <line>
      <bufferGeometry ref={geometryRef} />
      <lineBasicMaterial color="#22d3ee" linewidth={2} />
    </line>
  )
}

export function MpsTargetScene({ distance, pickedAngle, onPick }: MpsTargetSceneProps) {
  const markerPos = angleToMarkerPosition(pickedAngle, distance)

  function handleGroundClick(e: ThreeEvent<MouseEvent>) {
    e.stopPropagation()
    const { x, z } = e.point
    // Дедзона у центра: слишком близкие к роботу клики игнорируем
    // (там угол скачет от микродвижений мыши).
    if (groundPointRadius(x, z) < distance * CENTER_DEADZONE_FRACTION) return
    onPick(groundPointToAngle(x, z))
  }

  // Камера наклонно-сверху, сзади робота: +X («вперёд») уходит вверх кадра.
  // Стартовые значения — тонкая подстройка под читаемость допустима.
  const camPos: [number, number, number] = [-distance * 0.7, distance * 1.9, 0]

  return (
    <div className="relative w-full h-full">
      <Canvas
        camera={{ position: camPos, fov: 50, near: 0.01, far: 100 }}
        shadows
      >
        <color attach="background" args={['#1a1a2e']} />

        <ambientLight intensity={0.9} />
        <directionalLight
          position={[2, 3, 1]}
          intensity={1.8}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <directionalLight position={[-1, 2, -1]} intensity={0.7} />
        <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]} />

        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.6}
          cellColor="#3f3f5c"
          sectionSize={0.5}
          sectionThickness={1.2}
          sectionColor="#5a5a7a"
          fadeDistance={6}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid
        />

        {/* Кликабельная плоскость пола — мишень для raycast.
            opacity=0 (не visible=false!) — прозрачная, но raycast-able. */}
        <mesh
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, 0, 0]}
          onClick={handleGroundClick}
        >
          <planeGeometry args={[40, 40]} />
          <meshBasicMaterial transparent opacity={0} />
        </mesh>

        {/* Окружность радиуса N — «куда можно выбрать точку». */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
          <ringGeometry args={[distance - 0.015, distance + 0.015, 96]} />
          <meshBasicMaterial color="#22d3ee" side={THREE.DoubleSide} />
        </mesh>

        {/* Маркер старта — серая сфера в центре (под роботом). */}
        <mesh position={[0, 0.02, 0]}>
          <sphereGeometry args={[0.025, 12, 12]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>

        {/* Линия робот → цель. */}
        <TargetLine distance={distance} pickedAngle={pickedAngle} />

        {/* Маркер выбранной цели — оранжевый конус остриём вниз. */}
        <mesh position={markerPos} rotation={[Math.PI, 0, 0]}>
          <coneGeometry args={[0.04, 0.1, 16]} />
          <meshStandardMaterial
            color="#f97316"
            emissive="#f97316"
            emissiveIntensity={0.4}
          />
        </mesh>

        <Suspense fallback={
          <Html center>
            <div className="text-zinc-300 text-sm bg-zinc-900/80 px-3 py-2 rounded border border-zinc-700 backdrop-blur whitespace-nowrap">
              Загрузка модели…
            </div>
          </Html>
        }>
          <RobotModel
            yaw={0}
            pitch={0}
            roll={0}
            posX={0}
            posY={0}
            stationary
            noSmooth
          />
        </Suspense>

        <OrbitControls
          target={[0, 0, 0]}
          maxPolarAngle={Math.PI / 2 - 0.05}
          minDistance={distance * 0.6}
          maxDistance={distance * 4}
          enableDamping
          dampingFactor={0.1}
        />
      </Canvas>
    </div>
  )
}
