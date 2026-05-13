import { useState, useCallback, useRef, Suspense } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, Grid, Html } from '@react-three/drei'
import { useRobotState } from '@/hooks/useRobotState'
import { useConnected } from '@/stores/selectors'
import { usePathTrail } from '@/hooks/usePathTrail'
import { api } from '@/lib/api'
import { RobotModel } from '@/components/3d/RobotModel'
import { PathTrail } from '@/components/3d/PathTrail'
import { PlannedPathTrail } from '@/components/3d/PlannedPathTrail'
import { ImuVectors } from '@/components/3d/ImuVectors'
import { InfoPanel } from '@/components/3d/InfoPanel'
import { SlamMap3D } from '@/components/3d/SlamMap3D'
import { CoverageHeatmap } from '@/components/3d/CoverageHeatmap'
import { CompassHUD } from '@/components/3d/CompassHUD'
import { DistanceRings } from '@/components/3d/DistanceRings'
import * as THREE from 'three'

/** Helper: smoothly updates OrbitControls target to follow robot position */
function CameraTarget({ posX, posY }: { posX: number; posY: number }) {
  const controlsRef = useRef<any>(null)
  const targetRef = useRef({ posX, posY })
  targetRef.current = { posX, posY }

  useFrame(() => {
    if (!controlsRef.current) return
    const t = controlsRef.current.target as THREE.Vector3
    const tx = targetRef.current.posX
    const tz = -targetRef.current.posY
    // Smooth follow (lerp factor 0.1)
    t.x += (tx - t.x) * 0.1
    t.z += (tz - t.z) * 0.1
    t.y = 0.05
    controlsRef.current.update()
  })

  return (
    <OrbitControls
      ref={controlsRef}
      makeDefault
      maxPolarAngle={Math.PI / 2 - 0.05}
      minDistance={0.1}
      maxDistance={5}
      enableDamping
      dampingFactor={0.1}
    />
  )
}

export function Visualization3DPage() {
  const state = useRobotState()
  const connected = useConnected()
  const [clearSignal, setClearSignal] = useState(0)
  const [useEkf, setUseEkf] = useState(true)

  const hasEkf = state?.imu_has_ekf ?? false

  // Select YPR source based on toggle
  const yprSource = (useEkf && hasEkf)
    ? (state?.imu_ypr_ekf ?? [0, 0, 0])
    : (state?.imu_ypr_raw ?? state?.imu_ypr ?? [0, 0, 0])

  const yaw = yprSource[0]
  const pitch = yprSource[1]
  const roll = yprSource[2]
  const accel: [number, number, number] = state?.imu_accel ?? [0, 0, 9.81]
  const gyro: [number, number, number] = state?.imu_gyro ?? [0, 0, 0]
  const posX = state?.pose?.x ?? 0
  const posY = state?.pose?.y ?? 0
  // Fallback: если backend не эмитит stationary (старый sim) — считаем «едет»
  // только когда есть линейная или угловая скорость. Без этого PathTrail бы
  // молчал в сим-режиме без поля stationary.
  const stationary = state?.stationary ?? (
    Math.abs(state?.velocity?.linear ?? 0) < 0.005 &&
    Math.abs(state?.velocity?.angular ?? 0) < 0.01
  )
  const linearVel = state?.velocity?.linear ?? 0
  const angularVel = state?.velocity?.angular ?? 0
  const ekfBias: [number, number, number] | null = state?.imu_ekf_bias ?? null
  const recordedPath = state?.recorded_path ?? null
  const isReplaying = state?.path_recorder?.state === 'replaying'
  const slamMap = state?.slam_map ?? null

  // Накопитель траектории — общий для PathTrail (рендер) и InfoPanel (метраж).
  const trail = usePathTrail(posX, posY, stationary, clearSignal)

  const handleClearPath = useCallback(() => {
    setClearSignal(prev => prev + 1)
  }, [])

  const handleResetHome = useCallback(() => {
    api.resetPosition()
    setClearSignal(prev => prev + 1)  // also clear visual trail
  }, [])

  const handleToggleEkf = useCallback(() => {
    setUseEkf(prev => !prev)
  }, [])

  // SLAM status: считаем препятствия — поле для диагностики «карты нет».
  // Если slam_map null — Pi не публикует samurai/{id}/slam_map (slam_map_node
  // не запущен или SLAM ещё не построил карту). UI показывает «ожидание»,
  // чтобы пользователь не думал что дашборд сломан.
  const slamObstacleCount = slamMap?.obstacles?.length ?? 0
  const slamStatus: 'offline' | 'empty' | 'active' =
    slamMap === null ? 'offline'
      : slamObstacleCount === 0 ? 'empty'
      : 'active'

  return (
    <div className="relative w-full h-screen bg-[#1a1a2e]">
      {/* Connection status */}
      <div className="absolute top-3 right-3 z-10 flex items-center gap-2">
        <a
          href="/dashboard"
          className="px-3 py-1 text-xs bg-zinc-800 hover:bg-zinc-700 text-zinc-300 rounded border border-zinc-600 transition-colors"
        >
          Dashboard
        </a>
        <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`} />
        <span className="text-xs text-zinc-400">
          {connected ? 'Подключён' : 'Нет связи'}
        </span>
      </div>

      {/* SLAM status badge — диагностика «карты нет» */}
      <div className="absolute top-3 left-3 z-10 flex items-center gap-2 px-3 py-1.5 rounded bg-zinc-900/80 border border-zinc-700 backdrop-blur">
        <div className={`w-2 h-2 rounded-full ${
          slamStatus === 'active' ? 'bg-green-500'
            : slamStatus === 'empty' ? 'bg-yellow-500'
            : 'bg-zinc-500'
        }`} />
        <span className="text-[11px] text-zinc-300 font-mono">
          SLAM:{' '}
          {slamStatus === 'active' && (
            <span className="text-green-400">{slamObstacleCount} obstacles</span>
          )}
          {slamStatus === 'empty' && (
            <span className="text-yellow-400">карта пуста (нет препятствий)</span>
          )}
          {slamStatus === 'offline' && (
            <span className="text-zinc-400">offline — Pi не публикует slam_map</span>
          )}
        </span>
      </div>

      {/* 3D Canvas */}
      <Canvas
        camera={{ position: [0.5, 0.4, 0.5], fov: 50, near: 0.01, far: 100 }}
        shadows
      >
        <color attach="background" args={['#1a1a2e']} />

        {/* Lighting — brighter scene */}
        <ambientLight intensity={0.9} />
        <directionalLight
          position={[2, 3, 1]}
          intensity={1.8}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <directionalLight position={[-1, 2, -1]} intensity={0.7} />
        <directionalLight position={[0, 1, -2]} intensity={0.4} />
        <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]} />

        {/* Ground grid */}
        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.6}
          cellColor="#3f3f5c"
          sectionSize={0.5}
          sectionThickness={1.2}
          sectionColor="#5a5a7a"
          fadeDistance={5}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid
        />

        {/* Ground plane for shadows */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.001, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <shadowMaterial opacity={0.2} />
        </mesh>

        {/* Axis labels at origin */}
        <axesHelper args={[0.3]} />

        {/* Robot */}
        <Suspense fallback={
          <Html center>
            <div className="text-zinc-300 text-sm bg-zinc-900/80 px-3 py-2 rounded border border-zinc-700 backdrop-blur whitespace-nowrap">
              Загрузка модели…
            </div>
          </Html>
        }>
          <RobotModel
            yaw={yaw}
            pitch={pitch}
            roll={roll}
            posX={posX}
            posY={posY}
            stationary={stationary}
          />
        </Suspense>

        {/* Distance rings from origin (0.5m step) */}
        <DistanceRings stepM={0.5} maxM={5.0} />

        {/* Path trail (real-time odometry trace) — bounded only by softCap (~200m) */}
        <PathTrail points={trail.points} />

        {/* Planned return path (from path recorder) */}
        <PlannedPathTrail
          path={recordedPath}
          replaying={isReplaying}
        />

        {/* SLAM map obstacles + detected objects */}
        <SlamMap3D slamMap={slamMap} />

        {/* Coverage heatmap from trail */}
        <CoverageHeatmap trail={slamMap?.trail ?? null} cellSize={slamMap?.info?.resolution ?? 0.05} />

        {/* IMU vectors */}
        <ImuVectors
          posX={posX}
          posY={posY}
          yaw={yaw}
          accel={accel}
          gyro={gyro}
        />

        {/* Camera controls — smoothly follows robot position */}
        <CameraTarget posX={posX} posY={posY} />
      </Canvas>

      {/* Info overlay */}
      <InfoPanel
        yaw={yaw}
        pitch={pitch}
        roll={roll}
        accel={accel}
        gyro={gyro}
        posX={posX}
        posY={posY}
        stationary={stationary}
        linearVel={linearVel}
        angularVel={angularVel}
        pathDistance={trail.totalDistance}
        onClearPath={handleClearPath}
        onResetHome={handleResetHome}
        useEkf={useEkf}
        hasEkf={hasEkf}
        onToggleEkf={handleToggleEkf}
        ekfBias={ekfBias}
        yprRaw={state?.imu_ypr_raw ?? null}
      />

      {/* HUD-compass in bottom-right corner */}
      <CompassHUD yaw={yaw} />
    </div>
  )
}
