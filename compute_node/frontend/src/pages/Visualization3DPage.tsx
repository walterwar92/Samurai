import { useState, useCallback } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Grid, Environment } from '@react-three/drei'
import { useRobotState } from '@/hooks/useRobotState'
import { useSocket } from '@/providers/SocketProvider'
import { RobotModel } from '@/components/3d/RobotModel'
import { PathTrail } from '@/components/3d/PathTrail'
import { ImuVectors } from '@/components/3d/ImuVectors'
import { InfoPanel } from '@/components/3d/InfoPanel'

export function Visualization3DPage() {
  const state = useRobotState()
  const { connected } = useSocket()
  const [clearSignal, setClearSignal] = useState(0)

  const yaw = state?.imu_ypr?.[0] ?? 0
  const pitch = state?.imu_ypr?.[1] ?? 0
  const roll = state?.imu_ypr?.[2] ?? 0
  const accel: [number, number, number] = state?.imu_accel ?? [0, 0, 9.81]
  const gyro: [number, number, number] = state?.imu_gyro ?? [0, 0, 0]
  const posX = state?.pose?.x ?? 0
  const posY = state?.pose?.y ?? 0
  const linearVel = state?.velocity?.linear ?? 0
  const angularVel = state?.velocity?.angular ?? 0

  const handleClearPath = useCallback(() => {
    setClearSignal(prev => prev + 1)
  }, [])

  return (
    <div className="relative w-full h-screen bg-zinc-950">
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

      {/* 3D Canvas */}
      <Canvas
        camera={{ position: [0.5, 0.4, 0.5], fov: 50, near: 0.01, far: 100 }}
        shadows
      >
        <color attach="background" args={['#09090b']} />

        {/* Lighting */}
        <ambientLight intensity={0.4} />
        <directionalLight
          position={[2, 3, 1]}
          intensity={1.2}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <directionalLight position={[-1, 2, -1]} intensity={0.3} />

        {/* Ground grid */}
        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.5}
          cellColor="#27272a"
          sectionSize={0.5}
          sectionThickness={1}
          sectionColor="#3f3f46"
          fadeDistance={5}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid
        />

        {/* Ground plane for shadows */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.001, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <shadowMaterial opacity={0.3} />
        </mesh>

        {/* Axis labels at origin */}
        <axesHelper args={[0.3]} />

        {/* Robot */}
        <RobotModel
          yaw={yaw}
          pitch={pitch}
          roll={roll}
          posX={posX}
          posY={posY}
        />

        {/* Path trail */}
        <PathTrail
          posX={posX}
          posY={posY}
          clearSignal={clearSignal}
        />

        {/* IMU vectors */}
        <ImuVectors
          posX={posX}
          posY={posY}
          yaw={yaw}
          accel={accel}
          gyro={gyro}
        />

        {/* Camera controls */}
        <OrbitControls
          makeDefault
          target={[posX, 0.05, -posY]}
          maxPolarAngle={Math.PI / 2 - 0.05}
          minDistance={0.1}
          maxDistance={5}
          enableDamping
          dampingFactor={0.1}
        />
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
        linearVel={linearVel}
        angularVel={angularVel}
        onClearPath={handleClearPath}
      />
    </div>
  )
}
