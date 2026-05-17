import { Suspense, useEffect, useMemo, useRef, useState } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, Grid, Html } from '@react-three/drei'
import * as THREE from 'three'
import type { MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'
import { RobotModel } from '@/components/3d/RobotModel'

function statusText(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':         return 'достигнуто'
    case 'aborted':         return 'прервано'
    case 'timeout':         return 'таймаут'
    case 'timeout_settle':  return 'таймаут стабилизации'
    case 'error':           return 'ошибка'
    case 'running':         return 'выполняется'
  }
}

interface Mps3DSceneProps {
  telemetry: MpsTelemetryPoint[]
  distance: number
  status: ScenarioStatus
}

interface Sample {
  t: number
  s: number
  theta: number
  x: number
  y: number
}

interface LiveProgress {
  t: number
  s: number
  index: number  // последний пройденный индекс семпла (для AnimatedTrail)
}

function prepareSamples(telemetry: MpsTelemetryPoint[]): Sample[] {
  const out: Sample[] = []
  for (const p of telemetry) {
    const t = p.t
    const s = p.x[0]
    const theta = p.x[2]
    if (
      !Number.isFinite(t) ||
      !Number.isFinite(s) ||
      !Number.isFinite(theta)
    ) continue
    out.push({
      t,
      s,
      theta,
      x: s * Math.cos(theta),
      y: s * Math.sin(theta),
    })
  }
  return out
}

interface AnimatedRobotProps {
  samples: Sample[]
  progressRef: React.MutableRefObject<LiveProgress | null>
}

function AnimatedRobot({ samples, progressRef }: AnimatedRobotProps) {
  const startTimeRef = useRef<number | null>(null)
  const doneRef = useRef(false)
  const [pose, setPose] = useState({
    posX: samples[0]?.x ?? 0,
    posY: samples[0]?.y ?? 0,
    yawDeg: ((samples[0]?.theta ?? 0) * 180) / Math.PI,
    stationary: false,
  })

  // Visibility-фикс: при возврате во вкладку пересчитываем startTime,
  // чтобы машинка не «прыгнула» к финалу.
  useEffect(() => {
    function onVisibility() {
      if (document.visibilityState === 'visible' && startTimeRef.current !== null) {
        const lastElapsed = progressRef.current?.t ?? 0
        startTimeRef.current = performance.now() - lastElapsed * 1000
      }
    }
    document.addEventListener('visibilitychange', onVisibility)
    return () => document.removeEventListener('visibilitychange', onVisibility)
  }, [progressRef])

  useFrame(() => {
    if (doneRef.current) return
    if (samples.length === 0) return
    if (startTimeRef.current === null) {
      startTimeRef.current = performance.now()
    }
    const elapsed = (performance.now() - startTimeRef.current) / 1000
    const last = samples[samples.length - 1]

    if (elapsed >= last.t) {
      doneRef.current = true
      setPose({
        posX: last.x,
        posY: last.y,
        yawDeg: (last.theta * 180) / Math.PI,
        stationary: true,
      })
      progressRef.current = { t: last.t, s: last.s, index: samples.length - 1 }
      return
    }

    // Бинарный поиск интервала [lo, hi]: samples[lo].t <= elapsed < samples[hi].t
    let lo = 0
    let hi = samples.length - 1
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1
      if (samples[mid].t <= elapsed) lo = mid
      else hi = mid
    }
    const a = samples[lo]
    const b = samples[hi]
    const span = b.t - a.t
    const alpha = span > 0 ? (elapsed - a.t) / span : 0
    const x = a.x + (b.x - a.x) * alpha
    const y = a.y + (b.y - a.y) * alpha
    const theta = a.theta + (b.theta - a.theta) * alpha
    const s = a.s + (b.s - a.s) * alpha
    setPose({
      posX: x,
      posY: y,
      yawDeg: (theta * 180) / Math.PI,
      stationary: false,
    })
    progressRef.current = { t: elapsed, s, index: lo }
  })

  return (
    <RobotModel
      yaw={pose.yawDeg}
      pitch={0}
      roll={0}
      posX={pose.posX}
      posY={pose.posY}
      stationary={pose.stationary}
      noSmooth
    />
  )
}

interface AnimatedTrailProps {
  samples: Sample[]
  progressRef: React.MutableRefObject<LiveProgress | null>
}

function AnimatedTrail({ samples, progressRef }: AnimatedTrailProps) {
  const geometryRef = useRef<THREE.BufferGeometry | null>(null)

  // Префиллим все точки на маунте; setDrawRange ограничивает рисуемую часть.
  // [x, 0.02, -y] — конвертация мир → Three (Three Z смотрит «на нас» = -y_world).
  const positions = useMemo(() => {
    const arr = new Float32Array(samples.length * 3)
    for (let i = 0; i < samples.length; i++) {
      arr[i * 3 + 0] = samples[i].x
      arr[i * 3 + 1] = 0.02
      arr[i * 3 + 2] = -samples[i].y
    }
    return arr
  }, [samples])

  useEffect(() => {
    const geom = geometryRef.current
    if (!geom) return
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geom.setDrawRange(0, 0)
    return () => {
      geom.dispose()
    }
  }, [positions])

  useFrame(() => {
    const geom = geometryRef.current
    if (!geom) return
    const idx = progressRef.current?.index ?? 0
    geom.setDrawRange(0, idx + 1)
  })

  return (
    <line>
      <bufferGeometry ref={geometryRef} />
      <lineBasicMaterial color="#2563eb" linewidth={2} />
    </line>
  )
}

interface SceneInfoOverlayProps {
  progressRef: React.MutableRefObject<LiveProgress | null>
  status: ScenarioStatus
}

function SceneInfoOverlay({ progressRef, status }: SceneInfoOverlayProps) {
  // Обновляем HTML каждый кадр через requestAnimationFrame, без React state.
  const tRef = useRef<HTMLSpanElement | null>(null)
  const sRef = useRef<HTMLSpanElement | null>(null)
  useEffect(() => {
    let raf = 0
    const tick = () => {
      const p = progressRef.current
      if (tRef.current && p) tRef.current.textContent = p.t.toFixed(2)
      if (sRef.current && p) sRef.current.textContent = p.s.toFixed(2)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [progressRef])

  return (
    <div className="absolute bottom-3 left-3 text-xs font-mono text-zinc-300 bg-zinc-900/70 backdrop-blur px-2 py-1 rounded border border-zinc-700">
      t = <span ref={tRef}>0.00</span>с • s = <span ref={sRef}>0.00</span>м • {statusText(status)}
    </div>
  )
}

export function Mps3DScene({ telemetry, distance, status }: Mps3DSceneProps) {
  const samples = useMemo(() => prepareSamples(telemetry), [telemetry])
  const progressRef = useRef<LiveProgress | null>(null)

  // Защитная ветвь — реально Mps3DOverlay уже отфильтровал по validTelemetryCount,
  // но на случай прямого вызова Mps3DScene извне.
  if (samples.length < 2) {
    return (
      <div className="flex h-full items-center justify-center bg-[#1a1a2e]">
        <p className="text-sm text-zinc-400">Нет валидных данных для 3D-визуализации</p>
      </div>
    )
  }

  // Стартовая позиция камеры: смотрит сверху-сбоку на центр траектории.
  const cameraTarget: [number, number, number] = [distance / 2, 0, 0]
  const cameraPos: [number, number, number] = [distance / 2 + 1.0, 1.2, 1.2]

  return (
    <div className="relative w-full h-full">
      <Canvas
        camera={{ position: cameraPos, fov: 50, near: 0.01, far: 100 }}
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
        <directionalLight position={[0, 1, -2]} intensity={0.4} />
        <hemisphereLight args={['#4a90d9', '#2a2a4a', 0.5]} />

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

        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.001, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <shadowMaterial opacity={0.2} />
        </mesh>

        <axesHelper args={[0.3]} />

        {/* Маркер старта: серая сфера в (0,0,0) */}
        <mesh position={[0, 0.02, 0]}>
          <sphereGeometry args={[0.02, 12, 12]} />
          <meshStandardMaterial color="#94a3b8" />
        </mesh>

        {/* Маркер цели: зелёное кольцо в (D, 0, 0) */}
        <mesh position={[distance, 0.02, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.04, 0.005, 8, 32]} />
          <meshStandardMaterial color="#16a34a" emissive="#16a34a" emissiveIntensity={0.4} />
        </mesh>

        <Suspense fallback={
          <Html center>
            <div className="text-zinc-300 text-sm bg-zinc-900/80 px-3 py-2 rounded border border-zinc-700 backdrop-blur whitespace-nowrap">
              Загрузка модели…
            </div>
          </Html>
        }>
          <AnimatedRobot samples={samples} progressRef={progressRef} />
        </Suspense>

        <AnimatedTrail samples={samples} progressRef={progressRef} />

        <OrbitControls
          target={cameraTarget}
          maxPolarAngle={Math.PI / 2 - 0.05}
          minDistance={0.3}
          maxDistance={10}
          enableDamping
          dampingFactor={0.1}
        />
      </Canvas>

      <SceneInfoOverlay progressRef={progressRef} status={status} />
    </div>
  )
}
