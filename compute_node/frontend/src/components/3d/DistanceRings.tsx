import { useMemo } from 'react'
import { Html } from '@react-three/drei'
import * as THREE from 'three'

interface DistanceRingsProps {
  /** Шаг колец, м. По умолчанию 0.5 — совпадает с шагом маркеров на PathTrail. */
  stepM?: number
  /** Максимальный радиус, м. Дальше колец не рисуем. */
  maxM?: number
}

/**
 * Концентрические кольца от (0,0) с подписями расстояния — для оценки
 * пройденного пути относительно «дома» (origin = точка старта/Reset Home).
 *
 * Кольца — тонкие ringGeometry на полу, подписи — Html у дальнего края
 * каждого кольца по оси +X. Не вращаются за камерой, всегда лежат плоско.
 */
export function DistanceRings({ stepM = 0.5, maxM = 5.0 }: DistanceRingsProps) {
  const radii = useMemo(() => {
    const out: number[] = []
    for (let r = stepM; r <= maxM + 1e-6; r += stepM) {
      out.push(Number(r.toFixed(2)))
    }
    return out
  }, [stepM, maxM])

  return (
    <group>
      {radii.map((r, i) => {
        // Каждое второе кольцо — чуть ярче, для визуального ритма (0.5, 1.5...)
        const major = i % 2 === 0
        const innerR = r - 0.005
        const outerR = r + 0.005
        return (
          <group key={r}>
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
              <ringGeometry args={[innerR, outerR, 96]} />
              <meshBasicMaterial
                color={major ? '#5a5a7a' : '#3f3f5c'}
                side={THREE.DoubleSide}
                transparent
                opacity={major ? 0.7 : 0.4}
              />
            </mesh>
            {/* Подпись расстояния — у точки (+r, 0) на полу. */}
            <Html
              position={[r, 0.01, 0]}
              center
              distanceFactor={1}
              style={{ pointerEvents: 'none' }}
            >
              <div
                style={{
                  fontFamily: 'ui-monospace, monospace',
                  fontSize: 10,
                  color: major ? '#a1a1aa' : '#71717a',
                  background: 'rgba(20, 20, 30, 0.6)',
                  padding: '1px 4px',
                  borderRadius: 3,
                  whiteSpace: 'nowrap',
                }}
              >
                {r.toFixed(1)} м
              </div>
            </Html>
          </group>
        )
      })}
    </group>
  )
}
