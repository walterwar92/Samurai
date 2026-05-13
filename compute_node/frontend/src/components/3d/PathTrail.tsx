import { useMemo } from 'react'
import { Line, Html } from '@react-three/drei'
import * as THREE from 'three'
import type { TrailPoint } from '@/hooks/usePathTrail'

interface PathTrailProps {
  /** Точки трассы из `usePathTrail`. */
  points: TrailPoint[]
  /** Толщина линии в пикселях (Line2 — настоящие px, не WebGL hairline). */
  lineWidth?: number
  /**
   * Шаг маркеров расстояния по треку, в метрах.
   * Маркер = кружок на полу с подписью пройденного расстояния.
   */
  markerStepM?: number
}

// Палитра градиента «старое → новое»:
// COLOR_OLD — тусклый серый для давних точек,
// COLOR_NEW — янтарь, совпадает с прошлой версией PathTrail (continuity для глаза).
const COLOR_OLD = new THREE.Color('#3f3f5c')
const COLOR_NEW = new THREE.Color('#f59e0b')

/**
 * Линия пройденного пути с градиентом по времени и дистанционными маркерами.
 *
 * Рендеринг: `<Line>` из @react-three/drei — wrapper над three-stdlib Line2,
 * поддерживает реальный пиксельный `lineWidth` (в отличие от `lineBasicMaterial`,
 * где WebGL всегда рисует 1px) и per-vertex `vertexColors`.
 *
 * Маркеры расстояния: каждые `markerStepM` метров — кольцо на полу +
 * Html-подпись (например «1.5 м»). Маркеры не удаляются вместе со старыми
 * точками — их позиции запоминаются в локальном useMemo и пересчитываются
 * только при росте трассы.
 */
export function PathTrail({
  points,
  lineWidth = 3,
  markerStepM = 0.5,
}: PathTrailProps) {
  // Подготавливаем массив координат + цветов для Line2.
  // Градиент строим по индексу: i/N → 0..1, от COLOR_OLD к COLOR_NEW.
  // Это «time gradient» в чистом виде — последние точки самые яркие.
  const { coords, colors } = useMemo(() => {
    if (points.length < 2) {
      return { coords: [] as [number, number, number][], colors: [] as [number, number, number][] }
    }
    const n = points.length
    const _coords: [number, number, number][] = new Array(n)
    const _colors: [number, number, number][] = new Array(n)
    const tmp = new THREE.Color()
    for (let i = 0; i < n; i++) {
      const p = points[i].pos
      _coords[i] = [p.x, p.y, p.z]
      const t = i / (n - 1)
      tmp.copy(COLOR_OLD).lerp(COLOR_NEW, t)
      _colors[i] = [tmp.r, tmp.g, tmp.b]
    }
    return { coords: _coords, colors: _colors }
  }, [points])

  // Маркеры дистанции: первый кратный markerStepM, попадающий между соседними точками.
  const markers = useMemo(() => {
    if (points.length < 2) return [] as { x: number; z: number; label: string }[]
    const out: { x: number; z: number; label: string }[] = []
    let nextMark = markerStepM
    for (let i = 1; i < points.length; i++) {
      const prev = points[i - 1]
      const cur = points[i]
      // Может перепрыгнуть несколько шагов за один сегмент при длинном tick'е.
      while (cur.dist >= nextMark && prev.dist < nextMark) {
        const segLen = cur.dist - prev.dist
        const t = segLen > 0 ? (nextMark - prev.dist) / segLen : 0
        const x = prev.pos.x + (cur.pos.x - prev.pos.x) * t
        const z = prev.pos.z + (cur.pos.z - prev.pos.z) * t
        out.push({ x, z, label: nextMark.toFixed(1) + ' м' })
        nextMark += markerStepM
      }
    }
    return out
  }, [points, markerStepM])

  if (coords.length < 2) return null

  return (
    <group>
      <Line
        points={coords}
        vertexColors={colors}
        lineWidth={lineWidth}
        transparent
        opacity={0.95}
      />
      {markers.map((m, i) => (
        <group key={i} position={[m.x, 0.006, m.z]}>
          <mesh rotation={[-Math.PI / 2, 0, 0]}>
            <ringGeometry args={[0.015, 0.025, 24]} />
            <meshBasicMaterial color="#f59e0b" side={THREE.DoubleSide} transparent opacity={0.9} />
          </mesh>
          <Html
            center
            distanceFactor={1}
            position={[0, 0.04, 0]}
            style={{ pointerEvents: 'none' }}
          >
            <div
              style={{
                fontFamily: 'ui-monospace, monospace',
                fontSize: '10px',
                color: '#fbbf24',
                background: 'rgba(20, 20, 30, 0.75)',
                padding: '1px 4px',
                borderRadius: 3,
                border: '1px solid rgba(245, 158, 11, 0.6)',
                whiteSpace: 'nowrap',
              }}
            >
              {m.label}
            </div>
          </Html>
        </group>
      ))}
    </group>
  )
}
