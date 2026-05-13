import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

export interface TrailPoint {
  /** Three.js world-space position (Y-up, mapping: world.x = robot.x, world.z = -robot.y). */
  pos: THREE.Vector3
  /** Cumulative distance from start of trail (m). */
  dist: number
}

export interface PathTrailData {
  /** All recorded points in order. Unbounded — каптится только `softCap` для защиты от 100K+. */
  points: TrailPoint[]
  /** Сумма длин сегментов в метрах. */
  totalDistance: number
}

interface UsePathTrailOptions {
  /** Минимальное расстояние между соседними точками (м). Меньше — точка не пишется. */
  minStep?: number
  /** Жёсткий потолок числа точек (защита от утечки памяти). Когда упрёмся — отбрасываем самые старые. */
  softCap?: number
}

/**
 * Накопление траектории робота. Сбрасывается по изменению `clearSignal`.
 *
 * Логика отделена от рендера, чтобы `PathTrail.tsx` (R3F) и `InfoPanel.tsx`
 * (HTML overlay) читали одни и те же данные — общий накопитель + пройденная дистанция.
 *
 * `stationary=true` блокирует запись точек — это снимает фантомный «дрейф» от
 * шума IMU/одометрии когда робот реально стоит.
 */
export function usePathTrail(
  posX: number,
  posY: number,
  stationary: boolean,
  clearSignal: number,
  opts: UsePathTrailOptions = {},
): PathTrailData {
  const minStep = opts.minStep ?? 0.01      // 1 см
  const softCap = opts.softCap ?? 20000     // ~200 м непрерывной езды

  const pointsRef = useRef<TrailPoint[]>([])
  const distRef = useRef(0)
  const lastClear = useRef(clearSignal)

  // Используем счётчик, а не сами points в state — массив мутируется in-place,
  // и React не пересоздаёт ссылку. Счётчик инкрементируется по событиям и
  // триггерит ре-рендер потребителей.
  const [, forceTick] = useState(0)

  useEffect(() => {
    if (clearSignal !== lastClear.current) {
      pointsRef.current = []
      distRef.current = 0
      lastClear.current = clearSignal
      forceTick((v) => v + 1)
    }
  }, [clearSignal])

  useEffect(() => {
    if (stationary) return

    const pts = pointsRef.current
    // Маппинг ROS-координат (x вперёд, y влево) в Three.js (X вправо, Z вперёд):
    // X_three = robot.x, Z_three = -robot.y. Y_three=0.005 — чуть над сеткой.
    const next = new THREE.Vector3(posX, 0.005, -posY)

    if (pts.length === 0) {
      pts.push({ pos: next, dist: 0 })
      forceTick((v) => v + 1)
      return
    }

    const prev = pts[pts.length - 1].pos
    const step = next.distanceTo(prev)
    if (step < minStep) return

    distRef.current += step
    pts.push({ pos: next, dist: distRef.current })

    if (pts.length > softCap) {
      // Срезаем самые старые ~1% точек, чтобы не делать shift на каждую новую.
      const drop = Math.ceil(softCap * 0.01)
      pts.splice(0, drop)
    }

    forceTick((v) => v + 1)
  }, [posX, posY, stationary, minStep, softCap])

  return {
    points: pointsRef.current,
    totalDistance: distRef.current,
  }
}
