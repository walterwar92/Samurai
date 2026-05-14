import { describe, it, expect } from 'vitest'
import {
  groundPointToAngle,
  angleToMarkerPosition,
  formatHeadingLabel,
  groundPointRadius,
  MARKER_HEIGHT,
} from './targetAngle'

describe('groundPointToAngle', () => {
  it('точка прямо перед роботом (+X) → φ ≈ 0', () => {
    expect(groundPointToAngle(2.0, 0)).toBeCloseTo(0, 6)
  })
  it('точка слева (−Z в Three = +Y в мире) → φ = +π/2', () => {
    expect(groundPointToAngle(0, -2.0)).toBeCloseTo(Math.PI / 2, 6)
  })
  it('точка справа (+Z в Three = −Y в мире) → φ = −π/2', () => {
    expect(groundPointToAngle(0, 2.0)).toBeCloseTo(-Math.PI / 2, 6)
  })
  it('точка сзади (−X) → |φ| = π', () => {
    expect(Math.abs(groundPointToAngle(-2.0, 0))).toBeCloseTo(Math.PI, 6)
  })
})

describe('angleToMarkerPosition', () => {
  it('φ=0 → маркер на (+N, h, 0)', () => {
    const [x, h, z] = angleToMarkerPosition(0, 2.0)
    expect(x).toBeCloseTo(2.0, 6)
    expect(h).toBe(MARKER_HEIGHT)
    expect(z).toBeCloseTo(0, 6)
  })
  it('φ=+π/2 → маркер на (0, h, −N)', () => {
    const [x, h, z] = angleToMarkerPosition(Math.PI / 2, 2.0)
    expect(x).toBeCloseTo(0, 6)
    expect(h).toBe(MARKER_HEIGHT)
    expect(z).toBeCloseTo(-2.0, 6)
  })
  it('радиус сохраняется для любого угла', () => {
    const [x, , z] = angleToMarkerPosition(0.7, 3.0)
    expect(Math.hypot(x, z)).toBeCloseTo(3.0, 6)
  })
  it('round-trip: angleToMarkerPosition → groundPointToAngle', () => {
    const phi = 0.9
    const [x, , z] = angleToMarkerPosition(phi, 2.5)
    expect(groundPointToAngle(x, z)).toBeCloseTo(phi, 6)
  })
})

describe('formatHeadingLabel', () => {
  it('φ≈0 → «прямо»', () => {
    expect(formatHeadingLabel(0)).toBe('прямо')
    expect(formatHeadingLabel(0.02)).toBe('прямо')
  })
  it('φ>0 → «+N°»', () => {
    expect(formatHeadingLabel(Math.PI / 4)).toBe('+45°')
  })
  it('φ<0 → «−N°»', () => {
    expect(formatHeadingLabel(-Math.PI / 4)).toBe('-45°')
  })
})

describe('groundPointRadius', () => {
  it('расстояние от центра (origin)', () => {
    expect(groundPointRadius(3, 4)).toBeCloseTo(5, 6)
  })
})
