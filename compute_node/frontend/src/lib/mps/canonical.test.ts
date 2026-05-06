import { describe, it, expect } from 'vitest'
import {
  detectPhysics,
  buildCanonical,
  DEFAULT_TAU_V,
  DEFAULT_TAU_OMEGA,
  CANONICAL_PATTERN_A,
  CANONICAL_PATTERN_B,
} from './canonical'
import type { MpsMatrices } from '@/types/mps'

function emptyMatrices(): MpsMatrices {
  return {
    A: Array.from({ length: 5 }, () => Array(5).fill(0)),
    B: Array.from({ length: 5 }, () => Array(2).fill(0)),
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

describe('buildCanonical', () => {
  it('returns 5x5 A and 5x2 B', () => {
    const { A, B } = buildCanonical(0.15, 0.10)
    expect(A.length).toBe(5)
    expect(A[0].length).toBe(5)
    expect(B.length).toBe(5)
    expect(B[0].length).toBe(2)
  })

  it('A[0][1] = 1 (ṡ ← v)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[0][1]).toBe(1)
  })

  it('A[1][1] = -1/tau_v', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[1][1]).toBeCloseTo(-1 / 0.15, 5)
  })

  it('A[2][3] = 1 (θ̇ ← ω)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[2][3]).toBe(1)
  })

  it('A[3][3] = -1/tau_omega', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[3][3]).toBeCloseTo(-1 / 0.10, 5)
  })

  it('A[4][1] = -1 (ė_int ← -v)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[4][1]).toBe(-1)
  })

  it('B[1][0] = 1/tau_v', () => {
    const { B } = buildCanonical(0.15, 0.10)
    expect(B[1][0]).toBeCloseTo(1 / 0.15, 5)
  })

  it('B[3][1] = 1/tau_omega', () => {
    const { B } = buildCanonical(0.15, 0.10)
    expect(B[3][1]).toBeCloseTo(1 / 0.10, 5)
  })

  it('all other entries are zero', () => {
    const { A, B } = buildCanonical(0.15, 0.10)
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        const isCanonical =
          (i === 0 && j === 1) ||
          (i === 1 && j === 1) ||
          (i === 2 && j === 3) ||
          (i === 3 && j === 3) ||
          (i === 4 && j === 1)
        if (!isCanonical) expect(A[i][j]).toBe(0)
      }
    }
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 2; j++) {
        const isCanonical = (i === 1 && j === 0) || (i === 3 && j === 1)
        if (!isCanonical) expect(B[i][j]).toBe(0)
      }
    }
  })
})

describe('detectPhysics', () => {
  it('extracts tau_v and tau_omega from canonical matrices', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A
    m.B = built.B
    const r = detectPhysics(m)
    expect(r.status).toBe('canonical')
    expect(r.tau_v).toBeCloseTo(0.15, 4)
    expect(r.tau_omega).toBeCloseTo(0.10, 4)
    expect(r.deviations).toEqual([])
  })

  it('tolerates rounding within 1e-3 relative', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[1][1] = -6.667
    m.B[1][0] = 6.667
    const r = detectPhysics(m)
    expect(r.status).toBe('canonical')
    expect(r.tau_v).toBeCloseTo(0.15, 2)
  })

  it('flags incoherent when A[1][1] and B[1][0] disagree', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[1][1] = -4.0
    const r = detectPhysics(m)
    expect(r.status).toBe('incoherent')
    expect(r.tau_v).toBeNull()
    expect(r.tau_omega).toBeCloseTo(0.10, 4)
  })

  it('flags non_canonical when a "should-be-zero" cell is non-zero', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[0][2] = 0.5
    const r = detectPhysics(m)
    expect(r.status).toBe('non_canonical')
    expect(r.deviations).toEqual([
      expect.objectContaining({
        matrix: 'A',
        row: 0,
        col: 2,
        expected: 0,
        actual: 0.5,
      }),
    ])
  })

  it('flags non_canonical when fixed-one cell is wrong', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[0][1] = 0.95
    const r = detectPhysics(m)
    expect(r.status).toBe('non_canonical')
    expect(r.deviations.find((d) => d.row === 0 && d.col === 1)).toBeDefined()
  })

  it('returns non_canonical for empty matrices (fixed ones become zero)', () => {
    const m = emptyMatrices()
    const r = detectPhysics(m)
    expect(r.status).toBe('non_canonical')
  })
})

describe('CANONICAL_PATTERN exports', () => {
  it('CANONICAL_PATTERN_A has 5 entries', () => {
    expect(CANONICAL_PATTERN_A.length).toBe(5)
  })
  it('CANONICAL_PATTERN_B has 2 entries', () => {
    expect(CANONICAL_PATTERN_B.length).toBe(2)
  })
  it('DEFAULT_TAU_V is 0.15', () => {
    expect(DEFAULT_TAU_V).toBe(0.15)
  })
  it('DEFAULT_TAU_OMEGA is 0.10', () => {
    expect(DEFAULT_TAU_OMEGA).toBe(0.10)
  })
})
