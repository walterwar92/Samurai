import type { MpsMatrices } from '@/types/mps'

export const DEFAULT_TAU_V = 0.15
export const DEFAULT_TAU_OMEGA = 0.10
export const DEFAULT_V_ZERO = 0.20

const TOL_REL = 1e-3
const TOL_ABS = 1e-9

type CellRole =
  | { kind: 'fixed'; value: number }
  | { kind: 'tunable'; param: 'tau_v' | 'tau_omega'; expr: 'neg_inv' | 'inv' }

export const CANONICAL_PATTERN_A: Array<{ row: number; col: number; role: CellRole }> = [
  { row: 0, col: 1, role: { kind: 'fixed', value: 1 } },
  { row: 1, col: 1, role: { kind: 'tunable', param: 'tau_v', expr: 'neg_inv' } },
  { row: 2, col: 3, role: { kind: 'fixed', value: 1 } },
  { row: 3, col: 3, role: { kind: 'tunable', param: 'tau_omega', expr: 'neg_inv' } },
  { row: 4, col: 2, role: { kind: 'fixed', value: -1 } },
]

export const CANONICAL_PATTERN_B: Array<{ row: number; col: number; role: CellRole }> = [
  { row: 1, col: 0, role: { kind: 'tunable', param: 'tau_v', expr: 'inv' } },
  { row: 3, col: 1, role: { kind: 'tunable', param: 'tau_omega', expr: 'inv' } },
]

export type CanonicalStatus = 'canonical' | 'incoherent' | 'non_canonical'

export interface CellDeviation {
  matrix: 'A' | 'B'
  row: number
  col: number
  expected: number
  actual: number
}

export interface PhysicsExtraction {
  tau_v: number | null
  tau_omega: number | null
  status: CanonicalStatus
  deviations: CellDeviation[]
}

function approxEqual(a: number, b: number): boolean {
  const denom = Math.max(TOL_ABS, Math.abs(b))
  return Math.abs(a - b) / denom < TOL_REL
}

function expectedValue(role: CellRole, tau_v: number, tau_omega: number): number {
  if (role.kind === 'fixed') return role.value
  const tau = role.param === 'tau_v' ? tau_v : tau_omega
  return role.expr === 'neg_inv' ? -1 / tau : 1 / tau
}

export function buildCanonical(
  tau_v: number,
  tau_omega: number,
): { A: number[][]; B: number[][] } {
  const A = Array.from({ length: 5 }, () => Array(5).fill(0))
  const B = Array.from({ length: 5 }, () => Array(2).fill(0))
  for (const { row, col, role } of CANONICAL_PATTERN_A) {
    A[row][col] = expectedValue(role, tau_v, tau_omega)
  }
  for (const { row, col, role } of CANONICAL_PATTERN_B) {
    B[row][col] = expectedValue(role, tau_v, tau_omega)
  }
  return { A, B }
}

function tauFromCell(
  role: Extract<CellRole, { kind: 'tunable' }>,
  actual: number,
): number | null {
  if (Math.abs(actual) < TOL_ABS) return null
  if (role.expr === 'neg_inv') return -1 / actual
  return 1 / actual
}

export function detectPhysics(m: MpsMatrices): PhysicsExtraction {
  const deviations: CellDeviation[] = []

  const inPatternA = new Set(CANONICAL_PATTERN_A.map(({ row, col }) => `${row},${col}`))
  const inPatternB = new Set(CANONICAL_PATTERN_B.map(({ row, col }) => `${row},${col}`))

  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 5; j++) {
      if (inPatternA.has(`${i},${j}`)) continue
      if (Math.abs(m.A[i][j]) > TOL_ABS) {
        deviations.push({ matrix: 'A', row: i, col: j, expected: 0, actual: m.A[i][j] })
      }
    }
  }
  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 2; j++) {
      if (inPatternB.has(`${i},${j}`)) continue
      if (Math.abs(m.B[i][j]) > TOL_ABS) {
        deviations.push({ matrix: 'B', row: i, col: j, expected: 0, actual: m.B[i][j] })
      }
    }
  }

  const tauVCandidates: number[] = []
  const tauOmegaCandidates: number[] = []
  for (const { row, col, role } of CANONICAL_PATTERN_A) {
    const actual = m.A[row][col]
    if (role.kind === 'fixed') {
      if (!approxEqual(actual, role.value)) {
        deviations.push({ matrix: 'A', row, col, expected: role.value, actual })
      }
    } else {
      const tau = tauFromCell(role, actual)
      if (tau !== null && tau > 0) {
        if (role.param === 'tau_v') tauVCandidates.push(tau)
        else tauOmegaCandidates.push(tau)
      }
    }
  }
  for (const { row, col, role } of CANONICAL_PATTERN_B) {
    const actual = m.B[row][col]
    if (role.kind !== 'tunable') continue
    const tau = tauFromCell(role, actual)
    if (tau !== null && tau > 0) {
      if (role.param === 'tau_v') tauVCandidates.push(tau)
      else tauOmegaCandidates.push(tau)
    }
  }

  function reconcile(candidates: number[]): number | null {
    if (candidates.length === 0) return null
    const first = candidates[0]
    for (const c of candidates.slice(1)) {
      if (!approxEqual(c, first)) return null
    }
    return candidates.reduce((s, x) => s + x, 0) / candidates.length
  }

  const tau_v = reconcile(tauVCandidates)
  const tau_omega = reconcile(tauOmegaCandidates)

  let status: CanonicalStatus
  if (deviations.length === 0 && tau_v !== null && tau_omega !== null) {
    status = 'canonical'
  } else if (
    deviations.length === 0 &&
    (tau_v === null || tau_omega === null) &&
    (tauVCandidates.length > 0 || tauOmegaCandidates.length > 0)
  ) {
    status = 'incoherent'
  } else {
    status = 'non_canonical'
  }

  return { tau_v, tau_omega, status, deviations }
}
