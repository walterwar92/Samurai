import { describe, it, expect } from 'vitest'
import { tokenForCell, cellForToken, TOKENS, TOKEN_BY_ID } from './tokenMap'

describe('tokenForCell', () => {
  it('returns coef_s_v for A[0][1]', () => {
    expect(tokenForCell('A', 0, 1)).toBe('coef_s_v')
  })
  it('returns coef_v_v for A[1][1]', () => {
    expect(tokenForCell('A', 1, 1)).toBe('coef_v_v')
  })
  it('returns coef_omega_omega for A[3][3]', () => {
    expect(tokenForCell('A', 3, 3)).toBe('coef_omega_omega')
  })
  it('returns coef_v_uv for B[1][0]', () => {
    expect(tokenForCell('B', 1, 0)).toBe('coef_v_uv')
  })
  it('returns coef_omega_uomega for B[3][1]', () => {
    expect(tokenForCell('B', 3, 1)).toBe('coef_omega_uomega')
  })
  it('returns null for non-canonical cell A[0][2]', () => {
    expect(tokenForCell('A', 0, 2)).toBeNull()
  })
  it('returns null for B[0][0]', () => {
    expect(tokenForCell('B', 0, 0)).toBeNull()
  })
})

describe('cellForToken', () => {
  it('inverse for canonical cells', () => {
    expect(cellForToken('coef_s_v')).toEqual({ matrix: 'A', row: 0, col: 1 })
    expect(cellForToken('coef_v_v')).toEqual({ matrix: 'A', row: 1, col: 1 })
    expect(cellForToken('coef_v_uv')).toEqual({ matrix: 'B', row: 1, col: 0 })
  })
  it('returns null for unknown token', () => {
    expect(cellForToken('coef_nonexistent')).toBeNull()
  })
})

describe('TOKENS metadata', () => {
  it('has 7 canonical tokens', () => {
    expect(TOKENS.length).toBe(7)
  })
  it('TOKEN_BY_ID is keyed by id', () => {
    expect(TOKEN_BY_ID['coef_v_v']).toBeDefined()
    expect(TOKEN_BY_ID['coef_v_v']?.equationRow).toBe(1)
  })
})
