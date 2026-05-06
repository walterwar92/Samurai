export type MatrixName = 'A' | 'B' | 'C' | 'D'

export interface TokenMeta {
  id: string
  matrix: 'A' | 'B'
  row: number
  col: number
  equationRow: 0 | 1 | 2 | 3 | 4
  description: string
}

export const TOKENS: TokenMeta[] = [
  { id: 'coef_s_v',         matrix: 'A', row: 0, col: 1, equationRow: 0, description: '∂ṡ/∂v = 1' },
  { id: 'coef_v_v',         matrix: 'A', row: 1, col: 1, equationRow: 1, description: '∂v̇/∂v = −1/τ_v' },
  { id: 'coef_theta_omega', matrix: 'A', row: 2, col: 3, equationRow: 2, description: '∂θ̇/∂ω = 1' },
  { id: 'coef_omega_omega', matrix: 'A', row: 3, col: 3, equationRow: 3, description: '∂ω̇/∂ω = −1/τ_ω' },
  { id: 'coef_eint_v',      matrix: 'A', row: 4, col: 1, equationRow: 4, description: '∂ė_int/∂v = −1' },
  { id: 'coef_v_uv',        matrix: 'B', row: 1, col: 0, equationRow: 1, description: '∂v̇/∂u_v = 1/τ_v' },
  { id: 'coef_omega_uomega',matrix: 'B', row: 3, col: 1, equationRow: 3, description: '∂ω̇/∂u_ω = 1/τ_ω' },
]

export const TOKEN_BY_ID: Record<string, TokenMeta | undefined> = TOKENS.reduce(
  (acc, t) => {
    acc[t.id] = t
    return acc
  },
  {} as Record<string, TokenMeta>,
)

export function tokenForCell(
  matrix: MatrixName,
  row: number,
  col: number,
): string | null {
  if (matrix !== 'A' && matrix !== 'B') return null
  const t = TOKENS.find((t) => t.matrix === matrix && t.row === row && t.col === col)
  return t?.id ?? null
}

export function cellForToken(
  id: string,
): { matrix: 'A' | 'B'; row: number; col: number } | null {
  const t = TOKEN_BY_ID[id]
  if (!t) return null
  return { matrix: t.matrix, row: t.row, col: t.col }
}
