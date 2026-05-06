import { createContext } from 'react'

export type EquationIndex = 0 | 1 | 2 | 3 | 4
export type MatrixName = 'A' | 'B' | 'C' | 'D'
export type VectorName = 'Q' | 'R' | 'u_min' | 'u_max'

export interface HoveredCell {
  matrix: MatrixName
  row: number
  col: number
}

export interface HoveredVector {
  name: VectorName
  index: number
}

export interface MpsHighlightState {
  equation: EquationIndex | null
  cell: HoveredCell | null
  vector: HoveredVector | null
}

export interface MpsHighlightContextValue {
  hovered: MpsHighlightState
  setEquation: (i: EquationIndex | null) => void
  setCell: (c: HoveredCell | null) => void
  setVector: (v: HoveredVector | null) => void
  clearAll: () => void
}

export const EMPTY_STATE: MpsHighlightState = {
  equation: null,
  cell: null,
  vector: null,
}

const NOOP_VALUE: MpsHighlightContextValue = {
  hovered: EMPTY_STATE,
  setEquation: () => {},
  setCell: () => {},
  setVector: () => {},
  clearAll: () => {},
}

export const MpsHighlightContext = createContext<MpsHighlightContextValue>(NOOP_VALUE)
