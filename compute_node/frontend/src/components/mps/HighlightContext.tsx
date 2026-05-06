import { createContext, useCallback, useMemo, useState } from 'react'
import type { ReactNode } from 'react'

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

const EMPTY_STATE: MpsHighlightState = {
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

interface MpsHighlightProviderProps {
  children: ReactNode
}

function cellsEqual(a: HoveredCell | null, b: HoveredCell | null): boolean {
  if (a === b) return true
  if (!a || !b) return false
  return a.matrix === b.matrix && a.row === b.row && a.col === b.col
}

function vectorsEqual(a: HoveredVector | null, b: HoveredVector | null): boolean {
  if (a === b) return true
  if (!a || !b) return false
  return a.name === b.name && a.index === b.index
}

export function MpsHighlightProvider({ children }: MpsHighlightProviderProps) {
  const [hovered, setHovered] = useState<MpsHighlightState>(EMPTY_STATE)

  const setEquation = useCallback((equation: EquationIndex | null) => {
    setHovered((h) => (h.equation === equation ? h : { ...h, equation }))
  }, [])

  const setCell = useCallback((cell: HoveredCell | null) => {
    setHovered((h) => (cellsEqual(h.cell, cell) ? h : { ...h, cell }))
  }, [])

  const setVector = useCallback((vector: HoveredVector | null) => {
    setHovered((h) => (vectorsEqual(h.vector, vector) ? h : { ...h, vector }))
  }, [])

  const clearAll = useCallback(() => {
    setHovered(EMPTY_STATE)
  }, [])

  const value = useMemo<MpsHighlightContextValue>(
    () => ({ hovered, setEquation, setCell, setVector, clearAll }),
    [hovered, setEquation, setCell, setVector, clearAll],
  )

  return <MpsHighlightContext.Provider value={value}>{children}</MpsHighlightContext.Provider>
}
