import { useCallback, useMemo, useState } from 'react'
import type { ReactNode } from 'react'
import {
  EMPTY_STATE,
  MpsHighlightContext,
  type EquationIndex,
  type HoveredCell,
  type HoveredVector,
  type MpsHighlightContextValue,
  type MpsHighlightState,
} from './highlight-context-value'

export type { EquationIndex, HoveredCell, HoveredVector, MpsHighlightState, MatrixName, VectorName } from './highlight-context-value'

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
