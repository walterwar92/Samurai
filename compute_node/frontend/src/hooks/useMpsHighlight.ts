import { useContext } from 'react'
import { MpsHighlightContext } from '@/components/mps/HighlightContext'

export function useMpsHighlight() {
  return useContext(MpsHighlightContext)
}
