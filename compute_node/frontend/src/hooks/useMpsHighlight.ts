import { useContext } from 'react'
import { MpsHighlightContext } from '@/components/mps/highlight-context-value'

export function useMpsHighlight() {
  return useContext(MpsHighlightContext)
}
