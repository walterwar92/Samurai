import { Badge } from '@/components/ui/badge'
import { FSM_COLORS } from '@/lib/constants'
import type { FsmState } from '@/types/robot'

interface FsmBadgeProps {
  state: FsmState
}

export function FsmBadge({ state }: FsmBadgeProps) {
  const color = FSM_COLORS[state] || '#37474f'

  return (
    <Badge
      className="text-xs font-bold tracking-wider px-3 py-1 border-0"
      style={{ backgroundColor: color, color: '#fff' }}
    >
      {state}
    </Badge>
  )
}
