import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { FsmBadge } from './FsmBadge'
import { COLOUR_RU, COLOUR_CSS, ACTION_RU } from '@/lib/constants'
import type { RobotState } from '@/types/robot'

interface FsmStatePanelProps {
  state: RobotState | null
}

export function FsmStatePanel({ state }: FsmStatePanelProps) {
  const status = state?.status
  const pose = state?.pose

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Состояние
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        <div className="flex items-center gap-3">
          <FsmBadge state={status?.state || 'IDLE'} />
        </div>

        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="space-y-1">
            <span className="text-muted-foreground">Цель:</span>
            <span
              className="ml-2 font-medium"
              style={{ color: COLOUR_CSS[status?.target_colour || ''] || undefined }}
            >
              {COLOUR_RU[status?.target_colour || ''] || '—'}
            </span>
          </div>
          <div className="space-y-1">
            <span className="text-muted-foreground">Действие:</span>
            <span className="ml-2 font-medium">
              {ACTION_RU[status?.target_action || ''] || '—'}
            </span>
          </div>
          <div className="space-y-1">
            <span className="text-muted-foreground">X:</span>
            <span className="ml-2 font-mono">{pose ? pose.x.toFixed(2) : '—'} м</span>
          </div>
          <div className="space-y-1">
            <span className="text-muted-foreground">Y:</span>
            <span className="ml-2 font-mono">{pose ? pose.y.toFixed(2) : '—'} м</span>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
