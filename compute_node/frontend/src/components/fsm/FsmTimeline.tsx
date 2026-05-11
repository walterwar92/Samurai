import type { FsmState } from '@/types/robot'
import { FSM_ORDER, FSM_SHORT } from './fsm-styles'
import { cn } from '@/lib/utils'

export interface FsmTimelineProps {
  current: FsmState
  className?: string
}

/**
 * FsmTimeline — 7 сегментов в линию, активный coral, пройденные muted-bright,
 * будущие — почти невидимые. Под каждым сегментом — сокращённый label
 * моноспейсом (IDL / SRCH / TGT / APR / GRB / CAL / RTN).
 */
export function FsmTimeline({ current, className }: FsmTimelineProps) {
  const idx = FSM_ORDER.indexOf(current)
  return (
    <div className={cn('flex items-end gap-1.5', className)}>
      {FSM_ORDER.map((s, i) => {
        const isActive = i === idx
        const isPast = i < idx
        return (
          <div key={s} className="flex flex-col items-center gap-1.5 flex-1 min-w-0">
            <div
              className={cn(
                'h-1 w-full rounded-full transition-colors duration-standard',
                isActive
                  ? 'bg-accent'
                  : isPast
                    ? 'bg-foreground-muted/60'
                    : 'bg-surface-3',
              )}
            />
            <span
              className={cn(
                'font-mono text-micro tracking-wider',
                isActive ? 'text-accent' : 'text-foreground-faint',
              )}
            >
              {FSM_SHORT[s]}
            </span>
          </div>
        )
      })}
    </div>
  )
}
