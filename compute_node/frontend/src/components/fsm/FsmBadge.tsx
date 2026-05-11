import { cn } from '@/lib/utils'
import { FSM_DOT_HEX } from './fsm-styles'
import type { FsmState } from '@/types/robot'

export interface FsmBadgeProps {
  state: FsmState
  /** Опциональная цель (цвет/действие). Рендерится после `→`. */
  target?: string
  /** Компактный режим — меньше паддинги, для overlay на CameraFeed. */
  compact?: boolean
  className?: string
}

/**
 * FsmBadge — pill с пульсирующей точкой и uppercase state-text моноспейсом.
 * Не пульсирует при IDLE. Цвет точки — из FSM_DOT_HEX (inline style — потому
 * что мы не можем менять `bg-` динамически через переменную в Tailwind).
 */
export function FsmBadge({ state, target, compact, className }: FsmBadgeProps) {
  return (
    <div
      className={cn(
        'inline-flex items-center gap-2 rounded-pill border border-subtle bg-surface-2',
        compact ? 'px-2 py-1' : 'px-3 py-1.5',
        className,
      )}
    >
      <span
        className={cn(
          'h-2 w-2 rounded-full shrink-0',
          state !== 'IDLE' && 'animate-pulse-soft',
        )}
        style={{ backgroundColor: FSM_DOT_HEX[state] }}
      />
      <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
        {state}
      </span>
      {target && (
        <span className="font-mono text-micro text-foreground-faint">→ {target}</span>
      )}
    </div>
  )
}
