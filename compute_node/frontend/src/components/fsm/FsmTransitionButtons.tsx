import { Button } from '@/components/ui/button'
import { ALL_STATES, FSM_COLORS } from '@/lib/constants'
import { useFsmTransition } from '@/hooks/useFsmTransition'
import type { FsmState } from '@/types/robot'

interface FsmTransitionButtonsProps {
  currentState: FsmState
}

export function FsmTransitionButtons({ currentState }: FsmTransitionButtonsProps) {
  const { transitionTo, pending } = useFsmTransition()
  return (
    <div className="flex flex-wrap gap-1">
      {ALL_STATES.map((s) => (
        <Button
          key={s}
          variant="outline"
          size="sm"
          disabled={pending}
          className="text-[9px] h-6 px-1.5 font-mono"
          style={{
            borderColor: FSM_COLORS[s],
            color: s === currentState ? '#fff' : FSM_COLORS[s],
            backgroundColor: s === currentState ? FSM_COLORS[s] : 'transparent',
          }}
          onClick={() => transitionTo(s)}
        >
          {s}
        </Button>
      ))}
    </div>
  )
}
