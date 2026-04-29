import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import { useFsmTransition } from '@/hooks/useFsmTransition'

export function EmergencyStop() {
  const { transitionTo, pending } = useFsmTransition()

  const handleStop = async () => {
    await api.emergencyStop()
    await transitionTo('IDLE')
  }

  return (
    <Button
      variant="destructive"
      className="w-full font-bold text-sm py-3"
      disabled={pending}
      onClick={handleStop}
    >
      ЭКСТРЕННАЯ ОСТАНОВКА
    </Button>
  )
}
