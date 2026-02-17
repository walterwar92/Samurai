import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

export function EmergencyStop() {
  const handleStop = async () => {
    await api.emergencyStop()
    await api.forceTransition('IDLE')
  }

  return (
    <Button
      variant="destructive"
      className="w-full font-bold text-sm py-3"
      onClick={handleStop}
    >
      ЭКСТРЕННАЯ ОСТАНОВКА
    </Button>
  )
}
