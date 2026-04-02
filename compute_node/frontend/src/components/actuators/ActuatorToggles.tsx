import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import type { Actuators } from '@/types/robot'

interface ActuatorTogglesProps {
  actuators: Actuators | undefined
  collisionGuardEnabled?: boolean
}

export function ActuatorToggles({ actuators, collisionGuardEnabled }: ActuatorTogglesProps) {
  const clawOpen = actuators?.claw_open ?? false
  const guardOn = collisionGuardEnabled ?? false

  return (
    <div className="flex gap-2">
      <Button
        variant={clawOpen ? 'default' : 'outline'}
        size="sm"
        className="flex-1 text-xs"
        onClick={() => api.setClaw(!clawOpen)}
      >
        Клешня: {clawOpen ? 'ОТКРЫТА' : 'ЗАКРЫТА'}
      </Button>
      <Button
        variant={guardOn ? 'default' : 'outline'}
        size="sm"
        className={`flex-1 text-xs ${guardOn ? 'bg-green-600 hover:bg-green-700' : 'border-red-500 text-red-500 hover:bg-red-500/10'}`}
        onClick={() => api.setCollisionGuard(!guardOn)}
      >
        Защита: {guardOn ? 'ВКЛ' : 'ВЫКЛ'}
      </Button>
    </div>
  )
}
