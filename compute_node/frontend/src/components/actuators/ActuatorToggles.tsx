import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import type { Actuators } from '@/types/robot'

interface ActuatorTogglesProps {
  actuators: Actuators | undefined
}

export function ActuatorToggles({ actuators }: ActuatorTogglesProps) {
  const clawOpen = actuators?.claw_open ?? false
  const laserOn = actuators?.laser_on ?? false

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
        variant={laserOn ? 'destructive' : 'outline'}
        size="sm"
        className="flex-1 text-xs"
        onClick={() => api.setLaser(!laserOn)}
      >
        Лазер: {laserOn ? 'ВКЛ' : 'ВЫКЛ'}
      </Button>
    </div>
  )
}
