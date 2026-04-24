import { cn } from '@/lib/utils'
import { ROBOTS, useRobot, type RobotId } from '@/providers/RobotProvider'
import { useNavigate } from 'react-router-dom'

export function RobotSelector() {
  const { activeRobot, setActiveRobot } = useRobot()
  const navigate = useNavigate()

  const pick = (id: RobotId) => {
    setActiveRobot(id)
    // Vpered пока без дашборда в реальном времени — показываем его страницу
    if (id === 'vpered') {
      navigate('/vpered')
    } else {
      navigate('/dashboard')
    }
  }

  return (
    <div className="flex items-center gap-1 bg-muted/40 rounded-md p-0.5 border border-border/60">
      {(Object.keys(ROBOTS) as RobotId[]).map((id) => {
        const r = ROBOTS[id]
        const active = activeRobot === id
        return (
          <button
            key={id}
            onClick={() => pick(id)}
            title={`${r.name} — ${r.controller}`}
            className={cn(
              'flex items-center gap-1.5 px-2.5 py-1 rounded text-[11px] font-medium transition-colors',
              active
                ? 'bg-primary text-primary-foreground shadow-sm'
                : 'text-muted-foreground hover:text-foreground hover:bg-accent/30',
            )}
          >
            <span
              className={cn(
                'w-1.5 h-1.5 rounded-full',
                r.online ? 'bg-samurai-green' : 'bg-samurai-red',
              )}
            />
            {r.name}
          </button>
        )
      })}
    </div>
  )
}
