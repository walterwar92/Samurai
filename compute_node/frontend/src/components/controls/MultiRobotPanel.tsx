import { useState, useEffect, useCallback } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface RobotInfo {
  id: string
  connected: boolean
  state?: string
  battery?: number
  lastSeen: number
}

interface MultiRobotPanelProps {
  currentRobotId?: string
}

export function MultiRobotPanel({ currentRobotId = 'robot1' }: MultiRobotPanelProps) {
  const [robots, setRobots] = useState<RobotInfo[]>([
    { id: 'robot1', connected: true, lastSeen: Date.now() },
  ])
  const [activeId, setActiveId] = useState(currentRobotId)

  // Poll for connected robots
  useEffect(() => {
    const poll = async () => {
      try {
        const res = await fetch('/api/multi_robot/list')
        if (res.ok) {
          const data = await res.json()
          if (data.robots) {
            setRobots(data.robots)
          }
        }
      } catch {
        // API not available yet — single robot mode
      }
    }

    poll()
    const interval = setInterval(poll, 5000)
    return () => clearInterval(interval)
  }, [])

  const handleSwitch = useCallback(async (id: string) => {
    setActiveId(id)
    try {
      await fetch('/api/multi_robot/switch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ robot_id: id }),
      })
    } catch {
      // Fallback: reload with query param
      window.location.href = `/dashboard?robot=${id}`
    }
  }, [])

  const handleCallRobot = useCallback(async (id: string) => {
    try {
      await fetch('/api/multi_robot/call', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ robot_id: id }),
      })
    } catch { /* ignore */ }
  }, [])

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Роботы ({robots.length})
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="space-y-1.5">
          {robots.map((robot) => {
            const isActive = robot.id === activeId
            const stale = Date.now() - robot.lastSeen > 10000

            return (
              <div
                key={robot.id}
                className={`flex items-center justify-between p-1.5 rounded border text-xs transition-colors ${
                  isActive
                    ? 'bg-blue-900/40 border-blue-600'
                    : 'bg-zinc-800/50 border-zinc-700 hover:border-zinc-500'
                }`}
              >
                <div className="flex items-center gap-2">
                  <div className={`w-2 h-2 rounded-full ${
                    robot.connected && !stale ? 'bg-green-500' : 'bg-red-500'
                  }`} />
                  <span className={isActive ? 'text-blue-200 font-medium' : 'text-zinc-300'}>
                    {robot.id}
                  </span>
                  {robot.state && (
                    <span className="text-[10px] text-zinc-500">{robot.state}</span>
                  )}
                  {robot.battery != null && robot.battery >= 0 && (
                    <span className={`text-[10px] ${
                      robot.battery < 20 ? 'text-red-400' : 'text-zinc-500'
                    }`}>
                      {robot.battery}%
                    </span>
                  )}
                </div>

                <div className="flex gap-1">
                  {!isActive && (
                    <button
                      onClick={() => handleSwitch(robot.id)}
                      className="px-2 py-0.5 text-[10px] bg-zinc-700 hover:bg-zinc-600 rounded transition-colors"
                    >
                      Выбрать
                    </button>
                  )}
                  <button
                    onClick={() => handleCallRobot(robot.id)}
                    className="px-2 py-0.5 text-[10px] bg-zinc-700 hover:bg-zinc-600 rounded transition-colors"
                    title="Позвать робота к текущему роботу"
                  >
                    Позвать
                  </button>
                </div>
              </div>
            )
          })}
        </div>
      </CardContent>
    </Card>
  )
}
