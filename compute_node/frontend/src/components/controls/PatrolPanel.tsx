import { useState } from 'react'
import { MapPin, Play, Square, Pause, Trash2 } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

interface PatrolPanelProps {
  patrol: { active: boolean; paused: boolean; current_waypoint_idx: number; total: number } | null
}

export function PatrolPanel({ patrol }: PatrolPanelProps) {
  const [waypoints, setWaypoints] = useState<{ x: number; y: number; yaw: number }[]>([])

  const addWaypoint = (x: number, y: number) => {
    setWaypoints((prev) => [...prev, { x, y, yaw: 0 }])
  }

  const removeWaypoint = (idx: number) => {
    setWaypoints((prev) => prev.filter((_, i) => i !== idx))
  }

  const sendWaypoints = () => {
    api.setPatrolWaypoints(waypoints)
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Патрулирование
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex gap-1.5">
          <Button
            size="sm"
            variant={patrol?.active && !patrol?.paused ? 'default' : 'outline'}
            className="flex-1 h-7 text-[10px]"
            onClick={() => {
              sendWaypoints()
              api.patrolCommand('start')
            }}
          >
            <Play size={12} className="mr-1" /> Старт
          </Button>
          <Button
            size="sm"
            variant="outline"
            className="flex-1 h-7 text-[10px]"
            onClick={() => api.patrolCommand('pause')}
          >
            <Pause size={12} className="mr-1" /> Пауза
          </Button>
          <Button
            size="sm"
            variant="destructive"
            className="flex-1 h-7 text-[10px]"
            onClick={() => api.patrolCommand('stop')}
          >
            <Square size={12} className="mr-1" /> Стоп
          </Button>
        </div>

        {patrol && patrol.active && (
          <div className="text-xs text-muted-foreground">
            Точка {patrol.current_waypoint_idx + 1} / {patrol.total}
            {patrol.paused && ' (пауза)'}
          </div>
        )}

        {waypoints.length > 0 && (
          <div className="space-y-1">
            <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
              Точки маршрута ({waypoints.length})
            </span>
            {waypoints.map((wp, i) => (
              <div key={i} className="flex items-center gap-1.5 text-[10px]">
                <MapPin size={10} className="text-cyan-400" />
                <span className="font-mono">
                  ({wp.x.toFixed(1)}, {wp.y.toFixed(1)})
                </span>
                <button
                  className="ml-auto text-muted-foreground hover:text-red-400"
                  onClick={() => removeWaypoint(i)}
                >
                  <Trash2 size={10} />
                </button>
              </div>
            ))}
          </div>
        )}

        <div className="text-[10px] text-muted-foreground italic">
          Нажмите на карту для добавления точек
        </div>
      </CardContent>
    </Card>
  )
}
