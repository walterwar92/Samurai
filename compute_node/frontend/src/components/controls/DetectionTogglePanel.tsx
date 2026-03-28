import { api } from '@/lib/api'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface DetectionTogglePanelProps {
  detectionEnabled: boolean
  obstacleAvoidanceEnabled: boolean
}

export function DetectionTogglePanel({
  detectionEnabled,
  obstacleAvoidanceEnabled,
}: DetectionTogglePanelProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Детекция и обход
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 flex flex-wrap gap-2">
        {/* YOLO detection toggle */}
        <button
          onClick={() => api.setDetectionEnabled(!detectionEnabled)}
          className={`px-3 py-1.5 text-xs rounded border transition-colors font-medium ${
            detectionEnabled
              ? 'bg-emerald-900/80 border-emerald-600 text-emerald-300 hover:bg-emerald-800/80'
              : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
          }`}
        >
          {detectionEnabled ? 'YOLO ВКЛ' : 'YOLO ВЫКЛ'}
        </button>

        {/* Obstacle avoidance toggle */}
        <button
          onClick={() => api.setObstacleAvoidance(!obstacleAvoidanceEnabled)}
          className={`px-3 py-1.5 text-xs rounded border transition-colors font-medium ${
            obstacleAvoidanceEnabled
              ? 'bg-amber-900/80 border-amber-600 text-amber-300 hover:bg-amber-800/80'
              : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
          }`}
        >
          {obstacleAvoidanceEnabled ? 'Обход препятствий ВКЛ' : 'Обход препятствий ВЫКЛ'}
        </button>
      </CardContent>
    </Card>
  )
}
