import { useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import type { ScenarioSource } from '@/types/mps'

interface ScenarioControlsProps {
  defaultDistance?: number
  defaultVTarget?: number
  running: boolean
  onRun: (params: { distance: number; v_target: number; source: ScenarioSource }) => void
  onAbort: () => void
  source: ScenarioSource
  onSourceChange: (s: ScenarioSource) => void
  progress?: number
}

export function ScenarioControls({
  defaultDistance = 2.0,
  defaultVTarget = 0.15,
  running,
  onRun,
  onAbort,
  source,
  onSourceChange,
  progress,
}: ScenarioControlsProps) {
  const [distance, setDistance] = useState(defaultDistance)
  const [vTarget, setVTarget] = useState(defaultVTarget)

  const distanceOk = distance > 0 && distance <= 5.0
  const vTargetOk = vTarget > 0 && vTarget <= 0.30
  const valid = distanceOk && vTargetOk

  return (
    <Card>
      <CardHeader>
        <CardTitle>Сценарий «доехать в (D, 0) и развернуться к φ»</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center gap-3 flex-wrap">
          <div className="flex items-center gap-1">
            <label className="text-xs font-mono text-muted-foreground">D, м</label>
            <Input
              type="number"
              min={0.1}
              max={5.0}
              step={0.1}
              value={String(distance)}
              onChange={(e) => setDistance(Number(e.target.value))}
              disabled={running}
              className={[
                'h-8 text-sm w-20 font-mono',
                !distanceOk ? 'border-red-500' : '',
              ].join(' ')}
              aria-label="distance"
            />
          </div>
          <div className="flex items-center gap-1">
            <label className="text-xs font-mono text-muted-foreground">v_target</label>
            <Input
              type="number"
              min={0.05}
              max={0.30}
              step={0.05}
              value={String(vTarget)}
              onChange={(e) => setVTarget(Number(e.target.value))}
              disabled={running}
              className={[
                'h-8 text-sm w-20 font-mono',
                !vTargetOk ? 'border-red-500' : '',
              ].join(' ')}
              aria-label="v_target"
            />
            <span className="text-xs text-muted-foreground">м/с</span>
          </div>

          <div className="flex rounded-md border overflow-hidden text-xs">
            <button
              type="button"
              onClick={() => onSourceChange('sim')}
              disabled={running}
              className={[
                'px-3 py-1 transition-colors',
                source === 'sim'
                  ? 'bg-primary text-primary-foreground'
                  : 'bg-transparent hover:bg-muted',
              ].join(' ')}
            >
              Sim
            </button>
            <button
              type="button"
              onClick={() => onSourceChange('robot')}
              disabled={running}
              className={[
                'px-3 py-1 border-l transition-colors',
                source === 'robot'
                  ? 'bg-primary text-primary-foreground'
                  : 'bg-transparent hover:bg-muted',
              ].join(' ')}
            >
              Robot
            </button>
          </div>

          <div className="ml-auto flex gap-2">
            <Button
              size="sm"
              onClick={() => onRun({ distance, v_target: vTarget, source })}
              disabled={!valid || running}
            >
              {running ? '⏳ Прогон…' : `▶ Run on ${source === 'sim' ? 'Sim' : 'Robot'}`}
            </Button>
            <Button size="sm" variant="destructive" onClick={onAbort} disabled={!running}>
              ⏹ Abort
            </Button>
          </div>
        </div>

        {running && progress !== undefined && progress >= 0 && (
          <div className="h-1.5 bg-muted rounded overflow-hidden">
            <div
              className="h-full bg-primary transition-all duration-150"
              style={{ width: `${Math.min(100, Math.max(0, progress * 100))}%` }}
            />
          </div>
        )}

        <div className="text-xs text-muted-foreground">
          ⓘ Sim: ~100 мс. Robot: ≈ D / v_target секунд физически.
        </div>
      </CardContent>
    </Card>
  )
}
