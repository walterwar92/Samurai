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
}

export function ScenarioControls({
  defaultDistance = 2.0,
  defaultVTarget = 0.15,
  running,
  onRun,
  onAbort,
  source,
  onSourceChange,
}: ScenarioControlsProps) {
  const [distance, setDistance] = useState(defaultDistance)
  const [vTarget, setVTarget] = useState(defaultVTarget)

  const distanceOk = distance > 0 && distance <= 5.0
  const vTargetOk = vTarget > 0 && vTarget <= 0.30
  const valid = distanceOk && vTargetOk

  return (
    <Card>
      <CardHeader>
        <CardTitle>Сценарий</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center gap-2">
          <label className="text-xs text-muted-foreground w-32">Дистанция D, м</label>
          <Input
            value={String(distance)}
            onChange={(e) => setDistance(Number(e.target.value))}
            type="number"
            min={0.1}
            max={5.0}
            step={0.1}
            className={['h-8 text-sm w-24', !distanceOk ? 'border-red-500' : ''].join(' ')}
            aria-label="distance"
          />
          <span className="text-xs text-muted-foreground">≤ 5.0</span>
        </div>

        <div className="flex items-center gap-2">
          <label className="text-xs text-muted-foreground w-32">v_target, м/с</label>
          <Input
            value={String(vTarget)}
            onChange={(e) => setVTarget(Number(e.target.value))}
            type="number"
            min={0.05}
            max={0.30}
            step={0.05}
            className={['h-8 text-sm w-24', !vTargetOk ? 'border-red-500' : ''].join(' ')}
            aria-label="v_target"
          />
          <span className="text-xs text-muted-foreground">≤ 0.30</span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-xs text-muted-foreground w-32">Источник</span>
          <div className="flex rounded-md border overflow-hidden text-xs">
            <button
              type="button"
              onClick={() => onSourceChange('sim')}
              className={[
                'px-3 py-1',
                source === 'sim' ? 'bg-primary text-primary-foreground' : 'bg-transparent',
              ].join(' ')}
            >
              Sim
            </button>
            <button
              type="button"
              onClick={() => onSourceChange('robot')}
              className={[
                'px-3 py-1 border-l',
                source === 'robot' ? 'bg-primary text-primary-foreground' : 'bg-transparent',
              ].join(' ')}
            >
              Robot
            </button>
          </div>
        </div>

        <div className="flex gap-2">
          <Button
            size="sm"
            onClick={() =>
              onRun({ distance, v_target: vTarget, source })
            }
            disabled={!valid || running}
          >
            {running ? 'Идёт прогон…' : `Run on ${source === 'sim' ? 'Sim' : 'Robot'}`}
          </Button>
          <Button
            size="sm"
            variant="destructive"
            onClick={onAbort}
            disabled={!running}
          >
            Abort
          </Button>
        </div>
      </CardContent>
    </Card>
  )
}
