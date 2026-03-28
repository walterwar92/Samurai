import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface CalibrationPanelProps {
  calibration: {
    state?: string
    type?: string
    progress?: number
  } | null
  calibrationResult: {
    type?: string
    scale_factor?: number
    recommendation?: string
    odom_distance?: number
    actual_distance?: number
    odom_angle_deg?: number
  } | null
}

async function sendCalCmd(command: string) {
  await fetch('/api/calibration/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ command }),
  })
}

export function CalibrationPanel({ calibration, calibrationResult }: CalibrationPanelProps) {
  const [showResult, setShowResult] = useState(true)
  const state = calibration?.state ?? 'idle'
  const isRunning = state === 'running'

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Калибровка одометрии
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${
            isRunning ? 'bg-amber-500 animate-pulse' : 'bg-zinc-600'
          }`} />
          <span className="text-xs text-zinc-300">
            {isRunning ? `${calibration?.type === 'linear' ? 'Линейная' : 'Угловая'} калибровка...`
              : state === 'done' ? 'Готово' : 'Ожидание'}
          </span>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={() => sendCalCmd('start_linear')}
            disabled={isRunning}
            className={`px-2.5 py-1 text-xs rounded border transition-colors ${
              isRunning ? 'opacity-50' : ''
            } bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700`}
          >
            Линейная
          </button>
          <button
            onClick={() => sendCalCmd('start_angular')}
            disabled={isRunning}
            className={`px-2.5 py-1 text-xs rounded border transition-colors ${
              isRunning ? 'opacity-50' : ''
            } bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700`}
          >
            Угловая
          </button>
          <button
            onClick={() => sendCalCmd('stop')}
            className="px-2.5 py-1 text-xs rounded border bg-red-900/60 border-red-600 text-red-300 hover:bg-red-800/60 transition-colors"
          >
            Стоп
          </button>
        </div>

        {/* Progress */}
        {isRunning && calibration?.progress != null && (
          <div className="w-full bg-zinc-800 rounded-full h-1.5">
            <div
              className="bg-amber-500 h-1.5 rounded-full transition-all"
              style={{ width: `${Math.round(calibration.progress * 100)}%` }}
            />
          </div>
        )}

        {/* Result */}
        {calibrationResult && showResult && (
          <div className="bg-zinc-800/60 rounded p-2 text-xs space-y-1">
            <div className="flex justify-between">
              <span className="text-zinc-400">Тип:</span>
              <span className="text-zinc-200">
                {calibrationResult.type === 'linear' ? 'Линейная' : 'Угловая'}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-zinc-400">Scale factor:</span>
              <span className="text-amber-300 font-mono">{calibrationResult.scale_factor?.toFixed(4)}</span>
            </div>
            {calibrationResult.recommendation && (
              <div className="text-[10px] text-zinc-500 mt-1 break-all">
                {calibrationResult.recommendation}
              </div>
            )}
            <button
              onClick={() => setShowResult(false)}
              className="text-[10px] text-zinc-600 hover:text-zinc-400"
            >
              Скрыть
            </button>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
