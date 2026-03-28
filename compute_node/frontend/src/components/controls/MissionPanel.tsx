import { useState, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface MissionPanelProps {
  mission: { state?: string; name?: string; events_count?: number; progress?: number } | null
}

async function sendMissionCmd(command: string, name?: string) {
  await fetch('/api/mission/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ command, name: name ?? '' }),
  })
}

export function MissionPanel({ mission }: MissionPanelProps) {
  const [missions, setMissions] = useState<string[]>([])
  const [selectedMission, setSelectedMission] = useState('')
  const state = mission?.state ?? 'idle'
  const isRecording = state === 'recording'
  const isReplaying = state === 'replaying'

  useEffect(() => {
    fetch('/api/mission/list')
      .then(r => r.json())
      .then(d => { if (d.missions) setMissions(d.missions) })
      .catch(() => {})
  }, [state]) // refresh on state change

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Миссии
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        {/* Status */}
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${
            isRecording ? 'bg-red-500 animate-pulse'
            : isReplaying ? 'bg-emerald-500 animate-pulse'
            : 'bg-zinc-600'
          }`} />
          <span className="text-xs text-zinc-300">
            {isRecording ? `Запись: ${mission?.name ?? ''}` :
             isReplaying ? `Воспроизведение: ${mission?.name ?? ''}` :
             'Ожидание'}
          </span>
          {mission?.events_count != null && (
            <span className="text-[10px] text-zinc-500 ml-auto">
              {mission.events_count} событий
            </span>
          )}
        </div>

        {/* Controls */}
        <div className="flex flex-wrap gap-1.5">
          <button
            onClick={() => sendMissionCmd('record')}
            disabled={isRecording || isReplaying}
            className={`px-2.5 py-1 text-xs rounded border transition-colors ${
              isRecording
                ? 'bg-red-900/60 border-red-600 text-red-300'
                : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
            }`}
          >
            Запись
          </button>
          <button
            onClick={() => sendMissionCmd('stop')}
            disabled={!isRecording && !isReplaying}
            className="px-2.5 py-1 text-xs rounded border bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700 transition-colors"
          >
            Стоп
          </button>
          <button
            onClick={() => sendMissionCmd('save')}
            disabled={isReplaying}
            className="px-2.5 py-1 text-xs rounded border bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700 transition-colors"
          >
            Сохранить
          </button>
          <button
            onClick={() => sendMissionCmd('play', selectedMission)}
            disabled={isRecording || isReplaying}
            className="px-2.5 py-1 text-xs rounded border bg-emerald-900/60 border-emerald-600 text-emerald-300 hover:bg-emerald-800/60 transition-colors"
          >
            Воспроизвести
          </button>
        </div>

        {/* Saved missions selector */}
        {missions.length > 0 && (
          <select
            value={selectedMission}
            onChange={e => setSelectedMission(e.target.value)}
            className="w-full text-xs bg-zinc-800 border border-zinc-600 rounded px-2 py-1 text-zinc-300"
          >
            <option value="">Выберите миссию...</option>
            {missions.map(m => <option key={m} value={m}>{m}</option>)}
          </select>
        )}

        {/* Progress */}
        {isReplaying && mission?.progress != null && (
          <div className="w-full bg-zinc-800 rounded-full h-1.5">
            <div
              className="bg-emerald-500 h-1.5 rounded-full transition-all"
              style={{ width: `${Math.round(mission.progress * 100)}%` }}
            />
          </div>
        )}
      </CardContent>
    </Card>
  )
}
