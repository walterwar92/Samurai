import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface ExplorerPanelProps {
  explorer: { state?: string; strategy?: string; progress?: number; covered_cells?: number } | null
}

const strategies = [
  { id: 'frontier', label: 'Frontier' },
  { id: 'spiral', label: 'Спираль' },
  { id: 'zigzag', label: 'Зигзаг' },
]

async function sendExplorerCmd(command: string) {
  await fetch('/api/explorer/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ command }),
  })
}

export function ExplorerPanel({ explorer }: ExplorerPanelProps) {
  const state = explorer?.state ?? 'idle'
  const isActive = state !== 'idle' && state !== 'stopped'

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Авто-маппинг
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${isActive ? 'bg-emerald-500 animate-pulse' : 'bg-zinc-600'}`} />
          <span className="text-xs text-zinc-300 capitalize">{state}</span>
          {explorer?.covered_cells != null && (
            <span className="text-[10px] text-zinc-500 ml-auto">
              {explorer.covered_cells} клеток
            </span>
          )}
        </div>

        <div className="flex flex-wrap gap-1.5">
          {strategies.map(s => (
            <button
              key={s.id}
              onClick={() => sendExplorerCmd(s.id)}
              disabled={isActive}
              className={`px-2.5 py-1 text-xs rounded border transition-colors ${
                explorer?.strategy === s.id
                  ? 'bg-blue-900/60 border-blue-600 text-blue-300'
                  : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
              } ${isActive ? 'opacity-50' : ''}`}
            >
              {s.label}
            </button>
          ))}
          <button
            onClick={() => sendExplorerCmd('stop')}
            className="px-2.5 py-1 text-xs rounded border bg-red-900/60 border-red-600 text-red-300 hover:bg-red-800/60 transition-colors"
          >
            Стоп
          </button>
        </div>

        {isActive && explorer?.progress != null && (
          <div className="w-full bg-zinc-800 rounded-full h-1.5">
            <div
              className="bg-emerald-500 h-1.5 rounded-full transition-all"
              style={{ width: `${Math.round(explorer.progress * 100)}%` }}
            />
          </div>
        )}
      </CardContent>
    </Card>
  )
}
