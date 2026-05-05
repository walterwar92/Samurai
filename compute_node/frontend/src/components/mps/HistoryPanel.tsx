import { useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import type { MpsScenarioResult } from '@/types/mps'

interface HistoryPanelProps {
  history: MpsScenarioResult[]
  onSelect: (run: MpsScenarioResult) => void
  onReplay: (runId: string) => void
  onCompareChange: (selected: MpsScenarioResult[]) => void
  loading?: boolean
}

const STATUS_COLOR: Record<string, string> = {
  reached: 'text-green-600',
  timeout: 'text-amber-600',
  aborted: 'text-orange-600',
  error: 'text-red-600',
  running: 'text-blue-600',
}

export function HistoryPanel({
  history,
  onSelect,
  onReplay,
  onCompareChange,
  loading,
}: HistoryPanelProps) {
  const [selectedIds, setSelectedIds] = useState<Set<string>>(new Set())

  const selectedItems = useMemo(
    () => history.filter((h) => selectedIds.has(h.run_id)),
    [history, selectedIds],
  )

  function toggleCompare(runId: string) {
    setSelectedIds((prev) => {
      const next = new Set(prev)
      if (next.has(runId)) {
        next.delete(runId)
      } else {
        if (next.size >= 3) {
          // Только 3 одновременно (UI ограничение).
          return prev
        }
        next.add(runId)
      }
      const items = history.filter((h) => next.has(h.run_id))
      onCompareChange(items)
      return next
    })
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>История прогонов ({history.length})</CardTitle>
      </CardHeader>
      <CardContent>
        {loading ? (
          <div className="text-sm text-muted-foreground">Загрузка…</div>
        ) : history.length === 0 ? (
          <div className="text-sm text-muted-foreground">
            Пока ничего — запусти первый сценарий.
          </div>
        ) : (
          <ScrollArea className="h-64">
            <ul className="space-y-1 text-xs">
              {history.map((h) => {
                const sel = selectedIds.has(h.run_id)
                const ts = new Date(h.started_at).toLocaleTimeString()
                return (
                  <li
                    key={h.run_id}
                    className={[
                      'flex items-center gap-2 px-2 py-1 rounded',
                      sel ? 'bg-amber-500/10' : 'hover:bg-accent',
                    ].join(' ')}
                  >
                    <input
                      type="checkbox"
                      checked={sel}
                      onChange={() => toggleCompare(h.run_id)}
                      aria-label={`compare ${h.run_id}`}
                    />
                    <button
                      type="button"
                      onClick={() => onSelect(h)}
                      className="flex-1 text-left font-mono"
                    >
                      <div className="truncate">{h.run_id}</div>
                      <div className="text-muted-foreground">
                        {ts} · D={h.request.distance.toFixed(2)} ·{' '}
                        v={h.request.v_target.toFixed(2)} ·{' '}
                        <span className={STATUS_COLOR[h.status] ?? ''}>{h.status}</span>
                        {h.metrics && (
                          <>
                            {' · '}
                            ss_err={h.metrics.ss_error.toFixed(3)}
                          </>
                        )}
                      </div>
                    </button>
                    <Button
                      size="sm"
                      variant="ghost"
                      onClick={() => onReplay(h.run_id)}
                    >
                      Replay
                    </Button>
                  </li>
                )
              })}
            </ul>
          </ScrollArea>
        )}
        {selectedItems.length > 0 && (
          <div className="mt-2 text-xs text-muted-foreground">
            Compare: выбрано {selectedItems.length} (макс 3) — наложение в графиках.
          </div>
        )}
      </CardContent>
    </Card>
  )
}
