import { useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import type { MpsScenarioResult, ScenarioStatus } from '@/types/mps'

interface HistoryPanelProps {
  history: MpsScenarioResult[]
  onSelect: (run: MpsScenarioResult) => void
  onReplay: (runId: string) => void
  onCompareChange: (selected: MpsScenarioResult[]) => void
  loading?: boolean
}

const STATUS_TEXT_COLOR: Record<ScenarioStatus, string> = {
  running: 'text-blue-600',
  reached: 'text-green-600',
  timeout: 'text-amber-600',
  aborted: 'text-orange-600',
  error: 'text-red-600',
}
const STATUS_STRIPE_COLOR: Record<ScenarioStatus, string> = {
  running: 'bg-blue-500',
  reached: 'bg-green-500',
  timeout: 'bg-amber-500',
  aborted: 'bg-orange-500',
  error: 'bg-red-500',
}

const STATUS_OPTIONS: Array<{ value: ScenarioStatus | 'all'; label: string }> = [
  { value: 'all',     label: 'Все' },
  { value: 'reached', label: 'Достигнут' },
  { value: 'timeout', label: 'Timeout' },
  { value: 'aborted', label: 'Aborted' },
  { value: 'error',   label: 'Error' },
]

export function HistoryPanel({
  history,
  onSelect,
  onReplay,
  onCompareChange,
  loading,
}: HistoryPanelProps) {
  const [selectedIds, setSelectedIds] = useState<Set<string>>(new Set())
  const [filter, setFilter] = useState<ScenarioStatus | 'all'>('all')

  const filtered = useMemo(
    () => (filter === 'all' ? history : history.filter((h) => h.status === filter)),
    [history, filter],
  )

  function toggleCompare(runId: string) {
    setSelectedIds((prev) => {
      const next = new Set(prev)
      if (next.has(runId)) {
        next.delete(runId)
      } else {
        if (next.size >= 3) return prev
        next.add(runId)
      }
      const items = history.filter((h) => next.has(h.run_id))
      onCompareChange(items)
      return next
    })
  }

  function clearSelection() {
    setSelectedIds(new Set())
    onCompareChange([])
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between flex-wrap gap-2">
          <span>История прогонов ({history.length})</span>
          <span className="text-xs font-normal text-muted-foreground">
            Compare: {selectedIds.size}/3
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="flex items-center gap-2 mb-2 flex-wrap">
          <select
            value={filter}
            onChange={(e) => setFilter(e.target.value as ScenarioStatus | 'all')}
            className="h-7 text-xs border rounded px-2 bg-background"
            aria-label="status filter"
          >
            {STATUS_OPTIONS.map((o) => (
              <option key={o.value} value={o.value}>
                {o.label}
              </option>
            ))}
          </select>
          {selectedIds.size > 0 && (
            <Button size="sm" variant="ghost" onClick={clearSelection} className="h-7 text-xs">
              Очистить выбор
            </Button>
          )}
        </div>

        {loading ? (
          <div className="text-sm text-muted-foreground">Загрузка…</div>
        ) : filtered.length === 0 ? (
          <div className="text-sm text-muted-foreground">
            {history.length === 0
              ? 'Пока ничего — запусти первый сценарий.'
              : 'Нет прогонов с выбранным статусом.'}
          </div>
        ) : (
          <ScrollArea className="h-72">
            <ul className="space-y-1 text-xs">
              {filtered.map((h) => {
                const sel = selectedIds.has(h.run_id)
                const ts = new Date(h.started_at).toLocaleTimeString()
                return (
                  <li
                    key={h.run_id}
                    className={[
                      'relative flex items-center gap-2 pl-3 pr-2 py-1 rounded',
                      sel ? 'bg-amber-500/10' : 'hover:bg-accent',
                    ].join(' ')}
                  >
                    <span
                      className={[
                        'absolute left-0 top-1 bottom-1 w-1 rounded',
                        STATUS_STRIPE_COLOR[h.status],
                      ].join(' ')}
                    />
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
                      <div className="truncate">{h.run_id.slice(0, 8)}</div>
                      <div className="text-muted-foreground">
                        {ts} · D={h.request.distance.toFixed(2)} · v=
                        {h.request.v_target.toFixed(2)} ·{' '}
                        <span className={STATUS_TEXT_COLOR[h.status]}>{h.status}</span>
                        {h.metrics && <> · ss={h.metrics.ss_error.toFixed(3)}</>}
                      </div>
                    </button>
                    <Button
                      size="icon"
                      variant="ghost"
                      onClick={() => onReplay(h.run_id)}
                      className="h-7 w-7 text-xs"
                      aria-label={`replay ${h.run_id}`}
                      title="Replay"
                    >
                      ▶
                    </Button>
                  </li>
                )
              })}
            </ul>
          </ScrollArea>
        )}
      </CardContent>
    </Card>
  )
}
