import { useMemo, useState } from 'react'
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { MpsScenarioResult } from '@/types/mps'
import {
  HEADING_ERROR_SERIES,
  OMEGA_CMD_SERIES,
  buildExportPayload,
  buildHeadingErrorRows,
  formatTimestampForFile,
  shortRunId,
} from './DiagnosticsPanel.helpers'

interface DiagnosticsPanelProps {
  history: MpsScenarioResult[]
}

const PI = Math.PI

export function DiagnosticsPanel({ history }: DiagnosticsPanelProps) {
  const [selectedRunId, setSelectedRunId] = useState<string | null>(null)

  const selectedRun = useMemo<MpsScenarioResult | null>(() => {
    if (history.length === 0) return null
    if (selectedRunId) {
      const match = history.find((r) => r.run_id === selectedRunId)
      if (match) return match
    }
    // default — newest (history is ordered newest→oldest in compute_node).
    return history[0] ?? null
  }, [history, selectedRunId])

  const rows = useMemo(() => buildHeadingErrorRows(selectedRun), [selectedRun])

  const peakAbsHeadingError = useMemo(() => {
    if (rows.length === 0) return 0
    return rows.reduce((m, r) => Math.max(m, Math.abs(r[HEADING_ERROR_SERIES])), 0)
  }, [rows])

  const wrapWarning = peakAbsHeadingError > PI

  function handleExport() {
    const payload = buildExportPayload(history)
    const blob = new Blob([JSON.stringify(payload, null, 2)], {
      type: 'application/json',
    })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `mps-diagnostics-${formatTimestampForFile(new Date())}.json`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between gap-3 space-y-0">
        <CardTitle>Диагностика</CardTitle>
        <Button
          type="button"
          size="sm"
          variant="outline"
          onClick={handleExport}
          disabled={history.length === 0}
        >
          Скачать JSON ({history.length})
        </Button>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center gap-2 text-sm">
          <label htmlFor="diag-run-select" className="text-muted-foreground">
            Прогон:
          </label>
          <select
            id="diag-run-select"
            value={selectedRun?.run_id ?? ''}
            onChange={(e) => setSelectedRunId(e.target.value || null)}
            disabled={history.length === 0}
            className="border rounded px-2 py-1 text-sm bg-background"
          >
            {history.length === 0 && <option value="">— нет прогонов —</option>}
            {history.map((r) => {
              const phi = r.request.target_heading ?? 0
              const phiTxt =
                r.request.target_heading == null ? '' : ` φ=${phi.toFixed(2)}`
              return (
                <option key={r.run_id} value={r.run_id}>
                  {shortRunId(r.run_id)} · {r.status}
                  {phiTxt}
                </option>
              )
            })}
          </select>
          {selectedRun && (
            <span className="text-xs text-muted-foreground font-mono ml-auto">
              peak |dx[θ]| = {peakAbsHeadingError.toFixed(3)} рад
            </span>
          )}
        </div>

        {wrapWarning && (
          <div
            role="alert"
            className="rounded border border-red-300 bg-red-500/10 px-3 py-1.5 text-xs text-red-800"
          >
            ⚠ |dx[θ]| превышает π — MPC видит «угол через wrap». Скорее всего фаза
            DRIVE поймала разворот через 360°. Сравни с ω_cmd на графике.
          </div>
        )}

        {rows.length === 0 ? (
          <div className="h-72 flex items-center justify-center text-sm text-muted-foreground">
            Нет телеметрии — запусти MPS-сценарий или подожди завершения текущего.
          </div>
        ) : (
          <div className="h-72">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={rows} margin={{ top: 10, right: 24, left: 8, bottom: 16 }}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis
                  dataKey="t"
                  tickFormatter={(v: number) => v.toFixed(2)}
                  label={{ value: 't, с', position: 'insideBottom', offset: -8 }}
                />
                <YAxis />
                <Tooltip
                  formatter={(v: number) => v.toFixed(3)}
                  labelFormatter={(t: number) => `t = ${t.toFixed(3)} с`}
                />
                <Legend />
                <ReferenceLine y={0} stroke="#6b7280" />
                <ReferenceLine
                  y={PI}
                  stroke="#dc2626"
                  strokeDasharray="4 4"
                  label={{ value: '+π', position: 'right', fontSize: 11, fill: '#dc2626' }}
                />
                <ReferenceLine
                  y={-PI}
                  stroke="#dc2626"
                  strokeDasharray="4 4"
                  label={{ value: '−π', position: 'right', fontSize: 11, fill: '#dc2626' }}
                />
                <Line
                  type="monotone"
                  dataKey={HEADING_ERROR_SERIES}
                  stroke="#dc2626"
                  dot={false}
                  isAnimationActive={false}
                  strokeWidth={1.5}
                />
                <Line
                  type="monotone"
                  dataKey={OMEGA_CMD_SERIES}
                  stroke="#f59e0b"
                  dot={false}
                  isAnimationActive={false}
                  strokeWidth={1}
                  strokeOpacity={0.7}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}

        <p className="text-xs text-muted-foreground">
          dx[θ] = x[2] − target_heading (БЕЗ нормализации — именно то, что
          получает MPC). Если кривая выходит за ±π, MPC трактует курс «не той
          стороной» и крутит ω в насыщение.
        </p>
      </CardContent>
    </Card>
  )
}
