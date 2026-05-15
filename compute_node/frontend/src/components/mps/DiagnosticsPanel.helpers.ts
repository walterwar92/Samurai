import type { MpsScenarioResult } from '@/types/mps'

export const DIAG_SCHEMA_VERSION = '1.0'
export const HEADING_ERROR_SERIES = 'dx[θ]'
export const OMEGA_CMD_SERIES = 'ω_cmd'

export interface HeadingErrorRow {
  t: number
  [HEADING_ERROR_SERIES]: number
  [OMEGA_CMD_SERIES]: number
}

export function buildHeadingErrorRows(run: MpsScenarioResult | null): HeadingErrorRow[] {
  if (!run) return []
  const phi = run.request.target_heading ?? 0
  return run.telemetry.map((p) => ({
    t: p.t,
    [HEADING_ERROR_SERIES]: (p.x[2] ?? 0) - phi,
    [OMEGA_CMD_SERIES]: p.u[1] ?? 0,
  }))
}

export interface DiagnosticsExportPayload {
  exported_at: string
  schema_version: string
  runs: MpsScenarioResult[]
}

export function buildExportPayload(
  history: MpsScenarioResult[],
  now: Date = new Date(),
): DiagnosticsExportPayload {
  return {
    exported_at: now.toISOString(),
    schema_version: DIAG_SCHEMA_VERSION,
    runs: history,
  }
}

export function formatTimestampForFile(d: Date): string {
  const pad = (n: number) => String(n).padStart(2, '0')
  return (
    `${d.getUTCFullYear()}-${pad(d.getUTCMonth() + 1)}-${pad(d.getUTCDate())}` +
    `-${pad(d.getUTCHours())}${pad(d.getUTCMinutes())}${pad(d.getUTCSeconds())}`
  )
}

export function shortRunId(id: string): string {
  return id.length > 8 ? id.slice(0, 8) : id
}
