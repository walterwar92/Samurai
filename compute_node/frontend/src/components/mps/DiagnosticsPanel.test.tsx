import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { DiagnosticsPanel } from './DiagnosticsPanel'
import {
  buildExportPayload,
  buildHeadingErrorRows,
} from './DiagnosticsPanel.helpers'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

// Recharts использует ResizeObserver через ResponsiveContainer; в jsdom оно
// есть не везде и LineChart всё равно рендерится с шириной 0. Заменяем
// контейнер на div с фиксированными размерами — компоненты-дети рендерятся
// нормально, но мы их в assert'ах не трогаем (нет смысла гонять SVG в jsdom).
vi.mock('recharts', async () => {
  const actual = await vi.importActual<typeof import('recharts')>('recharts')
  return {
    ...actual,
    ResponsiveContainer: ({ children }: { children: React.ReactNode }) => (
      <div data-testid="recharts-container" style={{ width: 800, height: 300 }}>
        {children}
      </div>
    ),
  }
})

const PI = Math.PI

function makeResult(opts: {
  runId?: string
  targetHeading?: number | undefined
  telemetry?: MpsTelemetryPoint[]
  status?: MpsScenarioResult['status']
} = {}): MpsScenarioResult {
  return {
    run_id: opts.runId ?? 'run-1',
    started_at: '2026-05-15T10:00:00Z',
    finished_at: '2026-05-15T10:00:05Z',
    status: opts.status ?? 'reached',
    request: {
      distance: 1.0,
      v_target: 0.1,
      source: 'robot',
      target_heading: opts.targetHeading,
      schema_version: '1.0',
    },
    matrices_snapshot: {
      A: [], B: [], C: [], D: [],
      Q_diag: [], R_diag: [],
      horizon_N: 10,
      u_min: [], u_max: [],
      schema_version: '1.0',
    },
    telemetry: opts.telemetry ?? [
      { t: 0, x: [0, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.0 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

describe('buildHeadingErrorRows', () => {
  it('returns [] for null run', () => {
    expect(buildHeadingErrorRows(null)).toEqual([])
  })

  it('subtracts target_heading WITHOUT normalization (raw, what MPC sees)', () => {
    // target_heading=π, theta=-π+0.05 → dx = -π+0.05 - π = -2π+0.05 ≈ -6.23
    const run = makeResult({
      targetHeading: PI,
      telemetry: [
        { t: 0, x: [0, 0, -PI + 0.05, 0, 0], u: [0, 0.5], y: [], s_remaining: 1 },
      ],
    })
    const rows = buildHeadingErrorRows(run)
    expect(rows).toHaveLength(1)
    expect(rows[0]['dx[θ]']).toBeCloseTo(-2 * PI + 0.05, 4)
    expect(rows[0]['ω_cmd']).toBe(0.5)
    expect(rows[0].t).toBe(0)
  })

  it('treats missing target_heading as 0', () => {
    const run = makeResult({
      targetHeading: undefined,
      telemetry: [
        { t: 0, x: [0, 0, 0.3, 0, 0], u: [0, 0], y: [], s_remaining: 1 },
      ],
    })
    expect(buildHeadingErrorRows(run)[0]['dx[θ]']).toBeCloseTo(0.3, 6)
  })

  it('handles empty/short x arrays defensively', () => {
    const run = makeResult({
      targetHeading: 0,
      telemetry: [
        // x[2] и u[1] отсутствуют → defensive fallback на 0
        { t: 1, x: [0], u: [], y: [], s_remaining: 0 },
      ],
    })
    const rows = buildHeadingErrorRows(run)
    expect(rows[0]['dx[θ]']).toBe(0)
    expect(rows[0]['ω_cmd']).toBe(0)
  })
})

describe('buildExportPayload', () => {
  it('wraps history with timestamp + schema version', () => {
    const fixed = new Date('2026-05-15T19:42:01.000Z')
    const r1 = makeResult({ runId: 'a' })
    const r2 = makeResult({ runId: 'b' })
    const payload = buildExportPayload([r1, r2], fixed)
    expect(payload.exported_at).toBe('2026-05-15T19:42:01.000Z')
    expect(payload.schema_version).toBe('1.0')
    expect(payload.runs).toEqual([r1, r2])
  })

  it('handles empty history', () => {
    const payload = buildExportPayload([], new Date('2026-01-01T00:00:00Z'))
    expect(payload.runs).toEqual([])
  })
})

describe('DiagnosticsPanel — render', () => {
  it('shows empty state when no history', () => {
    render(<DiagnosticsPanel history={[]} />)
    expect(screen.getByRole('heading', { name: /Диагностика/ })).toBeInTheDocument()
    expect(screen.getByText(/нет прогонов/)).toBeInTheDocument()
    expect(screen.getByText(/Нет телеметрии/)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /Скачать JSON/ })).toBeDisabled()
  })

  it('selects newest run by default and shows peak |dx[θ]|', () => {
    const newest = makeResult({
      runId: 'newest',
      targetHeading: 0,
      telemetry: [
        { t: 0, x: [0, 0, 0.0, 0, 0], u: [0, 0], y: [], s_remaining: 1 },
        { t: 0.1, x: [0, 0, 0.7, 0, 0], u: [0, 0.5], y: [], s_remaining: 1 },
      ],
    })
    const older = makeResult({ runId: 'older', telemetry: [] })
    render(<DiagnosticsPanel history={[newest, older]} />)
    expect(screen.getByText(/peak \|dx\[θ\]\|/)).toHaveTextContent('0.700')
  })

  it('switches selection via select dropdown', () => {
    const a = makeResult({
      runId: 'aaaaaaaa-aaaaa',
      targetHeading: 0,
      telemetry: [
        { t: 0, x: [0, 0, 0.1, 0, 0], u: [0, 0], y: [], s_remaining: 1 },
      ],
    })
    const b = makeResult({
      runId: 'bbbbbbbb-bbbbb',
      targetHeading: 0,
      telemetry: [
        { t: 0, x: [0, 0, 1.5, 0, 0], u: [0, 0], y: [], s_remaining: 1 },
      ],
    })
    render(<DiagnosticsPanel history={[a, b]} />)
    // default — newest = a (history[0])
    expect(screen.getByText(/peak \|dx\[θ\]\|/)).toHaveTextContent('0.100')
    fireEvent.change(screen.getByLabelText(/Прогон:/), {
      target: { value: 'bbbbbbbb-bbbbb' },
    })
    expect(screen.getByText(/peak \|dx\[θ\]\|/)).toHaveTextContent('1.500')
  })

  it('shows wrap warning when peak |dx[θ]| > π', () => {
    const wrapping = makeResult({
      runId: 'wrap-1',
      targetHeading: PI,
      telemetry: [
        // theta=-π+0.05, dx = -2π+0.05 ≈ -6.23 — peak |·| > π
        { t: 0, x: [0, 0, -PI + 0.05, 0, 0], u: [0, 0.5], y: [], s_remaining: 1 },
      ],
    })
    render(<DiagnosticsPanel history={[wrapping]} />)
    expect(screen.getByRole('alert')).toHaveTextContent(/превышает π/)
  })

  it('does NOT show wrap warning when |dx[θ]| stays within ±π', () => {
    const ok = makeResult({
      runId: 'ok-1',
      targetHeading: 0,
      telemetry: [
        { t: 0, x: [0, 0, 0.5, 0, 0], u: [0, 0.1], y: [], s_remaining: 1 },
      ],
    })
    render(<DiagnosticsPanel history={[ok]} />)
    expect(screen.queryByRole('alert')).toBeNull()
  })
})

describe('DiagnosticsPanel — export', () => {
  let originalCreateObjectURL: typeof URL.createObjectURL
  let originalRevokeObjectURL: typeof URL.revokeObjectURL
  let createSpy: ReturnType<typeof vi.fn>
  let revokeSpy: ReturnType<typeof vi.fn>

  beforeEach(() => {
    originalCreateObjectURL = URL.createObjectURL
    originalRevokeObjectURL = URL.revokeObjectURL
    createSpy = vi.fn(() => 'blob:test-url')
    revokeSpy = vi.fn()
    URL.createObjectURL = createSpy as unknown as typeof URL.createObjectURL
    URL.revokeObjectURL = revokeSpy as unknown as typeof URL.revokeObjectURL
  })

  afterEach(() => {
    URL.createObjectURL = originalCreateObjectURL
    URL.revokeObjectURL = originalRevokeObjectURL
  })

  it('creates a Blob with JSON payload when Export clicked', async () => {
    const r = makeResult({ runId: 'export-test', targetHeading: 0.5 })
    render(<DiagnosticsPanel history={[r]} />)

    fireEvent.click(screen.getByRole('button', { name: /Скачать JSON/ }))

    expect(createSpy).toHaveBeenCalledTimes(1)
    const blob = createSpy.mock.calls[0][0] as Blob
    expect(blob).toBeInstanceOf(Blob)
    expect(blob.type).toBe('application/json')
    // jsdom-Blob: ни .text(), ни Response не возвращают содержимое; используем
    // FileReader — это работает.
    const text = await new Promise<string>((resolve, reject) => {
      const reader = new FileReader()
      reader.onload = () => resolve(reader.result as string)
      reader.onerror = () => reject(reader.error)
      reader.readAsText(blob)
    })
    const parsed = JSON.parse(text)
    expect(parsed.schema_version).toBe('1.0')
    expect(parsed.runs).toHaveLength(1)
    expect(parsed.runs[0].run_id).toBe('export-test')
    expect(parsed.runs[0].request.target_heading).toBe(0.5)
    expect(typeof parsed.exported_at).toBe('string')

    // и URL.revokeObjectURL вызывается, чтобы не утечь Blob
    expect(revokeSpy).toHaveBeenCalledWith('blob:test-url')
  })

  it('button disabled when history empty (no Blob created)', () => {
    render(<DiagnosticsPanel history={[]} />)
    fireEvent.click(screen.getByRole('button', { name: /Скачать JSON/ }))
    expect(createSpy).not.toHaveBeenCalled()
  })
})
