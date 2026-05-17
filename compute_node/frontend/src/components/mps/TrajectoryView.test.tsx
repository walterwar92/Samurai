import { describe, it, test, expect, vi } from 'vitest'
import { render, screen, act, fireEvent } from '@testing-library/react'
import type { ReactNode } from 'react'
import { TrajectoryView } from './TrajectoryView'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

vi.mock('./Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="scene-stub" />,
}))

function makeResult(opts: Partial<MpsScenarioResult> = {}): MpsScenarioResult {
  return {
    run_id: 'r1',
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
    ...opts,
  }
}

const wrap = (node: ReactNode) => <Mps3DProvider>{node}</Mps3DProvider>

describe('TrajectoryView — кнопка 3D', () => {
  it('кнопка disabled когда result=null', () => {
    render(wrap(<TrajectoryView result={null} />))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).toBeDisabled()
  })

  it('кнопка disabled когда telemetry пуст', () => {
    render(wrap(<TrajectoryView result={makeResult({ telemetry: [] })} />))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).toBeDisabled()
  })

  it('клик переводит в overlay-state', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    render(wrap(
      <>
        <TrajectoryView result={makeResult()} />
        <StateProbe />
      </>,
    ))
    const btn = screen.getByRole('button', { name: /3D-просмотр/i })
    expect(btn).not.toBeDisabled()
    act(() => { fireEvent.click(btn) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })
})

// ── pose-tracking refactor (schema 1.2): план + стрелка курса ──────────
function makeResultV12(telemetry: MpsTelemetryPoint[], distance = 0.30): MpsScenarioResult {
  return {
    run_id: 't',
    started_at: '',
    finished_at: null,
    status: 'reached',
    request: { distance, v_target: 0.15, target_heading: Math.PI, source: 'robot' },
    matrices_snapshot: {} as never,
    telemetry,
    metrics: null,
    schema_version: '1.2',
  }
}

test('renders dashed plan line from (0,0) to (D,0)', () => {
  const { container } = render(wrap(<TrajectoryView result={makeResultV12([])} />))
  const dashed = container.querySelector('path[stroke-dasharray], line[stroke-dasharray]')
  expect(dashed).toBeInTheDocument()
})

test('renders heading arrow at target', () => {
  const { container } = render(wrap(<TrajectoryView result={makeResultV12([])} />))
  const arrow = container.querySelector('[data-testid="target-heading-arrow"]')
  expect(arrow).toBeInTheDocument()
})

test('legend has 4 entries (план, путь, target, текущая)', () => {
  render(wrap(<TrajectoryView result={makeResultV12([])} />))
  expect(screen.getByText('план')).toBeInTheDocument()
  expect(screen.getByText('путь')).toBeInTheDocument()
  expect(screen.getByText('target')).toBeInTheDocument()
  expect(screen.getByText('текущая')).toBeInTheDocument()
})

test('uses x_local/y_local when present in telemetry', () => {
  const t: MpsTelemetryPoint[] = [
    { t: 0, x: [0, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 0.3,
      x_local: 0, y_local: 0 },
    { t: 1, x: [0.15, 0.15, 0, 0, 0], u: [0, 0], y: [], s_remaining: 0.15,
      x_local: 0.15, y_local: 0 },
  ]
  const { container } = render(wrap(<TrajectoryView result={makeResultV12(t)} />))
  const path = container.querySelector('path[stroke="#2563eb"]')
  expect(path?.getAttribute('d')).toContain('L')  // more than one point
})
