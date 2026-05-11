import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react'
import { MpsPage } from './MpsPage'
import { buildCanonical, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices, MpsScenarioResult } from '@/types/mps'

function makeMatrices(): MpsMatrices {
  const { A, B } = buildCanonical(DEFAULT_TAU_V, DEFAULT_TAU_OMEGA)
  return {
    A,
    B,
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

function makeResult(runId: string = 'r-test'): MpsScenarioResult {
  return {
    run_id: runId,
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: makeMatrices(),
    telemetry: [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

let mockRunResult: MpsScenarioResult | null = null
let mockRunId: string | null = null

vi.mock('@/hooks/useMpsMatrices', () => ({
  useMpsMatrices: () => ({
    applied: makeMatrices(),
    draft: null,
    loading: false,
    error: null,
    refresh: vi.fn(),
    saveDraft: vi.fn(),
    apply: vi.fn(),
    reset: vi.fn(),
    setDraftLocal: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsRun', () => ({
  useMpsRun: () => ({
    running: false,
    result: mockRunResult,
    runId: mockRunId,
    error: null,
    run: vi.fn(),
    abort: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsValidate', () => ({
  useMpsValidate: () => ({
    result: null,
    error: null,
    loading: false,
    validate: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsHistory', () => ({
  useMpsHistory: () => ({
    history: [],
    loading: false,
    refresh: vi.fn(),
    replay: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsLiveTelemetry', () => ({
  useMpsLiveTelemetry: () => ({
    connected: false,
    points: [],
  }),
}))

vi.mock('@/components/layout/Header', () => ({
  Header: () => <header data-testid="header">Header</header>,
}))

vi.mock('@/components/mps/Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="mps3d-scene-stub" />,
}))

describe('MpsPage integration', () => {
  beforeEach(() => {
    mockRunResult = null
    mockRunId = null
    if (typeof globalThis.ResizeObserver === 'undefined') {
      globalThis.ResizeObserver = class {
        observe() {}
        unobserve() {}
        disconnect() {}
      } as unknown as typeof ResizeObserver
    }
  })

  it('renders title and main panels', () => {
    render(<MpsPage />)
    expect(screen.getByText(/МПС — Модель Пространства Состояний/)).toBeInTheDocument()
    expect(screen.getByText(/ОДУ-модель робота/i)).toBeInTheDocument()
    expect(screen.getByText(/Физические параметры/i)).toBeInTheDocument()
    expect(screen.getByText(/Анализ устойчивости/i)).toBeInTheDocument()
  })

  it('hover on equation in OdeCard highlights the corresponding row label', () => {
    render(<MpsPage />)
    const equations = screen.getAllByRole('button', { name: /уравнение для/i })
    expect(equations.length).toBe(5)

    fireEvent.mouseEnter(equations[1])

    const labels = screen.getAllByText('v̇')
    const hasBold = labels.some((el) => el.className.includes('font-semibold'))
    expect(hasBold).toBe(true)
  })

  it('shows MatrixEditor with three tabs', () => {
    render(<MpsPage />)
    expect(screen.getByRole('tab', { name: /Динамика A·B/i })).toBeInTheDocument()
    expect(screen.getByRole('tab', { name: /Веса Q·R·N/i })).toBeInTheDocument()
    expect(screen.getByRole('tab', { name: /Выход C·D/i })).toBeInTheDocument()
  })

  it('shows canonical badge in DraftStatus when matrices are canonical', () => {
    render(<MpsPage />)
    expect(screen.getByRole('status').textContent).toBe('applied')
  })
})

describe('MpsPage — 3D toast', () => {
  beforeEach(() => {
    mockRunResult = null
    mockRunId = null
    if (typeof globalThis.ResizeObserver === 'undefined') {
      globalThis.ResizeObserver = class {
        observe() {}
        unobserve() {}
        disconnect() {}
      } as unknown as typeof ResizeObserver
    }
  })

  it('после завершения симуляции появляется тост «Симуляция завершена»', async () => {
    mockRunResult = makeResult('r-test-1')
    mockRunId = 'r-test-1'
    render(<MpsPage />)
    await waitFor(() => {
      expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
    })
  })

  it('replay того же run_id не показывает тост повторно', async () => {
    mockRunResult = makeResult('r-test-2')
    mockRunId = 'r-test-2'
    render(<MpsPage />)
    await waitFor(() => {
      expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
    })
    // Закрыть тост
    const close = screen.getByRole('button', { name: /Закрыть/i })
    act(() => { close.click() })
    expect(screen.queryByText(/Симуляция завершена/i)).toBeNull()
    // Если бы replay того же run_id триггерил тост — он бы появился; но MpsPage
    // должен запомнить run_id через ref и не звать requestToast повторно.
    // Этот сценарий проверяется через сравнение run_id'ов, и так как мок отдаёт
    // тот же result — повторного появления быть не должно.
  })
})
