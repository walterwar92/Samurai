import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { render, screen, act } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

function makeResult(runId = 'r1', status: MpsScenarioResult['status'] = 'reached'): MpsScenarioResult {
  return {
    run_id: runId,
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status,
    request: { distance: 2.5, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: [
      { t: 0, x: [0, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.5 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.4 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

// Тестовый компонент, поднимающий тост через useMps3D
function ToastHarness() {
  const mps3D = useMps3D()
  return (
    <button data-testid="trigger" onClick={() => mps3D.requestToast(makeResult('r1'))}>
      trigger
    </button>
  )
}

const renderWithProvider = (children: ReactNode = <ToastHarness />) =>
  render(<Mps3DProvider>{children}</Mps3DProvider>)

beforeEach(() => { vi.useFakeTimers() })
afterEach(() => { vi.useRealTimers() })

describe('Mps3DToast', () => {
  it('появляется в DOM после requestToast', () => {
    renderWithProvider()
    expect(screen.queryByText(/Симуляция завершена/i)).toBeNull()
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByText(/Симуляция завершена/i)).toBeInTheDocument()
  })

  it('клик «Показать в 3D» переводит в overlay-state провайдера', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    renderWithProvider(
      <>
        <ToastHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByTestId('kind').textContent).toBe('toasting')
    act(() => { screen.getByRole('button', { name: /Показать в 3D/i }).click() })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })

  it('клик ✕ возвращает в idle', () => {
    function StateProbe() {
      const { state } = useMps3D()
      return <div data-testid="kind">{state.kind}</div>
    }
    renderWithProvider(
      <>
        <ToastHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    act(() => { screen.getByRole('button', { name: /Закрыть/i }).click() })
    expect(screen.getByTestId('kind').textContent).toBe('idle')
  })

  it('подзаголовок содержит дистанцию и статус', () => {
    renderWithProvider()
    act(() => { screen.getByTestId('trigger').click() })
    // distance=2.5 → "s = 2.50 м"
    expect(screen.getByText(/s = 2\.50 м/)).toBeInTheDocument()
    expect(screen.getByText(/достигнуто/)).toBeInTheDocument()
  })
})
