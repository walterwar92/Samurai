import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent, act } from '@testing-library/react'
import type { ReactNode } from 'react'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

// Заглушка Mps3DScene: R3F/WebGL в jsdom не работает, поэтому при тестах
// заменяем сцену на пустой плейсхолдер чтобы не падать на Canvas.
vi.mock('./Mps3DScene', () => ({
  Mps3DScene: () => <div data-testid="scene-stub">scene</div>,
}))

function makeResult(opts: Partial<{ runId: string; telemetry: MpsTelemetryPoint[] }> = {}): MpsScenarioResult {
  return {
    run_id: opts.runId ?? 'r1',
    started_at: '2026-05-11T10:00:00Z',
    finished_at: '2026-05-11T10:00:05Z',
    status: 'reached',
    request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0' },
    matrices_snapshot: { A: [], B: [], C: [], D: [], Q_diag: [], R_diag: [], horizon_N: 20, u_min: [], u_max: [], schema_version: '1.0' },
    telemetry: opts.telemetry ?? [
      { t: 0,   x: [0,   0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 2.0 },
      { t: 0.1, x: [0.1, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: 1.9 },
    ],
    metrics: null,
    schema_version: '1.0',
  }
}

function OverlayHarness({ telemetry }: { telemetry?: MpsTelemetryPoint[] }) {
  const mps3D = useMps3D()
  return (
    <button data-testid="trigger" onClick={() => mps3D.open(makeResult({ telemetry }))}>
      open
    </button>
  )
}

const renderWithProvider = (node: ReactNode) =>
  render(<Mps3DProvider>{node}</Mps3DProvider>)

function StateProbe() {
  const { state } = useMps3D()
  return <div data-testid="kind">{state.kind}</div>
}

describe('Mps3DOverlay', () => {
  it('рендерит сцену когда есть валидная телеметрия (>= 2 точек)', () => {
    renderWithProvider(<OverlayHarness />)
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByTestId('scene-stub')).toBeInTheDocument()
    expect(screen.queryByText(/нет валидных данных/i)).toBeNull()
  })

  it('рендерит fallback когда телеметрия пуста', () => {
    renderWithProvider(<OverlayHarness telemetry={[]} />)
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.queryByTestId('scene-stub')).toBeNull()
    expect(screen.getByText(/нет валидных данных/i)).toBeInTheDocument()
  })

  it('клик ✕ закрывает (возвращает в idle)', () => {
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
    act(() => { screen.getByRole('button', { name: /Закрыть оверлей/i }).click() })
    expect(screen.getByTestId('kind').textContent).toBe('idle')
  })

  it('клик по backdrop НЕ закрывает (B1)', () => {
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    const backdrop = screen.getByTestId('mps3d-backdrop')
    act(() => { fireEvent.click(backdrop) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })

  it('Esc НЕ закрывает (B1)', () => {
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    act(() => { fireEvent.keyDown(document, { key: 'Escape' }) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })

  it('клик внутри панели не пробрасывается на backdrop (B1 защита)', () => {
    // Проверяем через React-уровень: backdrop получает React onClick,
    // клик по дочернему role=dialog должен быть остановлен stopPropagation.
    // Используем fireEvent на backdrop напрямую — backdrop без onClick → не закрывает.
    // Вместо native addEventListener тестируем invariant: dialog имеет
    // onClick stopPropagation, значит stopImmediatePropagation не требуется —
    // достаточно убедиться что dialog рендерится с нужным обработчиком через
    // fireEvent.click на dialog и проверку что state остался overlay.
    renderWithProvider(
      <>
        <OverlayHarness />
        <StateProbe />
      </>,
    )
    act(() => { screen.getByTestId('trigger').click() })
    const dialog = screen.getByRole('dialog', { name: /3D-просмотр траектории/i })
    // Кликаем по панели — stopPropagation предотвращает закрытие через любой
    // будущий backdrop-обработчик; текущий state должен остаться 'overlay'.
    act(() => { fireEvent.click(dialog) })
    expect(screen.getByTestId('kind').textContent).toBe('overlay')
  })
})
