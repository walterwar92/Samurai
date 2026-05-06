import { describe, it, expect } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { OdeCard } from './OdeCard'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { buildCanonical, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

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

describe('OdeCard', () => {
  it('renders the title', () => {
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText(/ОДУ-модель робота/i)).toBeInTheDocument()
  })

  it('has 5 equation rows', () => {
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} />
      </MpsHighlightProvider>,
    )
    const rows = screen.getAllByRole('button', { name: /уравнение/i })
    expect(rows.length).toBe(5)
  })

  it('shows numeric coefficients when numeric toggle is on', () => {
    function Wrapper() {
      return (
        <MpsHighlightProvider>
          <OdeCard matrices={makeMatrices()} showNumeric />
        </MpsHighlightProvider>
      )
    }
    const { container } = render(<Wrapper />)
    expect(container.textContent).toMatch(/-6\.67|−6\.67/)
  })

  it('hover row updates Context', () => {
    function ProbeState() {
      const { hovered } = useMpsHighlight()
      return <div data-testid="hl">{String(hovered.equation)}</div>
    }
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} />
        <ProbeState />
      </MpsHighlightProvider>,
    )
    const rows = screen.getAllByRole('button', { name: /уравнение/i })
    fireEvent.mouseEnter(rows[1])
    expect(screen.getByTestId('hl').textContent).toBe('1')
    fireEvent.mouseLeave(rows[1])
    expect(screen.getByTestId('hl').textContent).toBe('null')
  })

  it('shows deviation badge when matrices are non-canonical', () => {
    const m = makeMatrices()
    m.A[0][2] = 0.5
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={m} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText(/нестандартные члены/i)).toBeInTheDocument()
  })
})
