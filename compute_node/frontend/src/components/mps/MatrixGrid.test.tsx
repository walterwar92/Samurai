import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { MatrixGrid } from './MatrixGrid'

const baseProps = {
  matrix: 'A' as const,
  values: [
    [0, 1, 0, 0, 0],
    [0, -6.67, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, -10, 0],
    [0, -1, 0, 0, 0],
  ],
  rowLabels: ['ṡ', 'v̇', 'θ̇', 'ω̇', 'ė_int'],
  colLabels: ['s', 'v', 'θ', 'ω', 'e_int'],
}

describe('MatrixGrid', () => {
  it('renders correct number of input cells', () => {
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    const inputs = screen.getAllByRole('textbox')
    expect(inputs.length).toBe(25)
  })

  it('shows row and column labels', () => {
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText('ṡ')).toBeInTheDocument()
    expect(screen.getByText('e_int')).toBeInTheDocument()
  })

  it('marks dirty cell with data-dirty', () => {
    const applied = baseProps.values.map((row) => row.slice())
    applied[1][1] = -5.0
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} applied={applied} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    const dirtyCells = container.querySelectorAll('[data-dirty="true"]')
    expect(dirtyCells.length).toBe(1)
  })

  it('marks invalid cell with data-invalid', () => {
    const values = baseProps.values.map((row) => row.slice())
    values[0][0] = NaN
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} values={values} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    const invalid = container.querySelectorAll('[data-invalid="true"]')
    expect(invalid.length).toBeGreaterThan(0)
  })

  it('marks non-canonical cell with data-deviation', () => {
    const values = baseProps.values.map((row) => row.slice())
    values[0][2] = 0.5
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} values={values} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    const dev = container.querySelectorAll('[data-deviation="true"]')
    expect(dev.length).toBe(1)
  })

  it('emits onCell on input change', () => {
    const onCell = vi.fn()
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} onCell={onCell} />
      </MpsHighlightProvider>,
    )
    const inputs = screen.getAllByRole('textbox')
    fireEvent.change(inputs[0], { target: { value: '0.5' } })
    expect(onCell).toHaveBeenCalledWith(0, 0, '0.5')
  })

  it('shows canonical marker on 5 cells of A', () => {
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...baseProps} onCell={vi.fn()} />
      </MpsHighlightProvider>,
    )
    const markers = container.querySelectorAll('[data-canonical="true"]')
    expect(markers.length).toBe(5)
  })
})
