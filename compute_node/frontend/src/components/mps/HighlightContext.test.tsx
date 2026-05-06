import { describe, it, expect } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'

function Probe() {
  const { hovered, setEquation, setCell } = useMpsHighlight()
  return (
    <div>
      <div data-testid="state">{JSON.stringify(hovered)}</div>
      <button onClick={() => setEquation(1)}>set-eq-1</button>
      <button onClick={() => setEquation(null)}>clear-eq</button>
      <button onClick={() => setCell({ matrix: 'A', row: 1, col: 1 })}>set-cell</button>
    </div>
  )
}

describe('MpsHighlightProvider', () => {
  it('starts with empty state', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBeNull()
    expect(state.cell).toBeNull()
    expect(state.vector).toBeNull()
  })

  it('updates equation index', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-eq-1'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBe(1)
  })

  it('clears equation', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-eq-1'))
    fireEvent.click(screen.getByText('clear-eq'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBeNull()
  })

  it('updates cell', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-cell'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.cell).toEqual({ matrix: 'A', row: 1, col: 1 })
  })
})

describe('useMpsHighlight outside provider', () => {
  it('returns no-op state', () => {
    function NoProvider() {
      const { hovered } = useMpsHighlight()
      return <div data-testid="np">{JSON.stringify(hovered)}</div>
    }
    render(<NoProvider />)
    const state = JSON.parse(screen.getByTestId('np').textContent ?? '{}')
    expect(state.equation).toBeNull()
  })
})
