import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { PhysicsParams } from './PhysicsParams'
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

const DEFAULTS = { tau_v: DEFAULT_TAU_V, tau_omega: DEFAULT_TAU_OMEGA }

describe('PhysicsParams', () => {
  it('renders title', () => {
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getByText(/Физические параметры/i)).toBeInTheDocument()
  })

  it('shows canonical status when matrices are canonical', () => {
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getByText(/Каноническая форма/i)).toBeInTheDocument()
  })

  it('disables sliders and shows restore primary when non_canonical', () => {
    const m = makeMatrices()
    m.A[0][2] = 0.5
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    const sliders = screen.getAllByRole('slider')
    sliders.forEach((s) => expect(s).toBeDisabled())
    expect(screen.getByRole('button', { name: /Восстановить/i })).toBeInTheDocument()
  })

  it('slider τ_v change calls onPatch with updated A and B', () => {
    const onPatch = vi.fn()
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={onPatch}
        defaults={DEFAULTS}
      />,
    )
    const tauVSlider = screen.getByLabelText('τ_v') as HTMLInputElement
    fireEvent.change(tauVSlider, { target: { value: '0.20' } })
    expect(onPatch).toHaveBeenCalled()
    const patched = onPatch.mock.calls[0][0] as MpsMatrices
    expect(patched.A[1][1]).toBeCloseTo(-5.0, 4)
    expect(patched.B[1][0]).toBeCloseTo(5.0, 4)
  })

  it('shows incoherent when A and B disagree', () => {
    const m = makeMatrices()
    m.A[1][1] = -4.0
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getAllByText(/неоднозначно/i).length).toBeGreaterThan(0)
  })

  it('reset button resets specific tau to default', () => {
    const onPatch = vi.fn()
    const m = makeMatrices()
    const built = buildCanonical(0.20, DEFAULT_TAU_OMEGA)
    m.A = built.A
    m.B = built.B
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={onPatch}
        defaults={DEFAULTS}
      />,
    )
    fireEvent.click(screen.getByRole('button', { name: /Сбросить τ_v/i }))
    expect(onPatch).toHaveBeenCalled()
    const patched = onPatch.mock.calls[0][0] as MpsMatrices
    expect(patched.A[1][1]).toBeCloseTo(-1 / DEFAULT_TAU_V, 4)
  })

  it('patchCanonicalCells restores e_int canonical cell A[4][2] = -1 (ė_int = −θ)', () => {
    // Simulate a base matrix where A[4][2] was corrupted (cleared to 0).
    // patchCanonicalCells must restore A[4][2] = -1 (ė_int = −θ, Option D).
    const onPatch = vi.fn()
    const m = makeMatrices()
    m.A[4][2] = 0   // corrupt the correct canonical cell
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={onPatch}
        defaults={DEFAULTS}
      />,
    )
    const tauVSlider = screen.getByLabelText('τ_v') as HTMLInputElement
    fireEvent.change(tauVSlider, { target: { value: '0.20' } })
    expect(onPatch).toHaveBeenCalled()
    const patched = onPatch.mock.calls[0][0] as MpsMatrices
    // e_int canonical cell: A[4][2] = -1 (ė_int = −θ, Option D)
    expect(patched.A[4][2]).toBeCloseTo(-1, 4)
  })
})
