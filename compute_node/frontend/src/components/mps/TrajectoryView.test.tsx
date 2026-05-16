import { describe, it, expect, vi } from 'vitest'
import { render, screen, act, fireEvent } from '@testing-library/react'
import type { ReactNode } from 'react'
import { TrajectoryView } from './TrajectoryView'
import { Mps3DProvider, useMps3D } from './Mps3DProvider'
import type { MpsScenarioResult } from '@/types/mps'

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

describe('TrajectoryView — target позиция учитывает target_heading', () => {
  function getSvgState() {
    const svg = screen.getByRole('img', { name: /trajectory top-down/i })
    const circles = Array.from(svg.querySelectorAll('circle'))
    // По styling: первый круг — target (fill=none, stroke=#16a34a).
    const target = circles.find(
      (c) => c.getAttribute('fill') === 'none' && c.getAttribute('stroke') === '#16a34a',
    )!
    // current — fill=#dc2626 и r=5 (легендная точка — r=3).
    const current = circles.find(
      (c) => c.getAttribute('fill') === '#dc2626' && c.getAttribute('r') === '5',
    )
    const path = svg.querySelector('path[stroke="#2563eb"]')
    return {
      tx: parseFloat(target.getAttribute('cx') ?? '0'),
      ty: parseFloat(target.getAttribute('cy') ?? '0'),
      cx: current ? parseFloat(current.getAttribute('cx') ?? '0') : null,
      cy: current ? parseFloat(current.getAttribute('cy') ?? '0') : null,
      d: path?.getAttribute('d') ?? null,
    }
  }

  it('target_heading=0 (legacy): target лежит на оси Y=110 справа от центра', () => {
    render(wrap(<TrajectoryView result={makeResult()} />))
    const { tx, ty } = getSvgState()
    expect(tx).toBeGreaterThan(260) // правее центра X=260
    expect(ty).toBeCloseTo(110, 0)   // на горизонтальной оси
  })

  it('target_heading=π/4: target в правом верхнем квадранте (cy<110)', () => {
    const phi = Math.PI / 4
    const D = 2.0
    const result = makeResult({
      request: { distance: D, v_target: 0.2, source: 'sim', schema_version: '1.0', target_heading: phi },
      telemetry: [
        // финальная точка драйва: s=D, θ=φ → конечная (D·cos φ, D·sin φ)
        { t: 0, x: [0, 0, 0, 0, 0], u: [0, 0], y: [], s_remaining: D },
        { t: 1, x: [D, 0.2, phi, 0, 0], u: [0.2, 0], y: [], s_remaining: 0 },
      ],
    })
    render(wrap(<TrajectoryView result={result} />))
    const { tx, ty, cx, cy } = getSvgState()
    expect(tx).toBeGreaterThan(260)         // правее центра
    expect(ty).toBeLessThan(110)            // выше оси (cy инвертирован)
    // current должна совпасть с target (последняя точка пути == target)
    expect(cx).toBeCloseTo(tx, 1)
    expect(cy).toBeCloseTo(ty, 1)
  })

  it('target_heading=π/4: target внутри viewBox (0..220 по Y)', () => {
    const phi = Math.PI / 4
    const result = makeResult({
      request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0', target_heading: phi },
    })
    render(wrap(<TrajectoryView result={result} />))
    const { ty } = getSvgState()
    expect(ty).toBeGreaterThanOrEqual(0)
    expect(ty).toBeLessThanOrEqual(220)
  })

  it('target_heading=-π/3: target в правом нижнем квадранте (cy>110)', () => {
    const phi = -Math.PI / 3
    const result = makeResult({
      request: { distance: 2.0, v_target: 0.2, source: 'sim', schema_version: '1.0', target_heading: phi },
    })
    render(wrap(<TrajectoryView result={result} />))
    const { ty } = getSvgState()
    expect(ty).toBeGreaterThan(110)
    expect(ty).toBeLessThanOrEqual(220)
  })
})

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
