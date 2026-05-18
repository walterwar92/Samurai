import { describe, it, expect, vi } from 'vitest'
import { render, screen } from '@testing-library/react'
import { LiveStateVector } from './LiveStateVector'
import type { UseMpsLiveStateResult } from '@/hooks/useMpsLiveState'

vi.mock('@/hooks/useMpsLiveState', () => ({
  useMpsLiveState: () => mockResult,
}))

let mockResult: UseMpsLiveStateResult = {
  point: null,
  connected: false,
  stale: false,
  ageMs: null,
}

describe('LiveStateVector', () => {
  it('point=null + disconnected: рендерит «—» и бейдж disconnected', () => {
    mockResult = { point: null, connected: false, stale: false, ageMs: null }
    render(<LiveStateVector />)
    expect(screen.getByTestId('status-badge').textContent).toMatch(/disconnected/i)
    // «—» появляется в каждой строке вектора
    expect(screen.getAllByText('—').length).toBeGreaterThan(0)
  })

  it('живой фрейм: рендерит числа и бейдж live', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.0],
        u: [0.12, 0.0],
        scenario_active: false,
        run_id: null,
        schema_version: '1.0',
      },
      connected: true,
      stale: false,
      ageMs: 100,
    }
    render(<LiveStateVector />)
    expect(screen.getByTestId('status-badge').textContent).toMatch(/live/i)
    // s = +0.234
    expect(screen.getByText(/\+0\.234/)).toBeInTheDocument()
    // v = +0.118
    expect(screen.getByText(/\+0\.118/)).toBeInTheDocument()
    // idle + обновлено в подписи снизу
    expect(screen.getByTestId('scenario-caption').textContent).toMatch(/idle/i)
    expect(screen.getByTestId('scenario-caption').textContent).toMatch(/обновлено/)
  })

  it('stale: бейдж stale показывает возраст в секундах', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.0],
        u: [0.12, 0.0],
        scenario_active: false,
        run_id: null,
        schema_version: '1.0',
      },
      connected: true,
      stale: true,
      ageMs: 5_400,
    }
    render(<LiveStateVector />)
    const badge = screen.getByTestId('status-badge')
    expect(badge.textContent).toMatch(/stale/i)
    expect(badge.textContent).toMatch(/5\.4s/)
    // И подпись снизу тоже содержит возраст.
    expect(screen.getByTestId('scenario-caption').textContent).toMatch(/5\.4s/)
  })

  it('scenario_active=true: показывает префикс с обрезанным run_id', () => {
    mockResult = {
      point: {
        ts: 1.0,
        x: [0.234, 0.118, 0.05, 0.215, 0.42],
        u: [0.12, 0.05],
        scenario_active: true,
        run_id: 'abcdef1234567890',
        schema_version: '1.0',
      },
      connected: true,
      stale: false,
      ageMs: 100,
    }
    render(<LiveStateVector />)
    const caption = screen.getByTestId('scenario-caption')
    expect(caption.textContent).toMatch(/abcdef12/)
    expect(caption.textContent).not.toMatch(/^idle\b/i)
  })
})
