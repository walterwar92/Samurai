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
    expect(screen.getByText(/disconnected/i)).toBeInTheDocument()
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
    expect(screen.getByText(/live/i)).toBeInTheDocument()
    // s = +0.234
    expect(screen.getByText(/\+0\.234/)).toBeInTheDocument()
    // v = +0.118
    expect(screen.getByText(/\+0\.118/)).toBeInTheDocument()
    // idle подпись
    expect(screen.getByText(/idle/i)).toBeInTheDocument()
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
    expect(screen.getByText(/stale/i)).toBeInTheDocument()
    expect(screen.getByText(/5s|5\.4s/i)).toBeInTheDocument()
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
    expect(screen.getByText(/abcdef12/)).toBeInTheDocument()
    expect(screen.queryByText(/^idle\b/i)).not.toBeInTheDocument()
  })
})
