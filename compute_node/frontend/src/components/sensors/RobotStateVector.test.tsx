import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen } from '@testing-library/react'
import { RobotStateVector } from './RobotStateVector'
import type { RobotLiveStatePoint } from '@/types/robot'
import type { UseRobotLiveStateResult } from '@/hooks/useRobotLiveState'

vi.mock('@/hooks/useRobotLiveState')
import { useRobotLiveState } from '@/hooks/useRobotLiveState'
const mockHook = vi.mocked(useRobotLiveState)

const POINT: RobotLiveStatePoint = {
  ts: 1747574400,
  pose: { x: 0.234, y: 0.118, yaw_rad: -0.047, yaw_deg: -2.7 },
  vel: { linear: 0.118, angular: 0.215 },
  imu: {
    ypr_deg: [-2.7, 0.5, 0.0],
    gyro: [0.010, 0.020, 0.210],
    accel: [0.050, 0.020, 9.810],
    ekf_bias_deg: [0.06, -0.11, 0.17],
    has_ekf: true,
  },
  stationary: false,
  schema_version: '1.0',
}

function setHook(partial: Partial<UseRobotLiveStateResult>): void {
  mockHook.mockReturnValue({
    point: null,
    connected: false,
    stale: false,
    ageMs: null,
    ...partial,
  })
}

beforeEach(() => {
  mockHook.mockReset()
})

describe('RobotStateVector', () => {
  it('renders placeholder when point=null', () => {
    setHook({ point: null, connected: false })
    render(<RobotStateVector />)
    expect(screen.getByText('Состояние робота')).toBeInTheDocument()
    // Все числовые поля — «—»
    expect(screen.getAllByText('—').length).toBeGreaterThan(3)
    expect(screen.getByText(/disconnected/i)).toBeInTheDocument()
  })

  it('renders live frame with formatted values', () => {
    setHook({ point: POINT, connected: true, stale: false, ageMs: 100 })
    render(<RobotStateVector />)
    expect(screen.getByText(/\+0\.234/)).toBeInTheDocument()  // x
    expect(screen.getAllByText(/\+0\.118/).length).toBeGreaterThan(0)  // y (или vel.linear)
    expect(screen.getAllByText(/−2\.7°/).length).toBeGreaterThan(0)    // yaw_deg
    expect(screen.getByText(/EKF активен/)).toBeInTheDocument()
    expect(screen.getByText(/● live/)).toBeInTheDocument()
  })

  it('shows stale badge when stale=true', () => {
    setHook({ point: POINT, connected: true, stale: true, ageMs: 5300 })
    render(<RobotStateVector />)
    expect(screen.getByText(/stale/i)).toBeInTheDocument()
  })

  it('orientation header changes when has_ekf=false', () => {
    const noEkf: RobotLiveStatePoint = {
      ...POINT,
      imu: { ...POINT.imu, has_ekf: false, ekf_bias_deg: null },
    }
    setHook({ point: noEkf, connected: true })
    render(<RobotStateVector />)
    expect(screen.getByText(/raw fallback/i)).toBeInTheDocument()
  })

  it('bias row shows "—" when ekf_bias_deg=null', () => {
    const noBias: RobotLiveStatePoint = {
      ...POINT,
      imu: { ...POINT.imu, has_ekf: false, ekf_bias_deg: null },
    }
    setHook({ point: noBias, connected: true })
    render(<RobotStateVector />)
    // В секции bias должны быть прочерки.
    const biasRow = screen.getByText(/bias/i).parentElement
    expect(biasRow?.textContent).toMatch(/—/)
  })

  it('stationary=true shows STATIONARY label', () => {
    const stationary: RobotLiveStatePoint = { ...POINT, stationary: true }
    setHook({ point: stationary, connected: true })
    render(<RobotStateVector />)
    expect(screen.getByText(/STATIONARY/)).toBeInTheDocument()
  })
})
