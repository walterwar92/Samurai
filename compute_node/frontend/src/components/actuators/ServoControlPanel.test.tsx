import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { ServoControlPanel } from './ServoControlPanel'
import type { HeadState, ArmState } from '@/types/robot'

vi.mock('@/lib/api', () => ({
  api: {
    setArmJoint: vi.fn(),
    setHeadAngle: vi.fn(),
    armCommand: vi.fn(),
    armFreezeJoint: vi.fn(),
    armUnfreezeJoint: vi.fn(),
    headCommand: vi.fn(),
    centerHead: vi.fn(),
    homeArm: vi.fn(),
    armSavePreset: vi.fn(),
    armLoadPreset: vi.fn(),
    armDeletePreset: vi.fn(),
    armListPresets: vi.fn(),
    headSavePreset: vi.fn(),
    headLoadPreset: vi.fn(),
    headDeletePreset: vi.fn(),
    headListPresets: vi.fn(),
  },
}))

import { api } from '@/lib/api'

const head: HeadState = { angle: 90, frozen: false, locked: false }
const headFrozen: HeadState = { angle: 90, frozen: true, locked: false }
const armUnlocked: ArmState = {
  j1: 0, j2: 120, j3: 0, j4: 0,
  frozen: [false, false, false, false],
  locked: false,
}
const armFirstFrozen: ArmState = {
  j1: 50, j2: 120, j3: 0, j4: 0,
  frozen: [true, false, false, false],
  locked: false,
}

describe('ServoControlPanel — frozen slider drag', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('frozen arm slider triggers setArmJoint on change', () => {
    render(<ServoControlPanel head={head} arm={armFirstFrozen} />)
    const armSlider = screen.getByRole('slider', { name: /ch0: Основание/i }) as HTMLInputElement
    expect(armSlider.disabled).toBe(false)

    fireEvent.change(armSlider, { target: { value: '75' } })
    expect(api.setArmJoint).toHaveBeenCalledWith(1, 75)
  })

  it('frozen head slider stays disabled', () => {
    render(<ServoControlPanel head={headFrozen} arm={armUnlocked} />)
    const headSlider = screen.getByRole('slider', { name: /Угол поворота/i }) as HTMLInputElement
    expect(headSlider.disabled).toBe(true)

    fireEvent.change(headSlider, { target: { value: '120' } })
    expect(api.setHeadAngle).not.toHaveBeenCalled()
  })

  it('HOLD badge stays visible on frozen arm slider during interaction', () => {
    render(<ServoControlPanel head={head} arm={armFirstFrozen} />)
    // armFirstFrozen имеет ровно 1 frozen-сустав (ch0).
    const badges = screen.getAllByText('HOLD')
    expect(badges).toHaveLength(1)
  })

  it('non-frozen arm slider still triggers setArmJoint (regression)', () => {
    render(<ServoControlPanel head={head} arm={armUnlocked} />)
    const armSlider = screen.getByRole('slider', { name: /ch0: Основание/i }) as HTMLInputElement
    fireEvent.change(armSlider, { target: { value: '40' } })
    expect(api.setArmJoint).toHaveBeenCalledWith(1, 40)
  })
})
