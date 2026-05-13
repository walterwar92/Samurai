import { describe, it, expect } from 'vitest'
import {
  BALL_NAMES,
  BALL_HEX,
  BALL_BG_CLASS,
  BALL_TEXT_CLASS,
} from './ball-styles'

describe('ball-styles', () => {
  it('has 7 colors', () => {
    expect(BALL_NAMES).toHaveLength(7)
  })

  it('all hex values are valid 6-digit hex', () => {
    for (const c of BALL_NAMES) {
      expect(BALL_HEX[c]).toMatch(/^#[0-9A-F]{6}$/i)
    }
  })

  it('class maps cover all colors', () => {
    for (const c of BALL_NAMES) {
      expect(BALL_BG_CLASS[c]).toBe(`bg-ball-${c}`)
      expect(BALL_TEXT_CLASS[c]).toBe(`text-ball-${c}`)
    }
  })
})
