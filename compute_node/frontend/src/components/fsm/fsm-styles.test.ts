import { describe, it, expect } from 'vitest'
import {
  FSM_TEXT_CLASS,
  FSM_BG_CLASS,
  FSM_DOT_HEX,
  FSM_SHORT,
  FSM_ORDER,
} from './fsm-styles'

describe('fsm-styles', () => {
  it('TARGETING uses accent token, not own color', () => {
    expect(FSM_TEXT_CLASS.TARGETING).toBe('text-accent')
    expect(FSM_BG_CLASS.TARGETING).toBe('bg-accent')
  })

  it('IDLE uses fsm-idle muted token', () => {
    expect(FSM_TEXT_CLASS.IDLE).toBe('text-fsm-idle')
    expect(FSM_BG_CLASS.IDLE).toBe('bg-fsm-idle')
  })

  it('FSM_DOT_HEX has valid hex for all 7 states', () => {
    for (const s of FSM_ORDER) {
      expect(FSM_DOT_HEX[s]).toMatch(/^#[0-9A-F]{6}$/i)
    }
  })

  it('FSM_SHORT shortens names to 3-4 chars uppercase', () => {
    expect(FSM_SHORT.IDLE).toBe('IDL')
    expect(FSM_SHORT.SEARCHING).toBe('SRCH')
    expect(FSM_SHORT.RETURNING).toBe('RTN')
  })

  it('FSM_ORDER has 7 unique states in canonical order', () => {
    expect(FSM_ORDER).toHaveLength(7)
    expect(new Set(FSM_ORDER).size).toBe(7)
    expect(FSM_ORDER[0]).toBe('IDLE')
    expect(FSM_ORDER[6]).toBe('RETURNING')
  })
})
