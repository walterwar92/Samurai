import { render, screen } from '@testing-library/react'
import { describe, it, expect } from 'vitest'
import { FsmTimeline } from './FsmTimeline'

describe('FsmTimeline', () => {
  it('renders all 7 state shortcuts', () => {
    render(<FsmTimeline current="IDLE" />)
    for (const lbl of ['IDL', 'SRCH', 'TGT', 'APR', 'GRB', 'CAL', 'RTN']) {
      expect(screen.getByText(lbl)).toBeInTheDocument()
    }
  })

  it('marks current state with text-accent', () => {
    render(<FsmTimeline current="TARGETING" />)
    const tgt = screen.getByText('TGT')
    expect(tgt.className).toContain('text-accent')
  })

  it('marks non-current states as foreground-faint', () => {
    render(<FsmTimeline current="TARGETING" />)
    const idl = screen.getByText('IDL')
    expect(idl.className).toContain('text-foreground-faint')
  })
})
