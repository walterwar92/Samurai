import { describe, it, expect } from 'vitest'
import { render } from '@testing-library/react'
import { KatexFormula } from './KatexFormula'

describe('KatexFormula', () => {
  it('renders block formula', () => {
    const { container } = render(<KatexFormula formula="\\dot{s} = v" />)
    const katex = container.querySelector('.katex')
    expect(katex).not.toBeNull()
  })

  it('renders inline formula', () => {
    const { container } = render(<KatexFormula formula="x + 1" inline />)
    const katex = container.querySelector('.katex')
    expect(katex).not.toBeNull()
  })

  it('does not crash on malformed input', () => {
    const { container } = render(<KatexFormula formula="\\frac{" />)
    expect(container).toBeDefined()
  })
})
