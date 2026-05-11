import { render } from '@testing-library/react'
import { describe, it, expect } from 'vitest'
import { KatanaIcon } from './KatanaIcon'

describe('KatanaIcon', () => {
  it('renders SVG with currentColor stroke', () => {
    const { container } = render(<KatanaIcon className="text-accent h-5 w-5" />)
    const svg = container.querySelector('svg')
    expect(svg).not.toBeNull()
    expect(svg?.getAttribute('stroke')).toBe('currentColor')
    expect(svg?.getAttribute('viewBox')).toBe('0 0 24 24')
  })

  it('applies className prop', () => {
    const { container } = render(<KatanaIcon className="text-accent h-5 w-5" />)
    const svg = container.querySelector('svg')
    expect(svg?.className.baseVal).toContain('text-accent')
    expect(svg?.className.baseVal).toContain('h-5')
  })
})
