import { render, screen, fireEvent } from '@testing-library/react'
import { describe, it, expect, beforeEach } from 'vitest'
import { MemoryRouter } from 'react-router-dom'
import { Sidebar } from './Sidebar'

function renderWithRouter(initial = '/dashboard') {
  return render(
    <MemoryRouter initialEntries={[initial]}>
      <Sidebar />
    </MemoryRouter>,
  )
}

describe('Sidebar', () => {
  beforeEach(() => {
    localStorage.clear()
    document.documentElement.classList.remove('light')
  })

  it('renders all 6 nav labels when expanded', () => {
    renderWithRouter()
    for (const label of ['Dashboard', 'Admin', '3D View', 'Hardware', 'Samcan', 'MPS']) {
      expect(screen.getByText(label)).toBeInTheDocument()
    }
  })

  it('shows Samurai brand label when expanded', () => {
    renderWithRouter()
    expect(screen.getByText('Samurai')).toBeInTheDocument()
  })

  it('collapse button toggles width and persists', () => {
    const { container } = renderWithRouter()
    const aside = container.querySelector('aside')!
    expect(aside.className).toContain('w-[220px]')

    fireEvent.click(screen.getByLabelText(/collapse sidebar/i))
    expect(aside.className).toContain('w-[56px]')
    expect(localStorage.getItem('samurai.sidebarCollapsed')).toBe('1')

    fireEvent.click(screen.getByLabelText(/collapse sidebar/i))
    expect(aside.className).toContain('w-[220px]')
    expect(localStorage.getItem('samurai.sidebarCollapsed')).toBe('0')
  })

  it('respects initial collapsed state from localStorage', () => {
    localStorage.setItem('samurai.sidebarCollapsed', '1')
    const { container } = renderWithRouter()
    expect(container.querySelector('aside')!.className).toContain('w-[56px]')
  })
})
