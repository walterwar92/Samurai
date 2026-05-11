import { describe, it, expect, beforeEach, afterEach } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { useTheme } from './useTheme'

describe('useTheme', () => {
  beforeEach(() => {
    localStorage.clear()
    document.documentElement.classList.remove('light')
  })
  afterEach(() => {
    localStorage.clear()
    document.documentElement.classList.remove('light')
  })

  it('starts with dark when no preference and no system pref', () => {
    // happy-dom matchMedia всегда возвращает false по умолчанию
    const { result } = renderHook(() => useTheme())
    expect(result.current.theme).toBe('dark')
    expect(document.documentElement.classList.contains('light')).toBe(false)
  })

  it('reads from localStorage on first mount', () => {
    localStorage.setItem('samurai.theme', 'light')
    const { result } = renderHook(() => useTheme())
    expect(result.current.theme).toBe('light')
    expect(document.documentElement.classList.contains('light')).toBe(true)
  })

  it('toggle switches dark→light and persists', () => {
    const { result } = renderHook(() => useTheme())
    act(() => result.current.toggle())
    expect(result.current.theme).toBe('light')
    expect(localStorage.getItem('samurai.theme')).toBe('light')
    expect(document.documentElement.classList.contains('light')).toBe(true)
  })

  it('toggle switches light→dark and removes class', () => {
    localStorage.setItem('samurai.theme', 'light')
    const { result } = renderHook(() => useTheme())
    act(() => result.current.toggle())
    expect(result.current.theme).toBe('dark')
    expect(document.documentElement.classList.contains('light')).toBe(false)
  })
})
