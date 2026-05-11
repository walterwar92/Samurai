import { useCallback, useEffect, useState } from 'react'

export type Theme = 'dark' | 'light'

const STORAGE_KEY = 'samurai.theme'

/**
 * Управление dark/light темой. Класс `.light` ставится на `<html>`,
 * dark — это отсутствие класса (default). Состояние сохраняется в
 * localStorage. Первое значение — из localStorage, иначе из
 * `prefers-color-scheme`.
 */
export function useTheme() {
  const [theme, setTheme] = useState<Theme>(() => {
    if (typeof window === 'undefined') return 'dark'
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored === 'light' || stored === 'dark') return stored
    if (typeof window.matchMedia === 'function') {
      try {
        if (window.matchMedia('(prefers-color-scheme: light)').matches) return 'light'
      } catch {
        // matchMedia может бросить в jsdom/happy-dom — игнорируем
      }
    }
    return 'dark'
  })

  useEffect(() => {
    document.documentElement.classList.toggle('light', theme === 'light')
    localStorage.setItem(STORAGE_KEY, theme)
  }, [theme])

  const toggle = useCallback(() => {
    setTheme((t) => (t === 'dark' ? 'light' : 'dark'))
  }, [])

  return { theme, toggle, setTheme }
}
