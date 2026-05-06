import { useEffect, useRef } from 'react'
import katex from 'katex'

interface KatexFormulaProps {
  formula: string
  inline?: boolean
  className?: string
}

export function KatexFormula({ formula, inline = false, className }: KatexFormulaProps) {
  const ref = useRef<HTMLSpanElement>(null)

  useEffect(() => {
    if (!ref.current) return
    try {
      katex.render(formula, ref.current, {
        displayMode: !inline,
        throwOnError: false,
        errorColor: '#dc2626',
        strict: 'ignore',
      })
    } catch {
      if (ref.current) {
        ref.current.textContent = formula
        ref.current.style.color = '#dc2626'
      }
    }
  }, [formula, inline])

  return <span ref={ref} className={className} />
}
