import { useCallback, useEffect, useState } from 'react'
import { mpsApi } from '@/lib/mpsApi'
import type { MpsMatrices } from '@/types/mps'

interface UseMpsMatricesResult {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  loading: boolean
  error: string | null
  refresh: () => Promise<void>
  saveDraft: (m: MpsMatrices) => Promise<void>
  apply: () => Promise<void>
  reset: () => Promise<void>
  setDraftLocal: (m: MpsMatrices | null) => void
}

/** Обёртка над /matrices*. Хранит applied + draft, тонкая — без useMemo
 *  для глубокого сравнения матриц (это делает MatrixEditor сам). */
export function useMpsMatrices(): UseMpsMatricesResult {
  const [applied, setApplied] = useState<MpsMatrices | null>(null)
  const [draft, setDraft] = useState<MpsMatrices | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      const r = await mpsApi.getMatrices()
      setApplied(r.applied)
      setDraft(r.draft ?? null)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setLoading(false)
    }
  }, [])

  useEffect(() => {
    void refresh()
  }, [refresh])

  const saveDraft = useCallback(async (m: MpsMatrices) => {
    setError(null)
    try {
      const r = await mpsApi.setDraft(m)
      setDraft(r.matrices)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
      throw e
    }
  }, [])

  const apply = useCallback(async () => {
    setError(null)
    try {
      const r = await mpsApi.applyMatrices(draft ?? undefined)
      setApplied(r.matrices)
      setDraft(null)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
      throw e
    }
  }, [draft])

  const reset = useCallback(async () => {
    setError(null)
    try {
      const r = await mpsApi.resetMatrices()
      setApplied(r.matrices)
      setDraft(null)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
      throw e
    }
  }, [])

  return {
    applied,
    draft,
    loading,
    error,
    refresh,
    saveDraft,
    apply,
    reset,
    setDraftLocal: setDraft,
  }
}
