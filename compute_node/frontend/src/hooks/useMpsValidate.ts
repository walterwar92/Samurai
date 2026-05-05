import { useCallback, useState } from 'react'
import { mpsApi } from '@/lib/mpsApi'
import type { MpsMatrices, MpsValidateResult } from '@/types/mps'

interface UseMpsValidateResult {
  result: MpsValidateResult | null
  loading: boolean
  error: string | null
  validate: (m?: MpsMatrices | null) => Promise<MpsValidateResult | null>
  clear: () => void
}

export function useMpsValidate(): UseMpsValidateResult {
  const [result, setResult] = useState<MpsValidateResult | null>(null)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const validate = useCallback(async (m?: MpsMatrices | null) => {
    setLoading(true)
    setError(null)
    try {
      const r = await mpsApi.validate(m ?? null)
      setResult(r)
      return r
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
      return null
    } finally {
      setLoading(false)
    }
  }, [])

  const clear = useCallback(() => {
    setResult(null)
    setError(null)
  }, [])

  return { result, loading, error, validate, clear }
}
