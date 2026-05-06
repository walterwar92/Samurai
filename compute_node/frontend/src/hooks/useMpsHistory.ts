import { useCallback, useEffect, useState } from 'react'
import { mpsApi } from '@/lib/mpsApi'
import type { MpsScenarioResult } from '@/types/mps'

interface UseMpsHistoryResult {
  history: MpsScenarioResult[]
  loading: boolean
  error: string | null
  refresh: () => Promise<void>
  replay: (runId: string) => Promise<MpsScenarioResult | null>
}

export function useMpsHistory(): UseMpsHistoryResult {
  const [history, setHistory] = useState<MpsScenarioResult[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      const r = await mpsApi.history()
      setHistory(r.history)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setLoading(false)
    }
  }, [])

  useEffect(() => {
    void refresh()
  }, [refresh])

  const replay = useCallback(async (runId: string) => {
    try {
      const r = await mpsApi.replay(runId)
      await refresh()
      return r.result
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
      return null
    }
  }, [refresh])

  return { history, loading, error, refresh, replay }
}
