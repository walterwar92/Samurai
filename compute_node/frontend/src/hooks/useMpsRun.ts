import { useCallback, useEffect, useRef, useState } from 'react'
import { mpsApi } from '@/lib/mpsApi'
import type {
  MpsScenarioRequest,
  MpsScenarioResult,
} from '@/types/mps'

interface UseMpsRunResult {
  result: MpsScenarioResult | null
  running: boolean
  error: string | null
  /** Run scenario; sim returns immediately, robot polls until status≠running. */
  run: (req: MpsScenarioRequest) => Promise<MpsScenarioResult | null>
  abort: () => Promise<void>
  setResult: (r: MpsScenarioResult | null) => void
}

const POLL_INTERVAL_MS = 500
const POLL_MAX_TICKS = 600    // 5 минут потолок (~3·D/v_target всегда меньше)

/** Запуск сценария + автополл для source='robot'. Sim возвращает result сразу. */
export function useMpsRun(): UseMpsRunResult {
  const [result, setResult] = useState<MpsScenarioResult | null>(null)
  const [running, setRunning] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const pollHandle = useRef<ReturnType<typeof setInterval> | null>(null)

  const stopPolling = useCallback(() => {
    if (pollHandle.current !== null) {
      clearInterval(pollHandle.current)
      pollHandle.current = null
    }
  }, [])

  useEffect(() => () => stopPolling(), [stopPolling])

  const run = useCallback(
    async (req: MpsScenarioRequest): Promise<MpsScenarioResult | null> => {
      setError(null)
      setRunning(true)
      try {
        const r = await mpsApi.runScenario(req)
        if (r.result !== null) {
          // sim — синхронно
          setResult(r.result)
          setRunning(false)
          return r.result
        }
        // robot — async polling
        let ticks = 0
        return await new Promise<MpsScenarioResult | null>((resolve) => {
          pollHandle.current = setInterval(async () => {
            ticks += 1
            if (ticks > POLL_MAX_TICKS) {
              stopPolling()
              setRunning(false)
              resolve(null)
              return
            }
            try {
              const status = await mpsApi.scenarioStatus(r.run_id)
              setResult(status)
              if (status.status !== 'running') {
                stopPolling()
                setRunning(false)
                resolve(status)
              }
            } catch (err) {
              // 404 пока run ещё не закоммитился — пропускаем тик.
              if (ticks > 5) {
                setError(err instanceof Error ? err.message : String(err))
              }
            }
          }, POLL_INTERVAL_MS)
        })
      } catch (e) {
        setError(e instanceof Error ? e.message : String(e))
        setRunning(false)
        throw e
      }
    },
    [stopPolling],
  )

  const abort = useCallback(async () => {
    try {
      await mpsApi.abort()
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      stopPolling()
      setRunning(false)
    }
  }, [stopPolling])

  return { result, running, error, run, abort, setResult }
}
