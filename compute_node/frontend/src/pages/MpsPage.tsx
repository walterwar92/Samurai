import { useEffect, useMemo, useRef, useState } from 'react'
import { PageHeader } from '@/components/layout/PageHeader'
import { CalibrationPanel } from '@/components/controls/CalibrationPanel'
import { DiagnosticsPanel } from '@/components/mps/DiagnosticsPanel'
import { DraftStatus } from '@/components/mps/DraftStatus'
import { EigenvaluePanel } from '@/components/mps/EigenvaluePanel'
import { HistoryPanel } from '@/components/mps/HistoryPanel'
import { LiveStateVector } from '@/components/mps/LiveStateVector'
import { MatrixEditor } from '@/components/mps/MatrixEditor'
import { OdeCard } from '@/components/mps/OdeCard'
import { PhysicsParams } from '@/components/mps/PhysicsParams'
import { ResultPlots } from '@/components/mps/ResultPlots'
import { ScenarioControls } from '@/components/mps/ScenarioControls'
import { TrajectoryView } from '@/components/mps/TrajectoryView'
import { MpsHighlightProvider } from '@/components/mps/HighlightContext'
import { MpsTargetPicker } from '@/components/mps/MpsTargetPicker'
import { Mps3DProvider, useMps3D } from '@/components/mps/Mps3DProvider'
import { useMpsHistory } from '@/hooks/useMpsHistory'
import { useMpsLiveTelemetry } from '@/hooks/useMpsLiveTelemetry'
import { useMpsMatrices } from '@/hooks/useMpsMatrices'
import { useMpsRun } from '@/hooks/useMpsRun'
import { useMpsValidate } from '@/hooks/useMpsValidate'
import { useRobotState } from '@/hooks/useRobotState'
import { detectPhysics, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type {
  MpsMatrices,
  MpsScenarioRequest,
  MpsScenarioResult,
  ScenarioSource,
} from '@/types/mps'

export function MpsPage() {
  return (
    <MpsHighlightProvider>
      <Mps3DProvider>
        <MpsPageInner />
      </Mps3DProvider>
    </MpsHighlightProvider>
  )
}

function MpsPageInner() {
  const matricesHook = useMpsMatrices()
  const runHook = useMpsRun()
  const validateHook = useMpsValidate()
  const historyHook = useMpsHistory()
  const robotState = useRobotState()

  const [source, setSource] = useState<ScenarioSource>('sim')
  const [compareSelection, setCompareSelection] = useState<MpsScenarioResult[]>([])
  const [primaryResult, setPrimaryResult] = useState<MpsScenarioResult | null>(null)
  const [errors, setErrors] = useState<Array<{ id: string; kind: string; msg: string }>>([])
  const [picker, setPicker] = useState<{ distance: number; vTarget: number } | null>(null)

  const mps3D = useMps3D()
  const lastSeenRunIdRef = useRef<string | null>(null)

  useEffect(() => {
    if (matricesHook.applied) {
      void validateHook.validate(matricesHook.applied)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [matricesHook.applied])

  useEffect(() => {
    if (runHook.result) setPrimaryResult(runHook.result)
  }, [runHook.result])

  useEffect(() => {
    if (!primaryResult) return
    if (primaryResult.run_id === lastSeenRunIdRef.current) return
    if ((primaryResult.telemetry?.length ?? 0) < 2) return
    lastSeenRunIdRef.current = primaryResult.run_id
    mps3D.requestToast(primaryResult)
  }, [primaryResult, mps3D])

  useEffect(() => {
    const newErrors: typeof errors = []
    if (matricesHook.error)
      newErrors.push({
        id: `m-${Date.now()}`,
        kind: 'matrices',
        msg: matricesHook.error,
      })
    if (runHook.error)
      newErrors.push({ id: `r-${Date.now()}`, kind: 'run', msg: runHook.error })
    if (validateHook.error)
      newErrors.push({
        id: `v-${Date.now()}`,
        kind: 'validate',
        msg: validateHook.error,
      })
    if (newErrors.length > 0) {
      setErrors((prev) => [...newErrors, ...prev].slice(0, 5))
    }
  }, [matricesHook.error, runHook.error, validateHook.error])

  function dismissError(id: string) {
    setErrors((prev) => prev.filter((e) => e.id !== id))
  }

  const liveEnabled = source === 'robot' && runHook.running
  const live = useMpsLiveTelemetry({
    enabled: liveEnabled,
    // ВАЖНО: используем runHook.runId (выставляется СРАЗУ после POST), а не
    // runHook.result?.run_id, который для robot-mode null до первого polling-
    // тика. Без этого WS подписывался с undefined → телеметрия не маршрутилась
    // → робот в UI не «едет».
    runId: runHook.runId ?? undefined,
    onFinished: (r) => setPrimaryResult(r),
    onError: (msg) => {
      setErrors((prev) => [{ id: `ws-${Date.now()}`, kind: 'ws', msg }, ...prev].slice(0, 5))
    },
  })

  const isClosedLoopStable = validateHook.result?.is_closed_loop_stable ?? true
  const isPlantStable = validateHook.result?.is_plant_stable ?? true

  const canonicalStatus = useMemo(() => {
    const m = matricesHook.draft ?? matricesHook.applied
    return m ? detectPhysics(m).status : undefined
  }, [matricesHook.draft, matricesHook.applied])

  const validationStatus: 'unknown' | 'stable' | 'unstable' = useMemo(() => {
    if (!validateHook.result) return 'unknown'
    return validateHook.result.is_closed_loop_stable ? 'stable' : 'unstable'
  }, [validateHook.result])

  const lastValidatedAt = useMemo(() => {
    return validateHook.result ? new Date().toLocaleTimeString() : null
  }, [validateHook.result])

  const scenarioProgress = useMemo(() => {
    if (!runHook.running) return undefined
    if (live.points.length > 0) {
      const lastPoint = live.points[live.points.length - 1]
      const D = primaryResult?.request.distance ?? runHook.result?.request.distance
      if (!D || D <= 0) return undefined
      const s = lastPoint.x[0] ?? 0
      return Math.min(1, s / D)
    }
    return undefined
  }, [runHook.running, live.points, primaryResult, runHook.result])

  function handleRun(req: MpsScenarioRequest) {
    if (req.source === 'robot') {
      // На роботе — сперва выбор цели в 3D-пикере; прогон по «Старт».
      setPicker({ distance: req.distance, vTarget: req.v_target })
      return
    }
    void runHook.run(req).then((r) => {
      if (r) {
        setPrimaryResult(r)
        void historyHook.refresh()
      }
    })
  }

  function startRobotRun(targetHeading: number) {
    if (!picker) return
    const req: MpsScenarioRequest = {
      distance: picker.distance,
      v_target: picker.vTarget,
      source: 'robot',
      target_heading: targetHeading,
    }
    setPicker(null)
    void runHook.run(req).then((r) => {
      if (r) {
        setPrimaryResult(r)
        void historyHook.refresh()
      }
    })
  }

  function handleAbort() {
    void runHook.abort()
  }

  function handleReplay(runId: string) {
    void historyHook.replay(runId).then((r) => {
      if (r) setPrimaryResult(r)
    })
  }

  const overlays = useMemo(
    () => compareSelection.filter((r) => r.run_id !== primaryResult?.run_id),
    [compareSelection, primaryResult?.run_id],
  )

  return (
    <div className="min-h-screen">
      <PageHeader title="МПС" />
      <div className="p-3 max-w-[1920px] mx-auto">
        <div className="flex items-center justify-between mb-3 gap-2 flex-wrap">
          <h1 className="text-2xl font-semibold">МПС — Модель Пространства Состояний</h1>
          <div className="flex items-center gap-2">
            <DraftStatus
              applied={matricesHook.applied}
              draft={matricesHook.draft}
              isStable={isClosedLoopStable}
              isValid={true}
              canonicalStatus={canonicalStatus}
            />
            <span className="text-xs text-muted-foreground">
              WS: {live.connected ? 'connected' : 'idle'}
            </span>
          </div>
        </div>

        {errors.length > 0 && (
          <div className="space-y-1 mb-3">
            {errors.map((e) => (
              <div
                key={e.id}
                className="flex items-center gap-2 rounded border border-red-300 bg-red-500/10 px-3 py-1.5 text-xs"
                role="alert"
              >
                <span className="text-red-700 font-medium">⚠ {e.kind}:</span>
                <span className="flex-1 text-red-800">{e.msg}</span>
                <button
                  type="button"
                  onClick={() => dismissError(e.id)}
                  className="text-red-600 hover:text-red-900"
                  aria-label="dismiss"
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-[minmax(360px,_35%)_1fr] gap-4">
          <aside className="space-y-3 lg:sticky lg:top-16 lg:self-start lg:max-h-[calc(100vh-5rem)] lg:overflow-y-auto">
            <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
            <PhysicsParams
              applied={matricesHook.applied}
              draft={matricesHook.draft}
              onPatch={(m) => void matricesHook.saveDraft(m)}
              defaults={{ tau_v: DEFAULT_TAU_V, tau_omega: DEFAULT_TAU_OMEGA }}
            />
            <CalibrationPanel
              coeffs={robotState?.calibration_coeffs ?? null}
              profiles={robotState?.calibration_profiles ?? null}
            />
          </aside>

          <main className="space-y-3 min-w-0">
            <MatrixEditor
              applied={matricesHook.applied}
              draft={matricesHook.draft}
              onChange={(m: MpsMatrices) => void matricesHook.saveDraft(m)}
              onApply={() => void matricesHook.apply()}
              onValidate={() =>
                void validateHook.validate(matricesHook.draft ?? matricesHook.applied)
              }
              onReset={() => void matricesHook.reset()}
              saving={matricesHook.loading}
              validationStatus={validationStatus}
            />

            <EigenvaluePanel
              open={validateHook.result?.eigenvalues_ad ?? []}
              closed={validateHook.result?.eigenvalues_closed ?? []}
              isPlantStable={isPlantStable}
              isClosedLoopStable={isClosedLoopStable}
              warnings={validateHook.result?.warnings ?? []}
              lastValidatedAt={lastValidatedAt}
            />

            <div className="grid grid-cols-1 md:grid-cols-[minmax(260px,_32%)_1fr] gap-3 items-start">
              <LiveStateVector />
              <ScenarioControls
                running={runHook.running}
                onRun={handleRun}
                onAbort={handleAbort}
                source={source}
                onSourceChange={setSource}
                progress={scenarioProgress}
              />
            </div>

            <ResultPlots
              primary={primaryResult}
              overlays={overlays}
              liveTelemetry={liveEnabled ? live.points : undefined}
            />

            <TrajectoryView
              result={primaryResult}
              liveTelemetry={liveEnabled ? live.points : undefined}
            />

            <HistoryPanel
              history={historyHook.history}
              onSelect={setPrimaryResult}
              onReplay={handleReplay}
              onCompareChange={setCompareSelection}
              loading={historyHook.loading}
            />

            <DiagnosticsPanel history={historyHook.history} />
          </main>
        </div>
      </div>
      {picker && (
        <MpsTargetPicker
          distance={picker.distance}
          vTarget={picker.vTarget}
          onConfirm={startRobotRun}
          onCancel={() => setPicker(null)}
        />
      )}
    </div>
  )
}
