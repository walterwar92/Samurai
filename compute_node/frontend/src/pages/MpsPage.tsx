import { useEffect, useMemo, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { DraftStatus } from '@/components/mps/DraftStatus'
import { EigenvaluePanel } from '@/components/mps/EigenvaluePanel'
import { HistoryPanel } from '@/components/mps/HistoryPanel'
import { MatrixEditor } from '@/components/mps/MatrixEditor'
import { ResultPlots } from '@/components/mps/ResultPlots'
import { ScenarioControls } from '@/components/mps/ScenarioControls'
import { TrajectoryView } from '@/components/mps/TrajectoryView'
import { TuningSliders } from '@/components/mps/TuningSliders'
import { ValidationBadge } from '@/components/mps/ValidationBadge'
import { useMpsHistory } from '@/hooks/useMpsHistory'
import { useMpsLiveTelemetry } from '@/hooks/useMpsLiveTelemetry'
import { useMpsMatrices } from '@/hooks/useMpsMatrices'
import { useMpsRun } from '@/hooks/useMpsRun'
import { useMpsValidate } from '@/hooks/useMpsValidate'
import type {
  MpsMatrices,
  MpsScenarioRequest,
  MpsScenarioResult,
  ScenarioSource,
} from '@/types/mps'

/**
 * `/mps` — учебно-исследовательский UI для модуля МПС.
 *
 * Layout: левая колонка (40%) = редактор + контролы; правая (60%) =
 * графики и визуализация. Layout адаптивный: на узких экранах два
 * блока ставятся друг под друга (`lg:` breakpoint = 1024px).
 *
 * Все данные через REST `/api/v1/mps/*`. Live-mode — `/ws/mps/telemetry`
 * включается только когда `source='robot'` и есть active run.
 *
 * Контракт payload-ов: docs/mps/api.md.
 */
export function MpsPage() {
  const matricesHook = useMpsMatrices()
  const runHook = useMpsRun()
  const validateHook = useMpsValidate()
  const historyHook = useMpsHistory()

  const [source, setSource] = useState<ScenarioSource>('sim')
  const [compareSelection, setCompareSelection] = useState<MpsScenarioResult[]>([])
  const [primaryResult, setPrimaryResult] = useState<MpsScenarioResult | null>(null)

  // Last-applied auto-validate, чтобы Eigen-панель сразу показывала состояние.
  useEffect(() => {
    if (matricesHook.applied) {
      void validateHook.validate(matricesHook.applied)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [matricesHook.applied])

  // Sync hook → page state.
  useEffect(() => {
    if (runHook.result) setPrimaryResult(runHook.result)
  }, [runHook.result])

  // Live mode только для robot прогонов.
  const liveEnabled = source === 'robot' && runHook.running
  const live = useMpsLiveTelemetry({
    enabled: liveEnabled,
    runId: runHook.result?.run_id,
    onFinished: (r) => setPrimaryResult(r),
    onError: (msg) => console.warn('[MPS WS error]', msg),
  })

  const isClosedLoopStable = validateHook.result?.is_closed_loop_stable ?? true
  const isValid = matricesHook.draft !== null
    ? true   // draft validation в MatrixEditor; здесь — заглушка
    : true

  function handleRun(req: MpsScenarioRequest) {
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
      if (r) {
        setPrimaryResult(r)
      }
    })
  }

  const overlays = useMemo(
    () => compareSelection.filter((r) => r.run_id !== primaryResult?.run_id),
    [compareSelection, primaryResult?.run_id],
  )

  return (
    <div className="min-h-screen p-3 max-w-[1920px] mx-auto">
      <div className="flex items-center justify-between mb-3 gap-2">
        <h1 className="text-2xl font-semibold">МПС — Модель Пространства Состояний</h1>
        <div className="flex items-center gap-2">
          <DraftStatus
            applied={matricesHook.applied}
            draft={matricesHook.draft}
            isStable={isClosedLoopStable}
            isValid={isValid}
          />
          <span className="text-xs text-muted-foreground">
            WS: {live.connected ? 'connected' : 'idle'}
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-[2fr_3fr] gap-3">
        {/* Левая колонка: редактор + контролы */}
        <div className="space-y-3">
          <MatrixEditor
            applied={matricesHook.applied}
            draft={matricesHook.draft}
            onChange={(m: MpsMatrices) => {
              void matricesHook.saveDraft(m)
            }}
            onApply={() => {
              void matricesHook.apply()
            }}
            onValidate={() => {
              void validateHook.validate(matricesHook.draft ?? matricesHook.applied)
            }}
            onReset={() => {
              void matricesHook.reset()
            }}
            saving={matricesHook.loading}
          />

          <Card>
            <CardHeader>
              <CardTitle>Validate</CardTitle>
            </CardHeader>
            <CardContent>
              <ValidationBadge result={validateHook.result} />
            </CardContent>
          </Card>

          <HistoryPanel
            history={historyHook.history}
            onSelect={setPrimaryResult}
            onReplay={handleReplay}
            onCompareChange={setCompareSelection}
            loading={historyHook.loading}
          />
        </div>

        {/* Правая колонка: графики + визуализация */}
        <div className="space-y-3">
          <ScenarioControls
            running={runHook.running}
            onRun={handleRun}
            onAbort={handleAbort}
            source={source}
            onSourceChange={setSource}
          />

          <ResultPlots
            primary={primaryResult}
            overlays={overlays}
            liveTelemetry={liveEnabled ? live.points : undefined}
          />

          <EigenvaluePanel
            open={validateHook.result?.eigenvalues_ad ?? []}
            closed={validateHook.result?.eigenvalues_closed ?? []}
            isPlantStable={validateHook.result?.is_plant_stable ?? true}
            isClosedLoopStable={isClosedLoopStable}
          />

          <TrajectoryView
            result={primaryResult}
            liveTelemetry={liveEnabled ? live.points : undefined}
          />

          <TuningSliders
            applied={matricesHook.applied}
            onSimResult={setPrimaryResult}
            onPromote={(m) => void matricesHook.saveDraft(m)}
          />

          {(matricesHook.error || runHook.error || validateHook.error) && (
            <Card>
              <CardHeader>
                <CardTitle className="text-red-600">Ошибки</CardTitle>
              </CardHeader>
              <CardContent className="text-xs space-y-1">
                {matricesHook.error && <div>matrices: {matricesHook.error}</div>}
                {runHook.error && <div>run: {runHook.error}</div>}
                {validateHook.error && <div>validate: {validateHook.error}</div>}
              </CardContent>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}
