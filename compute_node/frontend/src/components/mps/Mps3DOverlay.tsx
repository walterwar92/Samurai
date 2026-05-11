// Tailwind v3 has no 'duration-250' step; 200ms is the closest standard token.
import { useEffect, useMemo, useState } from 'react'
import { createPortal } from 'react-dom'
import { X } from 'lucide-react'
import type { MpsScenarioResult, MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'
import { useMps3D } from './Mps3DProvider'
import { Mps3DScene } from './Mps3DScene'

function statusText(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'достигнуто'
    case 'aborted':  return 'прервано'
    case 'timeout':  return 'таймаут'
    case 'error':    return 'ошибка'
    case 'running':  return 'выполняется'
  }
}

function statusColor(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':  return 'text-emerald-400'
    case 'aborted':
    case 'timeout':  return 'text-amber-400'
    case 'error':    return 'text-red-400'
    case 'running':  return 'text-zinc-300'
  }
}

function validTelemetryCount(t: MpsTelemetryPoint[]): number {
  let count = 0
  for (const p of t) {
    if (
      Number.isFinite(p.t) &&
      Number.isFinite(p.x?.[0]) &&
      Number.isFinite(p.x?.[2])
    ) count++
  }
  return count
}

interface OverlayPanelProps {
  result: MpsScenarioResult
  onClose: () => void
}

function OverlayPanel({ result, onClose }: OverlayPanelProps) {
  // Анимация появления: backdrop fade + panel scale-95→100
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  const hasValidTelemetry = useMemo(() => validTelemetryCount(result.telemetry) >= 2, [result.telemetry])

  return (
    <div
      data-testid="mps3d-backdrop"
      className={[
        'fixed inset-0 z-[70] flex items-start justify-center',
        'bg-black/70 transition-opacity duration-200',
        entered ? 'opacity-100' : 'opacity-0',
      ].join(' ')}
      // Клик по backdrop НЕ закрывает (B1). onClick намеренно отсутствует.
    >
      <div
        className={[
          'relative mt-[7.5vh] w-[90vw] h-[85vh] overflow-hidden',
          'rounded-lg border border-zinc-700 bg-[#1a1a2e]',
          'transition-transform duration-200 ease-out',
          entered ? 'scale-100' : 'scale-95',
        ].join(' ')}
        role="dialog"
        aria-label="3D-просмотр траектории"
      >
        {/* Шапка */}
        <div className="absolute top-0 left-0 right-0 z-10 flex items-center justify-between px-4 py-2 border-b border-zinc-700 bg-zinc-900/80 backdrop-blur">
          <div className="flex items-center gap-3">
            <span className="text-sm font-medium text-zinc-100">3D-просмотр траектории</span>
            <span className="text-xs text-zinc-400">
              s = {result.request.distance.toFixed(2)} м •{' '}
              <span className={statusColor(result.status)}>{statusText(result.status)}</span>
            </span>
          </div>
          <button
            type="button"
            onClick={onClose}
            className="text-zinc-400 hover:text-zinc-100 transition-colors p-1 -m-1"
            aria-label="Закрыть оверлей"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* Содержимое: сцена или fallback */}
        <div className="absolute inset-0 pt-10">
          {hasValidTelemetry ? (
            <Mps3DScene
              telemetry={result.telemetry}
              distance={result.request.distance}
              status={result.status}
            />
          ) : (
            <div className="flex h-full items-center justify-center">
              <p className="text-sm text-zinc-400">Нет валидных данных для 3D-визуализации</p>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

export function Mps3DOverlay() {
  const { state, close } = useMps3D()
  if (state.kind !== 'overlay') return null
  return createPortal(
    <OverlayPanel result={state.result} onClose={close} />,
    document.body,
  )
}
