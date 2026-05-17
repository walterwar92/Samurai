import { useEffect, useState } from 'react'
import { createPortal } from 'react-dom'
import { Box, X } from 'lucide-react'
import type { MpsScenarioResult, ScenarioStatus } from '@/types/mps'
import { useMps3D, TOAST_MS } from './Mps3DProvider'

function statusText(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':         return 'достигнуто'
    case 'aborted':         return 'прервано'
    case 'timeout':         return 'таймаут'
    case 'timeout_settle':  return 'таймаут стабилизации'
    case 'error':           return 'ошибка'
    case 'running':         return 'выполняется'
  }
}

function statusColor(s: ScenarioStatus): string {
  switch (s) {
    case 'reached':         return 'text-emerald-400'
    case 'aborted':
    case 'timeout':
    case 'timeout_settle':  return 'text-amber-400'
    case 'error':           return 'text-red-400'
    case 'running':         return 'text-zinc-300'
  }
}

interface ToastCardProps {
  result: MpsScenarioResult
  onOpen: () => void
  onClose: () => void
}

function ToastCard({ result, onOpen, onClose }: ToastCardProps) {
  // Анимация появления: переход с translate-y-4 opacity-0 → translate-y-0 opacity-100.
  // Состояние entered флипается на mount → следующий тик React'а.
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  // Tailwind v3 has no 'duration-250' step; 200ms is the closest standard token.
  return (
    <div
      className={[
        'fixed bottom-4 right-4 z-[60] w-[320px]',
        'rounded border border-zinc-700 bg-zinc-900/95 shadow-xl backdrop-blur',
        'transition-all duration-200 ease-out',
        entered ? 'translate-y-0 opacity-100' : 'translate-y-4 opacity-0',
      ].join(' ')}
      role="status"
      aria-live="polite"
      aria-label="Симуляция завершена"
    >
      <div className="p-3">
        <div className="flex items-start gap-2">
          <Box className="w-4 h-4 mt-0.5 text-zinc-300 shrink-0" />
          <div className="flex-1 min-w-0">
            <div className="text-sm font-medium text-zinc-100">Симуляция завершена</div>
            <div className="text-xs text-zinc-400 mt-0.5">
              s = {result.request.distance.toFixed(2)} м •{' '}
              <span className={statusColor(result.status)}>{statusText(result.status)}</span>
            </div>
          </div>
          <button
            type="button"
            onClick={onClose}
            className="text-zinc-500 hover:text-zinc-200 transition-colors p-0.5 -m-0.5"
            aria-label="Закрыть"
          >
            <X className="w-4 h-4" />
          </button>
        </div>
        <button
          type="button"
          onClick={onOpen}
          className="mt-2 w-full rounded bg-blue-600 hover:bg-blue-500 transition-colors px-3 py-1.5 text-xs font-medium text-white"
        >
          Показать в 3D
        </button>
      </div>
      {/* Прогресс-полоска: убывает за TOAST_MS секунд */}
      <div className="h-0.5 bg-zinc-800 overflow-hidden rounded-b">
        <div
          className="h-full bg-blue-500/70 w-full origin-left"
          style={{
            animation: `mps3d-toast-progress ${TOAST_MS}ms linear forwards`,
          }}
        />
      </div>
      <style>{`
        @keyframes mps3d-toast-progress {
          from { transform: scaleX(1); }
          to   { transform: scaleX(0); }
        }
      `}</style>
    </div>
  )
}

export function Mps3DToast() {
  const { state, open, close } = useMps3D()
  if (state.kind !== 'toasting') return null
  return createPortal(
    <ToastCard
      result={state.result}
      onOpen={() => open(state.result)}
      onClose={close}
    />,
    document.body,
  )
}
