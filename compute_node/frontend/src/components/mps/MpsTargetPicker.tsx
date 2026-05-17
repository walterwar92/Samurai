import { useEffect, useMemo, useState } from 'react'
import { createPortal } from 'react-dom'
import { X } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { MpsTargetScene } from './MpsTargetScene'
import { formatHeadingLabel } from '@/lib/targetAngle'

interface MpsTargetPickerProps {
  /** Радиус окружности N (= дистанция сценария), м. */
  distance: number
  /** Целевая скорость — показывается в подвале, в прогон уходит как есть. */
  vTarget: number
  /** Подтверждение: пользователь нажал «Старт». Передаёт финальный курс φ (рад) —
   *  куда робот будет смотреть ПОСЛЕ прибытия в (D, 0) локального фрейма старта. */
  onConfirm: (targetHeading: number) => void
  /** Отмена: ✕ или клик по backdrop. Прогон не запускается. */
  onCancel: () => void
}

export function MpsTargetPicker({
  distance,
  vTarget,
  onConfirm,
  onCancel,
}: MpsTargetPickerProps) {
  // φ предвыбран в 0 («прямо») — «Старт» активна сразу.
  const [pickedAngle, setPickedAngle] = useState(0)

  // Анимация появления (как в Mps3DOverlay).
  const [entered, setEntered] = useState(false)
  useEffect(() => {
    const id = requestAnimationFrame(() => setEntered(true))
    return () => cancelAnimationFrame(id)
  }, [])

  const headingLabel = useMemo(() => formatHeadingLabel(pickedAngle), [pickedAngle])

  return createPortal(
    <div
      data-testid="mps-target-backdrop"
      onClick={onCancel}
      className={[
        'fixed inset-0 z-[70] flex items-center justify-center',
        'bg-black/70 transition-opacity duration-200',
        entered ? 'opacity-100' : 'opacity-0',
      ].join(' ')}
    >
      <div
        onClick={(e) => e.stopPropagation()}
        className={[
          'relative w-[640px] h-[520px] overflow-hidden flex flex-col',
          'rounded-lg border border-zinc-700 bg-[#1a1a2e]',
          'transition-transform duration-200 ease-out',
          entered ? 'scale-100' : 'scale-95',
        ].join(' ')}
        role="dialog"
        aria-modal="true"
        aria-label="Выбор цели для робота"
      >
        {/* Шапка */}
        <div className="flex items-center justify-between px-4 py-2 border-b border-zinc-700 bg-zinc-900/80">
          <span className="text-sm font-medium text-zinc-100">Финальный курс после прибытия в (D, 0)</span>
          <button
            type="button"
            onClick={onCancel}
            className="text-zinc-400 hover:text-zinc-100 transition-colors p-1 -m-1"
            aria-label="Закрыть выбор цели"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* 3D-сцена */}
        <div className="flex-1 min-h-0">
          <MpsTargetScene
            distance={distance}
            pickedAngle={pickedAngle}
            onPick={setPickedAngle}
          />
        </div>

        {/* Подвал: курс + дистанция + «Старт» */}
        <div className="flex items-center justify-between px-4 py-3 border-t border-zinc-700 bg-zinc-900/80">
          <div className="text-xs font-mono text-zinc-300">
            Финальный курс φ: <span className="text-cyan-400">{headingLabel}</span>
            {' • '}
            Дистанция: <span className="text-zinc-100">{distance.toFixed(2)} м</span>
            {' • '}
            v_target: <span className="text-zinc-100">{vTarget.toFixed(2)} м/с</span>
          </div>
          <Button size="sm" onClick={() => onConfirm(pickedAngle)}>
            ▶ Старт
          </Button>
        </div>
      </div>
    </div>,
    document.body,
  )
}
