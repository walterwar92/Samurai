import { useCallback, useEffect, useRef, useState } from 'react'
import { cn } from '@/lib/utils'

type Dir = 'F' | 'B' | 'L' | 'R' | 'S'

interface VirtualJoystickProps {
  /** Колбэк при смене направления (срабатывает только когда направление меняется). */
  onDirChange: (dir: Dir) => void
  size?: number
  disabled?: boolean
}

/**
 * Touch/mouse drag-джойстик. Возвращает дискретное направление на основе
 * угла отклонения от центра. Команда отправляется ОДИН раз при смене направления —
 * прошивке Samcan это правильный режим (она держит mode пока не пришлёт STOP).
 */
export function VirtualJoystick({ onDirChange, size = 220, disabled }: VirtualJoystickProps) {
  const ref = useRef<HTMLDivElement>(null)
  const [pos, setPos] = useState<{ x: number; y: number } | null>(null)
  const lastDirRef = useRef<Dir>('S')

  const compute = useCallback(
    (clientX: number, clientY: number) => {
      const el = ref.current
      if (!el) return
      const rect = el.getBoundingClientRect()
      const cx = rect.left + rect.width / 2
      const cy = rect.top + rect.height / 2
      let dx = clientX - cx
      let dy = clientY - cy
      const r = rect.width / 2 - 24
      const dist = Math.hypot(dx, dy)
      if (dist > r) {
        dx = (dx / dist) * r
        dy = (dy / dist) * r
      }
      setPos({ x: dx, y: dy })

      // dead zone
      if (dist < r * 0.25) {
        if (lastDirRef.current !== 'S') {
          lastDirRef.current = 'S'
          onDirChange('S')
        }
        return
      }
      // angle 0=вправо, 90=вниз
      const angle = (Math.atan2(dy, dx) * 180) / Math.PI
      let dir: Dir
      if (angle > -45 && angle <= 45)         dir = 'R'
      else if (angle > 45 && angle <= 135)    dir = 'B' // вниз = назад (работает если DIR_BACKWARD задан)
      else if (angle > -135 && angle <= -45)  dir = 'F'
      else                                    dir = 'L'

      if (dir !== lastDirRef.current) {
        lastDirRef.current = dir
        onDirChange(dir)
      }
    },
    [onDirChange],
  )

  const release = useCallback(() => {
    setPos(null)
    if (lastDirRef.current !== 'S') {
      lastDirRef.current = 'S'
      onDirChange('S')
    }
  }, [onDirChange])

  // Глобальные mouse/touch события чтобы не терять при выходе курсора
  useEffect(() => {
    if (!pos) return
    const onMove = (e: PointerEvent) => compute(e.clientX, e.clientY)
    const onUp = () => release()
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', onUp)
    window.addEventListener('pointercancel', onUp)
    return () => {
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', onUp)
      window.removeEventListener('pointercancel', onUp)
    }
  }, [pos, compute, release])

  const knobX = pos?.x ?? 0
  const knobY = pos?.y ?? 0
  const active = !!pos

  return (
    <div
      ref={ref}
      onPointerDown={e => !disabled && (e.preventDefault(), compute(e.clientX, e.clientY))}
      className={cn(
        'relative rounded-full select-none touch-none',
        'bg-gradient-to-br from-zinc-800 to-zinc-900',
        'border-2 border-border',
        'shadow-[inset_0_4px_24px_rgba(0,0,0,0.6)]',
        disabled && 'opacity-40 cursor-not-allowed',
      )}
      style={{ width: size, height: size }}
    >
      {/* directional hints */}
      <DirHint label="F" angle={0}   color="rgb(34 197 94)" />
      <DirHint label="L" angle={270} color="rgb(56 189 248)" />
      <DirHint label="R" angle={90}  color="rgb(56 189 248)" />
      <DirHint label="B" angle={180} color="rgb(234 179 8)" />

      {/* center crosshair */}
      <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
        <div className="w-[1px] h-full bg-zinc-700/30" />
        <div className="absolute w-full h-[1px] bg-zinc-700/30" />
      </div>

      {/* knob */}
      <div
        className={cn(
          'absolute top-1/2 left-1/2 rounded-full pointer-events-none',
          'bg-gradient-to-br from-primary to-primary/60',
          'shadow-[0_4px_18px_rgba(56,189,248,0.5)]',
          active ? 'transition-none' : 'transition-transform duration-300 ease-out',
        )}
        style={{
          width: 64,
          height: 64,
          transform: `translate(calc(-50% + ${knobX}px), calc(-50% + ${knobY}px))`,
        }}
      />
    </div>
  )
}

function DirHint({ label, angle, color }: { label: string; angle: number; color: string }) {
  return (
    <div
      className="absolute inset-0 flex items-start justify-center pointer-events-none"
      style={{ transform: `rotate(${angle}deg)` }}
    >
      <span
        className="text-[11px] font-bold mt-2"
        style={{ color, transform: `rotate(${-angle}deg)` }}
      >
        {label}
      </span>
    </div>
  )
}
