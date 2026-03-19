import { useState, useEffect, useRef, useCallback } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'
import type { HeadState, ArmState } from '@/types/robot'

/**
 * Servo mapping (PCA9685):
 *   CH0 — Основание   home=0   [0; 120]
 *   CH1 — Сустав 1    home=120 [0; 145]
 *   CH2 — Сустав 2    home=0   [0; 180]
 *   CH3 — Клешня      home=0   [0; 180]  (0=открыта, 180=закрыта)
 *   CH4 — Голова       home=0   locked (всегда 0°)
 */

const ARM_JOINTS = [
  { label: 'Основание',  min: 0, max: 120, home: 0   },
  { label: 'Сустав 1',   min: 0, max: 145, home: 120 },
  { label: 'Сустав 2',   min: 0, max: 180, home: 0   },
  { label: 'Клешня',     min: 0, max: 180, home: 0   },
]

const THROTTLE_MS = 80

/* ─── single servo slider with local state ─────────────────────── */

interface ServoSliderProps {
  label: string
  /** value coming from server (remote truth) */
  remoteValue: number
  min?: number
  max?: number
  onCommit: (v: number) => void
  disabled?: boolean
}

function ServoSlider({
  label,
  remoteValue,
  min = 0,
  max = 180,
  onCommit,
  disabled = false,
}: ServoSliderProps) {
  const [local, setLocal] = useState(remoteValue)
  const dragging = useRef(false)

  // sync from server only when NOT dragging
  useEffect(() => {
    if (!dragging.current) setLocal(remoteValue)
  }, [remoteValue])

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (disabled) return
    const v = Number(e.target.value)
    setLocal(v)
    onCommit(v)
  }

  return (
    <div className={`space-y-1 ${disabled ? 'opacity-40' : ''}`}>
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-muted-foreground">{label}</span>
        <span className="text-[11px] font-mono font-semibold tabular-nums w-10 text-right">
          {Math.round(local)}&deg;
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        value={Math.round(local)}
        disabled={disabled}
        onPointerDown={() => { dragging.current = true }}
        onPointerUp={() => { dragging.current = false }}
        onLostPointerCapture={() => { dragging.current = false }}
        onChange={handleChange}
        className="w-full h-2 bg-muted rounded-full appearance-none cursor-pointer
                   disabled:cursor-not-allowed
                   [&::-webkit-slider-thumb]:appearance-none
                   [&::-webkit-slider-thumb]:w-4
                   [&::-webkit-slider-thumb]:h-4
                   [&::-webkit-slider-thumb]:rounded-full
                   [&::-webkit-slider-thumb]:bg-primary
                   [&::-webkit-slider-thumb]:cursor-grab
                   [&::-webkit-slider-thumb]:active:cursor-grabbing
                   [&::-webkit-slider-thumb]:shadow-md
                   [&::-webkit-slider-thumb]:border-2
                   [&::-webkit-slider-thumb]:border-background
                   [&::-moz-range-thumb]:w-4
                   [&::-moz-range-thumb]:h-4
                   [&::-moz-range-thumb]:rounded-full
                   [&::-moz-range-thumb]:bg-primary
                   [&::-moz-range-thumb]:border-2
                   [&::-moz-range-thumb]:border-background
                   [&::-moz-range-thumb]:cursor-grab
                   [&::-moz-range-thumb]:active:cursor-grabbing"
      />
      <div className="flex justify-between text-[9px] text-muted-foreground/50">
        <span>{min}&deg;</span>
        <span>{max}&deg;</span>
      </div>
    </div>
  )
}

/* ─── main panel ───────────────────────────────────────────────── */

interface ServoControlPanelProps {
  head: HeadState | null | undefined
  arm: ArmState | null | undefined
}

export function ServoControlPanel({ head, arm }: ServoControlPanelProps) {
  /* throttle refs — allow first call, then block for THROTTLE_MS */
  const armTimers = useRef<(ReturnType<typeof setTimeout> | null)[]>([null, null, null, null])

  const sendArm = useCallback((joint: number, angle: number) => {
    const idx = joint - 1
    if (armTimers.current[idx]) return
    api.setArmJoint(joint, angle)
    armTimers.current[idx] = setTimeout(() => { armTimers.current[idx] = null }, THROTTLE_MS)
  }, [])

  const headAngle = head?.angle ?? 0
  const armAngles = [
    arm?.j1 ?? 0,
    arm?.j2 ?? 120,
    arm?.j3 ?? 0,
    arm?.j4 ?? 0,
  ]

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Сервоприводы
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        {/* ── Head (Camera) — ch4, locked at 0° ───────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">ch4: Голова (камера)</span>
            <span className="text-[10px] text-muted-foreground">0° (зафикс.)</span>
          </div>
          <ServoSlider
            label="Угол поворота"
            remoteValue={headAngle}
            min={0}
            max={0}
            onCommit={() => {}}
            disabled
          />
        </div>

        <Separator />

        {/* ── Arm — ch0-ch3 ───────────────────────────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">Рука / Клешня — ch0-ch3</span>
            <Button
              variant="outline"
              size="sm"
              className="h-6 px-2 text-[10px]"
              onClick={() => api.homeArm()}
            >
              Домой
            </Button>
          </div>
          {ARM_JOINTS.map((joint, i) => (
            <ServoSlider
              key={i}
              label={`ch${i}: ${joint.label}`}
              remoteValue={armAngles[i]}
              min={joint.min}
              max={joint.max}
              onCommit={(v) => sendArm(i + 1, v)}
            />
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
