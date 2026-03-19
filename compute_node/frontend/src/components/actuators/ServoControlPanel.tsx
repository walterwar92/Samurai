import { useState, useEffect, useRef, useCallback } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'
import type { HeadState, ArmState } from '@/types/robot'

const ARM_LABELS = ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня']
const THROTTLE_MS = 80

/* ─── single servo slider with local state ─────────────────────── */

interface ServoSliderProps {
  label: string
  /** value coming from server (remote truth) */
  remoteValue: number
  min?: number
  max?: number
  onCommit: (v: number) => void
}

function ServoSlider({
  label,
  remoteValue,
  min = 0,
  max = 180,
  onCommit,
}: ServoSliderProps) {
  const [local, setLocal] = useState(remoteValue)
  const dragging = useRef(false)

  // sync from server only when NOT dragging
  useEffect(() => {
    if (!dragging.current) setLocal(remoteValue)
  }, [remoteValue])

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const v = Number(e.target.value)
    setLocal(v)
    onCommit(v)
  }

  return (
    <div className="space-y-1">
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
        onPointerDown={() => { dragging.current = true }}
        onPointerUp={() => { dragging.current = false }}
        onLostPointerCapture={() => { dragging.current = false }}
        onChange={handleChange}
        className="w-full h-2 bg-muted rounded-full appearance-none cursor-pointer
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
  const headTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const armTimers = useRef<(ReturnType<typeof setTimeout> | null)[]>([null, null, null, null])

  const sendHead = useCallback((value: number) => {
    if (headTimer.current) return
    api.setHeadAngle(value)
    headTimer.current = setTimeout(() => { headTimer.current = null }, THROTTLE_MS)
  }, [])

  const sendArm = useCallback((joint: number, angle: number) => {
    const idx = joint - 1
    if (armTimers.current[idx]) return
    api.setArmJoint(joint, angle)
    armTimers.current[idx] = setTimeout(() => { armTimers.current[idx] = null }, THROTTLE_MS)
  }, [])

  const headAngle = head?.angle ?? 90
  const armAngles = [
    arm?.j1 ?? 90,
    arm?.j2 ?? 90,
    arm?.j3 ?? 90,
    arm?.j4 ?? 90,
  ]

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          🦾 Сервоприводы
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        {/* ── Head (Camera) — ch4 ─────────────────────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">📷 Голова (камера) — ch4</span>
            <Button
              variant="outline"
              size="sm"
              className="h-6 px-2 text-[10px]"
              onClick={() => api.centerHead()}
            >
              Центр
            </Button>
          </div>
          <ServoSlider
            label="Угол поворота"
            remoteValue={headAngle}
            onCommit={sendHead}
          />
        </div>

        <Separator />

        {/* ── Arm — ch0-ch3 ───────────────────────────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">🦾 Рука / Клешня — ch0–ch3</span>
            <Button
              variant="outline"
              size="sm"
              className="h-6 px-2 text-[10px]"
              onClick={() => api.homeArm()}
            >
              Домой
            </Button>
          </div>
          {ARM_LABELS.map((label, i) => (
            <ServoSlider
              key={i}
              label={`ch${i}: ${label}`}
              remoteValue={armAngles[i]}
              onCommit={(v) => sendArm(i + 1, v)}
            />
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
