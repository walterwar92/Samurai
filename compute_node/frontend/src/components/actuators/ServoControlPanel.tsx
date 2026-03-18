import { useState, useCallback } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'
import type { HeadState, ArmState } from '@/types/robot'

const ARM_LABELS = ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня']

interface ServoSliderProps {
  label: string
  value: number
  min?: number
  max?: number
  onChange: (v: number) => void
}

function ServoSlider({ label, value, min = 0, max = 180, onChange }: ServoSliderProps) {
  return (
    <div className="space-y-1">
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-muted-foreground">{label}</span>
        <span className="text-[11px] font-mono font-semibold tabular-nums w-10 text-right">
          {Math.round(value)}&deg;
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        value={Math.round(value)}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-full h-1.5 bg-muted rounded-full appearance-none cursor-pointer
                   [&::-webkit-slider-thumb]:appearance-none
                   [&::-webkit-slider-thumb]:w-3.5
                   [&::-webkit-slider-thumb]:h-3.5
                   [&::-webkit-slider-thumb]:rounded-full
                   [&::-webkit-slider-thumb]:bg-primary
                   [&::-webkit-slider-thumb]:cursor-pointer
                   [&::-webkit-slider-thumb]:shadow-sm
                   [&::-moz-range-thumb]:w-3.5
                   [&::-moz-range-thumb]:h-3.5
                   [&::-moz-range-thumb]:rounded-full
                   [&::-moz-range-thumb]:bg-primary
                   [&::-moz-range-thumb]:border-0
                   [&::-moz-range-thumb]:cursor-pointer"
      />
      <div className="flex justify-between text-[9px] text-muted-foreground/50">
        <span>{min}&deg;</span>
        <span>{max}&deg;</span>
      </div>
    </div>
  )
}

interface ServoControlPanelProps {
  head: HeadState | null | undefined
  arm: ArmState | null | undefined
}

export function ServoControlPanel({ head, arm }: ServoControlPanelProps) {
  const [headThrottle, setHeadThrottle] = useState(false)
  const [armThrottle, setArmThrottle] = useState(false)

  const throttledHeadChange = useCallback((value: number) => {
    if (headThrottle) return
    setHeadThrottle(true)
    api.setHeadAngle(value)
    setTimeout(() => setHeadThrottle(false), 80)
  }, [headThrottle])

  const throttledArmChange = useCallback((joint: number, angle: number) => {
    if (armThrottle) return
    setArmThrottle(true)
    api.setArmJoint(joint, angle)
    setTimeout(() => setArmThrottle(false), 80)
  }, [armThrottle])

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
          Сервоприводы
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        {/* Head (Camera) — single servo ch4 */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">Голова (камера) — ch4</span>
            <Button
              variant="outline"
              size="sm"
              className="h-5 px-2 text-[10px]"
              onClick={() => api.centerHead()}
            >
              Центр
            </Button>
          </div>
          <ServoSlider
            label="Угол поворота"
            value={headAngle}
            onChange={(v) => throttledHeadChange(v)}
          />
        </div>

        <Separator />

        {/* Arm — ch0-ch3 */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-xs font-medium">Рука / Клешня — ch0-ch3</span>
            <Button
              variant="outline"
              size="sm"
              className="h-5 px-2 text-[10px]"
              onClick={() => api.homeArm()}
            >
              Домой
            </Button>
          </div>
          {ARM_LABELS.map((label, i) => (
            <ServoSlider
              key={i}
              label={`ch${i}: ${label}`}
              value={armAngles[i]}
              onChange={(v) => throttledArmChange(i + 1, v)}
            />
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
