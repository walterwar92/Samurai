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
 *   CH4 — Голова       home=90  [0; 180]
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
  remoteValue: number
  min?: number
  max?: number
  onCommit: (v: number) => void
  disabled?: boolean
  frozen?: boolean
  onToggleFreeze?: () => void
}

function ServoSlider({
  label,
  remoteValue,
  min = 0,
  max = 180,
  onCommit,
  disabled = false,
  frozen = false,
  onToggleFreeze,
}: ServoSliderProps) {
  const [local, setLocal] = useState(remoteValue)
  const dragging = useRef(false)

  useEffect(() => {
    if (!dragging.current) setLocal(remoteValue)
  }, [remoteValue])

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (disabled || frozen) return
    const v = Number(e.target.value)
    setLocal(v)
    onCommit(v)
  }

  return (
    <div className={`space-y-1 ${disabled ? 'opacity-40' : ''}`}>
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] text-muted-foreground">{label}</span>
          {frozen && (
            <span className="text-[9px] px-1 py-0.5 rounded bg-blue-900/50 text-blue-300 font-medium">
              HOLD
            </span>
          )}
        </div>
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] font-mono font-semibold tabular-nums w-10 text-right">
            {Math.round(local)}&deg;
          </span>
          {onToggleFreeze && (
            <button
              onClick={onToggleFreeze}
              className={`text-[9px] px-1.5 py-0.5 rounded transition-colors ${
                frozen
                  ? 'bg-blue-600 text-white hover:bg-blue-500'
                  : 'bg-zinc-700 text-zinc-400 hover:bg-zinc-600 hover:text-zinc-200'
              }`}
              title={frozen ? 'Разморозить' : 'Заморозить'}
            >
              {frozen ? '❄' : '🔓'}
            </button>
          )}
        </div>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        value={Math.round(local)}
        disabled={disabled || frozen}
        onPointerDown={() => { dragging.current = true }}
        onPointerUp={() => { dragging.current = false }}
        onLostPointerCapture={() => { dragging.current = false }}
        onChange={handleChange}
        className={`w-full h-2 rounded-full appearance-none cursor-pointer
                   disabled:cursor-not-allowed
                   ${frozen ? 'bg-blue-900/40' : 'bg-muted'}
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
                   [&::-moz-range-thumb]:active:cursor-grabbing`}
      />
      <div className="flex justify-between text-[9px] text-muted-foreground/50">
        <span>{min}&deg;</span>
        <span>{max}&deg;</span>
      </div>
    </div>
  )
}

/* ─── preset section (shared for arm and head) ─────────────────── */

interface PresetSectionProps {
  presets: string[]
  onSave: (name: string) => void
  onLoad: (name: string) => void
  onDelete: (name: string) => void
  onRefresh: () => void
}

function PresetSection({ presets, onSave, onLoad, onDelete, onRefresh }: PresetSectionProps) {
  const [showSave, setShowSave] = useState(false)
  const [name, setName] = useState('')

  const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-cyan-500'

  const handleSave = () => {
    if (name.trim()) {
      onSave(name.trim())
      setName('')
      setShowSave(false)
      setTimeout(onRefresh, 500)
    }
  }

  return (
    <div className="space-y-1.5">
      <div className="flex items-center justify-between">
        <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
          Пресеты
        </span>
        <div className="flex gap-1">
          <button
            onClick={() => setShowSave(!showSave)}
            className="text-[10px] px-1.5 py-0.5 rounded bg-zinc-700 text-zinc-300 hover:bg-zinc-600"
          >
            {showSave ? 'Отмена' : '+ Сохранить'}
          </button>
          <button
            onClick={onRefresh}
            className="text-[10px] text-zinc-500 hover:text-zinc-300 px-1"
            title="Обновить"
          >
            ↻
          </button>
        </div>
      </div>

      {showSave && (
        <div className="bg-zinc-800/60 rounded p-2 space-y-1.5">
          <input
            className={inputCls}
            value={name}
            onChange={e => setName(e.target.value)}
            placeholder="Имя пресета"
            onKeyDown={e => e.key === 'Enter' && handleSave()}
          />
          <Button
            size="sm"
            className="text-xs h-7 w-full"
            onClick={handleSave}
            disabled={!name.trim()}
          >
            Сохранить
          </Button>
        </div>
      )}

      {presets.length === 0 ? (
        <span className="text-[10px] text-zinc-600">Нет сохранённых пресетов</span>
      ) : (
        <div className="space-y-1">
          {presets.map(p => (
            <div
              key={p}
              className="flex items-center justify-between p-1.5 rounded text-xs bg-zinc-800/40 border border-zinc-700/30"
            >
              <span className="font-medium text-zinc-200 truncate">{p}</span>
              <div className="flex gap-1 ml-1.5 shrink-0">
                <button
                  onClick={() => onLoad(p)}
                  className="text-[10px] px-1.5 py-0.5 rounded bg-cyan-900/40 text-cyan-300 hover:bg-cyan-800/50"
                >
                  Загр.
                </button>
                <button
                  onClick={() => { onDelete(p); setTimeout(onRefresh, 500) }}
                  className="text-[10px] px-1.5 py-0.5 rounded bg-red-900/40 text-red-400 hover:bg-red-800/50"
                >
                  ✕
                </button>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}

/* ─── status badge ─────────────────────────────────────────────── */

function StatusBadge({ locked, frozen }: { locked: boolean; frozen: boolean }) {
  if (locked) {
    return (
      <span className="text-[9px] px-1.5 py-0.5 rounded bg-amber-900/50 text-amber-300 font-medium">
        LOCKED
      </span>
    )
  }
  if (frozen) {
    return (
      <span className="text-[9px] px-1.5 py-0.5 rounded bg-blue-900/50 text-blue-300 font-medium">
        FROZEN
      </span>
    )
  }
  return (
    <span className="text-[9px] px-1.5 py-0.5 rounded bg-emerald-900/50 text-emerald-300 font-medium">
      ACTIVE
    </span>
  )
}

/* ─── main panel ───────────────────────────────────────────────── */

interface ServoControlPanelProps {
  head: HeadState | null | undefined
  arm: ArmState | null | undefined
  armPresets?: string[]
  headPresets?: string[]
}

export function ServoControlPanel({ head, arm, armPresets = [], headPresets = [] }: ServoControlPanelProps) {
  const armTimers = useRef<(ReturnType<typeof setTimeout> | null)[]>([null, null, null, null])
  const headTimer = useRef<ReturnType<typeof setTimeout> | null>(null)

  // Request preset lists on mount
  useEffect(() => {
    api.armListPresets()
    api.headListPresets()
  }, [])

  const sendArm = useCallback((joint: number, angle: number) => {
    const idx = joint - 1
    if (armTimers.current[idx]) return
    api.setArmJoint(joint, angle)
    armTimers.current[idx] = setTimeout(() => { armTimers.current[idx] = null }, THROTTLE_MS)
  }, [])

  const sendHead = useCallback((angle: number) => {
    if (headTimer.current) return
    api.setHeadAngle(angle)
    headTimer.current = setTimeout(() => { headTimer.current = null }, THROTTLE_MS)
  }, [])

  const headAngle  = head?.angle ?? 90
  const headLocked = head?.locked ?? true
  const headFrozen = head?.frozen ?? false

  const armLocked = arm?.locked ?? true
  const armFrozen = arm?.frozen ?? [false, false, false, false]
  const anyArmFrozen = armFrozen.some(Boolean)

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
        {/* ── Head (Camera) — ch4 ──────────────────────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-1.5">
              <span className="text-xs font-medium">ch4: Голова (камера)</span>
              <StatusBadge locked={headLocked} frozen={headFrozen} />
            </div>
            <div className="flex gap-1">
              {headLocked ? (
                <Button
                  variant="outline"
                  size="sm"
                  className="h-6 px-2 text-[10px]"
                  onClick={() => api.headCommand('unlock')}
                >
                  Разблокировать
                </Button>
              ) : (
                <>
                  <Button
                    variant="outline"
                    size="sm"
                    className="h-6 px-2 text-[10px]"
                    onClick={() => api.centerHead()}
                  >
                    Центр
                  </Button>
                  <Button
                    variant={headFrozen ? 'default' : 'outline'}
                    size="sm"
                    className={`h-6 px-2 text-[10px] ${headFrozen ? 'bg-blue-600 hover:bg-blue-500' : ''}`}
                    onClick={() => api.headCommand(headFrozen ? 'unfreeze' : 'freeze')}
                  >
                    {headFrozen ? 'Разморозить' : 'Заморозить'}
                  </Button>
                </>
              )}
            </div>
          </div>
          <ServoSlider
            label="Угол поворота"
            remoteValue={headAngle}
            min={0}
            max={180}
            onCommit={sendHead}
            disabled={headLocked}
            frozen={headFrozen}
            onToggleFreeze={() => api.headCommand(headFrozen ? 'unfreeze' : 'freeze')}
          />

          {/* Head presets */}
          <PresetSection
            presets={headPresets}
            onSave={(name) => api.headSavePreset(name)}
            onLoad={(name) => api.headLoadPreset(name)}
            onDelete={(name) => api.headDeletePreset(name)}
            onRefresh={() => api.headListPresets()}
          />
        </div>

        <Separator />

        {/* ── Arm — ch0-ch3 ───────────────────────────────── */}
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-1.5">
              <span className="text-xs font-medium">Рука / Клешня — ch0-ch3</span>
              <StatusBadge locked={armLocked} frozen={anyArmFrozen} />
            </div>
            <div className="flex gap-1">
              {armLocked ? (
                <Button
                  variant="outline"
                  size="sm"
                  className="h-6 px-2 text-[10px]"
                  onClick={() => api.armCommand('unlock')}
                >
                  Разблокировать
                </Button>
              ) : (
                <>
                  <Button
                    variant="outline"
                    size="sm"
                    className="h-6 px-2 text-[10px]"
                    onClick={() => api.homeArm()}
                  >
                    Домой
                  </Button>
                  <Button
                    variant={anyArmFrozen ? 'default' : 'outline'}
                    size="sm"
                    className={`h-6 px-2 text-[10px] ${anyArmFrozen ? 'bg-blue-600 hover:bg-blue-500' : ''}`}
                    onClick={() => api.armCommand(anyArmFrozen ? 'unfreeze' : 'freeze')}
                  >
                    {anyArmFrozen ? 'Разм. все' : 'Замор. все'}
                  </Button>
                </>
              )}
            </div>
          </div>
          {ARM_JOINTS.map((joint, i) => (
            <ServoSlider
              key={i}
              label={`ch${i}: ${joint.label}`}
              remoteValue={armAngles[i]}
              min={joint.min}
              max={joint.max}
              onCommit={(v) => sendArm(i + 1, v)}
              disabled={armLocked}
              frozen={armFrozen[i] ?? false}
              onToggleFreeze={() => {
                if (armFrozen[i]) {
                  api.armUnfreezeJoint(i + 1)
                } else {
                  api.armFreezeJoint(i + 1)
                }
              }}
            />
          ))}

          {/* Arm presets */}
          <Separator />
          <PresetSection
            presets={armPresets}
            onSave={(name) => api.armSavePreset(name)}
            onLoad={(name) => api.armLoadPreset(name)}
            onDelete={(name) => api.armDeletePreset(name)}
            onRefresh={() => api.armListPresets()}
          />
        </div>
      </CardContent>
    </Card>
  )
}
