import { useCallback, useEffect, useRef, useState } from 'react'
import {
  ArrowUp, Crosshair, Hand, Pause, Play, RotateCcw, Square, Wrench, Zap,
} from 'lucide-react'
import { Header } from '@/components/layout/Header'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import { useVpered } from '@/hooks/useVperedState'
import { cn } from '@/lib/utils'
import { HeadingCompass } from '@/components/vpered/HeadingCompass'
import { DistanceRadar } from '@/components/vpered/DistanceRadar'
import { MotorBars } from '@/components/vpered/MotorBars'
import { VirtualJoystick } from '@/components/vpered/VirtualJoystick'
import { TelemetryHistory } from '@/components/vpered/TelemetryHistory'
import { ArmVisualizer } from '@/components/vpered/ArmVisualizer'
import { VperedEventLog } from '@/components/vpered/VperedEventLog'
import { PresetManager } from '@/components/vpered/PresetManager'
import { ConnectionDiagnostics } from '@/components/vpered/ConnectionDiagnostics'

// ════════════════════════════════════════════════════════════
// SCENARIOS
// ════════════════════════════════════════════════════════════
const SCENARIOS: { id: string; label: string; descr: string; emoji: string }[] = [
  { id: 'fwd_stop',   label: 'Вперёд + стоп', descr: '2.5 с прямо, потом STOP',    emoji: '🏃' },
  { id: 'fwd_back',   label: 'Туда-обратно',  descr: 'вперёд → разворот → вперёд', emoji: '🔁' },
  { id: 'square',     label: 'Квадрат',       descr: '4× (прямо + правый поворот)', emoji: '⬜' },
  { id: 'wiggle',     label: 'Покачать',      descr: 'влево → вправо → центр',      emoji: '🔃' },
  { id: 'open_close', label: 'Open/Close',    descr: 'клешня × 2',                  emoji: '✋' },
  { id: 'grab_demo',  label: 'GRAB demo',     descr: 'open → forward → close → up', emoji: '🤏' },
]

// ════════════════════════════════════════════════════════════
// PAGE
// ════════════════════════════════════════════════════════════
export function VperedPage() {
  const api = useVpered()
  const { state, send, scenario, log } = api
  const tlm = state?.telemetry || {}
  const connected = !!(state?.connected && state?.telemetry_fresh)

  const [armAngle, setArmAngle]  = useState(90)
  const [baseAngle, setBaseAngle] = useState(90)
  const keysPressed = useRef(new Set<string>())

  // Keyboard WASD + Space + O/X/G/P
  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.repeat) return
      if ((e.target as HTMLElement)?.tagName === 'INPUT') return
      const k = e.key.toLowerCase()
      if (keysPressed.current.has(k)) return
      keysPressed.current.add(k)
      switch (k) {
        case 'w': send('F'); break
        case 's': send('S'); break
        case 'a': send('L'); break
        case 'd': send('R'); break
        case ' ': e.preventDefault(); send('S'); break
        case 'o': send('O'); break
        case 'x': send('X'); break
        case 'g': send('G'); break
        case 'p': send('P'); break
      }
    }
    const onKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      keysPressed.current.delete(k)
      if (k === 'w' || k === 'a' || k === 'd') send('S')
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
    }
  }, [send])

  // sync sliders once from telemetry
  const syncedRef = useRef(false)
  useEffect(() => {
    if (syncedRef.current) return
    if (typeof tlm.a === 'number' && typeof tlm.b === 'number') {
      setArmAngle(tlm.a); setBaseAngle(tlm.b)
      syncedRef.current = true
    }
  }, [tlm.a, tlm.b])

  const fetchLog = useCallback(() => log(), [log])

  const mode = tlm.m || 'IDLE'
  const modeStyle =
    mode === 'FWD'  ? 'bg-emerald-500/20 text-emerald-300 border-emerald-500/40' :
    mode === 'LEFT' || mode === 'RIGHT' ? 'bg-sky-500/20 text-sky-300 border-sky-500/40' :
                                            'bg-zinc-500/10 text-zinc-300 border-zinc-500/30'

  return (
    <div className="min-h-screen bg-gradient-to-b from-zinc-950 to-black">
      <Header />

      {/* Heavy banner with live mode */}
      <div className="max-w-[1600px] mx-auto px-3 pt-3">
        <div className="flex items-center justify-between gap-3 rounded-lg border border-border bg-zinc-950/60 backdrop-blur p-3">
          <div className="flex items-center gap-3">
            <div className={cn('px-3 py-1 rounded-md border text-[11px] font-bold tracking-wider uppercase', modeStyle)}>
              {mode}
            </div>
            <div className="flex items-center gap-2 text-[11px] text-muted-foreground">
              <Crosshair className="w-3.5 h-3.5" />
              target θ = 0°
            </div>
            {tlm.ob ? (
              <Badge variant="destructive" className="text-[10px] animate-pulse">⚠ OBSTACLE STOP</Badge>
            ) : null}
          </div>
          <div className="flex items-center gap-2">
            <Badge
              variant={connected ? 'default' : 'destructive'}
              className={cn('text-[10px]', connected && 'bg-emerald-600/80 hover:bg-emerald-600/80')}
            >
              {connected ? `● LIVE · ${state?.port || 'COM?'}` : '○ OFFLINE'}
            </Badge>
            <Button size="sm" variant="destructive" className="h-7 text-xs px-3" onClick={() => send('S')}>
              <Square className="w-3 h-3 mr-1" /> EMERGENCY STOP
            </Button>
          </div>
        </div>
      </div>

      {/* Diagnostics — показывается только когда нет связи */}
      <div className="max-w-[1600px] mx-auto px-3 pt-3">
        <ConnectionDiagnostics api={api} />
      </div>

      {/* Main 3-col grid */}
      <div className="max-w-[1600px] mx-auto p-3 grid grid-cols-1 xl:grid-cols-[1fr_1fr_380px] gap-3">

        {/* ═══════════════ COL 1 — TELEMETRY VISUALS ═══════════════ */}
        <div className="space-y-3">
          {/* Compass + Motor bars side-by-side */}
          <Card className="overflow-hidden">
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold flex items-center gap-2">
                <Crosshair className="w-3 h-3" /> Курсодержание
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="flex items-center gap-3 justify-around">
                <HeadingCompass theta={tlm.th} omega={tlm.om} size={240} />
                <MotorBars left={tlm.L || 0} right={tlm.R || 0} />
              </div>
            </CardContent>
          </Card>

          {/* Distance radar */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Ультразвук вперёд
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <DistanceRadar distance={tlm.d} obstacle={!!tlm.ob} />
            </CardContent>
          </Card>

          {/* Sparklines */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                История (30 с)
              </CardTitle>
            </CardHeader>
            <CardContent className="p-2">
              <TelemetryHistory th={tlm.th} d={tlm.d} L={tlm.L} R={tlm.R} />
            </CardContent>
          </Card>
        </div>

        {/* ═══════════════ COL 2 — CONTROLS ═══════════════ */}
        <div className="space-y-3">
          {/* Joystick */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <div className="flex items-center justify-between">
                <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                  Движение · джойстик или WASD
                </CardTitle>
                <span className="text-[10px] text-muted-foreground">удерживай = едет</span>
              </div>
            </CardHeader>
            <CardContent className="p-4 flex flex-col items-center gap-3">
              <VirtualJoystick onDirChange={dir => send(dir)} disabled={!connected} />
              <div className="grid grid-cols-3 gap-2 w-full max-w-sm">
                <Button size="sm" variant="outline" className="text-xs" onClick={() => send('L')} disabled={!connected}>
                  ◀ L (A)
                </Button>
                <Button size="sm" variant="destructive" className="text-xs" onClick={() => send('S')}>
                  <Pause className="w-3 h-3 mr-1" /> STOP
                </Button>
                <Button size="sm" variant="outline" className="text-xs" onClick={() => send('R')} disabled={!connected}>
                  R (D) ▶
                </Button>
              </div>
            </CardContent>
          </Card>

          {/* Arm + Claw */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold flex items-center gap-2">
                <Hand className="w-3 h-3" /> Рука и клешня
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3 space-y-3">
              <div className="flex items-start gap-3">
                <div className="flex-1">
                  <ArmVisualizer
                    arm={tlm.a ?? 90}
                    base={tlm.b ?? 90}
                    claw={tlm.c ?? 90}
                  />
                </div>
              </div>

              <div className="grid grid-cols-5 gap-1.5">
                <ActionBtn onClick={() => send('O')} color="emerald" label="OPEN" hint="O" disabled={!connected} />
                <ActionBtn onClick={() => send('X')} color="amber"   label="CLOSE" hint="X" disabled={!connected} />
                <ActionBtn onClick={() => send('G')} color="primary" label="GRAB"  hint="G" disabled={!connected} />
                <ActionBtn onClick={() => send('P')} color="sky"     label="PARK"  hint="P" disabled={!connected} />
                <ActionBtn onClick={() => send('D')} color="zinc"    label="DETACH" disabled={!connected} />
              </div>

              <Separator />

              <ServoSlider
                label="ARM"
                value={armAngle}
                onChange={setArmAngle}
                onCommit={v => send('M', v)}
                liveValue={tlm.a}
                disabled={!connected}
              />
              <ServoSlider
                label="BASE"
                value={baseAngle}
                onChange={setBaseAngle}
                onCommit={v => send('N', v)}
                liveValue={tlm.b}
                disabled={!connected}
              />

              <Separator />

              <PresetManager
                api={api}
                liveArm={tlm.a}
                liveBase={tlm.b}
                liveClaw={tlm.c}
                sliderArm={armAngle}
                sliderBase={baseAngle}
                disabled={!connected}
              />
            </CardContent>
          </Card>

          {/* Diagnostics */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold flex items-center gap-2">
                <Wrench className="w-3 h-3" /> Диагностика
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="grid grid-cols-3 gap-1.5">
                <DiagBtn icon={Zap}      label="Calibrate"  onClick={() => send('C')} disabled={!connected} />
                <DiagBtn icon={RotateCcw} label="Zero θ"    onClick={() => send('Z')} disabled={!connected} />
                <DiagBtn icon={Play}     label="Kick"       onClick={() => send('K')} disabled={!connected} />
                <DiagBtn               label="Toggle TLM" onClick={() => send('T')} disabled={!connected} />
                <DiagBtn               label="Help"       onClick={() => send('H')} disabled={!connected} />
                <DiagBtn               label="Emergency"  onClick={() => send('S')} destructive />
              </div>
            </CardContent>
          </Card>
        </div>

        {/* ═══════════════ COL 3 — SIDEBAR ═══════════════ */}
        <div className="space-y-3">
          {/* Quick stats */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Быстрая сводка
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3 grid grid-cols-2 gap-2 text-xs">
              <Stat label="Режим"       value={mode} />
              <Stat label="Порт"        value={state?.port || '—'} mono />
              <Stat label="θ"           value={fmt(tlm.th, '°')} mono />
              <Stat label="ω"           value={fmt(tlm.om, '°/s')} mono />
              <Stat label="Расстояние"  value={fmt(tlm.d, ' см')} mono />
              <Stat label="Препятствие" value={tlm.ob ? '⚠ STOP' : 'нет'} />
              <Stat label="L PWM"       value={fmt(tlm.L)} mono />
              <Stat label="R PWM"       value={fmt(tlm.R)} mono />
            </CardContent>
          </Card>

          {/* Scenarios */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Сценарии
              </CardTitle>
            </CardHeader>
            <CardContent className="p-2 grid grid-cols-1 gap-1.5">
              {SCENARIOS.map(s => (
                <button
                  key={s.id}
                  onClick={() => scenario(s.id)}
                  disabled={!connected}
                  className={cn(
                    'group flex items-center gap-2 rounded-md border border-border/60 bg-gradient-to-r',
                    'from-zinc-900/60 to-zinc-900/30 hover:border-primary/60 hover:from-primary/10',
                    'px-2.5 py-2 transition-all text-left',
                    'disabled:opacity-40 disabled:cursor-not-allowed',
                  )}
                >
                  <span className="text-base">{s.emoji}</span>
                  <div className="min-w-0 flex-1">
                    <div className="text-xs font-semibold truncate">{s.label}</div>
                    <div className="text-[10px] text-muted-foreground truncate">{s.descr}</div>
                  </div>
                  <ArrowUp className="w-3 h-3 rotate-45 opacity-0 group-hover:opacity-100 transition-opacity" />
                </button>
              ))}
            </CardContent>
          </Card>

          {/* Log */}
          <Card>
            <CardHeader className="py-2 px-3 border-b border-border/50">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Событийный лог
              </CardTitle>
            </CardHeader>
            <CardContent className="p-2">
              <VperedEventLog fetchLog={fetchLog} />
            </CardContent>
          </Card>
        </div>
      </div>

      {/* Keyboard hint footer */}
      <div className="max-w-[1600px] mx-auto px-3 pb-4">
        <div className="flex flex-wrap gap-1.5 justify-center text-[10px] text-muted-foreground">
          <Kbd>W</Kbd> вперёд
          <Kbd>A</Kbd> лево
          <Kbd>D</Kbd> право
          <Kbd>Space</Kbd>/<Kbd>S</Kbd> стоп
          <span className="mx-2 text-zinc-700">·</span>
          <Kbd>O</Kbd> open
          <Kbd>X</Kbd> close
          <Kbd>G</Kbd> grab
          <Kbd>P</Kbd> park
        </div>
      </div>
    </div>
  )
}

// ════════════════════════════════════════════════════════════
// SUBCOMPONENTS
// ════════════════════════════════════════════════════════════

function Kbd({ children }: { children: React.ReactNode }) {
  return (
    <kbd className="px-1.5 py-0.5 rounded border border-border bg-zinc-900 text-zinc-300 font-mono text-[10px]">
      {children}
    </kbd>
  )
}

function fmt(v: number | undefined, suffix = ''): string {
  if (v === undefined || v === null) return '—'
  return `${typeof v === 'number' ? v.toFixed(1) : v}${suffix}`
}

function Stat({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className="flex flex-col p-1.5 bg-zinc-900/40 rounded border border-border/40">
      <span className="text-[9px] uppercase tracking-wider text-muted-foreground">{label}</span>
      <span className={cn('text-[12px] font-semibold text-zinc-100 truncate', mono && 'font-mono')}>
        {value}
      </span>
    </div>
  )
}

const COLOR_MAP: Record<string, string> = {
  emerald: 'bg-emerald-600/80 hover:bg-emerald-600 text-white',
  amber:   'bg-amber-600/80 hover:bg-amber-600 text-white',
  primary: 'bg-primary/90 hover:bg-primary text-primary-foreground',
  sky:     'bg-sky-600/80 hover:bg-sky-600 text-white',
  zinc:    'bg-zinc-700/80 hover:bg-zinc-700 text-white',
}

function ActionBtn({
  onClick, color, label, hint, disabled,
}: {
  onClick: () => void; color: string; label: string; hint?: string; disabled?: boolean
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={cn(
        'relative flex flex-col items-center justify-center py-2 rounded-md text-[11px] font-bold tracking-wider',
        'border border-border/60 transition-all shadow-sm',
        'disabled:opacity-40 disabled:cursor-not-allowed',
        COLOR_MAP[color],
      )}
    >
      {label}
      {hint && (
        <span className="absolute top-0.5 right-1 text-[8px] opacity-70 font-mono">{hint}</span>
      )}
    </button>
  )
}

function DiagBtn({
  icon: Icon, label, onClick, disabled, destructive,
}: {
  icon?: React.ComponentType<{ className?: string }>; label: string
  onClick: () => void; disabled?: boolean; destructive?: boolean
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={cn(
        'flex items-center justify-center gap-1 py-1.5 rounded-md border text-[10px] font-medium',
        'border-border/60 bg-zinc-900/40 hover:bg-zinc-800 transition-colors',
        'disabled:opacity-40 disabled:cursor-not-allowed',
        destructive && 'bg-red-900/30 hover:bg-red-900/60 border-red-700/50 text-red-200',
      )}
    >
      {Icon && <Icon className="w-3 h-3" />}
      {label}
    </button>
  )
}

function ServoSlider({
  label, value, onChange, onCommit, liveValue, disabled,
}: {
  label: string; value: number; onChange: (v: number) => void; onCommit: (v: number) => void
  liveValue?: number; disabled?: boolean
}) {
  return (
    <div>
      <div className="flex justify-between items-center mb-1">
        <span className="text-[11px] text-muted-foreground">{label}</span>
        <span className="text-[10px] font-mono text-zinc-400">
          set: <b className="text-primary">{value}°</b>
          {liveValue !== undefined && liveValue !== value && (
            <> · live: <span className="text-zinc-200">{liveValue}°</span></>
          )}
        </span>
      </div>
      <input
        type="range" min={0} max={180} value={value}
        disabled={disabled}
        onChange={e => onChange(parseInt(e.target.value, 10))}
        onMouseUp={() => onCommit(value)}
        onTouchEnd={() => onCommit(value)}
        className="w-full accent-primary disabled:opacity-40"
      />
    </div>
  )
}
