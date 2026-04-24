import { useEffect, useRef, useState } from 'react'
import { Header } from '@/components/layout/Header'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useVpered } from '@/hooks/useVperedState'
import { cn } from '@/lib/utils'

const SCENARIOS: { id: string; label: string; descr: string }[] = [
  { id: 'fwd_stop',   label: 'Вперёд + стоп',   descr: '2.5 сек прямо, потом STOP' },
  { id: 'fwd_back',   label: 'Туда-обратно',    descr: 'вперёд → разворот → вперёд' },
  { id: 'square',     label: 'Квадрат',         descr: '4× (вперёд → правый поворот)' },
  { id: 'wiggle',     label: 'Покачать',        descr: 'влево → вправо → центр' },
  { id: 'open_close', label: 'Open/Close клешня', descr: 'открыть → закрыть × 2' },
  { id: 'grab_demo',  label: 'GRAB sequence',   descr: 'open → forward → close → up' },
]

export function VperedPage() {
  const { state, send, scenario, log } = useVpered()
  const [armAngle, setArmAngle] = useState(90)
  const [baseAngle, setBaseAngle] = useState(90)
  const [logLines, setLogLines] = useState<string[]>([])
  const tlm = state?.telemetry || {}
  const connected = !!(state?.connected && state?.telemetry_fresh)
  const [keysPressed] = useState(() => new Set<string>())

  // Keyboard control: WASD + space to stop, O/X для клешни, G grab
  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.repeat) return
      const k = e.key.toLowerCase()
      if (keysPressed.has(k)) return
      keysPressed.add(k)
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
      keysPressed.delete(k)
      // Авто-стоп для w/a/d (отпустил клавишу — стоп)
      if (k === 'w' || k === 'a' || k === 'd') send('S')
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
    }
  }, [send, keysPressed])

  // Sync sliders from telemetry on first fresh read
  const syncedRef = useRef(false)
  useEffect(() => {
    if (syncedRef.current) return
    if (typeof tlm.a === 'number' && typeof tlm.b === 'number') {
      setArmAngle(tlm.a)
      setBaseAngle(tlm.b)
      syncedRef.current = true
    }
  }, [tlm.a, tlm.b])

  const refreshLog = async () => setLogLines(await log())

  return (
    <div className="min-h-screen">
      <Header />

      <div className="max-w-[1400px] mx-auto p-3 grid grid-cols-1 lg:grid-cols-[1fr_360px] gap-3">
        {/* ========== LEFT — Controls ========== */}
        <div className="space-y-3">
          {/* Connection */}
          <Card>
            <CardHeader className="py-3 px-4">
              <div className="flex items-center justify-between">
                <CardTitle className="text-base">Vpered (Arduino Uno via USB)</CardTitle>
                <Badge variant={connected ? 'default' : 'destructive'} className="text-[10px]">
                  {connected ? `ONLINE · ${state?.port || 'COM?'}` : 'OFFLINE'}
                </Badge>
              </div>
            </CardHeader>
          </Card>

          {/* Drive controls */}
          <Card>
            <CardHeader className="py-2 px-3">
              <div className="flex items-center justify-between">
                <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                  Движение (или клавиатура: W A S D / Space)
                </CardTitle>
                <span className="text-[10px] text-muted-foreground">
                  держи кнопку, отпусти = стоп
                </span>
              </div>
            </CardHeader>
            <CardContent className="p-4">
              <div className="grid grid-cols-3 gap-2 max-w-xs mx-auto">
                <div />
                <DriveBtn label="▲ Вперёд" cmd="F" stopOnRelease send={send} />
                <div />
                <DriveBtn label="◀ Лево" cmd="L" stopOnRelease send={send} />
                <DriveBtn label="■ STOP" cmd="S" send={send} variant="destructive" />
                <DriveBtn label="Право ▶" cmd="R" stopOnRelease send={send} />
                <div />
                <DriveBtn label="▼ Назад" cmd="B" stopOnRelease send={send} disabled />
                <div />
              </div>
              <div className="text-[10px] text-muted-foreground mt-3 text-center">
                «Назад» отключено: для реверса нужно перепрошить биты 74HC595.
              </div>
            </CardContent>
          </Card>

          {/* Claw + Arm */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Клешня и рука
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3 space-y-3">
              <div className="flex gap-2 flex-wrap">
                <Button size="sm" onClick={() => send('O')}>Open (O)</Button>
                <Button size="sm" onClick={() => send('X')}>Close (X)</Button>
                <Button size="sm" variant="default" onClick={() => send('G')}>GRAB (G)</Button>
                <Button size="sm" variant="outline" onClick={() => send('P')}>Park (P)</Button>
                <Button size="sm" variant="outline" onClick={() => send('D')}>Detach</Button>
              </div>

              <Separator />

              <ServoSlider
                label="ARM"
                value={armAngle}
                onChange={setArmAngle}
                onCommit={v => send('M', v)}
                liveValue={tlm.a}
              />
              <ServoSlider
                label="BASE"
                value={baseAngle}
                onChange={setBaseAngle}
                onCommit={v => send('N', v)}
                liveValue={tlm.b}
              />
            </CardContent>
          </Card>

          {/* Scenarios */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Сценарии
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
                {SCENARIOS.map(s => (
                  <button
                    key={s.id}
                    onClick={() => scenario(s.id)}
                    disabled={!connected}
                    className={cn(
                      'text-left rounded-md border border-border bg-muted/20 hover:bg-accent/20',
                      'px-3 py-2 transition-colors disabled:opacity-40 disabled:cursor-not-allowed',
                    )}
                  >
                    <div className="text-xs font-medium">{s.label}</div>
                    <div className="text-[10px] text-muted-foreground">{s.descr}</div>
                  </button>
                ))}
              </div>
            </CardContent>
          </Card>

          {/* Diag */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Диагностика
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3 flex gap-2 flex-wrap">
              <Button size="sm" variant="outline" onClick={() => send('C')} disabled={!connected}>
                Калибровка (C)
              </Button>
              <Button size="sm" variant="outline" onClick={() => send('Z')} disabled={!connected}>
                Zero heading (Z)
              </Button>
              <Button size="sm" variant="outline" onClick={() => send('K')} disabled={!connected}>
                Kick (K)
              </Button>
              <Button size="sm" variant="outline" onClick={() => send('T')} disabled={!connected}>
                Toggle telemetry (T)
              </Button>
              <Button size="sm" variant="outline" onClick={() => send('H')} disabled={!connected}>
                Help (H)
              </Button>
            </CardContent>
          </Card>
        </div>

        {/* ========== RIGHT — Telemetry ========== */}
        <div className="space-y-3">
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Телеметрия
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="space-y-1.5 text-xs">
                <Row label="Режим" value={tlm.m || '—'} mono />
                <Row label="θ (heading)" value={fmt(tlm.th, '°')} mono />
                <Row label="ω (фильтр)" value={fmt(tlm.om, '°/s')} mono />
                <Row label="Расстояние" value={fmt(tlm.d, ' см')} mono />
                <Row label="L PWM" value={fmt(tlm.L)} mono />
                <Row label="R PWM" value={fmt(tlm.R)} mono />
                <Row label="ARM/BASE/CLAW" value={`${tlm.a ?? '—'} / ${tlm.b ?? '—'} / ${tlm.c ?? '—'}`} mono />
                <Row label="Препятствие" value={tlm.ob ? '⚠ STOP' : 'свободно'} />
              </div>
            </CardContent>
          </Card>

          {/* Log */}
          <Card>
            <CardHeader className="py-2 px-3">
              <div className="flex items-center justify-between">
                <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                  Serial log
                </CardTitle>
                <Button size="sm" variant="outline" className="h-6 text-[10px]" onClick={refreshLog}>
                  Обновить
                </Button>
              </div>
            </CardHeader>
            <CardContent className="p-0">
              <ScrollArea className="h-[280px]">
                <pre className="p-3 text-[10px] font-mono leading-tight text-zinc-300 whitespace-pre-wrap">
                  {logLines.length ? logLines.join('\n') : 'нажми «Обновить» чтобы получить лог'}
                </pre>
              </ScrollArea>
            </CardContent>
          </Card>

          {/* Hints */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Подсказки
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3 text-xs text-muted-foreground space-y-1.5">
              <p>Перед первой ездой нажми <b>Калибровка (C)</b> — робот стоит на полу,
                сделает калибровку гироскопа + автокалибровку знака.</p>
              <p>Если ARM падает после простоя — серво авто-detach. Жми Park чтобы
                поднять обратно.</p>
              <p><b>WASD</b> на клавиатуре: удерживай — едет, отпустил — стоп.</p>
              <p>Запуск bridge: <code>python compute_node/vpered_bridge.py --port COM3</code></p>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  )
}

/* ====================== HELPERS ====================== */

function fmt(v: number | undefined, suffix = ''): string {
  if (v === undefined || v === null) return '—'
  return `${typeof v === 'number' ? v.toFixed(1) : v}${suffix}`
}

function Row({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className="flex justify-between gap-2">
      <span className="text-muted-foreground">{label}</span>
      <span className={cn(mono && 'font-mono', 'text-zinc-200')}>{value}</span>
    </div>
  )
}

interface DriveBtnProps {
  label: string
  cmd: string
  send: (c: string, arg?: number) => void
  stopOnRelease?: boolean
  variant?: 'default' | 'destructive' | 'outline'
  disabled?: boolean
}

function DriveBtn({ label, cmd, send, stopOnRelease, variant = 'default', disabled }: DriveBtnProps) {
  return (
    <Button
      variant={variant}
      disabled={disabled}
      className="h-14 text-xs select-none"
      onMouseDown={() => !disabled && send(cmd)}
      onMouseUp={() => stopOnRelease && !disabled && send('S')}
      onMouseLeave={() => stopOnRelease && !disabled && send('S')}
      onTouchStart={() => !disabled && send(cmd)}
      onTouchEnd={() => stopOnRelease && !disabled && send('S')}
    >
      {label}
    </Button>
  )
}

interface ServoSliderProps {
  label: string
  value: number
  onChange: (v: number) => void
  onCommit: (v: number) => void
  liveValue?: number
}

function ServoSlider({ label, value, onChange, onCommit, liveValue }: ServoSliderProps) {
  return (
    <div>
      <div className="flex justify-between items-center mb-1">
        <span className="text-[11px] text-muted-foreground">{label}</span>
        <span className="text-[10px] font-mono text-zinc-400">
          set: <b className="text-zinc-200">{value}°</b>
          {liveValue !== undefined && liveValue !== value && (
            <> · live: {liveValue}°</>
          )}
        </span>
      </div>
      <input
        type="range"
        min={0}
        max={180}
        value={value}
        onChange={e => onChange(parseInt(e.target.value, 10))}
        onMouseUp={() => onCommit(value)}
        onTouchEnd={() => onCommit(value)}
        className="w-full accent-primary"
      />
    </div>
  )
}
