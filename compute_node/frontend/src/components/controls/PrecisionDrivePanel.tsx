import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'

/* ── Types ── */

export interface PrecisionDriveStatus {
  state?: string
  scenario?: string
  leg?: number
  total_legs?: number
  distance_done_cm?: number
  distance_target_cm?: number
  heading_error_deg?: number
  lateral_error_cm?: number
  disturbance?: string
}

export interface PrecisionDriveResult {
  success?: boolean
  detail?: string
  scenario?: string
  ts?: number
}

interface Props {
  status: PrecisionDriveStatus | null
  result: PrecisionDriveResult | null
}

/* ── API helper ── */

async function sendPD(body: object) {
  await fetch('/api/precision_drive/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}

/* ── Scenarios ── */

const SCENARIOS = [
  { id: 'line',    label: 'Прямая',  icon: '↕', desc: 'Вперёд и назад' },
  { id: 'cross',   label: 'Крест',   icon: '✚', desc: 'Все 4 стороны' },
  { id: 'square',  label: 'Квадрат', icon: '▢', desc: 'Квадрат по часовой' },
  { id: 'zigzag',  label: 'Зигзаг',  icon: '⚡', desc: 'Зигзаг 4 отрезка' },
]

const DIRECTIONS = [
  { id: 'forward',  label: '▲',  title: 'Вперёд' },
  { id: 'left',     label: '◄',  title: 'Влево' },
  { id: 'backward', label: '▼',  title: 'Назад' },
  { id: 'right',    label: '►',  title: 'Вправо' },
]

const DISTANCES = [10, 20, 30, 50, 100]

/* ── State helpers ── */

const STATE_LABELS: Record<string, string> = {
  idle:        'Ожидание',
  aligning:    'Выравнивание',
  driving:     'Движение',
  turning:     'Поворот',
  paused_push: 'Пауза (толчок)',
  paused_lift: 'Пауза (подъём)',
  paused_user: 'Пауза (ручная)',
  settling:    'Стабилизация',
  done:        'Готово',
}

function stateColor(s?: string): string {
  if (!s || s === 'idle') return 'bg-zinc-600'
  if (s === 'driving' || s === 'turning' || s === 'aligning') return 'bg-emerald-500 animate-pulse'
  if (s.startsWith('paused_lift')) return 'bg-red-500 animate-pulse'
  if (s.startsWith('paused')) return 'bg-amber-500 animate-pulse'
  if (s === 'settling') return 'bg-amber-400 animate-pulse'
  if (s === 'done') return 'bg-cyan-500'
  return 'bg-zinc-600'
}

function disturbanceLabel(d?: string): string | null {
  if (!d || d === 'none') return null
  if (d === 'push') return 'Толчок обнаружен'
  if (d === 'lift') return 'Робот поднят!'
  return d
}

/* ── Component ── */

export function PrecisionDrivePanel({ status, result }: Props) {
  const [distance, setDistance] = useState(30)
  const [showResult, setShowResult] = useState(true)

  const st = status?.state ?? 'idle'
  const isActive = st !== 'idle' && st !== 'done'
  const progress = (status?.distance_target_cm && status?.distance_target_cm > 0)
    ? Math.min(100, (status.distance_done_cm ?? 0) / status.distance_target_cm * 100)
    : 0

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Точное движение
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2.5">

        {/* ── Status indicator ── */}
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full shrink-0 ${stateColor(st)}`} />
          <span className="text-xs text-zinc-300">
            {STATE_LABELS[st] || st}
          </span>
          {status?.scenario && isActive && (
            <span className="text-[10px] text-zinc-500 ml-auto">
              {status.scenario} — шаг {status.leg}/{status.total_legs}
            </span>
          )}
        </div>

        {/* ── Disturbance alert ── */}
        {disturbanceLabel(status?.disturbance) && (
          <div className="bg-red-900/40 border border-red-600/50 rounded px-2 py-1 text-xs text-red-300 animate-pulse">
            ⚠ {disturbanceLabel(status?.disturbance)}
          </div>
        )}

        {/* ── Progress bar ── */}
        {isActive && status?.distance_target_cm != null && status.distance_target_cm > 0 && (
          <div className="space-y-1">
            <div className="flex justify-between text-[10px] text-zinc-500">
              <span>{(status.distance_done_cm ?? 0).toFixed(1)} см</span>
              <span>{status.distance_target_cm.toFixed(0)} см</span>
            </div>
            <div className="w-full bg-zinc-800 rounded-full h-2">
              <div
                className="bg-cyan-500 h-2 rounded-full transition-all duration-300"
                style={{ width: `${Math.round(progress)}%` }}
              />
            </div>
            {/* Errors */}
            <div className="flex gap-3 text-[10px] text-zinc-500">
              <span>Курс: <span className="text-zinc-300 font-mono">{(status.heading_error_deg ?? 0).toFixed(1)}°</span></span>
              <span>Бок: <span className="text-zinc-300 font-mono">{(status.lateral_error_cm ?? 0).toFixed(1)} см</span></span>
            </div>
          </div>
        )}

        <Separator />

        {/* ── Distance selector ── */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
            Дистанция (см)
          </span>
          <div className="flex gap-1">
            {DISTANCES.map((d) => (
              <button
                key={d}
                onClick={() => setDistance(d)}
                className={`flex-1 px-1.5 py-1 text-xs rounded border transition-colors ${
                  distance === d
                    ? 'bg-cyan-600/40 border-cyan-500 text-cyan-200'
                    : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
                }`}
              >
                {d}
              </button>
            ))}
          </div>
        </div>

        {/* ── Single direction drive ── */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
            Одиночное движение
          </span>
          <div className="grid grid-cols-3 gap-1 w-[120px] mx-auto">
            {/* Row 1: forward */}
            <div />
            <button
              disabled={isActive}
              onClick={() => sendPD({ action: 'drive', distance_cm: distance, direction: 'forward' })}
              title="Вперёд"
              className={`px-2 py-1.5 text-sm rounded border transition-colors ${
                isActive ? 'opacity-40' : 'bg-zinc-800 border-zinc-600 text-zinc-300 hover:bg-zinc-700'
              }`}
            >
              ▲
            </button>
            <div />
            {/* Row 2: left, stop, right */}
            <button
              disabled={isActive}
              onClick={() => sendPD({ action: 'drive', distance_cm: distance, direction: 'left' })}
              title="Влево"
              className={`px-2 py-1.5 text-sm rounded border transition-colors ${
                isActive ? 'opacity-40' : 'bg-zinc-800 border-zinc-600 text-zinc-300 hover:bg-zinc-700'
              }`}
            >
              ◄
            </button>
            <button
              onClick={() => sendPD({ action: 'stop' })}
              title="Стоп"
              className="px-2 py-1.5 text-sm rounded border bg-red-900/60 border-red-600 text-red-300 hover:bg-red-800/60 transition-colors"
            >
              ■
            </button>
            <button
              disabled={isActive}
              onClick={() => sendPD({ action: 'drive', distance_cm: distance, direction: 'right' })}
              title="Вправо"
              className={`px-2 py-1.5 text-sm rounded border transition-colors ${
                isActive ? 'opacity-40' : 'bg-zinc-800 border-zinc-600 text-zinc-300 hover:bg-zinc-700'
              }`}
            >
              ►
            </button>
            {/* Row 3: backward */}
            <div />
            <button
              disabled={isActive}
              onClick={() => sendPD({ action: 'drive', distance_cm: distance, direction: 'backward' })}
              title="Назад"
              className={`px-2 py-1.5 text-sm rounded border transition-colors ${
                isActive ? 'opacity-40' : 'bg-zinc-800 border-zinc-600 text-zinc-300 hover:bg-zinc-700'
              }`}
            >
              ▼
            </button>
            <div />
          </div>
        </div>

        <Separator />

        {/* ── Scenarios ── */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
            Сценарии
          </span>
          <div className="grid grid-cols-2 gap-1.5">
            {SCENARIOS.map((sc) => (
              <button
                key={sc.id}
                disabled={isActive}
                onClick={() => sendPD({ action: 'scenario', name: sc.id, distance_cm: distance })}
                title={sc.desc}
                className={`flex flex-col items-center gap-0.5 px-2 py-2 text-xs rounded border transition-colors ${
                  isActive
                    ? 'opacity-40'
                    : status?.scenario === sc.id && st !== 'idle'
                      ? 'bg-cyan-600/30 border-cyan-500 text-cyan-200'
                      : 'bg-zinc-800 border-zinc-600 text-zinc-400 hover:bg-zinc-700'
                }`}
              >
                <span className="text-base">{sc.icon}</span>
                <span>{sc.label}</span>
              </button>
            ))}
          </div>
        </div>

        {/* ── Result ── */}
        {result && showResult && (
          <>
            <Separator />
            <div className={`rounded p-2 text-xs space-y-1 ${
              result.success ? 'bg-emerald-900/30 border border-emerald-700/40' : 'bg-red-900/30 border border-red-700/40'
            }`}>
              <div className="flex items-center gap-1.5">
                <span>{result.success ? '✓' : '✗'}</span>
                <span className={result.success ? 'text-emerald-300' : 'text-red-300'}>
                  {result.success ? 'Выполнено' : 'Ошибка'}
                </span>
                {result.scenario && (
                  <span className="text-zinc-500 ml-auto">{result.scenario}</span>
                )}
              </div>
              {result.detail && (
                <div className="text-[10px] text-zinc-500 break-all">
                  {result.detail}
                </div>
              )}
              <button
                onClick={() => setShowResult(false)}
                className="text-[10px] text-zinc-600 hover:text-zinc-400"
              >
                Скрыть
              </button>
            </div>
          </>
        )}
      </CardContent>
    </Card>
  )
}
