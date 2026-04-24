import { useEffect, useRef, useState } from 'react'
import { CheckCircle, CircleDot, Pause, Play, SkipForward, Square } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'
import type { VperedApi } from '@/hooks/useVperedState'

interface ReverseFinderProps {
  api: VperedApi
  disabled?: boolean
}

// Популярные кандидаты на быструю проверку (если не помогли — auto-scan).
const QUICK: { value: number; note: string }[] = [
  { value: 92,  note: 'FWD reference' },
  { value: 163, note: 'FWD XOR 0xFF' },
  { value: 172, note: 'nibble swap' },
  { value: 240, note: 'high nibble' },
  { value: 15,  note: 'low nibble' },
  { value: 195, note: 'ends' },
]

// Режимы перебора
type Scanset = 'all' | 'two-bits' | 'three-bits'

function buildScanSet(kind: Scanset): number[] {
  if (kind === 'all') return Array.from({ length: 256 }, (_, i) => i)
  const out: number[] = []
  const targetBits = kind === 'two-bits' ? 2 : 3
  for (let v = 0; v <= 255; v++) {
    let c = 0
    for (let b = 0; b < 8; b++) if (v & (1 << b)) c++
    if (c === targetBits) out.push(v)
  }
  return out
}

const toBin = (v: number) => v.toString(2).padStart(8, '0')

/**
 * Панель подбора DIR-байта с автоперебором и тестированием отдельных
 * значений.
 */
export function ReverseFinder({ api, disabled }: ReverseFinderProps) {
  const [custom, setCustom] = useState(163)
  const [lastTried, setLastTried] = useState<number | null>(null)
  const [found, setFound] = useState<number | null>(null)

  // Auto-scan state
  const [scanSet, setScanSet] = useState<Scanset>('two-bits')
  const [scanning, setScanning] = useState(false)
  const [scanIdx, setScanIdx] = useState(0)
  const [intervalMs, setIntervalMs] = useState(1200)
  const scanRef = useRef({ running: false, idx: 0, list: [] as number[], ms: 1200 })

  const testOnce = async (n: number) => {
    setLastTried(n)
    await api.send('Y', n)
  }

  /** Останавливает скан и моторы. */
  const stopScan = async () => {
    scanRef.current.running = false
    setScanning(false)
    await api.send('S')
  }

  /** Запуск скана. Перебираем список с интервалом intervalMs. */
  const startScan = async () => {
    const list = buildScanSet(scanSet)
    scanRef.current = { running: true, idx: 0, list, ms: intervalMs }
    setScanning(true)
    setScanIdx(0)

    while (scanRef.current.running && scanRef.current.idx < list.length) {
      const v = list[scanRef.current.idx]
      setLastTried(v)
      setScanIdx(scanRef.current.idx)
      try { await api.send('Y', v) } catch { break }
      // Ждём нужное время — проверяя флаг каждые 50 мс для отзывчивости
      const waitUntil = performance.now() + scanRef.current.ms
      while (performance.now() < waitUntil && scanRef.current.running) {
        await new Promise(r => setTimeout(r, 50))
      }
      if (!scanRef.current.running) break
      scanRef.current.idx++
    }
    scanRef.current.running = false
    setScanning(false)
    api.send('S').catch(() => {})
  }

  /** Остановить скан и зафиксировать текущее значение как найденное. */
  const markFound = async () => {
    const current = scanRef.current.list[scanRef.current.idx] ?? lastTried
    if (current !== null && current !== undefined) {
      setFound(current)
    }
    await stopScan()
  }

  /** Пропустить текущий шаг и ускориться на следующий. */
  const skipNext = () => {
    // Просто прерываем ожидание — цикл сам перейдёт на idx+1
    scanRef.current.idx++
  }

  useEffect(() => () => { scanRef.current.running = false }, [])

  const list = buildScanSet(scanSet)
  const progress = scanning ? (scanIdx / Math.max(1, list.length)) * 100 : 0

  return (
    <div className="space-y-3">
      {/* Quick manual try */}
      <div>
        <div className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold mb-1.5">
          Быстрая проверка
        </div>
        <div className="grid grid-cols-3 gap-1.5">
          {QUICK.map(q => (
            <button
              key={q.value}
              onClick={() => testOnce(q.value)}
              disabled={disabled || scanning}
              className={cn(
                'flex flex-col items-start rounded border border-border/60 bg-zinc-900/40',
                'hover:bg-amber-950/30 hover:border-amber-700/60 transition-colors',
                'px-2 py-1 text-left',
                'disabled:opacity-40 disabled:cursor-not-allowed',
                lastTried === q.value && 'border-amber-500/70 bg-amber-950/40',
              )}
            >
              <span className="text-xs font-bold tabular-nums text-zinc-200">{q.value}</span>
              <span className="text-[9px] text-muted-foreground truncate">{q.note}</span>
            </button>
          ))}
        </div>
      </div>

      {/* Auto-scan */}
      <div className="rounded-md border border-border/60 bg-zinc-950/40 p-2.5 space-y-2">
        <div className="flex items-center gap-2">
          <CircleDot className={cn('w-3 h-3', scanning ? 'text-emerald-400 animate-pulse' : 'text-muted-foreground')} />
          <span className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
            Авто-перебор
          </span>
          {scanning && (
            <span className="text-[10px] text-zinc-300 ml-auto">
              {scanIdx + 1} / {list.length}
            </span>
          )}
        </div>

        {/* Текущее значение во время сканирования — крупно */}
        {scanning && (
          <div className="text-center py-2 border border-amber-700/40 bg-amber-950/30 rounded">
            <div className="text-[9px] uppercase text-amber-400/70 tracking-wider">текущий байт</div>
            <div className="text-3xl font-bold font-mono text-amber-200 tabular-nums">
              {list[scanIdx]}
            </div>
            <div className="text-[10px] font-mono text-amber-400/80">
              0b{toBin(list[scanIdx])}
            </div>
          </div>
        )}

        {/* progress */}
        <div className="h-1 rounded bg-zinc-800 overflow-hidden">
          <div
            className="h-full bg-emerald-500 transition-all"
            style={{ width: `${progress}%` }}
          />
        </div>

        {/* Режим */}
        <div className="flex gap-1">
          {([
            ['two-bits',   '2 бита', '28 вариантов — быстро'],
            ['three-bits', '3 бита', '56 вариантов'],
            ['all',        'Все',    '256 — долго'],
          ] as const).map(([k, label, hint]) => (
            <button
              key={k}
              onClick={() => setScanSet(k)}
              disabled={scanning}
              title={hint}
              className={cn(
                'flex-1 px-2 py-1 rounded text-[10px] font-medium border transition-colors',
                scanSet === k
                  ? 'bg-primary/20 border-primary/60 text-primary'
                  : 'bg-zinc-900/40 border-border hover:bg-zinc-800',
                'disabled:opacity-40 disabled:cursor-not-allowed',
              )}
            >
              {label}
              <span className="block text-[8px] text-muted-foreground">{hint}</span>
            </button>
          ))}
        </div>

        {/* Интервал */}
        <div className="flex items-center gap-2 text-[10px]">
          <span className="text-muted-foreground">шаг:</span>
          <input
            type="range" min={600} max={2500} step={100}
            value={intervalMs}
            onChange={e => setIntervalMs(parseInt(e.target.value, 10))}
            disabled={scanning}
            className="flex-1 accent-primary disabled:opacity-40"
          />
          <span className="font-mono text-zinc-300 w-12">{(intervalMs / 1000).toFixed(1)}с</span>
        </div>

        {/* Controls */}
        <div className="grid grid-cols-3 gap-1.5">
          {!scanning ? (
            <Button
              size="sm" variant="default" className="col-span-3 h-8 text-[11px] bg-emerald-600/90 hover:bg-emerald-600"
              onClick={startScan}
              disabled={disabled}
            >
              <Play className="w-3 h-3 mr-1" /> Старт автоперебора
            </Button>
          ) : (
            <>
              <Button
                size="sm" variant="default" className="h-8 text-[11px] bg-emerald-600/90 hover:bg-emerald-600"
                onClick={markFound}
              >
                <CheckCircle className="w-3 h-3 mr-1" /> ЕДЕТ!
              </Button>
              <Button
                size="sm" variant="outline" className="h-8 text-[11px]"
                onClick={skipNext}
              >
                <SkipForward className="w-3 h-3 mr-1" /> Далее
              </Button>
              <Button
                size="sm" variant="destructive" className="h-8 text-[11px]"
                onClick={stopScan}
              >
                <Square className="w-3 h-3 mr-1" /> Стоп
              </Button>
            </>
          )}
        </div>

        <div className="text-[9px] text-muted-foreground leading-snug">
          Подними робот. Нажми «Старт» — каждые {(intervalMs / 1000).toFixed(1)}с
          будет подаваться новый байт. Следи за колёсами. Когда поехало назад
          синхронно — жми «ЕДЕТ!» (текущий байт зафиксируется).
        </div>
      </div>

      {/* Свой ввод */}
      <div className="flex items-center gap-2">
        <span className="text-[10px] text-muted-foreground shrink-0">ручной:</span>
        <input
          type="number" min={0} max={255} value={custom}
          onChange={e => setCustom(Math.max(0, Math.min(255, parseInt(e.target.value || '0', 10))))}
          disabled={disabled || scanning}
          className="flex-1 px-2 py-1 text-xs font-mono rounded bg-zinc-900 border border-border focus:border-primary outline-none disabled:opacity-40"
        />
        <Button size="sm" variant="outline" className="h-7 text-[10px]"
          disabled={disabled || scanning} onClick={() => testOnce(custom)}>
          <Play className="w-3 h-3 mr-1" /> Тест
        </Button>
      </div>

      {/* найденное */}
      {found !== null && (
        <div className="rounded-md border border-emerald-700/50 bg-emerald-950/30 p-2 space-y-1">
          <div className="flex items-center gap-1.5">
            <CheckCircle className="w-3 h-3 text-emerald-400" />
            <span className="text-[10px] uppercase tracking-wider text-emerald-300 font-semibold">
              Найдено!
            </span>
          </div>
          <div className="text-[10px] text-emerald-200/80">
            Впиши в <code className="bg-emerald-950/60 px-1.5 rounded">vpered_uno.ino</code>:
          </div>
          <code className="block text-[11px] font-mono bg-emerald-950/60 px-2 py-1 rounded text-emerald-200">
            const uint8_t DIR_BACKWARD = {found};  {'// 0b' + toBin(found)}
          </code>
          <Button
            size="sm" variant="outline" className="w-full h-6 text-[10px]"
            onClick={() => navigator.clipboard?.writeText(`const uint8_t DIR_BACKWARD = ${found};`)}
          >
            Скопировать в буфер
          </Button>
        </div>
      )}

      {/* Тест stop */}
      <Button
        size="sm" variant="outline" className="w-full h-7 text-[10px]"
        onClick={() => api.send('S')}
        disabled={disabled}
      >
        <Pause className="w-3 h-3 mr-1" /> Остановить моторы (если зависли)
      </Button>
    </div>
  )
}
