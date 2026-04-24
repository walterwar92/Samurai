import { useState } from 'react'
import { CircleDot, Play, RotateCw } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'
import type { VperedApi } from '@/hooks/useVperedState'

interface ReverseFinderProps {
  api: VperedApi
  disabled?: boolean
}

// Типичные кандидаты для 74HC595+H-bridge шасси (если FORWARD = 92).
// Большинство значений — попытки инверсии битов / соседних мотор-конфигураций.
const CANDIDATES: { value: number; bin: string; note: string }[] = [
  { value: 163, bin: '10100011', note: '92 XOR 0xFF' },
  { value: 172, bin: '10101100', note: 'top nibble swap' },
  { value: 35,  bin: '00100011', note: 'low nibble swap' },
  { value: 76,  bin: '01001100', note: 'sym pair' },
  { value: 240, bin: '11110000', note: 'all top high' },
  { value: 80,  bin: '01010000', note: 'half forward' },
  { value: 95,  bin: '01011111', note: 'forward + low' },
  { value: 116, bin: '01110100', note: 'middle bits' },
]

/**
 * Панель подбора DIR-байта для обратного хода.
 * Шлёт команду Y<num> на Arduino — оба колеса крутятся 1 сек этим направлением.
 * Когда найден байт когда оба колеса едут НАЗАД — пользователь вписывает
 * его в DIR_BACKWARD в скетче и перепрошивает.
 */
export function ReverseFinder({ api, disabled }: ReverseFinderProps) {
  const [custom, setCustom] = useState<number>(163)
  const [lastTried, setLastTried] = useState<number | null>(null)
  const [found, setFound] = useState<number | null>(null)

  const test = async (n: number) => {
    setLastTried(n)
    await api.send('Y', n)
  }

  return (
    <div className="space-y-2.5">
      <div className="flex items-center gap-2">
        <RotateCw className="w-3 h-3 text-amber-400" />
        <span className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Подбор reverse (DIR-байт)
        </span>
        {lastTried !== null && (
          <span className="text-[10px] text-zinc-400 ml-auto">
            последний: <b className="font-mono text-zinc-200">{lastTried}</b>
          </span>
        )}
      </div>

      <div className="text-[10px] text-muted-foreground">
        Подними робота чтобы колёса крутились свободно. Жми кнопки —
        моторы прогоняются 1 сек. Когда оба колеса едут <b>назад</b> и в
        одну сторону — запомни число и впиши в <code>DIR_BACKWARD</code>
        в <code>vpered_uno.ino</code>.
      </div>

      {/* кандидаты */}
      <div className="grid grid-cols-2 gap-1.5">
        {CANDIDATES.map(c => (
          <button
            key={c.value}
            onClick={() => test(c.value)}
            disabled={disabled}
            className={cn(
              'group flex flex-col items-start rounded border border-border/60 bg-zinc-900/40',
              'hover:bg-amber-950/30 hover:border-amber-700/60 transition-colors',
              'px-2 py-1.5 text-left',
              'disabled:opacity-40 disabled:cursor-not-allowed',
              lastTried === c.value && 'border-amber-500/70 bg-amber-950/40',
            )}
          >
            <div className="flex items-center gap-1.5 w-full">
              <CircleDot className="w-2.5 h-2.5 text-amber-400 opacity-50 group-hover:opacity-100" />
              <span className="text-xs font-bold tabular-nums text-zinc-200">{c.value}</span>
              <span className="text-[9px] font-mono text-muted-foreground ml-auto">
                0b{c.bin}
              </span>
            </div>
            <div className="text-[9px] text-muted-foreground pl-4">{c.note}</div>
          </button>
        ))}
      </div>

      {/* ручной ввод */}
      <div className="flex items-center gap-2 pt-1">
        <span className="text-[10px] text-muted-foreground shrink-0">свой:</span>
        <input
          type="number" min={0} max={255} value={custom}
          onChange={e => setCustom(Math.max(0, Math.min(255, parseInt(e.target.value || '0', 10))))}
          disabled={disabled}
          className="flex-1 px-2 py-1 text-xs font-mono rounded bg-zinc-900 border border-border focus:border-primary outline-none"
        />
        <Button size="sm" variant="outline" className="h-7 text-[10px]"
          disabled={disabled} onClick={() => test(custom)}>
          <Play className="w-3 h-3 mr-1" /> Тест
        </Button>
      </div>

      {/* отметить найденное */}
      <div className="flex items-center gap-2 pt-1 border-t border-border/40">
        <span className="text-[10px] text-muted-foreground">подошло?</span>
        <input
          type="number" min={0} max={255}
          value={found ?? ''}
          placeholder="—"
          onChange={e => setFound(e.target.value ? Math.max(0, Math.min(255, parseInt(e.target.value, 10))) : null)}
          className="w-16 px-2 py-0.5 text-xs font-mono rounded bg-emerald-950/40 border border-emerald-800/50 focus:border-emerald-500 outline-none text-emerald-300"
        />
        {found !== null && (
          <span className="text-[10px] text-emerald-300 truncate">
            впиши: <code className="bg-emerald-950/60 px-1.5 rounded">DIR_BACKWARD = {found};</code>
          </span>
        )}
      </div>
    </div>
  )
}
