import { useEffect, useRef, useState } from 'react'
import { ScrollArea } from '@/components/ui/scroll-area'
import { cn } from '@/lib/utils'

interface VperedEventLogProps {
  /** Функция получения последних строк лога. */
  fetchLog: () => Promise<string[]>
  /** Интервал обновления, мс. */
  intervalMs?: number
}

type Kind = 'tlm' | 'cmd' | 'info' | 'warn' | 'error' | 'ok'

function classify(line: string): Kind {
  if (line.startsWith('T,')) return 'tlm'
  const up = line.toUpperCase()
  if (up.includes('STOP') || up.includes('OBS') || up.includes('WATCHDOG')) return 'warn'
  if (up.includes('ERROR') || up.startsWith('?')) return 'error'
  if (up.includes('READY') || up === 'OK' || up.includes('DONE')) return 'ok'
  if (['F','B','L','R','S','O','X','G','P','C','K','Z','D','H','T'].includes(up.trim())) return 'cmd'
  if (up.includes('CALIB') || up.includes('KICK') || up.includes('CAL')) return 'info'
  if (up.includes('FWD') || up.includes('LEFT') || up.includes('RIGHT') || up.includes('OPEN') || up.includes('CLOSE') || up.includes('GRAB') || up.includes('PARK')) return 'cmd'
  return 'info'
}

const STYLES: Record<Kind, string> = {
  tlm:   'text-zinc-600',
  cmd:   'text-sky-400',
  info:  'text-zinc-300',
  warn:  'text-amber-400',
  error: 'text-red-400',
  ok:    'text-emerald-400',
}

export function VperedEventLog({ fetchLog, intervalMs = 1000 }: VperedEventLogProps) {
  const [lines, setLines] = useState<string[]>([])
  const [showTlm, setShowTlm] = useState(false)
  const scrollRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    let alive = true
    const tick = async () => {
      const fresh = await fetchLog()
      if (alive) setLines(fresh)
    }
    tick()
    const id = setInterval(tick, intervalMs)
    return () => { alive = false; clearInterval(id) }
  }, [fetchLog, intervalMs])

  // Auto-scroll
  useEffect(() => {
    const el = scrollRef.current?.querySelector('[data-radix-scroll-area-viewport]') as HTMLElement | null
    if (el) el.scrollTop = el.scrollHeight
  }, [lines])

  const filtered = showTlm ? lines : lines.filter(l => !l.startsWith('T,'))
  const visible = filtered.slice(-120)

  return (
    <div className="flex flex-col h-full">
      <div className="flex justify-between items-center mb-2">
        <div className="flex gap-1.5 text-[9px]">
          <Legend color="rgb(56 189 248)" label="CMD" />
          <Legend color="rgb(34 197 94)" label="OK" />
          <Legend color="rgb(234 179 8)" label="WARN" />
          <Legend color="rgb(239 68 68)" label="ERR" />
        </div>
        <label className="flex items-center gap-1.5 text-[10px] text-muted-foreground cursor-pointer select-none">
          <input
            type="checkbox"
            checked={showTlm}
            onChange={e => setShowTlm(e.target.checked)}
            className="accent-primary"
          />
          показывать телеметрию
        </label>
      </div>
      <div ref={scrollRef} className="flex-1 min-h-0">
        <ScrollArea className="h-[260px]">
          <div className="p-2 font-mono text-[10px] leading-[1.35] space-y-0.5">
            {visible.length === 0 ? (
              <div className="text-muted-foreground">нет событий</div>
            ) : (
              visible.map((line, i) => (
                <div key={`${i}-${line.slice(0, 30)}`} className={cn(STYLES[classify(line)], 'whitespace-pre-wrap break-all')}>
                  {line}
                </div>
              ))
            )}
          </div>
        </ScrollArea>
      </div>
    </div>
  )
}

function Legend({ color, label }: { color: string; label: string }) {
  return (
    <span className="flex items-center gap-1 text-muted-foreground">
      <span className="w-1.5 h-1.5 rounded-full" style={{ background: color }} />
      {label}
    </span>
  )
}
