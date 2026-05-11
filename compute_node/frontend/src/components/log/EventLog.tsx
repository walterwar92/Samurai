import { memo } from 'react'
import { Card, CardContent, CardHeader, CardTitle, CardSubtitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import { cn } from '@/lib/utils'
import type { LogEntry } from '@/types/robot'

interface EventLogProps {
  log: LogEntry[]
}

/**
 * EventLog — журнал voice/system событий. Моноспейс таймстампы,
 * fade-in-up для самой свежей записи (наверху).
 */
export const EventLog = memo(function EventLog({ log }: EventLogProps) {
  const items = !log || log.length === 0 ? [] : [...log].reverse()
  return (
    <Card>
      <CardHeader right={items.length > 0 ? <CardSubtitle>{items.length} events</CardSubtitle> : null}>
        <CardTitle>Журнал событий</CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <ScrollArea className="h-[180px]">
          <ul className="divide-y divide-subtle font-mono text-small">
            {items.length === 0 ? (
              <li className="px-3 py-2 text-foreground-faint italic">Нет событий</li>
            ) : (
              items.map((entry, i) => (
                <li
                  key={`${entry.time}-${i}`}
                  className={cn(
                    'grid grid-cols-[auto_1fr] items-center gap-3 px-3 py-1.5',
                    i === 0 && 'animate-fade-in-up',
                  )}
                >
                  <span className="text-foreground-faint tabular-nums shrink-0">
                    {entry.time}
                  </span>
                  <span className="text-foreground-muted truncate">{entry.text}</span>
                </li>
              ))
            )}
          </ul>
        </ScrollArea>
      </CardContent>
    </Card>
  )
})
