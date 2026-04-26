import { memo } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import type { LogEntry } from '@/types/robot'

interface EventLogProps {
  log: LogEntry[]
}

export const EventLog = memo(function EventLog({ log }: EventLogProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Журнал событий
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <ScrollArea className="h-[180px]">
          <div className="p-3 space-y-1">
            {(!log || log.length === 0) ? (
              <p className="text-xs text-muted-foreground">Нет событий</p>
            ) : (
              [...log].reverse().map((entry, i) => (
                <div key={i} className="flex gap-2 text-xs py-1 border-b border-border/50 last:border-0">
                  <span className="text-muted-foreground font-mono shrink-0">{entry.time}</span>
                  <span>{entry.text}</span>
                </div>
              ))
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  )
})
