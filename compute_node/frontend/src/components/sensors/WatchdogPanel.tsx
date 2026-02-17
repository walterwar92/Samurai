import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface WatchdogPanelProps {
  watchdog: Record<string, { alive: boolean; last_seen_sec: number }> | null
}

export function WatchdogPanel({ watchdog }: WatchdogPanelProps) {
  if (!watchdog || Object.keys(watchdog).length === 0) {
    return (
      <Card>
        <CardHeader className="py-2 px-3">
          <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
            Здоровье нод
          </CardTitle>
        </CardHeader>
        <CardContent className="p-3">
          <span className="text-xs text-muted-foreground">Нет данных</span>
        </CardContent>
      </Card>
    )
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Здоровье нод
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="grid grid-cols-1 gap-1">
          {Object.entries(watchdog).map(([topic, status]) => (
            <div key={topic} className="flex items-center gap-2 text-xs">
              <div
                className="w-2 h-2 rounded-full shrink-0"
                style={{ backgroundColor: status.alive ? '#66bb6a' : '#ef5350' }}
              />
              <span className="truncate font-mono text-[10px]">{topic}</span>
              {status.last_seen_sec >= 0 && (
                <span className="text-muted-foreground ml-auto text-[10px]">
                  {status.last_seen_sec.toFixed(0)}с
                </span>
              )}
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
