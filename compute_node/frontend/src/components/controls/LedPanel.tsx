import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

const LED_MODES = [
  { id: 'off',     label: 'Выкл',   color: undefined },
  { id: 'white',   label: 'Белый',  color: '#fafafa' },
  { id: 'red',     label: 'Красн.', color: '#ef5350' },
  { id: 'green',   label: 'Зелён.', color: '#66bb6a' },
  { id: 'blue',    label: 'Синий',  color: '#42a5f5' },
  { id: 'rainbow', label: '🌈',     color: undefined },
  { id: 'pulse',   label: '💗',     color: undefined },
  { id: 'police',  label: '🚨',     color: undefined },
]

export function LedPanel() {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          LED
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="flex flex-wrap gap-1.5">
          {LED_MODES.map((m) => (
            <Button
              key={m.id}
              size="sm"
              variant="outline"
              className="text-xs h-7 px-2.5"
              style={m.color ? { borderColor: m.color, color: m.color } : undefined}
              onClick={() => api.setLed(m.id)}
            >
              {m.label}
            </Button>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
