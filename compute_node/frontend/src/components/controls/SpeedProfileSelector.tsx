import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

const PROFILES = [
  { id: 'slow', label: 'Медленно', desc: '0.10 м/с' },
  { id: 'normal', label: 'Нормально', desc: '0.20 м/с' },
  { id: 'fast', label: 'Быстро', desc: '0.30 м/с' },
]

interface SpeedProfileSelectorProps {
  activeProfile: string
}

export function SpeedProfileSelector({ activeProfile }: SpeedProfileSelectorProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Профиль скорости
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="flex gap-1.5">
          {PROFILES.map((p) => (
            <Button
              key={p.id}
              size="sm"
              variant={activeProfile === p.id ? 'default' : 'outline'}
              className="flex-1 text-[10px] h-8"
              onClick={() => api.setSpeedProfile(p.id)}
            >
              <div className="text-center">
                <div>{p.label}</div>
                <div className="text-[9px] opacity-60">{p.desc}</div>
              </div>
            </Button>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
