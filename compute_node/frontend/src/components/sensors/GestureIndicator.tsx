import { Hand } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

const GESTURE_LABELS: Record<string, string> = {
  stop: 'Стоп (ладонь)',
  grab: 'Захват (кулак)',
  forward: 'Вперёд (палец вверх)',
  follow: 'Следуй (большой палец)',
  point_left: 'Влево',
  point_right: 'Вправо',
}

interface GestureIndicatorProps {
  gesture: string
}

export function GestureIndicator({ gesture }: GestureIndicatorProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Жесты
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="flex items-center gap-2 text-xs">
          <Hand size={14} className={gesture ? 'text-green-400' : 'text-muted-foreground'} />
          <span>{gesture ? (GESTURE_LABELS[gesture] || gesture) : 'Нет жеста'}</span>
        </div>
      </CardContent>
    </Card>
  )
}
