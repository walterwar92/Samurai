import { UserRound } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

interface FollowMePanelProps {
  followMe: { active: boolean; tracking: boolean; distance: number } | null
}

export function FollowMePanel({ followMe }: FollowMePanelProps) {
  const active = followMe?.active ?? false

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Следуй за мной
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <Button
          size="sm"
          variant={active ? 'destructive' : 'default'}
          className="w-full h-8 text-xs"
          onClick={() => api.followMeCommand(active ? 'stop' : 'start')}
        >
          <UserRound size={14} className="mr-1.5" />
          {active ? 'Остановить' : 'Начать слежение'}
        </Button>

        {active && followMe && (
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div className="flex flex-col">
              <span className="text-[10px] text-muted-foreground">Статус</span>
              <span className={followMe.tracking ? 'text-green-400' : 'text-yellow-400'}>
                {followMe.tracking ? 'Отслеживаю' : 'Ищу...'}
              </span>
            </div>
            <div className="flex flex-col">
              <span className="text-[10px] text-muted-foreground">Дистанция</span>
              <span className="font-mono">{followMe.distance.toFixed(2)} м</span>
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
