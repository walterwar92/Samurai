import { cn } from '@/lib/utils'
import { useSocket } from '@/providers/SocketProvider'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Link } from 'react-router-dom'

interface HeaderProps {
  isAdmin?: boolean
  simTime?: number
  onDebugOpen?: () => void
}

export function Header({ isAdmin, simTime, onDebugOpen }: HeaderProps) {
  const { connected } = useSocket()

  return (
    <header className="flex items-center justify-between px-5 py-3 bg-card border-b border-border sticky top-0 z-50">
      <div className="flex items-center gap-4">
        <h1 className="text-lg font-bold tracking-widest text-primary">SAMURAI</h1>
        {isAdmin && (
          <Badge variant="destructive" className="text-[10px] tracking-wider">
            ADMIN
          </Badge>
        )}
        <nav className="flex items-center gap-2 ml-4">
          <Link to="/dashboard">
            <Button variant="ghost" size="sm" className="text-xs">
              Панель
            </Button>
          </Link>
          <Link to="/admin">
            <Button variant="ghost" size="sm" className="text-xs">
              Админ
            </Button>
          </Link>
          <Link to="/3d">
            <Button variant="ghost" size="sm" className="text-xs">
              3D
            </Button>
          </Link>
        </nav>
      </div>

      <div className="flex items-center gap-4">
        {simTime !== undefined && (
          <span className="text-xs text-muted-foreground tabular-nums">
            {simTime.toFixed(1)}s
          </span>
        )}
        {onDebugOpen && (
          <Button variant="outline" size="sm" className="text-xs" onClick={onDebugOpen}>
            Все данные
          </Button>
        )}
        <div className="flex items-center gap-2 text-xs text-muted-foreground">
          <div
            className={cn(
              'w-2 h-2 rounded-full transition-colors',
              connected ? 'bg-samurai-green' : 'bg-samurai-red'
            )}
          />
          {connected ? 'Подключено' : 'Отключено'}
        </div>
      </div>
    </header>
  )
}
