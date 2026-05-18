import { useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import { Power } from 'lucide-react'

import { cn } from '@/lib/utils'
import { useConnected } from '@/stores/selectors'
import { useRobot } from '@/providers/RobotProvider'
import { api } from '@/lib/api'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog'
import { RobotSelector } from '@/components/layout/RobotSelector'

type ShutdownPhase = 'idle' | 'confirming' | 'shutting_down' | 'done'

interface HeaderProps {
  isAdmin?: boolean
  simTime?: number
  onDebugOpen?: () => void
}

export function Header({ isAdmin, simTime, onDebugOpen }: HeaderProps) {
  const connected = useConnected()
  const { activeRobot } = useRobot()
  const { pathname } = useLocation()
  const [phase, setPhase] = useState<ShutdownPhase>('idle')

  const navLink = (to: string, label: string) => {
    const active = pathname === to || (to === '/dashboard' && pathname === '/')
    return (
      <Link to={to}>
        <Button
          variant="ghost"
          size="sm"
          className={cn('text-xs', active && 'bg-accent/15 text-accent')}
        >
          {label}
        </Button>
      </Link>
    )
  }

  const isSamcan = activeRobot === 'samcan'

  const handleConfirmShutdown = async () => {
    setPhase('shutting_down')
    try {
      await api.shutdownAll()
    } catch {
      // Бэк скорее всего уже умер до ответа — это OK
    }
    setPhase('done')
  }

  return (
    <>
      <header className="flex items-center justify-between px-5 py-3 bg-card border-b border-border sticky top-0 z-50">
        <div className="flex items-center gap-4">
          <h1 className="text-lg font-bold tracking-widest text-primary">
            {isSamcan ? 'SAMCAN' : 'SAMURAI'}
          </h1>
          {isAdmin && (
            <Badge variant="destructive" className="text-[10px] tracking-wider">
              ADMIN
            </Badge>
          )}
          <RobotSelector />
          {!isSamcan && (
            <nav className="flex items-center gap-2 ml-2">
              {navLink('/dashboard', 'Панель')}
              {navLink('/admin', 'Админ')}
              {navLink('/3d', '3D Карта')}
              {navLink('/hardware', 'Оборудование')}
              {navLink('/mps', 'МПС')}
            </nav>
          )}
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
                isSamcan ? 'bg-samurai-red' : connected ? 'bg-samurai-green' : 'bg-samurai-red'
              )}
            />
            {isSamcan ? 'Нет связи (USB)' : connected ? 'Подключено' : 'Отключено'}
          </div>
          {!isSamcan && (
            <Button
              variant="destructive"
              size="sm"
              className="text-xs h-8 w-8 p-0"
              title="Выключить робота и дашборд"
              onClick={() => setPhase('confirming')}
            >
              <Power className="h-4 w-4" />
            </Button>
          )}
        </div>
      </header>

      <Dialog
        open={phase === 'confirming' || phase === 'shutting_down'}
        onOpenChange={(open) => {
          if (!open && phase === 'confirming') setPhase('idle')
        }}
      >
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Выключить всё?</DialogTitle>
            <DialogDescription>
              Робот (Pi) и дашборд (ПК) будут остановлены. Это нельзя отменить.
            </DialogDescription>
          </DialogHeader>
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => setPhase('idle')}
              disabled={phase === 'shutting_down'}
            >
              Отмена
            </Button>
            <Button
              variant="destructive"
              onClick={handleConfirmShutdown}
              disabled={phase === 'shutting_down'}
            >
              {phase === 'shutting_down' ? 'Выключаю…' : 'Выключить'}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {phase === 'done' && (
        <div className="fixed inset-0 z-[100] bg-background/95 backdrop-blur-sm flex items-center justify-center">
          <div className="text-center space-y-3">
            <Power className="h-12 w-12 mx-auto text-destructive" />
            <h2 className="text-2xl font-bold tracking-wide">Выключение…</h2>
            <p className="text-muted-foreground">Можно закрыть вкладку.</p>
          </div>
        </div>
      )}
    </>
  )
}
