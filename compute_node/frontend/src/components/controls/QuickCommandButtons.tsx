import { Button } from '@/components/ui/button'
import { useSocket } from '@/providers/SocketProvider'
import { QUICK_COMMANDS } from '@/lib/constants'

interface QuickCommandButtonsProps {
  showReset?: boolean
}

export function QuickCommandButtons({ showReset }: QuickCommandButtonsProps) {
  const { sendCommand, resetSim } = useSocket()

  return (
    <div className="flex flex-wrap gap-1.5">
      {QUICK_COMMANDS.map((cmd) => (
        <Button
          key={cmd.label}
          variant={cmd.variant === 'destructive' ? 'destructive' : 'outline'}
          size="sm"
          className="text-xs h-7 px-2.5"
          style={
            cmd.color
              ? { borderColor: cmd.color, color: cmd.color }
              : undefined
          }
          onClick={() => sendCommand(cmd.command)}
        >
          {cmd.label}
        </Button>
      ))}
      {showReset && (
        <Button
          variant="outline"
          size="sm"
          className="text-xs h-7 px-2.5 border-samurai-orange text-samurai-orange"
          onClick={resetSim}
        >
          Сброс
        </Button>
      )}
    </div>
  )
}
