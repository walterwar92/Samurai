import { useEffect, useState } from 'react'
import { AlertTriangle, RefreshCw, Usb } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'
import type { SamcanApi, SamcanDiag } from '@/hooks/useSamcanState'

interface ConnectionDiagnosticsProps {
  api: SamcanApi
  /** Скрывать баннер если true (когда реально всё работает). */
  hideWhenConnected?: boolean
}

/**
 * Показывает почему нет связи: какой порт пытался открыть bridge,
 * какая ошибка, и список всех доступных портов — чтобы пользователь
 * мог перезапустить с правильным --samcan-port.
 */
export function ConnectionDiagnostics({ api, hideWhenConnected = true }: ConnectionDiagnosticsProps) {
  const [diag, setDiag] = useState<SamcanDiag | null>(null)
  const [loading, setLoading] = useState(false)

  const refresh = async () => {
    setLoading(true)
    setDiag(await api.diag())
    setLoading(false)
  }

  // Poll раз в 3 сек пока идёт страница
  useEffect(() => {
    refresh()
    const id = setInterval(refresh, 3000)
    return () => clearInterval(id)
  }, [])

  // Определяем «реально подключено» по свежести telemetry
  const isReallyOnline = !!(api.state?.connected && api.state?.telemetry_fresh)
  if (hideWhenConnected && isReallyOnline) return null

  // Если bridge недоступен (нет ответа на /diag вообще)
  if (!diag) {
    return (
      <Card>
        <div className="flex items-start gap-2 p-3 border border-red-900/40 bg-red-950/30 rounded-md">
          <AlertTriangle className="w-4 h-4 text-red-400 flex-none mt-0.5" />
          <div className="text-xs flex-1">
            <div className="font-semibold text-red-300">Bridge не отвечает</div>
            <div className="text-muted-foreground mt-1">
              Python-процесс <code>samcan_bridge.py</code> не запущен или падает.
              Проверь <code>/tmp/samcan_bridge.log</code> и перезапусти
              <code> ./start_laptop_robot.sh</code>.
            </div>
          </div>
        </div>
      </Card>
    )
  }

  const connected = diag.connected
  const port = diag.port
  const attempted = diag.attempted_port
  const err = diag.last_error
  const rx = diag.rx_count
  const ports = diag.available_ports || []

  // Подключён, но телеметрия не приходит — значит Arduino на порту, но
  // не шлёт данные (reset после открытия Serial, не тот скетч, или бит-рейт)
  const silent = connected && rx === 0

  return (
    <Card>
      <div
        className={cn(
          'flex items-start gap-2 p-3 rounded-md border',
          isReallyOnline
            ? 'bg-emerald-950/20 border-emerald-800/40'
            : silent
              ? 'bg-amber-950/30 border-amber-800/40'
              : 'bg-red-950/30 border-red-800/40',
        )}
      >
        <Usb className={cn(
          'w-4 h-4 flex-none mt-0.5',
          isReallyOnline ? 'text-emerald-400' : silent ? 'text-amber-400' : 'text-red-400',
        )} />

        <div className="text-xs flex-1 min-w-0 space-y-2">
          <div className="flex items-center gap-2">
            <div className={cn(
              'font-semibold',
              isReallyOnline ? 'text-emerald-300' : silent ? 'text-amber-300' : 'text-red-300',
            )}>
              {isReallyOnline
                ? `Подключено: ${port}`
                : silent
                  ? `Порт ${port} открыт, но телеметрия не идёт`
                  : connected
                    ? `Открыт ${port}, но нет ответа`
                    : attempted
                      ? `Не удалось открыть ${attempted}`
                      : 'Arduino-порт не найден'}
            </div>
            <Button
              size="sm" variant="outline" className="h-6 text-[10px] ml-auto"
              onClick={refresh} disabled={loading}
            >
              <RefreshCw className={cn('w-3 h-3 mr-1', loading && 'animate-spin')} />
              Обновить
            </Button>
          </div>

          {err && (
            <div className="font-mono text-[10px] text-red-300 bg-red-950/50 px-2 py-1 rounded border border-red-900/40">
              {err}
            </div>
          )}

          {silent && (
            <div className="text-muted-foreground">
              Возможные причины:
              <ul className="list-disc pl-4 mt-0.5 space-y-0.5">
                <li>Arduino сбросилась при открытии порта — подожди 2 сек, это норма</li>
                <li>На Uno залит не samcan_uno.ino — проверь что Serial на 9600</li>
                <li>Телеметрия выключена в скетче — отправь <code>T</code> чтобы включить</li>
              </ul>
            </div>
          )}

          {ports.length > 0 && !isReallyOnline && (
            <div>
              <div className="text-muted-foreground mb-1">Доступные порты на системе:</div>
              <div className="space-y-1">
                {ports.map(p => (
                  <div
                    key={p.device}
                    className={cn(
                      'flex items-center justify-between px-2 py-1 rounded border font-mono text-[10px]',
                      p.device === attempted
                        ? 'border-sky-700/60 bg-sky-950/40 text-sky-200'
                        : 'border-border/60 bg-zinc-900/30 text-zinc-300',
                    )}
                  >
                    <span>
                      <span className="font-bold">{p.device}</span>
                      {p.description && (
                        <span className="text-muted-foreground ml-2">· {p.description}</span>
                      )}
                    </span>
                    {p.device === attempted && (
                      <span className="text-[9px] text-sky-300 uppercase tracking-wider">
                        attempted
                      </span>
                    )}
                  </div>
                ))}
              </div>
              <div className="mt-2 text-muted-foreground text-[10px]">
                Если нужен другой порт, перезапусти:
                <br />
                <code className="text-zinc-300 bg-muted/40 px-1.5 py-0.5 rounded">
                  ./start_laptop_robot.sh --samcan-port {ports[0]?.device || 'COM3'}
                </code>
              </div>
            </div>
          )}

          {ports.length === 0 && (
            <div className="text-muted-foreground">
              Система не видит ни одного COM-порта. Проверь что Arduino подключён
              и CH340 (или аналогичный) драйвер установлен.
            </div>
          )}
        </div>
      </div>
    </Card>
  )
}

/* ── inline Card (без лишних wrapper'ов) ── */
function Card({ children }: { children: React.ReactNode }) {
  return <div className="rounded-lg">{children}</div>
}
