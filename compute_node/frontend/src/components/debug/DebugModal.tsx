import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Separator } from '@/components/ui/separator'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { COLOUR_CSS, COLOUR_RU } from '@/lib/constants'
import type { RobotState } from '@/types/robot'

interface DebugModalProps {
  open: boolean
  onClose: () => void
  state: RobotState | null
}

export function DebugModal({ open, onClose, state }: DebugModalProps) {
  if (!state) return null

  return (
    <Dialog open={open} onOpenChange={(v) => !v && onClose()}>
      <DialogContent className="max-w-3xl max-h-[85vh] p-0">
        <DialogHeader className="px-6 pt-6 pb-2">
          <DialogTitle>Полные данные робота</DialogTitle>
        </DialogHeader>
        <ScrollArea className="px-6 pb-6 max-h-[70vh]">
          <div className="space-y-4">
            {/* Simulation */}
            <Section title="Симуляция">
              <KV label="Время" value={`${state.sim_time?.toFixed(1)} с`} />
              <KV
                label="Арена"
                value={`${state.arena_size?.width ?? '?'} x ${state.arena_size?.height ?? '?'} м`}
              />
            </Section>

            <Separator />

            {/* FSM */}
            <Section title="FSM">
              <KV label="Состояние" value={state.status?.state} />
              <KV label="Цвет цели" value={COLOUR_RU[state.status?.target_colour] || '—'} />
              <KV label="Действие" value={state.status?.target_action || '—'} />
              <KV label="lost_frames" value={String(state.lost_frames ?? 0)} />
            </Section>

            <Separator />

            {/* Pose */}
            <Section title="Позиция робота">
              <KV label="X" value={`${state.pose?.x?.toFixed(3)} м`} />
              <KV label="Y" value={`${state.pose?.y?.toFixed(3)} м`} />
              <KV label="Yaw (рад)" value={state.pose?.yaw?.toFixed(3)} />
              <KV label="Yaw (град)" value={`${state.pose?.yaw_deg?.toFixed(1)}°`} />
            </Section>

            <Separator />

            {/* Velocity */}
            <Section title="Скорости">
              <KV label="Линейная" value={`${state.velocity?.linear?.toFixed(3)} м/с`} />
              <KV label="Угловая" value={`${state.velocity?.angular?.toFixed(3)} рад/с`} />
            </Section>

            <Separator />

            {/* Sensors */}
            <Section title="Сенсоры">
              <KV label="Дальномер" value={`${state.range_m?.toFixed(3)} м`} />
              <KV label="Yaw" value={`${state.imu_ypr?.[0]?.toFixed(1)}°`} />
              <KV label="Pitch" value={`${state.imu_ypr?.[1]?.toFixed(1)}°`} />
              <KV label="Roll" value={`${state.imu_ypr?.[2]?.toFixed(1)}°`} />
              <KV label="Gyro Z" value={state.imu_gyro_z?.toFixed(3)} />
              <KV label="Accel X" value={state.imu_accel_x?.toFixed(3)} />
            </Section>

            <Separator />

            {/* Actuators */}
            <Section title="Актуаторы">
              <KV
                label="Клешня"
                value={state.actuators?.claw_open ? 'ОТКРЫТА' : 'ЗАКРЫТА'}
                highlight={state.actuators?.claw_open}
              />
            </Section>

            <Separator />

            {/* Remembered ball */}
            {state.remembered_ball && (
              <>
                <Section title="Запомненный мяч (approach)">
                  <KV label="X" value={state.remembered_ball.x?.toFixed(3)} />
                  <KV label="Y" value={state.remembered_ball.y?.toFixed(3)} />
                </Section>
                <Separator />
              </>
            )}

            {/* Last known target */}
            {state.last_known_target && (
              <>
                <Section title="Последняя цель (search)">
                  <KV label="X" value={state.last_known_target.x?.toFixed(3)} />
                  <KV label="Y" value={state.last_known_target.y?.toFixed(3)} />
                </Section>
                <Separator />
              </>
            )}

            {/* A* Path */}
            <Section title={`Маршрут A* (${state.planned_path?.length ?? 0} точек)`}>
              {state.planned_path && state.planned_path.length > 0 && (
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead className="text-xs">#</TableHead>
                      <TableHead className="text-xs">X</TableHead>
                      <TableHead className="text-xs">Y</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {state.planned_path.map(([x, y], i) => (
                      <TableRow key={i}>
                        <TableCell className="text-xs py-1">{i}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{x.toFixed(3)}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{y.toFixed(3)}</TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              )}
            </Section>

            <Separator />

            {/* Detections */}
            <Section title={`Детекции камеры (${state.all_detections?.length ?? 0})`}>
              {state.all_detections && state.all_detections.length > 0 && (
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead className="text-xs">Цвет</TableHead>
                      <TableHead className="text-xs">Класс</TableHead>
                      <TableHead className="text-xs">Увер.</TableHead>
                      <TableHead className="text-xs">Дист.</TableHead>
                      <TableHead className="text-xs">x,y,w,h</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {state.all_detections.map((d, i) => (
                      <TableRow key={i}>
                        <TableCell className="text-xs py-1">
                          <span className="flex items-center gap-1">
                            <span
                              className="w-2 h-2 rounded-full inline-block"
                              style={{ backgroundColor: COLOUR_CSS[d.colour] }}
                            />
                            {COLOUR_RU[d.colour] || d.colour}
                          </span>
                        </TableCell>
                        <TableCell className="text-xs py-1">{d.class || '—'}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">
                          {(d.conf * 100).toFixed(0)}%
                        </TableCell>
                        <TableCell className="text-xs py-1 font-mono">
                          {d.distance?.toFixed(2)} м
                        </TableCell>
                        <TableCell className="text-xs py-1 font-mono">
                          {d.x},{d.y},{d.w},{d.h}
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              )}
            </Section>

            <Separator />

            {/* Balls */}
            <Section title={`Мячи на арене (${state.balls?.length ?? 0})`}>
              {state.balls && state.balls.length > 0 && (
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead className="text-xs">ID</TableHead>
                      <TableHead className="text-xs">Цвет</TableHead>
                      <TableHead className="text-xs">X</TableHead>
                      <TableHead className="text-xs">Y</TableHead>
                      <TableHead className="text-xs">Статус</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {state.balls.map((b) => (
                      <TableRow key={b.id}>
                        <TableCell className="text-xs py-1">{b.id}</TableCell>
                        <TableCell className="text-xs py-1">
                          <span className="flex items-center gap-1">
                            <span
                              className="w-2 h-2 rounded-full inline-block"
                              style={{ backgroundColor: COLOUR_CSS[b.colour] }}
                            />
                            {COLOUR_RU[b.colour] || b.colour}
                          </span>
                        </TableCell>
                        <TableCell className="text-xs py-1 font-mono">{b.x?.toFixed(2)}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{b.y?.toFixed(2)}</TableCell>
                        <TableCell className="text-xs py-1">
                          <span className={b.grabbed ? 'text-samurai-red' : 'text-samurai-green'}>
                            {b.grabbed ? 'Захвачен' : 'Активен'}
                          </span>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              )}
            </Section>

            <Separator />

            {/* Zones */}
            <Section title={`Запретные зоны (${state.zones?.length ?? 0})`}>
              {state.zones && state.zones.length > 0 && (
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead className="text-xs">ID</TableHead>
                      <TableHead className="text-xs">X1</TableHead>
                      <TableHead className="text-xs">Y1</TableHead>
                      <TableHead className="text-xs">X2</TableHead>
                      <TableHead className="text-xs">Y2</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {state.zones.map((z) => (
                      <TableRow key={z.id}>
                        <TableCell className="text-xs py-1">{z.id}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{z.x1?.toFixed(2)}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{z.y1?.toFixed(2)}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{z.x2?.toFixed(2)}</TableCell>
                        <TableCell className="text-xs py-1 font-mono">{z.y2?.toFixed(2)}</TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              )}
            </Section>
          </div>
        </ScrollArea>
      </DialogContent>
    </Dialog>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div>
      <h3 className="text-xs font-semibold uppercase tracking-wider text-muted-foreground mb-2">
        {title}
      </h3>
      <div className="space-y-1">{children}</div>
    </div>
  )
}

function KV({
  label,
  value,
  highlight,
}: {
  label: string
  value?: string
  highlight?: boolean
}) {
  return (
    <div className="flex items-center justify-between text-xs">
      <span className="text-muted-foreground">{label}</span>
      <span className={highlight ? 'text-samurai-green font-semibold' : 'font-mono'}>
        {value ?? '—'}
      </span>
    </div>
  )
}
