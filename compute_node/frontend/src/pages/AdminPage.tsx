import { useRobotState } from '@/hooks/useRobotState'
import { Header } from '@/components/layout/Header'
import { CameraFeed } from '@/components/camera/CameraFeed'
import { MapCanvas } from '@/components/map/MapCanvas'
import { FsmBadge } from '@/components/fsm/FsmBadge'
import { FsmTransitionButtons } from '@/components/fsm/FsmTransitionButtons'
import { SensorPanel } from '@/components/sensors/SensorPanel'
import { ActuatorToggles } from '@/components/actuators/ActuatorToggles'
import { ServoControlPanel } from '@/components/actuators/ServoControlPanel'
import { JoystickControl } from '@/components/joystick/JoystickControl'
import { EmergencyStop } from '@/components/controls/EmergencyStop'
import { PrecisionDrivePanel } from '@/components/controls/PrecisionDrivePanel'
import { DetectionTable } from '@/components/detection/DetectionTable'
import { BallsTable } from '@/components/detection/BallsTable'
import { CommandInput } from '@/components/controls/CommandInput'
import { QuickCommandButtons } from '@/components/controls/QuickCommandButtons'
import { PatrolPanel } from '@/components/controls/PatrolPanel'
import { FollowMePanel } from '@/components/controls/FollowMePanel'
import { PathRecorderPanel } from '@/components/controls/PathRecorderPanel'
import { SpeedProfileSelector } from '@/components/controls/SpeedProfileSelector'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'
import { COLOUR_RU, COLOUR_CSS, ACTION_RU } from '@/lib/constants'

export function AdminPage() {
  const state = useRobotState()
  const status = state?.status
  const pose = state?.pose

  return (
    <div className="min-h-screen">
      <Header isAdmin simTime={state?.sim_time} />

      <div className="grid grid-cols-1 lg:grid-cols-[1fr_1fr_340px] gap-2.5 p-2.5 max-w-[1920px] mx-auto min-h-[calc(100vh-48px)]">
        {/* Col 1, Row 1: Camera */}
        <CameraFeed />

        {/* Col 2, Row 1: Map */}
        <MapCanvas state={state} />

        {/* Col 3: Right sidebar (spans all rows) */}
        <div className="lg:row-span-3">
          <ScrollArea className="h-[calc(100vh-60px)]">
            <div className="space-y-2.5 pr-2">

              {/* ── СОСТОЯНИЕ FSM ── */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Состояние FSM
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3 space-y-3">
                  <FsmBadge state={status?.state || 'IDLE'} />

                  <div className="space-y-1.5 text-xs">
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Цель</span>
                      <span
                        className="font-medium"
                        style={{ color: COLOUR_CSS[status?.target_colour || ''] || undefined }}
                      >
                        {COLOUR_RU[status?.target_colour || ''] || '—'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Действие</span>
                      <span>{ACTION_RU[status?.target_action || ''] || '—'}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Позиция</span>
                      <span className="font-mono">
                        {pose ? `${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}` : '—'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Курс</span>
                      <span className="font-mono">
                        {pose?.yaw_deg !== undefined ? `${pose.yaw_deg.toFixed(1)}°` : '—'}
                      </span>
                    </div>
                  </div>

                  <Separator />

                  <div>
                    <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                      Принудительный переход
                    </span>
                    <FsmTransitionButtons currentState={status?.state || 'IDLE'} />
                  </div>
                </CardContent>
              </Card>

              {/* ── СЕНСОРЫ ── */}
              <SensorPanel state={state} expanded />

              {/* ── СИСТЕМА ── */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Система
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3">
                  <div className="space-y-1.5 text-xs">
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Батарея</span>
                      <span className="font-mono">
                        {state?.battery_voltage !== undefined
                          ? `${state.battery_voltage.toFixed(1)}В`
                          : '—'}
                        {state?.battery_percent !== undefined && (
                          <span className="text-muted-foreground ml-1">
                            ({state.battery_percent}%)
                          </span>
                        )}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Темп. CPU</span>
                      <span className="font-mono">
                        {state?.cpu_temp !== undefined ? `${state.cpu_temp.toFixed(1)}°C` : '—'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Профиль</span>
                      <span className="font-medium">{state?.speed_profile || 'normal'}</span>
                    </div>
                  </div>
                </CardContent>
              </Card>

              {/* ── УПРАВЛЕНИЕ РОБОТОМ ── */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Управление роботом
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3 space-y-3">
                  {/* Актуаторы */}
                  <div>
                    <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                      Актуаторы
                    </span>
                    <ActuatorToggles actuators={state?.actuators} collisionGuardEnabled={state?.collision_guard_enabled} />
                  </div>

                  <Separator />

                  {/* Джойстик скорости */}
                  <div>
                    <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                      Джойстик скорости
                    </span>
                    <JoystickControl />
                  </div>
                </CardContent>
              </Card>

              {/* ── Сервоприводы ── */}
              <ServoControlPanel head={state?.head} arm={state?.arm} />

              {/* ── Точное движение ── */}
              <PrecisionDrivePanel
                status={state?.precision_drive ?? null}
                result={state?.precision_drive_result ?? null}
              />

              {/* ── Emergency Stop ── */}
              <EmergencyStop />
            </div>
          </ScrollArea>
        </div>

        {/* Col 1, Row 2: Commands + Modes + Path + Speed */}
        <Card>
          <CardHeader className="py-2 px-3">
            <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
              Команды
            </CardTitle>
          </CardHeader>
          <CardContent className="p-3 space-y-3">
            <CommandInput />
            <QuickCommandButtons showReset />

            <Separator />

            {/* Режимы */}
            <div>
              <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                Режимы
              </span>
              <div className="flex gap-1.5 flex-wrap">
                <Button size="sm" variant="outline" className="text-xs h-7 px-2.5" onClick={() => api.patrolCommand('start')}>
                  ▶ Патруль
                </Button>
                <Button size="sm" variant={state?.patrol?.active ? 'destructive' : 'outline'} className="text-xs h-7 px-2.5" onClick={() => api.patrolCommand('stop')}>
                  ■ Патруль
                </Button>
                <Button size="sm" variant="outline" className="text-xs h-7 px-2.5" onClick={() => api.followMeCommand('start')}>
                  ▶ Следуй
                </Button>
                <Button size="sm" variant={state?.follow_me?.active ? 'destructive' : 'outline'} className="text-xs h-7 px-2.5" onClick={() => api.followMeCommand('stop')}>
                  ■ Следуй
                </Button>
              </div>
            </div>

            <Separator />

            {/* Запись пути */}
            <div>
              <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                Запись пути
              </span>
              <div className="flex gap-1.5">
                <PathBtn
                  state={state?.path_recorder?.state}
                />
              </div>
            </div>

            <Separator />

            {/* Скорость */}
            <div>
              <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                Скорость
              </span>
              <SpeedBtns currentProfile={state?.speed_profile} />
            </div>
          </CardContent>
        </Card>

        {/* Col 2, Row 2: Detection + Balls */}
        <div className="space-y-2.5">
          <DetectionTable
            detections={state?.all_detections ?? []}
            closest={state?.detection ?? null}
          />
          <BallsTable balls={state?.balls ?? []} />
        </div>
      </div>
    </div>
  )
}

/* ── Inline helper components ── */

import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'

function PathBtn({ state }: { state?: string }) {
  const recording = state === 'recording'
  const replaying = state === 'replaying'
  return (
    <>
      <Button
        size="sm"
        variant={recording ? 'default' : 'outline'}
        className="text-xs h-7 px-2.5"
        onClick={() => api.pathRecorderCommand(recording ? 'stop' : 'record')}
      >
        ● Запись
      </Button>
      <Button
        size="sm"
        variant="destructive"
        className="text-xs h-7 px-2.5"
        onClick={() => api.pathRecorderCommand('stop')}
      >
        ■ Стоп
      </Button>
      <Button
        size="sm"
        variant={replaying ? 'default' : 'outline'}
        className="text-xs h-7 px-2.5"
        onClick={() => api.pathRecorderCommand('replay')}
      >
        ▶ Воспр.
      </Button>
    </>
  )
}

const SPEED_PROFILES = [
  { id: 'slow', label: 'Медл.' },
  { id: 'normal', label: 'Норм.' },
  { id: 'fast', label: 'Быстр.' },
]

function SpeedBtns({ currentProfile }: { currentProfile?: string }) {
  const active = currentProfile || 'normal'
  return (
    <div className="flex gap-1.5">
      {SPEED_PROFILES.map((p) => (
        <Button
          key={p.id}
          size="sm"
          variant={active === p.id ? 'default' : 'outline'}
          className="text-xs h-7 px-2.5 flex-1"
          onClick={() => api.setSpeedProfile(p.id)}
        >
          {p.label}
        </Button>
      ))}
    </div>
  )
}
