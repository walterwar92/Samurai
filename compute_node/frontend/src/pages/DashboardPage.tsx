import { useState } from 'react'
import { useRobotState } from '@/hooks/useRobotState'
import { useSensorHistory } from '@/hooks/useSensorHistory'
import { Header } from '@/components/layout/Header'
import { CameraFeed } from '@/components/camera/CameraFeed'
import { MapCanvas } from '@/components/map/MapCanvas'
import { FsmBadge } from '@/components/fsm/FsmBadge'
import { SensorPanel } from '@/components/sensors/SensorPanel'
import { DetectionBanner } from '@/components/detection/DetectionBanner'
import { DetectionTable } from '@/components/detection/DetectionTable'
import { BallsTable } from '@/components/detection/BallsTable'
import { EventLog } from '@/components/log/EventLog'
import { CommandInput } from '@/components/controls/CommandInput'
import { QuickCommandButtons } from '@/components/controls/QuickCommandButtons'
import { DebugModal } from '@/components/debug/DebugModal'
import { GamepadController } from '@/components/controls/GamepadController'
import { PathRecorderPanel } from '@/components/controls/PathRecorderPanel'
import { ActuatorToggles } from '@/components/actuators/ActuatorToggles'
import { LedPanel } from '@/components/controls/LedPanel'
import { SensorCharts } from '@/components/charts/SensorCharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'
import { COLOUR_RU, COLOUR_CSS, ACTION_RU } from '@/lib/constants'
import { api } from '@/lib/api'
import { Button } from '@/components/ui/button'

export function DashboardPage() {
  const state = useRobotState()
  const sensorHistory = useSensorHistory(state)
  const [debugOpen, setDebugOpen] = useState(false)
  const status = state?.status
  const pose = state?.pose

  return (
    <div className="min-h-screen">
      <Header onDebugOpen={() => setDebugOpen(true)} simTime={state?.sim_time} />

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
                      <span className="text-muted-foreground">Позиция (см)</span>
                      <span className="font-mono">
                        {pose ? `${pose.x.toFixed(1)}, ${pose.y.toFixed(1)}` : '—'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-muted-foreground">Курс</span>
                      <span className="font-mono">
                        {pose?.yaw_deg !== undefined ? `${pose.yaw_deg.toFixed(1)}°` : '—'}
                      </span>
                    </div>
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

              {/* ── Актуаторы ── */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Актуаторы
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3">
                  <ActuatorToggles actuators={state?.actuators} collisionGuardEnabled={state?.collision_guard_enabled} />
                </CardContent>
              </Card>

              {/* ── Геймпад ── */}
              <GamepadController />

              {/* ── LED ── */}
              <LedPanel />

              {/* ── Обнаружение ── */}
              <DetectionBanner detection={state?.detection ?? null} />
            </div>
          </ScrollArea>
        </div>

        {/* Col 1, Row 2: Commands */}
        <Card>
          <CardHeader className="py-2 px-3">
            <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
              Команды
            </CardTitle>
          </CardHeader>
          <CardContent className="p-3 space-y-3">
            <CommandInput />
            <QuickCommandButtons />

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
            <PathRecorderPanel pathRecorder={state?.path_recorder ?? null} />

            <Separator />

            {/* Скорость */}
            <div>
              <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                Скорость
              </span>
              <div className="flex gap-1.5">
                {[
                  { id: 'slow', label: 'Медл.' },
                  { id: 'normal', label: 'Норм.' },
                  { id: 'fast', label: 'Быстр.' },
                ].map((p) => (
                  <Button
                    key={p.id}
                    size="sm"
                    variant={(state?.speed_profile || 'normal') === p.id ? 'default' : 'outline'}
                    className="text-xs h-7 px-2.5 flex-1"
                    onClick={() => api.setSpeedProfile(p.id)}
                  >
                    {p.label}
                  </Button>
                ))}
              </div>
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

        {/* Row 3: Speed Chart + Sensor Charts (col 1-2) */}
        <div className="lg:col-span-2">
          <SensorCharts data={sensorHistory} />
        </div>

        {/* Row 4: Event Log (col 1-2) */}
        <div className="lg:col-span-2">
          <EventLog log={state?.voice_log ?? []} />
        </div>
      </div>

      <DebugModal open={debugOpen} onClose={() => setDebugOpen(false)} state={state} />
    </div>
  )
}
