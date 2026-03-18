import { useRobotState } from '@/hooks/useRobotState'
import { Header } from '@/components/layout/Header'
import { CameraFeed } from '@/components/camera/CameraFeed'
import { MapCanvas } from '@/components/map/MapCanvas'
import { FsmStatePanel } from '@/components/fsm/FsmStatePanel'
import { FsmTransitionButtons } from '@/components/fsm/FsmTransitionButtons'
import { SensorPanel } from '@/components/sensors/SensorPanel'
import { ActuatorToggles } from '@/components/actuators/ActuatorToggles'
import { ServoControlPanel } from '@/components/actuators/ServoControlPanel'
import { JoystickControl } from '@/components/joystick/JoystickControl'
import { EmergencyStop } from '@/components/controls/EmergencyStop'
import { DetectionTable } from '@/components/detection/DetectionTable'
import { BallsTable } from '@/components/detection/BallsTable'
import { EventLog } from '@/components/log/EventLog'
import { CommandInput } from '@/components/controls/CommandInput'
import { QuickCommandButtons } from '@/components/controls/QuickCommandButtons'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'

export function AdminPage() {
  const state = useRobotState()

  return (
    <div className="min-h-screen">
      <Header isAdmin simTime={state?.sim_time} />

      <div className="grid grid-cols-1 lg:grid-cols-[1fr_1fr_340px] gap-2.5 p-2.5 max-w-[1920px] mx-auto min-h-[calc(100vh-48px)]">
        {/* Col 1, Row 1: Camera */}
        <CameraFeed />

        {/* Col 2, Row 1: Map */}
        <MapCanvas state={state} />

        {/* Col 3: Sidebar (spans all rows) */}
        <div className="lg:row-span-3">
          <ScrollArea className="h-[calc(100vh-60px)]">
            <div className="space-y-2.5 pr-2">
              {/* FSM */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    FSM
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3 space-y-3">
                  <FsmStatePanel state={state} />
                  <Separator />
                  <div>
                    <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
                      Принудительный переход
                    </span>
                    <FsmTransitionButtons currentState={state?.status?.state || 'IDLE'} />
                  </div>
                </CardContent>
              </Card>

              {/* Sensors (expanded) */}
              <SensorPanel state={state} expanded />

              {/* Actuators */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Актуаторы
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3">
                  <ActuatorToggles actuators={state?.actuators} />
                </CardContent>
              </Card>

              {/* Servo Control (Head + Arm) */}
              <ServoControlPanel head={state?.head} arm={state?.arm} />

              {/* Joystick */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Управление
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3">
                  <JoystickControl />
                </CardContent>
              </Card>

              {/* Emergency Stop */}
              <EmergencyStop />
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
          <CardContent className="p-3 space-y-2">
            <CommandInput />
            <QuickCommandButtons showReset />
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

        {/* Row 3: Event Log (col 1-2) */}
        <div className="lg:col-span-2">
          <EventLog log={state?.voice_log ?? []} />
        </div>
      </div>
    </div>
  )
}
