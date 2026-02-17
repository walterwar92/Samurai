import { useState } from 'react'
import { useRobotState } from '@/hooks/useRobotState'
import { Header } from '@/components/layout/Header'
import { CameraFeed } from '@/components/camera/CameraFeed'
import { MapCanvas } from '@/components/map/MapCanvas'
import { FsmStatePanel } from '@/components/fsm/FsmStatePanel'
import { SensorPanel } from '@/components/sensors/SensorPanel'
import { DetectionBanner } from '@/components/detection/DetectionBanner'
import { EventLog } from '@/components/log/EventLog'
import { CommandInput } from '@/components/controls/CommandInput'
import { QuickCommandButtons } from '@/components/controls/QuickCommandButtons'
import { DebugModal } from '@/components/debug/DebugModal'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

export function DashboardPage() {
  const state = useRobotState()
  const [debugOpen, setDebugOpen] = useState(false)

  return (
    <div className="min-h-screen">
      <Header onDebugOpen={() => setDebugOpen(true)} />

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 p-3 max-w-[1400px] mx-auto">
        {/* Row 1: Camera + Map */}
        <CameraFeed />
        <MapCanvas state={state} />

        {/* Row 2: FSM State + Sensors */}
        <FsmStatePanel state={state} />
        <SensorPanel state={state} />

        {/* Detection Banner */}
        <div className="lg:col-span-2">
          <DetectionBanner detection={state?.detection ?? null} />
        </div>

        {/* Event Log */}
        <div className="lg:col-span-2">
          <EventLog log={state?.voice_log ?? []} />
        </div>

        {/* Commands */}
        <div className="lg:col-span-2">
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
        </div>
      </div>

      <DebugModal open={debugOpen} onClose={() => setDebugOpen(false)} state={state} />
    </div>
  )
}
