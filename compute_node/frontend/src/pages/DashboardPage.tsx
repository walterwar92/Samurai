import { useState } from 'react'
import { useRobotState } from '@/hooks/useRobotState'
import { useSensorHistory } from '@/hooks/useSensorHistory'
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
import { BatteryIndicator } from '@/components/sensors/BatteryIndicator'
import { TemperatureIndicator } from '@/components/sensors/TemperatureIndicator'
import { WatchdogPanel } from '@/components/sensors/WatchdogPanel'
import { SpeedProfileSelector } from '@/components/controls/SpeedProfileSelector'
import { GestureIndicator } from '@/components/sensors/GestureIndicator'
import { QrDetectionBanner } from '@/components/detection/QrDetectionBanner'
import { PatrolPanel } from '@/components/controls/PatrolPanel'
import { MapManagerPanel } from '@/components/map/MapManagerPanel'
import { FollowMePanel } from '@/components/controls/FollowMePanel'
import { PathRecorderPanel } from '@/components/controls/PathRecorderPanel'
import { ServoControlPanel } from '@/components/actuators/ServoControlPanel'
import { SensorCharts } from '@/components/charts/SensorCharts'
import { DetectionTogglePanel } from '@/components/controls/DetectionTogglePanel'
import { CalibrationPanel } from '@/components/controls/CalibrationPanel'
import { ExplorerPanel } from '@/components/controls/ExplorerPanel'
import { MissionPanel } from '@/components/controls/MissionPanel'
import { GamepadController } from '@/components/controls/GamepadController'
import { MultiRobotPanel } from '@/components/controls/MultiRobotPanel'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

export function DashboardPage() {
  const state = useRobotState()
  const sensorHistory = useSensorHistory(state)
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

        {/* Row 3: Battery / Temperature / Speed / Gesture */}
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 lg:col-span-2">
          <BatteryIndicator
            voltage={state?.battery_voltage ?? 0}
            percent={state?.battery_percent ?? 0}
          />
          <TemperatureIndicator temp={state?.cpu_temp ?? 0} />
          <SpeedProfileSelector activeProfile={state?.speed_profile ?? 'normal'} />
          <GestureIndicator gesture={state?.gesture ?? ''} />
        </div>

        {/* Detection toggle + banner */}
        <div className="lg:col-span-2">
          <DetectionTogglePanel
            detectionEnabled={state?.detection_enabled ?? true}
            obstacleAvoidanceEnabled={state?.obstacle_avoidance_enabled ?? false}
          />
        </div>
        <div className="lg:col-span-2">
          <DetectionBanner detection={state?.detection ?? null} />
        </div>
        {state?.qr_detection && (
          <div className="lg:col-span-2">
            <QrDetectionBanner qr={state.qr_detection} />
          </div>
        )}

        {/* Row 4: Control Panels */}
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 lg:col-span-2">
          <PatrolPanel patrol={state?.patrol ?? null} />
          <FollowMePanel followMe={state?.follow_me ?? null} />
          <PathRecorderPanel pathRecorder={state?.path_recorder ?? null} />
          <MapManagerPanel />
          <CalibrationPanel
            calibration={state?.calibration ?? null}
            calibrationResult={state?.calibration_result ?? null}
          />
          <ExplorerPanel explorer={state?.explorer ?? null} />
          <MissionPanel mission={state?.mission ?? null} />
          <MultiRobotPanel />
        </div>

        {/* Gamepad Controller */}
        <div className="lg:col-span-2">
          <GamepadController />
        </div>

        {/* Servo Control Panel */}
        <div className="lg:col-span-2">
          <ServoControlPanel head={state?.head ?? null} arm={state?.arm ?? null} />
        </div>

        {/* Watchdog */}
        <div className="lg:col-span-2">
          <WatchdogPanel watchdog={state?.watchdog ?? null} />
        </div>

        {/* Sensor Charts */}
        <div className="lg:col-span-2">
          <SensorCharts data={sensorHistory} />
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
