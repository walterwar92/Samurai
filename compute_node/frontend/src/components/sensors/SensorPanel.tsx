import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { RangeBar } from './RangeBar'
import { COLOUR_RU, COLOUR_CSS } from '@/lib/constants'
import type { RobotState } from '@/types/robot'

interface SensorPanelProps {
  state: RobotState | null
  expanded?: boolean
}

export function SensorPanel({ state, expanded }: SensorPanelProps) {
  const imu = state?.imu_ypr || [0, 0, 0]
  const det = state?.detection

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Сенсоры
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-3">
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Ультразвук
          </span>
          <RangeBar value={state?.range_m ?? -1} />
        </div>

        <div className="grid grid-cols-3 gap-2 text-xs">
          <SensorValue label="Yaw" value={`${imu[0].toFixed(1)}°`} />
          <SensorValue label="Pitch" value={`${imu[1].toFixed(1)}°`} />
          <SensorValue label="Roll" value={`${imu[2].toFixed(1)}°`} />
        </div>

        {expanded && (
          <div className="grid grid-cols-2 gap-2 text-xs">
            <SensorValue label="Gyro Z" value={`${(state?.imu_gyro_z ?? 0).toFixed(2)}`} />
            <SensorValue label="Accel X" value={`${(state?.imu_accel_x ?? 0).toFixed(2)}`} />
            <SensorValue label="Лин. ск." value={`${(state?.velocity?.linear ?? 0).toFixed(2)} м/с`} />
            <SensorValue label="Угл. ск." value={`${(state?.velocity?.angular ?? 0).toFixed(2)} рад/с`} />
          </div>
        )}

        {det && det.colour && (
          <div className="flex items-center gap-2 text-xs pt-1 border-t border-border">
            <div
              className="w-3 h-3 rounded-full shrink-0"
              style={{ backgroundColor: COLOUR_CSS[det.colour] }}
            />
            <span>
              {COLOUR_RU[det.colour] || det.colour} — {det.distance?.toFixed(2)} м
              <span className="text-muted-foreground ml-1">
                ({(det.conf * 100).toFixed(0)}%)
              </span>
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}

function SensorValue({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex flex-col">
      <span className="text-[10px] text-muted-foreground">{label}</span>
      <span className="font-mono">{value}</span>
    </div>
  )
}
