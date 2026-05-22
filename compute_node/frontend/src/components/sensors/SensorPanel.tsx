import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { RangeBar } from './RangeBar'
import { COLOUR_RU } from '@/lib/constants'
import { BALL_HEX, type BallColour, BALL_NAMES } from '@/components/detection/ball-styles'
import {
  useClosestDetection,
  useImuAccel,
  useImuGyro,
  useImuYpr,
  useUltrasonicRange,
  useVelocity,
} from '@/stores/selectors'

interface SensorPanelProps {
  expanded?: boolean
}

/**
 * SensorPanel — компактный обзор телеметрии. Читает поля state напрямую
 * через гранулярные селекторы — каждое поле триггерит re-render только
 * когда меняется (Z5, #6 Zustand).
 */
export function SensorPanel({ expanded }: SensorPanelProps) {
  const range = useUltrasonicRange()
  const imu = useImuYpr()
  const gyro = useImuGyro()
  const accel = useImuAccel()
  const velocity = useVelocity()
  const det = useClosestDetection()

  // Цвет детекции из новой десатурированной палитры; fallback на foreground-faint.
  const detColour = det?.colour as BallColour | undefined
  const detHex =
    detColour && BALL_NAMES.includes(detColour as BallColour)
      ? BALL_HEX[detColour as BallColour]
      : undefined

  return (
    <Card>
      <CardHeader>
        <CardTitle>Сенсоры</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="space-y-1">
          <SectionLabel>Ультразвук</SectionLabel>
          <RangeBar value={range} />
        </div>

        <div className="grid grid-cols-[repeat(3,minmax(0,1fr))] gap-2">
          <SensorValue label="Yaw" value={`${imu[0].toFixed(1)}°`} />
          <SensorValue label="Pitch" value={`${imu[1].toFixed(1)}°`} />
          <SensorValue label="Roll" value={`${imu[2].toFixed(1)}°`} />
        </div>

        {expanded && (
          <div className="grid grid-cols-[repeat(2,minmax(0,1fr))] gap-2">
            <SensorValue label="Gyro Z" value={gyro[2].toFixed(2)} />
            <SensorValue label="Accel X" value={accel[0].toFixed(2)} />
            <SensorValue
              label="Лин. ск."
              value={`${(velocity?.linear ?? 0).toFixed(2)} м/с`}
            />
            <SensorValue
              label="Угл. ск."
              value={`${(velocity?.angular ?? 0).toFixed(2)} рад/с`}
            />
          </div>
        )}

        {det && det.colour && (
          <div className="flex items-center gap-2 pt-2 border-t border-subtle">
            <div
              className="h-2.5 w-2.5 rounded-full border border-strong shrink-0"
              style={{ backgroundColor: detHex ?? 'hsl(var(--foreground-faint))' }}
            />
            <span className="text-body text-foreground">
              {COLOUR_RU[det.colour] || det.colour}{' '}
              <span className="font-mono tabular-nums text-foreground-muted">
                — {det.distance?.toFixed(2)} м
              </span>{' '}
              <span className="font-mono tabular-nums text-foreground-faint">
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
    <div className="flex flex-col gap-0.5">
      <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
        {label}
      </span>
      <span className="font-mono text-small tabular-nums text-foreground">{value}</span>
    </div>
  )
}

function SectionLabel({ children }: { children: React.ReactNode }) {
  return (
    <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
      {children}
    </span>
  )
}
