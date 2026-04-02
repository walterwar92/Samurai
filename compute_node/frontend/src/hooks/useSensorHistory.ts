import { useRef, useMemo } from 'react'
import type { RobotState } from '@/types/robot'

export interface SensorSample {
  t: number // seconds since start
  battery: number
  cpuTemp: number
  range: number
  imuYaw: number
  imuPitch: number
  imuRoll: number
  odomX: number
  odomY: number
  // Accelerometer (gravity-compensated)
  accelX: number
  accelY: number
  accelZ: number
  // Integrated speed from accelerometer (м/с)
  accelSpeed: number
  // Odometry speed
  linearSpeed: number
  angularSpeed: number
}

const MAX_SAMPLES = 300 // ~60s at 5Hz

export function useSensorHistory(state: RobotState | null) {
  const startRef = useRef(Date.now())
  const samplesRef = useRef<SensorSample[]>([])
  // Gravity baseline — computed from first N samples when robot is stationary
  const gravityRef = useRef<{ x: number; y: number; z: number; calibrated: boolean; count: number }>({
    x: 0, y: 0, z: 0, calibrated: false, count: 0,
  })
  // Accumulated velocity from accelerometer integration
  const integratedVelRef = useRef(0)
  const lastTimeRef = useRef<number | null>(null)

  const data = useMemo(() => {
    if (!state) return samplesRef.current

    const now = (Date.now() - startRef.current) / 1000

    // Raw accelerometer data (3-axis)
    const rawAx = state.imu_accel?.[0] ?? 0
    const rawAy = state.imu_accel?.[1] ?? 0
    const rawAz = state.imu_accel?.[2] ?? 0

    // Gravity calibration: accumulate first 20 samples as baseline
    const grav = gravityRef.current
    const CALIB_SAMPLES = 20
    if (!grav.calibrated) {
      grav.count++
      const alpha = 1 / grav.count
      grav.x += (rawAx - grav.x) * alpha
      grav.y += (rawAy - grav.y) * alpha
      grav.z += (rawAz - grav.z) * alpha
      if (grav.count >= CALIB_SAMPLES) {
        grav.calibrated = true
      }
    }

    // Compensated acceleration (remove gravity)
    const compAx = rawAx - grav.x
    const compAy = rawAy - grav.y
    const compAz = rawAz - grav.z

    // Integrate acceleration to get speed estimate
    const dt = lastTimeRef.current !== null ? (now - lastTimeRef.current) : 0
    lastTimeRef.current = now

    if (grav.calibrated && dt > 0 && dt < 1) {
      // Forward acceleration magnitude (use X and Y as horizontal plane)
      const horizontalAccel = Math.sqrt(compAx * compAx + compAy * compAy)
      // Sign: use compAx direction as "forward"
      const signedAccel = compAx >= 0 ? horizontalAccel : -horizontalAccel

      integratedVelRef.current += signedAccel * 9.81 * dt

      // Apply decay to counteract drift (high-pass effect)
      // Without this, noise accumulates indefinitely
      integratedVelRef.current *= 0.98

      // Clamp small values to zero (deadband)
      if (Math.abs(integratedVelRef.current) < 0.005) {
        integratedVelRef.current = 0
      }
    }

    const sample: SensorSample = {
      t: Math.round(now * 10) / 10,
      battery: state.battery_percent ?? 0,
      cpuTemp: state.cpu_temp ?? 0,
      range: state.range_m ?? 0,
      imuYaw: state.imu_ypr?.[0] ?? 0,
      imuPitch: state.imu_ypr?.[1] ?? 0,
      imuRoll: state.imu_ypr?.[2] ?? 0,
      odomX: state.pose?.x ?? 0,
      odomY: state.pose?.y ?? 0,
      accelX: compAx,
      accelY: compAy,
      accelZ: compAz,
      accelSpeed: integratedVelRef.current,
      linearSpeed: state.velocity?.linear ?? 0,
      angularSpeed: state.velocity?.angular ?? 0,
    }

    const arr = samplesRef.current
    // Deduplicate — don't add if same time bucket
    if (arr.length === 0 || arr[arr.length - 1].t !== sample.t) {
      arr.push(sample)
      if (arr.length > MAX_SAMPLES) arr.shift()
    }

    samplesRef.current = arr
    return [...arr]
  }, [state])

  return data
}
