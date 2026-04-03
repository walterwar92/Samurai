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
  // Fused speed from backend EKF (м/с) — true robot speed
  fusedSpeed: number
  // Odometry linear velocity (forward component, signed)
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

    // Compensated acceleration (remove gravity) — for graph display only
    const compAx = rawAx - grav.x
    const compAy = rawAy - grav.y
    const compAz = rawAz - grav.z

    // Use backend-computed fused speed (EKF + ZUPT).
    // This is the true robot speed — properly filtered, with instant
    // zero when stopped (ZUPT), and accurate during motion (accel+wheel fusion).
    const isStationary = state.stationary ?? false
    const fusedSpeed = isStationary ? 0 : (state.velocity?.speed ?? Math.abs(state.velocity?.linear ?? 0))

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
      fusedSpeed,
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
