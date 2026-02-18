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
}

const MAX_SAMPLES = 120 // ~30s at 4Hz

export function useSensorHistory(state: RobotState | null) {
  const startRef = useRef(Date.now())
  const samplesRef = useRef<SensorSample[]>([])

  const data = useMemo(() => {
    if (!state) return samplesRef.current

    const now = (Date.now() - startRef.current) / 1000

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
