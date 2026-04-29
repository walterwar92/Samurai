/**
 * Гранулярные селекторы Zustand store робота (#6, 2026-04).
 *
 * Каждый хук возвращает только нужное поле — компонент перерендеривается
 * ТОЛЬКО когда оно реально изменилось. Для composite-полей (Pose, Imu)
 * используем `useShallow` чтобы object identity не приводила к лишним
 * re-render'ам.
 *
 * Конвенция:
 *   useX()         — селектор данных
 *   useXAction()   — селектор action (send, resetSim, connect)
 *
 * Пример:
 *   const battery = useBatteryPercent()  // re-render только при изменении %
 *   const send = useSend()               // ссылка стабильна между рендерами
 */
import { useShallow } from 'zustand/react/shallow'

import { useRobotStore } from './robotStore'
import type {
  ArmState,
  BallInfo,
  Detection,
  ForbiddenZone,
  HeadState,
  LogEntry,
  MapInfo,
  PathRecorderStatus,
  PatrolStatus,
  RobotPose,
  RobotState,
  RobotStatus,
  RobotVelocity,
} from '@/types/robot'

// ── Connection ─────────────────────────────────────────────────────────
export const useConnected = () => useRobotStore((s) => s.connected)
export const useSimTime = () => useRobotStore((s) => s.state?.sim_time ?? 0)

// ── Robot core (FSM, pose, velocity) ───────────────────────────────────
export const useFsm = (): RobotStatus | null =>
  useRobotStore((s) => s.state?.status ?? null)
export const useFsmState = () =>
  useRobotStore((s) => s.state?.status?.state ?? 'IDLE')
export const useTargetColour = () =>
  useRobotStore((s) => s.state?.status?.target_colour ?? '')
export const useTargetAction = () =>
  useRobotStore((s) => s.state?.status?.target_action ?? '')

export const usePose = (): RobotPose | null =>
  useRobotStore(useShallow((s) => s.state?.pose ?? null))
export const useStationary = () => useRobotStore((s) => s.state?.stationary ?? true)

export const useVelocity = (): RobotVelocity | null =>
  useRobotStore(useShallow((s) => s.state?.velocity ?? null))
export const useLinearSpeed = () =>
  useRobotStore((s) => s.state?.velocity?.linear ?? 0)
export const useAngularSpeed = () =>
  useRobotStore((s) => s.state?.velocity?.angular ?? 0)

export const useSpeedProfile = () =>
  useRobotStore((s) => s.state?.speed_profile ?? 'normal')

// ── Sensors ────────────────────────────────────────────────────────────
export const useBatteryPercent = () =>
  useRobotStore((s) => s.state?.battery_percent ?? -1)
export const useBatteryVoltage = () =>
  useRobotStore((s) => s.state?.battery_voltage ?? -1)
export const useCpuTemp = () => useRobotStore((s) => s.state?.cpu_temp ?? -1)
export const useUltrasonicRange = () =>
  useRobotStore((s) => s.state?.range_m ?? -1)

export const useImuYpr = () =>
  useRobotStore(useShallow((s) => s.state?.imu_ypr ?? [0, 0, 0]))
export const useImuAccel = () =>
  useRobotStore(useShallow((s) => s.state?.imu_accel ?? [0, 0, 0]))
export const useImuGyro = () =>
  useRobotStore(useShallow((s) => s.state?.imu_gyro ?? [0, 0, 0]))
export const useImuHasEkf = () =>
  useRobotStore((s) => s.state?.imu_has_ekf ?? false)

// ── Actuators ──────────────────────────────────────────────────────────
export const useClawOpen = () =>
  useRobotStore((s) => s.state?.actuators?.claw_open ?? false)
export const useHead = (): HeadState | null =>
  useRobotStore(useShallow((s) => s.state?.head ?? null))
export const useArm = (): ArmState | null =>
  useRobotStore(useShallow((s) => s.state?.arm ?? null))
export const useArmPresets = () =>
  useRobotStore(useShallow((s) => s.state?.arm_presets ?? []))
export const useHeadPresets = () =>
  useRobotStore(useShallow((s) => s.state?.head_presets ?? []))

// ── Detection ──────────────────────────────────────────────────────────
export const useClosestDetection = (): Detection | null =>
  useRobotStore(useShallow((s) => s.state?.detection ?? null))
export const useAllDetections = (): Detection[] =>
  useRobotStore(useShallow((s) => s.state?.all_detections ?? []))
export const useBalls = (): BallInfo[] =>
  useRobotStore(useShallow((s) => s.state?.balls ?? []))
export const useDetectionEnabled = () =>
  useRobotStore((s) => s.state?.detection_enabled ?? true)

// ── Map ────────────────────────────────────────────────────────────────
export const useMapInfo = (): MapInfo | null =>
  useRobotStore(useShallow((s) => s.state?.map_info ?? null))
export const useScanPoints = () =>
  useRobotStore(useShallow((s) => s.state?.scan_points ?? []))
export const useZones = (): ForbiddenZone[] =>
  useRobotStore(useShallow((s) => s.state?.zones ?? []))
export const usePlannedPath = () =>
  useRobotStore(useShallow((s) => s.state?.planned_path ?? []))

// ── Logs ───────────────────────────────────────────────────────────────
export const useVoiceLog = (): LogEntry[] =>
  useRobotStore(useShallow((s) => s.state?.voice_log ?? []))

// ── Control toggles ────────────────────────────────────────────────────
export const useObstacleAvoidanceEnabled = () =>
  useRobotStore((s) => s.state?.obstacle_avoidance_enabled ?? false)
export const useCollisionGuardEnabled = () =>
  useRobotStore((s) => s.state?.collision_guard_enabled ?? false)
export const useTtsEnabled = () => useRobotStore((s) => s.state?.tts_enabled ?? true)

// ── High-level statuses (composite dicts) ─────────────────────────────
export const usePatrol = (): PatrolStatus | null =>
  useRobotStore(useShallow((s) => s.state?.patrol ?? null))
export const usePathRecorder = (): PathRecorderStatus | null =>
  useRobotStore(useShallow((s) => s.state?.path_recorder ?? null))
export const useRecordedPath = () =>
  useRobotStore(useShallow((s) => s.state?.recorded_path ?? null))
export const useFollowMe = () =>
  useRobotStore(useShallow((s) => s.state?.follow_me ?? null))
export const useSlamMap = () =>
  useRobotStore(useShallow((s) => s.state?.slam_map ?? null))
export const usePrecisionDrive = () =>
  useRobotStore(useShallow((s) => s.state?.precision_drive ?? null))
export const useCalibration = () =>
  useRobotStore(useShallow((s) => s.state?.calibration ?? null))
export const useCalibrationCoeffs = () =>
  useRobotStore(useShallow((s) => s.state?.calibration_coeffs ?? null))

// ── Actions (стабильные ссылки между рендерами) ────────────────────────
export const useSend = () => useRobotStore((s) => s.send)
export const useResetSim = () => useRobotStore((s) => s.resetSim)

// ── Convenience: re-export raw store and full state ────────────────────
export { useRobotStore } from './robotStore'

/** Если действительно нужен весь state (debug, snapshot) — используй это. */
export const useFullState = (): RobotState | null =>
  useRobotStore((s) => s.state)
