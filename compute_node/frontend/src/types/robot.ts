export interface RobotPose {
  x: number
  y: number
  yaw: number
  yaw_deg?: number
}

export interface RobotVelocity {
  linear: number
  angular: number
  speed: number
}

export interface RobotStatus {
  state: FsmState
  target_colour: string
  target_action: string
}

export interface Detection {
  colour: string
  class?: string
  x: number
  y: number
  w: number
  h: number
  conf: number
  distance: number
}

export interface BallInfo {
  id: number
  colour: string
  x: number
  y: number
  grabbed: boolean
}

export interface MapInfo {
  width: number
  height: number
  resolution: number
  origin_x: number
  origin_y: number
}

export interface ForbiddenZone {
  id: number | string
  x1: number
  y1: number
  x2: number
  y2: number
}

export interface Actuators {
  claw_open: boolean
}

export interface HeadState {
  angle: number
  frozen: boolean
  locked: boolean
}

export interface ArmState {
  j1: number
  j2: number
  j3: number
  j4: number
  frozen: boolean[]
  locked: boolean
}

export interface LogEntry {
  text: string
  time: string
}

export interface RememberedPosition {
  x: number
  y: number
}

export interface ArenaSize {
  width: number
  height: number
}

export type FsmState =
  | 'IDLE'
  | 'SEARCHING'
  | 'TARGETING'
  | 'APPROACHING'
  | 'GRABBING'
  | 'CALLING'
  | 'RETURNING'

export interface PatrolStatus {
  active: boolean
  paused: boolean
  current_waypoint_idx: number
  total: number
}

export interface PathRecorderStatus {
  state: string
  points_count: number
  current_idx: number
}

export interface FollowMeStatus {
  active: boolean
  tracking: boolean
  distance: number
}

export interface QrDetection {
  data: string
  timestamp?: number
}

export interface WatchdogNodeStatus {
  name: string
  alive: boolean
  last_seen?: number
}

// ── Composable RobotState slices (#47) ────────────────────────────────
// RobotState was a flat union with ~50 fields. Components only need a
// subset (IMU widget reads imu_*, FSM badge reads status, MapCanvas reads
// pose+scan_points+zones+map_info). Splitting into smaller interfaces
// lets useRobotStore selectors return just the slice a component needs,
// which:
//   - improves React.memo's shallow-equal check (smaller object → fewer
//     spurious re-renders),
//   - makes test fixtures terser (a fake IMU panel doesn't need to mock
//     speed_profile, calibration, etc.),
//   - documents which fields belong together.
// Backward compat: RobotState is the intersection of all slices, so every
// component that already reads `state.imu_ypr` or `state.pose` keeps
// compiling unchanged.

export interface RobotImuState {
  imu_ypr: [number, number, number]
  imu_gyro_z: number
  imu_accel_x: number
  imu_accel: [number, number, number]
  imu_gyro: [number, number, number]
  imu_ypr_raw: [number, number, number] | null
  imu_ypr_ekf: [number, number, number] | null
  imu_ekf_bias: [number, number, number] | null
  imu_has_ekf: boolean
}

export type OdometrySourceMode = 'wheel' | 'imu' | 'complementary' | 'ekf'

export interface OdometrySources {
  x_wheel: number
  y_wheel: number
  x_imu: number
  y_imu: number
  vx_imu: number
  vy_imu: number
  stationary_imu: boolean
  source: OdometrySourceMode
}

export interface RobotPoseState {
  pose: RobotPose
  odom_sources: OdometrySources
  stationary: boolean
  velocity: RobotVelocity
  speed_profile: string
}

export interface RobotDetectionState {
  detection: Detection | null
  all_detections: Detection[]
  balls: BallInfo[]
  remembered_ball: RememberedPosition | null
  last_known_target: RememberedPosition | null
  qr_detection: QrDetection | null
  detection_enabled: boolean
  lost_frames: number
}

export interface RobotMapState {
  map_info: MapInfo
  scan_points: [number, number][]
  zones: ForbiddenZone[]
  planned_path: [number, number][]
  slam_map: SlamMapData | null
  arena_size: ArenaSize
}

export interface RobotSensorsState {
  range_m: number
  battery_voltage: number
  battery_percent: number
  cpu_temp: number
  watchdog: Record<string, { alive: boolean; last_seen_sec: number }> | null
}

export interface RobotActuatorsState {
  actuators: Actuators
  head: HeadState | null
  arm: ArmState | null
  arm_presets: string[]
  head_presets: string[]
}

export interface RobotControlState {
  patrol: PatrolStatus | null
  path_recorder: PathRecorderStatus | null
  recorded_path: [number, number][] | null
  follow_me: FollowMeStatus | null
  obstacle_avoidance_enabled: boolean
  collision_guard_enabled: boolean
  explorer: { state?: string; strategy?: string; progress?: number; covered_cells?: number } | null
  mission: { state?: string; name?: string; events_count?: number; progress?: number } | null
  precision_drive: {
    state?: string
    scenario?: string
    leg?: number
    total_legs?: number
    distance_done_cm?: number
    distance_target_cm?: number
    heading_error_deg?: number
    lateral_error_cm?: number
    disturbance?: string
  } | null
  precision_drive_result: {
    success?: boolean
    detail?: string
    scenario?: string
    ts?: number
  } | null
  calibration: { state?: string; type?: string; progress?: number } | null
  calibration_result: {
    type?: string
    scale_factor?: number
    recommendation?: string
    odom_distance?: number
    actual_distance?: number
    odom_angle_deg?: number
  } | null
  calibration_coeffs: {
    profile: string
    scale_fwd: number
    scale_bwd: number
    motor_trim: number
  } | null
  calibration_profiles: {
    profiles: Record<string, {
      scale_fwd: number
      scale_bwd: number
      motor_trim: number
      description: string
    }>
    active: string
  } | null
}

export interface RobotSystemState {
  status: RobotStatus
  voice_log: LogEntry[]
  sim_time: number
  gesture: string
  tts_enabled: boolean
}

// Aggregate type — preserves the flat shape every existing component reads.
export interface RobotState
  extends RobotImuState,
    RobotPoseState,
    RobotDetectionState,
    RobotMapState,
    RobotSensorsState,
    RobotActuatorsState,
    RobotControlState,
    RobotSystemState {}

export interface SlamMapObstacle {
  0: number  // x
  1: number  // y
}

export interface SlamDetectedObject {
  id: string
  class: string
  colour: string
  x: number
  y: number
  conf: number
  dist: number
  count: number
}

export interface SlamMapData {
  obstacles: [number, number][]
  trail: [number, number][]
  robot: { x: number; y: number; theta: number }
  detected_objects: SlamDetectedObject[]
  info: {
    width: number
    height: number
    resolution: number
    origin_x: number
    origin_y: number
    size_m: number
  }
  stats: {
    occupied: number
    free: number
    unknown: number
    detected_objects: number
  }
}

// ── /ws/robot/live_state ──────────────────────────────────────────────
// Постоянный поток текущего состояния робота (pose + velocity + IMU) для
// панели на DashboardPage. Контракт: docs/superpowers/specs/
// 2026-05-19-robot-live-state-vector-design.md §2.
export const ROBOT_LIVE_STATE_SCHEMA = '1.0' as const

export interface RobotLiveStatePoint {
  /** Pi-clock unix-секунды */
  ts: number
  pose: {
    /** м, world frame */
    x: number
    y: number
    /** радианы */
    yaw_rad: number
    /** градусы (для удобства UI) */
    yaw_deg: number
  }
  vel: {
    /** м/с */
    linear: number
    /** рад/с */
    angular: number
  }
  imu: {
    /** [yaw, pitch, roll] в °, активный (EKF или raw-fallback) */
    ypr_deg: [number, number, number]
    /** [x, y, z] рад/с */
    gyro: [number, number, number]
    /** [x, y, z] м/с² */
    accel: [number, number, number]
    /** [x, y, z] °/с; null если has_ekf=false */
    ekf_bias_deg: [number, number, number] | null
    has_ekf: boolean
  }
  /** ZUPT (IMU-based stationary detector) */
  stationary: boolean
  schema_version: string
}

export interface RobotLiveStateWsFrame {
  type: 'live_state'
  point: RobotLiveStatePoint
}
