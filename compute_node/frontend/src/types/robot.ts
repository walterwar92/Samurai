export interface RobotPose {
  x: number
  y: number
  yaw: number
  yaw_deg?: number
}

export interface RobotVelocity {
  linear: number
  angular: number
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
  laser_on: boolean
}

export interface HeadState {
  angle: number
}

export interface ArmState {
  j1: number
  j2: number
  j3: number
  j4: number
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
  | 'BURNING'
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

export interface RobotState {
  status: RobotStatus
  detection: Detection | null
  all_detections: Detection[]
  range_m: number
  imu_ypr: [number, number, number]
  imu_gyro_z: number
  imu_accel_x: number
  imu_accel: [number, number, number]
  imu_gyro: [number, number, number]
  imu_ypr_raw: [number, number, number] | null
  imu_ypr_ekf: [number, number, number] | null
  imu_ekf_bias: [number, number, number] | null
  imu_has_ekf: boolean
  pose: RobotPose
  stationary: boolean
  velocity: RobotVelocity
  actuators: Actuators
  map_info: MapInfo
  scan_points: [number, number][]
  voice_log: LogEntry[]
  zones: ForbiddenZone[]
  planned_path: [number, number][]
  remembered_ball: RememberedPosition | null
  last_known_target: RememberedPosition | null
  balls: BallInfo[]
  sim_time: number
  arena_size: ArenaSize
  lost_frames: number
  // New fields
  battery_voltage: number
  battery_percent: number
  cpu_temp: number
  watchdog: Record<string, { alive: boolean; last_seen_sec: number }> | null
  patrol: PatrolStatus | null
  path_recorder: PathRecorderStatus | null
  recorded_path: [number, number][] | null
  follow_me: FollowMeStatus | null
  qr_detection: QrDetection | null
  gesture: string
  speed_profile: string
  head: HeadState | null
  arm: ArmState | null
  // SLAM map data from Pi ultrasonic
  slam_map: SlamMapData | null
  detection_enabled: boolean
  obstacle_avoidance_enabled: boolean
  // Calibration
  calibration: { state?: string; type?: string; progress?: number } | null
  calibration_result: {
    type?: string
    scale_factor?: number
    recommendation?: string
    odom_distance?: number
    actual_distance?: number
    odom_angle_deg?: number
  } | null
  // Explorer
  explorer: { state?: string; strategy?: string; progress?: number; covered_cells?: number } | null
  // Mission
  mission: { state?: string; name?: string; events_count?: number; progress?: number } | null
  // TTS
  tts_enabled: boolean
}

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
