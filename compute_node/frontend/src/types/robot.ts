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

export interface RobotState {
  status: RobotStatus
  detection: Detection | null
  all_detections: Detection[]
  range_m: number
  imu_ypr: [number, number, number]
  imu_gyro_z: number
  imu_accel_x: number
  pose: RobotPose
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
}
