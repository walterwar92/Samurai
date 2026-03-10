package com.samurai.robotcontrol.api

/**
 * Data models for the Samurai Robot REST API.
 * Handles both simulator and real-robot response formats.
 */

// ── Generic API envelope ──────────────────────────────────────────
data class ApiResponse<T>(
    val ok: Boolean,
    val data: T? = null,
    val error: String? = null
)

// ── Robot Pose ───────────────────────────────────────────────────
// Simulator: theta/theta_deg  |  Real robot: yaw
data class RobotPose(
    val x: Double = 0.0,
    val y: Double = 0.0,
    // Simulator
    val theta: Double = 0.0,
    val theta_deg: Double = 0.0,
    // Real robot
    val yaw: Double = 0.0,
) {
    val headingRad: Double get() = if (theta != 0.0) theta else yaw
    val headingDeg: Double get() = if (theta_deg != 0.0) theta_deg else Math.toDegrees(yaw)
}

// ── Velocity ─────────────────────────────────────────────────────
data class VelocityDetail(
    val linear_x: Double = 0.0,
    val linear_y: Double = 0.0,
    val angular_z: Double = 0.0,
)

// Simulator: linear/angular  |  Real robot: estimated/commanded
data class RobotVelocity(
    val linear: Double = 0.0,
    val angular: Double = 0.0,
    val max_linear: Double = 0.3,
    val max_angular: Double = 2.0,
    val estimated: VelocityDetail? = null,
    val commanded: VelocityDetail? = null,
) {
    val linearSpeed: Double get() = estimated?.linear_x ?: linear
    val angularSpeed: Double get() = estimated?.angular_z ?: angular
}

// ── IMU ──────────────────────────────────────────────────────────
data class GyroData(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0)
data class AccelData(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0)

// Simulator: flat fields  |  Real robot: yaw/pitch/roll + gyro{}/accel{}
data class ImuData(
    // Simulator
    val yaw_deg: Double = 0.0,
    val pitch_deg: Double = 0.0,
    val roll_deg: Double = 0.0,
    val gyro_z_rad_s: Double = 0.0,
    val accel_x_m_s2: Double = 0.0,
    // Real robot
    val yaw: Double = 0.0,
    val pitch: Double = 0.0,
    val roll: Double = 0.0,
    val gyro: GyroData = GyroData(),
    val accel: AccelData = AccelData(),
) {
    val yawDeg: Double   get() = if (yaw_deg != 0.0) yaw_deg else yaw
    val pitchDeg: Double get() = if (pitch_deg != 0.0) pitch_deg else pitch
    val rollDeg: Double  get() = if (roll_deg != 0.0) roll_deg else roll
    val gyroZ: Double    get() = if (gyro_z_rad_s != 0.0) gyro_z_rad_s else gyro.z
    val accelX: Double   get() = if (accel_x_m_s2 != 0.0) accel_x_m_s2 else accel.x
}

data class UltrasonicData(
    val range_m: Double = 2.0,
    val min_range: Double = 0.02,
    val max_range: Double = 2.0,
    val field_of_view_rad: Double = 0.26,
)

data class SensorData(
    val ultrasonic: UltrasonicData = UltrasonicData(),
    val imu: ImuData = ImuData(),
)

// ── Detection ────────────────────────────────────────────────────
// Simulator: colour/conf  |  Real robot: color/confidence
data class Detection(
    val colour: String = "",
    val color: String = "",
    val `class`: String = "",
    val x: Int = 0,
    val y: Int = 0,
    val w: Int = 0,
    val h: Int = 0,
    val conf: Double = 0.0,
    val confidence: Double = 0.0,
    val distance: Double = 0.0,
) {
    val displayColor: String get() = colour.ifEmpty { color }
    val displayConf: Double  get() = if (conf > 0) conf else confidence
}

data class DetectionResult(
    val count: Int = 0,
    val detections: List<Detection> = emptyList(), // simulator
    val objects: List<Detection> = emptyList(),    // real robot
) {
    val allDetections: List<Detection>
        get() = if (detections.isNotEmpty()) detections else objects
}

// ── Actuators ────────────────────────────────────────────────────
// Simulator: Boolean fields  |  Real robot: String fields "open"/"closed", "on"/"off"
data class ActuatorState(
    val claw_open: Boolean? = null,
    val laser_on: Boolean? = null,
    val claw: String? = null,
    val laser: String? = null,
) {
    val isClawOpen: Boolean get() = claw_open ?: (claw == "open")
    val isLaserOn: Boolean  get() = laser_on  ?: (laser == "on")
}

// ── FSM ──────────────────────────────────────────────────────────
data class FsmState(
    val state: String = "IDLE",
    val target_colour: String = "",
    val target_action: String = "",
    val all_states: List<String> = emptyList(),
)

// ── Arena (simulator) ────────────────────────────────────────────
data class ArenaInfo(
    val width: Double = 3.0,
    val height: Double = 3.0,
    val robot_radius: Double = 0.12,
    val ball_radius: Double = 0.02,
    val balls_total: Int = 0,
    val balls_remaining: Int = 0,
    val balls_grabbed: Int = 0,
)

data class BallInfo(
    val id: Int = 0,
    val colour: String = "",
    val x: Double = 0.0,
    val y: Double = 0.0,
    val radius: Double = 0.02,
    val grabbed: Boolean = false,
)

// ── Zones ────────────────────────────────────────────────────────
data class ForbiddenZone(
    val id: Int = 0,
    val x1: Double = 0.0,
    val y1: Double = 0.0,
    val x2: Double = 0.0,
    val y2: Double = 0.0,
)

// ── Map info ─────────────────────────────────────────────────────
data class MapInfo(
    // Simulator
    val width_px: Int = 0,
    val height_px: Int = 0,
    val resolution_m_per_px: Double = 0.01,
    val origin_x: Double = 0.0,
    val origin_y: Double = 0.0,
    val arena_width: Double = 3.0,
    val arena_height: Double = 3.0,
    // Real robot
    val width: Int = 0,
    val height: Int = 0,
    val resolution: Double = 0.05,
)

// ── Log entry ────────────────────────────────────────────────────
data class LogEntry(
    val text: String = "",
    val time: String = "",
    val type: String = "",
)

// ── Battery ──────────────────────────────────────────────────────
data class BatteryStatus(
    val voltage: Double = -1.0,
    val percentage: Int = -1,
    val status: String = "unknown",
)

// ── Temperature ──────────────────────────────────────────────────
data class TemperatureData(
    val value: Double = -1.0,
    val unit: String = "C",
)

// ── Watchdog ─────────────────────────────────────────────────────
data class WatchdogNode(
    val alive: Boolean = false,
    val last_seen: String = "",
)

// ── Autonomous modes ─────────────────────────────────────────────
data class PatrolStatus(
    val active: Boolean = false,
    val current_waypoint: Int = 0,
    val total_waypoints: Int = 0,
)

data class FollowMeStatus(
    val active: Boolean = false,
    val tracking: Boolean = false,
    val distance: Double = -1.0,
)

data class PathRecorderStatus(
    val recording: Boolean = false,
    val playing: Boolean = false,
    val current_name: String = "",
)

// ── Full aggregated robot state ───────────────────────────────────
data class RobotFullState(
    val pose: RobotPose = RobotPose(),
    val velocity: RobotVelocity = RobotVelocity(),
    val sensors: SensorData = SensorData(),
    val fsm: FsmState = FsmState(),
    val actuators: ActuatorState = ActuatorState(),
    val detections: DetectionResult = DetectionResult(),
    val closestDetection: Detection? = null,
    val arena: ArenaInfo = ArenaInfo(),
    val balls: List<BallInfo> = emptyList(),
    val zones: List<ForbiddenZone> = emptyList(),
    val log: List<LogEntry> = emptyList(),
    val path: List<List<Double>> = emptyList(),
    // New: real robot features
    val battery: BatteryStatus = BatteryStatus(),
    val temperature: TemperatureData = TemperatureData(),
    val watchdog: Map<String, WatchdogNode> = emptyMap(),
    val speedProfile: String = "normal",
    val patrol: PatrolStatus = PatrolStatus(),
    val followMe: FollowMeStatus = FollowMeStatus(),
    val pathRecorder: PathRecorderStatus = PathRecorderStatus(),
    val mapList: List<String> = emptyList(),
    val pathList: List<String> = emptyList(),
)
