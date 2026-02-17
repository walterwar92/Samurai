package com.samurai.robotcontrol.api

/**
 * Data models for the Samurai Robot REST API.
 * Maps to JSON responses from simulator (port 5000) or robot API.
 */

// ── Generic API envelope ──
data class ApiResponse<T>(
    val ok: Boolean,
    val data: T? = null,
    val error: String? = null
)

// ── Robot Pose ──
data class RobotPose(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val theta: Double = 0.0,
    val theta_deg: Double = 0.0
)

// ── Velocity ──
data class RobotVelocity(
    val linear: Double = 0.0,
    val angular: Double = 0.0,
    val max_linear: Double = 0.3,
    val max_angular: Double = 2.0
)

// ── Sensors ──
data class ImuData(
    val yaw_deg: Double = 0.0,
    val pitch_deg: Double = 0.0,
    val roll_deg: Double = 0.0,
    val gyro_z_rad_s: Double = 0.0,
    val accel_x_m_s2: Double = 0.0
)

data class UltrasonicData(
    val range_m: Double = 2.0,
    val min_range: Double = 0.02,
    val max_range: Double = 2.0,
    val field_of_view_rad: Double = 0.26
)

data class SensorData(
    val ultrasonic: UltrasonicData = UltrasonicData(),
    val imu: ImuData = ImuData()
)

// ── Detection ──
data class Detection(
    val colour: String = "",
    val `class`: String = "",
    val x: Int = 0,
    val y: Int = 0,
    val w: Int = 0,
    val h: Int = 0,
    val conf: Double = 0.0,
    val distance: Double = 0.0
)

data class DetectionResult(
    val count: Int = 0,
    val detections: List<Detection> = emptyList()
)

// ── Actuators ──
data class ActuatorState(
    val claw_open: Boolean = false,
    val laser_on: Boolean = false
)

// ── FSM ──
data class FsmState(
    val state: String = "IDLE",
    val target_colour: String = "",
    val target_action: String = "",
    val all_states: List<String> = emptyList()
)

// ── Arena ──
data class ArenaInfo(
    val width: Double = 3.0,
    val height: Double = 3.0,
    val robot_radius: Double = 0.12,
    val ball_radius: Double = 0.02,
    val balls_total: Int = 0,
    val balls_remaining: Int = 0,
    val balls_grabbed: Int = 0
)

data class BallInfo(
    val id: Int = 0,
    val colour: String = "",
    val x: Double = 0.0,
    val y: Double = 0.0,
    val radius: Double = 0.02,
    val grabbed: Boolean = false
)

// ── Zones ──
data class ForbiddenZone(
    val id: Int = 0,
    val x1: Double = 0.0,
    val y1: Double = 0.0,
    val x2: Double = 0.0,
    val y2: Double = 0.0
)

// ── Map info ──
data class MapInfo(
    val width_px: Int = 0,
    val height_px: Int = 0,
    val resolution_m_per_px: Double = 0.01,
    val origin_x: Double = 0.0,
    val origin_y: Double = 0.0,
    val arena_width: Double = 3.0,
    val arena_height: Double = 3.0
)

// ── Log entry ──
data class LogEntry(
    val text: String = "",
    val time: String = ""
)

// ── Full robot state (aggregated for polling) ──
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
    val path: List<List<Double>> = emptyList()
)
