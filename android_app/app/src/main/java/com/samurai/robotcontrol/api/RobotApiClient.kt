package com.samurai.robotcontrol.api

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Log
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.withContext
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import java.util.concurrent.TimeUnit

/**
 * REST API client for the Samurai robot / simulator.
 * Handles both simulator (nested "data" responses) and real robot (flat responses).
 */
class RobotApiClient {

    companion object {
        private const val TAG = "RobotAPI"
        private val JSON_TYPE = "application/json; charset=utf-8".toMediaType()
    }

    private val gson = Gson()
    private val client = OkHttpClient.Builder()
        .connectTimeout(10, TimeUnit.SECONDS)  // 10s — mDNS (.local) может резолвиться дольше
        .readTimeout(10, TimeUnit.SECONDS)
        .writeTimeout(5, TimeUnit.SECONDS)
        .build()

    private val _baseUrl = MutableStateFlow("")
    val baseUrl: StateFlow<String> = _baseUrl

    private val _connected = MutableStateFlow(false)
    val connected: StateFlow<Boolean> = _connected

    private val _state = MutableStateFlow(RobotFullState())
    val state: StateFlow<RobotFullState> = _state

    fun setBaseUrl(url: String) {
        _baseUrl.value = url.trimEnd('/')
    }

    // ═══════════════════════════════════════════════════════════
    // Poll all data
    // ═══════════════════════════════════════════════════════════

    /**
     * isRealRobot = true → parse /api/status from the real robot dashboard format
     * isRealRobot = false → poll individual simulator endpoints
     */
    suspend fun pollState(isRealRobot: Boolean = false) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        try {
            if (isRealRobot) pollRealRobot(base) else pollSimulator(base)
            _connected.value = true
        } catch (e: Exception) {
            Log.e(TAG, "Poll failed: ${e.message}")
            _connected.value = false
        }
    }

    // ── Simulator polling (individual endpoints) ─────────────────

    private fun pollSimulator(base: String) {
        val pose        = getPose(base)
        val velocity    = getVelocity(base)
        val sensors     = getSensors(base)
        val fsm         = getFsm(base)
        val actuators   = getActuators(base)
        val detections  = getDetections(base)
        val closest     = getClosestDetection(base)
        val balls       = getBalls(base)
        val zones       = getZones(base)
        val log         = getLog(base)
        val path        = getPath(base)

        _state.value = _state.value.copy(
            pose            = pose,
            velocity        = velocity,
            sensors         = sensors,
            fsm             = fsm,
            actuators       = actuators,
            detections      = detections,
            closestDetection = closest,
            balls           = balls,
            zones           = zones,
            log             = log,
            path            = path,
        )
    }

    // ── Real robot polling (/api/status) ─────────────────────────

    @Suppress("UNCHECKED_CAST")
    private fun pollRealRobot(base: String) {
        val statusJson = httpGet("$base/api/status") ?: return
        val map = gson.fromJson(statusJson, Map::class.java)

        // Pose: {x, y, yaw}
        val poseMap = map["pose"] as? Map<*, *>
        val pose = RobotPose(
            x   = poseMap.dbl("x"),
            y   = poseMap.dbl("y"),
            yaw = poseMap.dbl("yaw"),
        )

        // Velocity: {linear_x, linear_y, angular_z}
        val velMap = map["velocity"] as? Map<*, *>
        val velocity = RobotVelocity(
            estimated = VelocityDetail(
                linear_x  = velMap.dbl("linear_x"),
                linear_y  = velMap.dbl("linear_y"),
                angular_z = velMap.dbl("angular_z"),
            )
        )

        // Sensors: {ultrasonic: {range_m}, imu: {yaw, pitch, roll, gyro, accel}}
        val sensorsMap  = map["sensors"] as? Map<*, *>
        val ultMap      = (sensorsMap?.get("ultrasonic") as? Map<*, *>)
        val imuMap      = (sensorsMap?.get("imu") as? Map<*, *>)
        val gyroMap     = (imuMap?.get("gyro") as? Map<*, *>)
        val accelMap    = (imuMap?.get("accel") as? Map<*, *>)
        val sensors = SensorData(
            ultrasonic = UltrasonicData(range_m = ultMap.dbl("range_m", 2.0)),
            imu = ImuData(
                yaw   = imuMap.dbl("yaw"),
                pitch = imuMap.dbl("pitch"),
                roll  = imuMap.dbl("roll"),
                gyro  = GyroData(gyroMap.dbl("x"), gyroMap.dbl("y"), gyroMap.dbl("z")),
                accel = AccelData(accelMap.dbl("x"), accelMap.dbl("y"), accelMap.dbl("z")),
            )
        )

        // FSM: robot_status → {state, target_colour, target_action}
        val statusMap = map["robot_status"] as? Map<*, *>
        val fsm = FsmState(
            state         = (statusMap?.get("state") as? String) ?: "IDLE",
            target_colour = (statusMap?.get("target_colour") as? String) ?: "",
            target_action = (statusMap?.get("target_action") as? String) ?: "",
        )

        // Actuators: {claw: "open"/"closed", laser: "on"/"off"}
        val actMap = map["actuators"] as? Map<*, *>
        val actuators = ActuatorState(
            claw  = actMap?.get("claw")  as? String,
            laser = actMap?.get("laser") as? String,
        )

        // Battery: {voltage, percentage, status}
        val batMap = map["battery"] as? Map<*, *>
        val battery = BatteryStatus(
            voltage    = batMap.dbl("voltage", -1.0),
            percentage = batMap.dbl("percentage", -1.0).toInt(),
            status     = (batMap?.get("status") as? String) ?: "unknown",
        )

        // Temperature: {value, unit}
        val tempMap = map["temperature"] as? Map<*, *>
        val temperature = TemperatureData(
            value = tempMap.dbl("value", -1.0),
            unit  = (tempMap?.get("unit") as? String) ?: "C",
        )

        // Speed profile
        val speedProfile = (map["speed_profile"] as? String) ?: "normal"

        // Event log: [{text, time, type}]
        val logList = map["event_log"] as? List<*>
        val log = logList?.mapNotNull { entry ->
            val e = entry as? Map<*, *> ?: return@mapNotNull null
            LogEntry(
                text = (e["text"] as? String) ?: "",
                time = (e["time"] as? String) ?: "",
                type = (e["type"] as? String) ?: "",
            )
        } ?: emptyList()

        // Map list and path list (separate fast calls)
        val mapList  = fetchStringList("$base/api/map/list", "maps")
        val pathList = fetchStringList("$base/api/path_recorder/list", "paths")

        _state.value = _state.value.copy(
            pose          = pose,
            velocity      = velocity,
            sensors       = sensors,
            fsm           = fsm,
            actuators     = actuators,
            battery       = battery,
            temperature   = temperature,
            speedProfile  = speedProfile,
            log           = log,
            mapList       = mapList,
            pathList      = pathList,
        )
    }

    // Helper extension for Map<*, *>
    private fun Map<*, *>?.dbl(key: String, default: Double = 0.0): Double =
        (this?.get(key) as? Double) ?: default

    // ═══════════════════════════════════════════════════════════
    // GET endpoints (simulator)
    // ═══════════════════════════════════════════════════════════

    private fun getPose(base: String): RobotPose =
        parseData(httpGet("$base/api/robot/pose")) ?: RobotPose()

    private fun getVelocity(base: String): RobotVelocity =
        parseData(httpGet("$base/api/robot/velocity")) ?: RobotVelocity()

    private fun getSensors(base: String): SensorData =
        parseData(httpGet("$base/api/sensors")) ?: SensorData()

    private fun getFsm(base: String): FsmState =
        parseData(httpGet("$base/api/fsm")) ?: FsmState()

    private fun getActuators(base: String): ActuatorState =
        parseData(httpGet("$base/api/actuators")) ?: ActuatorState()

    private fun getDetections(base: String): DetectionResult =
        parseData(httpGet("$base/api/detection")) ?: DetectionResult()

    private fun getClosestDetection(base: String): Detection? =
        parseData<Detection>(httpGet("$base/api/detection/closest"))

    private fun getBalls(base: String): List<BallInfo> =
        parseDataList(httpGet("$base/api/arena/balls"), "balls") ?: emptyList()

    private fun getZones(base: String): List<ForbiddenZone> =
        parseDataList(httpGet("$base/api/zones"), "zones") ?: emptyList()

    private fun getLog(base: String, limit: Int = 30): List<LogEntry> =
        parseDataList(httpGet("$base/api/log?limit=$limit"), "log") ?: emptyList()

    private fun getPath(base: String): List<List<Double>> {
        val json = httpGet("$base/api/path") ?: return emptyList()
        return try {
            val obj = gson.fromJson(json, Map::class.java)
            @Suppress("UNCHECKED_CAST")
            val data = obj["data"] as? Map<String, Any> ?: return emptyList()
            @Suppress("UNCHECKED_CAST")
            (data["path"] as? List<List<Double>>) ?: emptyList()
        } catch (e: Exception) {
            emptyList()
        }
    }

    // ═══════════════════════════════════════════════════════════
    // POST endpoints — commands
    // ═══════════════════════════════════════════════════════════

    suspend fun sendCommand(text: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        // Both simulator and real robot accept "command" key
        httpPost("$base/api/fsm/command", mapOf("command" to text))
    }

    suspend fun setVelocity(linear: Double, angular: Double) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/robot/velocity", mapOf("linear" to linear, "angular" to angular))
    }

    suspend fun stop() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/robot/stop", emptyMap<String, Any>())
    }

    suspend fun setClaw(open: Boolean) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        // Send both formats: real robot uses "state" string, simulator uses boolean
        httpPost("$base/api/actuators/claw",
            mapOf("state" to if (open) "open" else "close", "open" to open))
    }

    suspend fun setLaser(on: Boolean) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/actuators/laser",
            mapOf("state" to if (on) "on" else "off", "on" to on))
    }

    suspend fun setSpeedProfile(profile: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/speed_profile", mapOf("profile" to profile))
    }

    suspend fun setPatrolCommand(command: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/patrol/command", mapOf("command" to command))
    }

    suspend fun setPatrolWaypoints(waypoints: List<List<Double>>) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/patrol/waypoints", mapOf("waypoints" to waypoints))
    }

    suspend fun setFollowMe(command: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/follow_me", mapOf("command" to command))
    }

    suspend fun setPathRecorderCommand(command: String, name: String? = null) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        val body = mutableMapOf<String, Any>("command" to command)
        if (name != null) body["name"] = name
        httpPost("$base/api/path_recorder/command", body)
    }

    suspend fun saveMap(name: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/map/save", mapOf("name" to name))
    }

    suspend fun loadMap(name: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/map/load", mapOf("name" to name))
    }

    suspend fun forceTransition(state: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/fsm/transition", mapOf("state" to state))
    }

    suspend fun addZone(x1: Double, y1: Double, x2: Double, y2: Double) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/zones", mapOf("x1" to x1, "y1" to y1, "x2" to x2, "y2" to y2))
    }

    suspend fun deleteZone(id: Int) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpDelete("$base/api/zones/$id")
    }

    suspend fun clearZones() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/zones/clear", emptyMap<String, Any>())
    }

    suspend fun resetSimulation() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext }
        httpPost("$base/api/robot/reset", emptyMap<String, Any>())
    }

    // ═══════════════════════════════════════════════════════════
    // Image endpoints
    // ═══════════════════════════════════════════════════════════

    suspend fun getCameraFrame(): Bitmap? = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext null }
        try {
            val request = Request.Builder().url("$base/api/camera/frame").build()
            client.newCall(request).execute().use { resp ->
                if (!resp.isSuccessful) return@withContext null
                val bytes = resp.body?.bytes() ?: return@withContext null
                BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
            }
        } catch (e: Exception) { null }
    }

    suspend fun getMapImage(): Bitmap? = withContext(Dispatchers.IO) {
        val base = _baseUrl.value.ifEmpty { return@withContext null }
        try {
            val request = Request.Builder().url("$base/api/map/image").build()
            client.newCall(request).execute().use { resp ->
                if (!resp.isSuccessful) return@withContext null
                val bytes = resp.body?.bytes() ?: return@withContext null
                BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
            }
        } catch (e: Exception) { null }
    }

    fun getCameraStreamUrl(): String {
        val base = _baseUrl.value
        return if (base.isNotEmpty()) "$base/video_feed" else ""
    }

    // ═══════════════════════════════════════════════════════════
    // HTTP helpers
    // ═══════════════════════════════════════════════════════════

    private fun httpGet(url: String): String? {
        return try {
            val request = Request.Builder().url(url).build()
            client.newCall(request).execute().use { resp ->
                if (resp.isSuccessful) resp.body?.string() else null
            }
        } catch (e: Exception) {
            Log.e(TAG, "GET $url failed: ${e.message}")
            null
        }
    }

    private fun httpPost(url: String, body: Any): String? {
        return try {
            val json = gson.toJson(body)
            val requestBody = json.toRequestBody(JSON_TYPE)
            val request = Request.Builder().url(url).post(requestBody).build()
            client.newCall(request).execute().use { resp -> resp.body?.string() }
        } catch (e: Exception) {
            Log.e(TAG, "POST $url failed: ${e.message}")
            null
        }
    }

    private fun httpDelete(url: String): String? {
        return try {
            val request = Request.Builder().url(url).delete().build()
            client.newCall(request).execute().use { resp -> resp.body?.string() }
        } catch (e: Exception) {
            Log.e(TAG, "DELETE $url failed: ${e.message}")
            null
        }
    }

    // ═══════════════════════════════════════════════════════════
    // Parsing helpers
    // ═══════════════════════════════════════════════════════════

    /**
     * Tries "data" key first (simulator format), then falls back to
     * the root object minus "ok"/"error" (real robot flat format).
     */
    private inline fun <reified T> parseData(json: String?): T? {
        json ?: return null
        return try {
            val map = gson.fromJson(json, Map::class.java)
            val src = if (map.containsKey("data")) {
                gson.toJson(map["data"])
            } else {
                gson.toJson(map.filterKeys { it != "ok" && it != "error" })
            }
            gson.fromJson(src, T::class.java)
        } catch (e: Exception) {
            Log.e(TAG, "Parse error: ${e.message}")
            null
        }
    }

    /**
     * Tries "data" key first, then the specified [key], then the root list.
     */
    private inline fun <reified T> parseDataList(json: String?, key: String = "data"): List<T>? {
        json ?: return null
        return try {
            val map = gson.fromJson(json, Map::class.java)
            val listSrc = map["data"] ?: map[key] ?: return emptyList()
            val listJson = gson.toJson(listSrc)
            val type = TypeToken.getParameterized(List::class.java, T::class.java).type
            gson.fromJson(listJson, type)
        } catch (e: Exception) {
            Log.e(TAG, "Parse list error: ${e.message}")
            null
        }
    }

    @Suppress("UNCHECKED_CAST")
    private fun fetchStringList(url: String, key: String): List<String> {
        val json = httpGet(url) ?: return emptyList()
        return try {
            val map = gson.fromJson(json, Map::class.java)
            (map[key] as? List<String>) ?: emptyList()
        } catch (e: Exception) {
            emptyList()
        }
    }
}
