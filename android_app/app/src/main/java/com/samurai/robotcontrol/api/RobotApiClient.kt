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
 * REST API client for the Samurai robot simulator / real robot.
 * Base URL example: http://192.168.1.100:5000
 */
class RobotApiClient {

    companion object {
        private const val TAG = "RobotAPI"
        private val JSON_TYPE = "application/json; charset=utf-8".toMediaType()
    }

    private val gson = Gson()
    private val client = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(5, TimeUnit.SECONDS)
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
    // Poll all data (called periodically from UI)
    // ═══════════════════════════════════════════════════════════

    suspend fun pollState() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext

        try {
            val pose = getPose(base)
            val velocity = getVelocity(base)
            val sensors = getSensors(base)
            val fsm = getFsm(base)
            val actuators = getActuators(base)
            val detections = getDetections(base)
            val closest = getClosestDetection(base)
            val balls = getBalls(base)
            val zones = getZones(base)
            val log = getLog(base)
            val path = getPath(base)

            _state.value = RobotFullState(
                pose = pose,
                velocity = velocity,
                sensors = sensors,
                fsm = fsm,
                actuators = actuators,
                detections = detections,
                closestDetection = closest,
                balls = balls,
                zones = zones,
                log = log,
                path = path
            )
            _connected.value = true
        } catch (e: Exception) {
            Log.e(TAG, "Poll failed: ${e.message}")
            _connected.value = false
        }
    }

    // ═══════════════════════════════════════════════════════════
    // GET endpoints
    // ═══════════════════════════════════════════════════════════

    private fun getPose(base: String): RobotPose {
        val json = httpGet("$base/api/robot/pose") ?: return RobotPose()
        return parseData(json) ?: RobotPose()
    }

    private fun getVelocity(base: String): RobotVelocity {
        val json = httpGet("$base/api/robot/velocity") ?: return RobotVelocity()
        return parseData(json) ?: RobotVelocity()
    }

    private fun getSensors(base: String): SensorData {
        val json = httpGet("$base/api/sensors") ?: return SensorData()
        return parseData(json) ?: SensorData()
    }

    private fun getFsm(base: String): FsmState {
        val json = httpGet("$base/api/fsm") ?: return FsmState()
        return parseData(json) ?: FsmState()
    }

    private fun getActuators(base: String): ActuatorState {
        val json = httpGet("$base/api/actuators") ?: return ActuatorState()
        return parseData(json) ?: ActuatorState()
    }

    private fun getDetections(base: String): DetectionResult {
        val json = httpGet("$base/api/detection") ?: return DetectionResult()
        return parseData(json) ?: DetectionResult()
    }

    private fun getClosestDetection(base: String): Detection? {
        val json = httpGet("$base/api/detection/closest") ?: return null
        return parseData<Detection>(json)
    }

    private fun getBalls(base: String): List<BallInfo> {
        val json = httpGet("$base/api/arena/balls") ?: return emptyList()
        return parseDataList(json) ?: emptyList()
    }

    private fun getZones(base: String): List<ForbiddenZone> {
        val json = httpGet("$base/api/zones") ?: return emptyList()
        return parseDataList(json) ?: emptyList()
    }

    private fun getLog(base: String, limit: Int = 30): List<LogEntry> {
        val json = httpGet("$base/api/log?limit=$limit") ?: return emptyList()
        return parseDataList(json) ?: emptyList()
    }

    private fun getPath(base: String): List<List<Double>> {
        val json = httpGet("$base/api/path") ?: return emptyList()
        try {
            val obj = gson.fromJson(json, Map::class.java)
            @Suppress("UNCHECKED_CAST")
            val data = obj["data"] as? Map<String, Any> ?: return emptyList()
            @Suppress("UNCHECKED_CAST")
            return (data["path"] as? List<List<Double>>) ?: emptyList()
        } catch (e: Exception) {
            return emptyList()
        }
    }

    // ═══════════════════════════════════════════════════════════
    // POST endpoints (commands)
    // ═══════════════════════════════════════════════════════════

    suspend fun sendCommand(text: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/fsm/command", mapOf("text" to text))
    }

    suspend fun setVelocity(linear: Double, angular: Double) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/robot/velocity", mapOf("linear" to linear, "angular" to angular))
    }

    suspend fun stop() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/robot/stop", emptyMap<String, Any>())
    }

    suspend fun setClaw(open: Boolean) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/actuators/claw", mapOf("open" to open))
    }

    suspend fun setLaser(on: Boolean) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/actuators/laser", mapOf("on" to on))
    }

    suspend fun forceTransition(state: String) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/fsm/transition", mapOf("state" to state))
    }

    suspend fun addZone(x1: Double, y1: Double, x2: Double, y2: Double) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/zones", mapOf("x1" to x1, "y1" to y1, "x2" to x2, "y2" to y2))
    }

    suspend fun deleteZone(id: Int) = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpDelete("$base/api/zones/$id")
    }

    suspend fun clearZones() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/zones/clear", emptyMap<String, Any>())
    }

    suspend fun resetSimulation() = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext
        httpPost("$base/api/robot/reset", emptyMap<String, Any>())
    }

    // ═══════════════════════════════════════════════════════════
    // Image endpoints
    // ═══════════════════════════════════════════════════════════

    suspend fun getCameraFrame(): Bitmap? = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext null
        try {
            val request = Request.Builder().url("$base/api/camera/frame").build()
            client.newCall(request).execute().use { response ->
                if (!response.isSuccessful) return@withContext null
                val bytes = response.body?.bytes() ?: return@withContext null
                BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
            }
        } catch (e: Exception) {
            null
        }
    }

    suspend fun getMapImage(): Bitmap? = withContext(Dispatchers.IO) {
        val base = _baseUrl.value
        if (base.isEmpty()) return@withContext null
        try {
            val request = Request.Builder().url("$base/api/map/image").build()
            client.newCall(request).execute().use { response ->
                if (!response.isSuccessful) return@withContext null
                val bytes = response.body?.bytes() ?: return@withContext null
                BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
            }
        } catch (e: Exception) {
            null
        }
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
            client.newCall(request).execute().use { response ->
                if (response.isSuccessful) response.body?.string() else null
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
            client.newCall(request).execute().use { response ->
                response.body?.string()
            }
        } catch (e: Exception) {
            Log.e(TAG, "POST $url failed: ${e.message}")
            null
        }
    }

    private fun httpDelete(url: String): String? {
        return try {
            val request = Request.Builder().url(url).delete().build()
            client.newCall(request).execute().use { response ->
                response.body?.string()
            }
        } catch (e: Exception) {
            Log.e(TAG, "DELETE $url failed: ${e.message}")
            null
        }
    }

    private inline fun <reified T> parseData(json: String): T? {
        return try {
            val map = gson.fromJson(json, Map::class.java)
            val data = map["data"] ?: return null
            val dataJson = gson.toJson(data)
            gson.fromJson(dataJson, T::class.java)
        } catch (e: Exception) {
            Log.e(TAG, "Parse error: ${e.message}")
            null
        }
    }

    private inline fun <reified T> parseDataList(json: String): List<T>? {
        return try {
            val map = gson.fromJson(json, Map::class.java)
            val data = map["data"] ?: return null
            val dataJson = gson.toJson(data)
            val type = TypeToken.getParameterized(List::class.java, T::class.java).type
            gson.fromJson(dataJson, type)
        } catch (e: Exception) {
            Log.e(TAG, "Parse list error: ${e.message}")
            null
        }
    }
}
