package com.samurai.robotcontrol.api

import android.util.Log
import com.google.gson.Gson
import io.socket.client.IO
import io.socket.client.Socket
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import org.json.JSONObject
import java.net.URI

/**
 * Socket.IO клиент для получения push-обновлений состояния робота.
 *
 * Сервер эмитирует событие `state_update` каждые 250ms (4 Hz).
 * Вместо HTTP-поллинга из MainActivity — push от сервера,
 * что устраняет HTTP-накладные расходы (~400 байт заголовков × 4/с).
 *
 * Использование:
 *   socketClient.connect("http://192.168.1.100:5000")
 *   // state обновляется автоматически при каждом событии сервера
 *   socketClient.disconnect()
 */
class RobotSocketClient {

    companion object {
        private const val TAG = "RobotSocket"
    }

    private val gson = Gson()
    private var socket: Socket? = null

    private val _connected = MutableStateFlow(false)
    val connected: StateFlow<Boolean> = _connected

    private val _state = MutableStateFlow(RobotFullState())
    val state: StateFlow<RobotFullState> = _state

    fun connect(serverUrl: String) {
        disconnect()
        try {
            val opts = IO.Options().apply {
                reconnection = true
                reconnectionAttempts = Int.MAX_VALUE
                reconnectionDelay = 1000
                timeout = 10000
            }
            socket = IO.socket(URI.create(serverUrl), opts).apply {
                on(Socket.EVENT_CONNECT) {
                    Log.i(TAG, "Socket.IO connected to $serverUrl")
                    _connected.value = true
                }
                on(Socket.EVENT_DISCONNECT) {
                    Log.i(TAG, "Socket.IO disconnected")
                    _connected.value = false
                }
                on(Socket.EVENT_CONNECT_ERROR) { args ->
                    Log.w(TAG, "Socket.IO connect error: ${args.firstOrNull()}")
                    _connected.value = false
                }
                on("state_update") { args ->
                    val json = args.firstOrNull() as? JSONObject ?: return@on
                    parseStateUpdate(json)
                }
                connect()
            }
        } catch (e: Exception) {
            Log.e(TAG, "Socket.IO init failed: ${e.message}")
        }
    }

    fun disconnect() {
        socket?.let {
            it.off()
            it.disconnect()
            it.close()
        }
        socket = null
        _connected.value = false
    }

    @Suppress("UNCHECKED_CAST")
    private fun parseStateUpdate(json: JSONObject) {
        try {
            val map = gson.fromJson(json.toString(), Map::class.java) as Map<*, *>

            // Pose
            val poseMap = map["pose"] as? Map<*, *>
            val pose = RobotPose(
                x   = poseMap.dbl("x"),
                y   = poseMap.dbl("y"),
                yaw = poseMap.dbl("yaw"),
            )

            // Velocity
            val velMap = map["velocity"] as? Map<*, *>
            val velocity = RobotVelocity(
                estimated = VelocityDetail(
                    linear_x  = velMap.dbl("linear_x"),
                    linear_y  = velMap.dbl("linear_y"),
                    angular_z = velMap.dbl("angular_z"),
                )
            )

            // Sensors
            val imuMap   = (map["imu"] as? Map<*, *>)
            val gyroMap  = (imuMap?.get("gyro")  as? Map<*, *>)
            val accelMap = (imuMap?.get("accel") as? Map<*, *>)
            val sensors = SensorData(
                ultrasonic = UltrasonicData(range_m = map.dbl("range_m", 2.0)),
                imu = ImuData(
                    yaw   = imuMap.dbl("yaw"),
                    pitch = imuMap.dbl("pitch"),
                    roll  = imuMap.dbl("roll"),
                    gyro  = GyroData(gyroMap.dbl("x"), gyroMap.dbl("y"), gyroMap.dbl("z")),
                    accel = AccelData(accelMap.dbl("x"), accelMap.dbl("y"), accelMap.dbl("z")),
                )
            )

            // FSM state
            val statusMap = map["status"] as? Map<*, *>
            val fsm = FsmState(
                state         = (statusMap?.get("state") as? String) ?: "IDLE",
                target_colour = (statusMap?.get("target_colour") as? String) ?: "",
                target_action = (statusMap?.get("target_action") as? String) ?: "",
            )

            // Actuators
            val actMap = map["actuators"] as? Map<*, *>
            val actuators = ActuatorState(
                claw  = actMap?.get("claw")  as? String,
                laser = actMap?.get("laser") as? String,
            )

            // Battery: Pi publishes "percent", старый формат — "percentage"
            val batMap = map["battery"] as? Map<*, *>
            val battery = BatteryStatus(
                voltage    = batMap.dbl("voltage", -1.0),
                percentage = (batMap?.get("percent") as? Double
                             ?: batMap?.get("percentage") as? Double ?: -1.0).toInt(),
                status     = (batMap?.get("status") as? String) ?: "unknown",
            )

            // Temperature
            val tempMap = map["temperature"] as? Map<*, *>
            val temperature = TemperatureData(
                value = tempMap.dbl("value", -1.0),
                unit  = (tempMap?.get("unit") as? String) ?: "C",
            )

            // Speed profile
            val speedProfile = (map["speed_profile"] as? String) ?: "normal"

            // Voice log
            val logList = map["voice_log"] as? List<*>
            val log = logList?.mapNotNull { entry ->
                val e = entry as? Map<*, *> ?: return@mapNotNull null
                LogEntry(
                    text = (e["text"] as? String) ?: "",
                    time = (e["time"] as? String) ?: "",
                    type = (e["type"] as? String) ?: "",
                )
            } ?: emptyList()

            _state.value = _state.value.copy(
                pose         = pose,
                velocity     = velocity,
                sensors      = sensors,
                fsm          = fsm,
                actuators    = actuators,
                battery      = battery,
                temperature  = temperature,
                speedProfile = speedProfile,
                log          = log,
            )
        } catch (e: Exception) {
            Log.e(TAG, "State parse error: ${e.message}")
        }
    }

    private fun Map<*, *>?.dbl(key: String, default: Double = 0.0): Double =
        (this?.get(key) as? Double) ?: default
}
