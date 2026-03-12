package com.samurai.robotcontrol.mqtt

import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import org.eclipse.paho.client.mqttv3.*
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence

/**
 * MQTT client for communication with the Samurai robot.
 * Publishes voice commands to  samurai/<robotId>/voice_command
 * Subscribes to  samurai/<robotId>/status  for state monitoring.
 */
class RobotMqttClient {

    companion object {
        private const val TAG = "RobotMQTT"
        private const val QOS = 1
    }

    private var client: MqttAsyncClient? = null

    private val _connected = MutableStateFlow(false)
    val connected: StateFlow<Boolean> = _connected

    private val _robotStatus = MutableStateFlow("")
    val robotStatus: StateFlow<String> = _robotStatus

    private var robotId = "robot1"

    // ── Connect ─────────────────────────────────────────────
    fun connect(brokerIp: String, robotIdParam: String = "robot1") {
        robotId = robotIdParam
        val serverUri = "tcp://$brokerIp:1883"
        try {
            client = MqttAsyncClient(
                serverUri, "samurai_android_${System.currentTimeMillis()}",
                MemoryPersistence()
            )
            val options = MqttConnectOptions().apply {
                isAutomaticReconnect = true
                isCleanSession = true
                connectionTimeout = 10
            }
            client?.setCallback(object : MqttCallbackExtended {
                override fun connectComplete(reconnect: Boolean, serverURI: String?) {
                    Log.i(TAG, "Connected to $serverURI (reconnect=$reconnect)")
                    _connected.value = true
                    subscribeToStatus()
                }

                override fun connectionLost(cause: Throwable?) {
                    Log.w(TAG, "Connection lost", cause)
                    _connected.value = false
                }

                override fun messageArrived(topic: String?, message: MqttMessage?) {
                    val payload = message?.payload?.toString(Charsets.UTF_8) ?: return
                    if (topic?.endsWith("/status") == true) {
                        _robotStatus.value = payload
                    }
                }

                override fun deliveryComplete(token: IMqttDeliveryToken?) {}
            })
            client?.connect(options)
        } catch (e: Exception) {
            Log.e(TAG, "Connect failed", e)
        }
    }

    // ── Disconnect ──────────────────────────────────────────
    fun disconnect() {
        try {
            client?.disconnect()
            _connected.value = false
        } catch (e: Exception) {
            Log.e(TAG, "Disconnect failed", e)
        }
    }

    // ── Publish voice command ───────────────────────────────
    fun sendVoiceCommand(command: String) {
        val topic = "samurai/$robotId/voice_command"
        try {
            client?.publish(topic, MqttMessage(command.toByteArray(Charsets.UTF_8)).apply {
                qos = QOS
            })
            Log.i(TAG, "Sent: [$topic] $command")
        } catch (e: Exception) {
            Log.e(TAG, "Publish failed", e)
        }
    }

    // ── Publish cmd_vel (joystick — QoS 0, высокая частота) ──
    fun sendCmdVel(linear: Double, angular: Double) {
        val topic = "samurai/$robotId/cmd_vel"
        val payload = """{"linear_x":$linear,"angular_z":$angular}"""
        try {
            client?.publish(topic, MqttMessage(payload.toByteArray(Charsets.UTF_8)).apply {
                qos = 0
            })
        } catch (e: Exception) {
            Log.e(TAG, "sendCmdVel failed", e)
        }
    }

    // ── Subscribe to status ─────────────────────────────────
    private fun subscribeToStatus() {
        val topic = "samurai/$robotId/status"
        try {
            client?.subscribe(topic, QOS)
            Log.i(TAG, "Subscribed to $topic")
        } catch (e: Exception) {
            Log.e(TAG, "Subscribe failed", e)
        }
    }
}
