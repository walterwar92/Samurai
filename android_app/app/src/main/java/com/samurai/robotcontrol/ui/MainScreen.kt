package com.samurai.robotcontrol.ui

import android.content.Context
import android.graphics.Bitmap
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.unit.dp
import com.samurai.robotcontrol.api.RobotApiClient
import com.samurai.robotcontrol.mqtt.RobotMqttClient
import com.samurai.robotcontrol.ui.screens.*
import com.samurai.robotcontrol.voice.VoskRecognizer
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

enum class AppTab(val label: String, val icon: ImageVector) {
    CONTROL ("Управление", Icons.Default.Gamepad),
    CAMERA  ("Камера",     Icons.Default.Videocam),
    MAP     ("Карта",      Icons.Default.Map),
    SENSORS ("Датчики",    Icons.Default.Sensors),
    AUTO    ("Авто",       Icons.Default.SmartToy),
    SETTINGS("Настройки",  Icons.Default.Settings),
}

enum class ConnectionMode { SIMULATOR, ROBOT }

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainScreen(applicationContext: Context) {
    val scope = rememberCoroutineScope()

    // Services
    val mqttClient = remember { RobotMqttClient() }
    val apiClient  = remember { RobotApiClient() }
    val vosk       = remember { VoskRecognizer(applicationContext) }

    // Connection state
    var connectionMode by remember { mutableStateOf(ConnectionMode.SIMULATOR) }
    var serverIp   by remember { mutableStateOf("raspberrypi.local") }
    var serverPort by remember { mutableStateOf("5000") }
    var robotId    by remember { mutableStateOf("robot1") }

    val mqttConnected by mqttClient.connected.collectAsState()
    val apiConnected  by apiClient.connected.collectAsState()
    val robotState    by apiClient.state.collectAsState()
    val robotStatus   by mqttClient.robotStatus.collectAsState()
    val recognisedText by vosk.recognisedText.collectAsState()
    val isListening   by vosk.isListening.collectAsState()
    val modelReady    by vosk.modelReady.collectAsState()

    var commandLog  by remember { mutableStateOf(listOf<String>()) }
    var selectedTab by remember { mutableStateOf(AppTab.CONTROL) }

    // Camera & Map images
    var cameraFrame by remember { mutableStateOf<Bitmap?>(null) }
    var mapImage    by remember { mutableStateOf<Bitmap?>(null) }

    // Connection status — API is always used for data; MQTT for voice in robot mode
    val isApiConnected  = apiConnected
    val isMqttConnected = mqttConnected
    val isConnected = isApiConnected || (connectionMode == ConnectionMode.ROBOT && isMqttConnected)

    val isRealRobot = connectionMode == ConnectionMode.ROBOT

    // Init Vosk
    LaunchedEffect(Unit) { vosk.initModel() }

    // Auto-send recognised voice text
    LaunchedEffect(recognisedText) {
        if (recognisedText.isNotEmpty() && isConnected) {
            when (connectionMode) {
                ConnectionMode.SIMULATOR -> apiClient.sendCommand(recognisedText)
                ConnectionMode.ROBOT     -> {
                    mqttClient.sendVoiceCommand(recognisedText)
                    // Also send via API so FSM log captures it
                    if (isApiConnected) apiClient.sendCommand(recognisedText)
                }
            }
            commandLog = (listOf("VOICE: $recognisedText") + commandLog).take(50)
        }
    }

    // Poll state (runs in both modes — real robot needs API URL set too)
    LaunchedEffect(connectionMode, apiConnected) {
        if (_baseUrl_isSet(apiClient)) {
            while (isActive) {
                apiClient.pollState(isRealRobot = isRealRobot)
                delay(250) // 4 Hz
            }
        }
    }

    // Poll camera frame (active when Camera tab open)
    LaunchedEffect(connectionMode, apiConnected, selectedTab) {
        if (selectedTab == AppTab.CAMERA && apiConnected) {
            while (isActive) {
                cameraFrame = apiClient.getCameraFrame()
                delay(100) // ~10 fps
            }
        }
    }

    // Poll map image (active when Map tab open)
    LaunchedEffect(connectionMode, apiConnected, selectedTab) {
        if (selectedTab == AppTab.MAP && apiConnected) {
            while (isActive) {
                mapImage = apiClient.getMapImage()
                delay(300) // ~3 fps
            }
        }
    }

    // Send command helper
    val sendCommand: (String) -> Unit = { text ->
        scope.launch {
            when (connectionMode) {
                ConnectionMode.SIMULATOR -> apiClient.sendCommand(text)
                ConnectionMode.ROBOT     -> {
                    mqttClient.sendVoiceCommand(text)
                    if (isApiConnected) apiClient.sendCommand(text)
                }
            }
            commandLog = (listOf("CMD: $text") + commandLog).take(50)
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("SAMURAI", style = MaterialTheme.typography.titleMedium) },
                actions = {
                    // Battery chip (if available)
                    val bat = robotState.battery
                    if (bat.percentage in 0..100) {
                        val batColor = when {
                            bat.percentage <= 20 -> MaterialTheme.colorScheme.error
                            bat.percentage <= 50 -> MaterialTheme.colorScheme.tertiary
                            else                 -> MaterialTheme.colorScheme.primary
                        }
                        Text("${bat.percentage}%",
                            style = MaterialTheme.typography.labelSmall,
                            color = batColor)
                        Spacer(Modifier.width(6.dp))
                    }

                    // Connection dot
                    val dotColor = if (isConnected) MaterialTheme.colorScheme.primary
                                   else MaterialTheme.colorScheme.error
                    Badge(containerColor = dotColor) {
                        Text(if (isConnected) "ON" else "OFF",
                            style = MaterialTheme.typography.labelSmall)
                    }
                    Spacer(Modifier.width(12.dp))
                },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.surfaceVariant
                )
            )
        },
        bottomBar = {
            NavigationBar {
                AppTab.entries.forEach { tab ->
                    NavigationBarItem(
                        selected = selectedTab == tab,
                        onClick  = { selectedTab = tab },
                        icon     = { Icon(tab.icon, contentDescription = tab.label) },
                        label    = { Text(tab.label, style = MaterialTheme.typography.labelSmall) }
                    )
                }
            }
        }
    ) { padding ->
        Box(modifier = Modifier.fillMaxSize().padding(padding)) {
            when (selectedTab) {

                AppTab.CONTROL -> ControlScreen(
                    robotState      = robotState,
                    robotStatus     = robotStatus,
                    connectionMode  = connectionMode,
                    isConnected     = isConnected,
                    isListening     = isListening,
                    modelReady      = modelReady,
                    recognisedText  = recognisedText,
                    commandLog      = commandLog,
                    vosk            = vosk,
                    sendCommand     = sendCommand,
                    onStop          = { scope.launch { apiClient.stop() } },
                    onReset         = { scope.launch { apiClient.resetSimulation() } },
                    onSetVelocity   = { lin, ang -> scope.launch { apiClient.setVelocity(lin, ang) } },
                    onSetClaw       = { open -> scope.launch { apiClient.setClaw(open) } },
                    onSetLaser      = { on   -> scope.launch { apiClient.setLaser(on) } },
                    onForceState    = { s    -> scope.launch { apiClient.forceTransition(s) } },
                )

                AppTab.CAMERA -> CameraScreen(
                    cameraFrame      = cameraFrame,
                    detections       = robotState.detections,
                    closestDetection = robotState.closestDetection,
                    connectionMode   = connectionMode,
                    streamUrl        = apiClient.getCameraStreamUrl(),
                )

                AppTab.MAP -> MapScreen(
                    mapImage       = mapImage,
                    zones          = robotState.zones,
                    balls          = robotState.balls,
                    pose           = robotState.pose,
                    path           = robotState.path,
                    mapList        = robotState.mapList,
                    connectionMode = connectionMode,
                    isConnected    = isConnected,
                    onAddZone      = { x1, y1, x2, y2 ->
                        scope.launch { apiClient.addZone(x1, y1, x2, y2) }
                    },
                    onDeleteZone   = { id -> scope.launch { apiClient.deleteZone(id) } },
                    onClearZones   = { scope.launch { apiClient.clearZones() } },
                    onSaveMap      = { name -> scope.launch { apiClient.saveMap(name) } },
                    onLoadMap      = { name -> scope.launch { apiClient.loadMap(name) } },
                )

                AppTab.SENSORS -> SensorsScreen(
                    robotState     = robotState,
                    connectionMode = connectionMode,
                )

                AppTab.AUTO -> AutoScreen(
                    robotState   = robotState,
                    isConnected  = isConnected,
                    onSpeedProfile = { profile ->
                        scope.launch { apiClient.setSpeedProfile(profile) }
                    },
                    onPatrolCommand = { cmd ->
                        scope.launch { apiClient.setPatrolCommand(cmd) }
                    },
                    onPatrolWaypoints = { wps ->
                        scope.launch { apiClient.setPatrolWaypoints(wps) }
                    },
                    onFollowMe = { cmd ->
                        scope.launch { apiClient.setFollowMe(cmd) }
                    },
                    onPathRecorderCommand = { cmd, name ->
                        scope.launch { apiClient.setPathRecorderCommand(cmd, name) }
                    },
                    onSaveMap = { name -> scope.launch { apiClient.saveMap(name) } },
                    onLoadMap = { name -> scope.launch { apiClient.loadMap(name) } },
                )

                AppTab.SETTINGS -> SettingsScreen(
                    connectionMode    = connectionMode,
                    serverIp          = serverIp,
                    serverPort        = serverPort,
                    robotId           = robotId,
                    isConnected       = isConnected,
                    mqttConnected     = mqttConnected,
                    apiConnected      = apiConnected,
                    onModeChange      = { connectionMode = it },
                    onServerIpChange  = { serverIp = it },
                    onServerPortChange= { serverPort = it },
                    onRobotIdChange   = { robotId = it },
                    onConnect = {
                        // Always set API URL for both modes
                        apiClient.setBaseUrl("http://$serverIp:$serverPort")
                        when (connectionMode) {
                            ConnectionMode.SIMULATOR -> scope.launch { apiClient.pollState() }
                            ConnectionMode.ROBOT     -> {
                                mqttClient.connect(serverIp, robotId)
                                scope.launch { apiClient.pollState(isRealRobot = true) }
                            }
                        }
                    },
                    onDisconnect = {
                        apiClient.setBaseUrl("")
                        if (connectionMode == ConnectionMode.ROBOT) mqttClient.disconnect()
                    },
                )
            }
        }
    }

    DisposableEffect(Unit) {
        onDispose {
            vosk.destroy()
            mqttClient.disconnect()
        }
    }
}

/** Check if base URL has been set on the API client. */
private fun _baseUrl_isSet(client: RobotApiClient): Boolean =
    client.baseUrl.value.isNotEmpty()
