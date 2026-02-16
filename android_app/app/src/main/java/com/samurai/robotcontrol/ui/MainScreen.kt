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
import com.samurai.robotcontrol.api.RobotApiClient
import com.samurai.robotcontrol.mqtt.RobotMqttClient
import com.samurai.robotcontrol.ui.screens.*
import com.samurai.robotcontrol.voice.VoskRecognizer
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

enum class AppTab(val label: String, val icon: ImageVector) {
    CONTROL("Управление", Icons.Default.Gamepad),
    CAMERA("Камера", Icons.Default.Videocam),
    MAP("Карта", Icons.Default.Map),
    SENSORS("Датчики", Icons.Default.Sensors),
    SETTINGS("Настройки", Icons.Default.Settings),
}

enum class ConnectionMode { SIMULATOR, ROBOT }

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainScreen(applicationContext: Context) {
    val scope = rememberCoroutineScope()

    // Services
    val mqttClient = remember { RobotMqttClient() }
    val apiClient = remember { RobotApiClient() }
    val vosk = remember { VoskRecognizer(applicationContext) }

    // Connection state
    var connectionMode by remember { mutableStateOf(ConnectionMode.SIMULATOR) }
    var serverIp by remember { mutableStateOf("192.168.1.100") }
    var serverPort by remember { mutableStateOf("5000") }
    var robotId by remember { mutableStateOf("robot1") }

    val mqttConnected by mqttClient.connected.collectAsState()
    val apiConnected by apiClient.connected.collectAsState()
    val robotState by apiClient.state.collectAsState()
    val robotStatus by mqttClient.robotStatus.collectAsState()
    val recognisedText by vosk.recognisedText.collectAsState()
    val isListening by vosk.isListening.collectAsState()
    val modelReady by vosk.modelReady.collectAsState()

    var commandLog by remember { mutableStateOf(listOf<String>()) }
    var selectedTab by remember { mutableStateOf(AppTab.CONTROL) }

    // Camera & Map images (polled separately for performance)
    var cameraFrame by remember { mutableStateOf<Bitmap?>(null) }
    var mapImage by remember { mutableStateOf<Bitmap?>(null) }

    val isConnected = when (connectionMode) {
        ConnectionMode.SIMULATOR -> apiConnected
        ConnectionMode.ROBOT -> mqttConnected
    }

    // Init Vosk
    LaunchedEffect(Unit) {
        vosk.initModel()
    }

    // Auto-send recognised voice text
    LaunchedEffect(recognisedText) {
        if (recognisedText.isNotEmpty() && isConnected) {
            when (connectionMode) {
                ConnectionMode.SIMULATOR -> apiClient.sendCommand(recognisedText)
                ConnectionMode.ROBOT -> mqttClient.sendVoiceCommand(recognisedText)
            }
            commandLog = (listOf("VOICE: $recognisedText") + commandLog).take(50)
        }
    }

    // Poll state from API (simulator mode)
    LaunchedEffect(connectionMode, apiConnected) {
        if (connectionMode == ConnectionMode.SIMULATOR) {
            while (isActive) {
                apiClient.pollState()
                delay(250) // 4 Hz
            }
        }
    }

    // Poll camera frame
    LaunchedEffect(connectionMode, apiConnected, selectedTab) {
        if (connectionMode == ConnectionMode.SIMULATOR && selectedTab == AppTab.CAMERA) {
            while (isActive) {
                cameraFrame = apiClient.getCameraFrame()
                delay(100) // ~10 fps
            }
        }
    }

    // Poll map image
    LaunchedEffect(connectionMode, apiConnected, selectedTab) {
        if (connectionMode == ConnectionMode.SIMULATOR && selectedTab == AppTab.MAP) {
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
                ConnectionMode.ROBOT -> mqttClient.sendVoiceCommand(text)
            }
            commandLog = (listOf("CMD: $text") + commandLog).take(50)
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Text("SAMURAI", style = MaterialTheme.typography.titleMedium)
                },
                actions = {
                    val dotColor = if (isConnected)
                        MaterialTheme.colorScheme.primary
                    else
                        MaterialTheme.colorScheme.error
                    Badge(containerColor = dotColor) {
                        Text(
                            if (isConnected) "ON" else "OFF",
                            style = MaterialTheme.typography.labelSmall
                        )
                    }
                    Spacer(Modifier.width(androidx.compose.ui.unit.Dp(12f)))
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
                        onClick = { selectedTab = tab },
                        icon = { Icon(tab.icon, contentDescription = tab.label) },
                        label = { Text(tab.label, style = MaterialTheme.typography.labelSmall) }
                    )
                }
            }
        }
    ) { padding ->
        Box(modifier = Modifier.fillMaxSize().padding(padding)) {
            when (selectedTab) {
                AppTab.CONTROL -> ControlScreen(
                    robotState = robotState,
                    robotStatus = robotStatus,
                    connectionMode = connectionMode,
                    isConnected = isConnected,
                    isListening = isListening,
                    modelReady = modelReady,
                    recognisedText = recognisedText,
                    commandLog = commandLog,
                    vosk = vosk,
                    sendCommand = sendCommand,
                    onStop = { scope.launch { apiClient.stop() } },
                    onReset = { scope.launch { apiClient.resetSimulation() } },
                    onSetVelocity = { lin, ang -> scope.launch { apiClient.setVelocity(lin, ang) } },
                    onSetClaw = { open -> scope.launch { apiClient.setClaw(open) } },
                    onSetLaser = { on -> scope.launch { apiClient.setLaser(on) } },
                    onForceState = { s -> scope.launch { apiClient.forceTransition(s) } },
                )

                AppTab.CAMERA -> CameraScreen(
                    cameraFrame = cameraFrame,
                    detections = robotState.detections,
                    closestDetection = robotState.closestDetection,
                    connectionMode = connectionMode,
                    streamUrl = apiClient.getCameraStreamUrl(),
                )

                AppTab.MAP -> MapScreen(
                    mapImage = mapImage,
                    zones = robotState.zones,
                    balls = robotState.balls,
                    pose = robotState.pose,
                    path = robotState.path,
                    connectionMode = connectionMode,
                    onAddZone = { x1, y1, x2, y2 ->
                        scope.launch { apiClient.addZone(x1, y1, x2, y2) }
                    },
                    onDeleteZone = { id -> scope.launch { apiClient.deleteZone(id) } },
                    onClearZones = { scope.launch { apiClient.clearZones() } },
                )

                AppTab.SENSORS -> SensorsScreen(
                    robotState = robotState,
                    connectionMode = connectionMode,
                )

                AppTab.SETTINGS -> SettingsScreen(
                    connectionMode = connectionMode,
                    serverIp = serverIp,
                    serverPort = serverPort,
                    robotId = robotId,
                    isConnected = isConnected,
                    mqttConnected = mqttConnected,
                    apiConnected = apiConnected,
                    onModeChange = { connectionMode = it },
                    onServerIpChange = { serverIp = it },
                    onServerPortChange = { serverPort = it },
                    onRobotIdChange = { robotId = it },
                    onConnect = {
                        when (connectionMode) {
                            ConnectionMode.SIMULATOR -> {
                                apiClient.setBaseUrl("http://$serverIp:$serverPort")
                                scope.launch { apiClient.pollState() }
                            }
                            ConnectionMode.ROBOT -> {
                                mqttClient.connect(serverIp, robotId)
                            }
                        }
                    },
                    onDisconnect = {
                        when (connectionMode) {
                            ConnectionMode.SIMULATOR -> apiClient.setBaseUrl("")
                            ConnectionMode.ROBOT -> mqttClient.disconnect()
                        }
                    },
                )
            }
        }
    }

    // Cleanup
    DisposableEffect(Unit) {
        onDispose {
            vosk.destroy()
            mqttClient.disconnect()
        }
    }
}
