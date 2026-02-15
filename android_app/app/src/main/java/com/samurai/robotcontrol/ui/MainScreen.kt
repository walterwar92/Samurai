package com.samurai.robotcontrol.ui

import android.content.Context
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.samurai.robotcontrol.mqtt.RobotMqttClient
import com.samurai.robotcontrol.voice.VoskRecognizer
import kotlinx.coroutines.launch

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainScreen(applicationContext: Context) {
    val scope = rememberCoroutineScope()

    // Services
    val mqttClient = remember { RobotMqttClient() }
    val vosk = remember { VoskRecognizer(applicationContext) }

    // State
    val isConnected by mqttClient.connected.collectAsState()
    val robotStatus by mqttClient.robotStatus.collectAsState()
    val recognisedText by vosk.recognisedText.collectAsState()
    val isListening by vosk.isListening.collectAsState()
    val modelReady by vosk.modelReady.collectAsState()

    var brokerIp by remember { mutableStateOf("192.168.1.100") }
    var robotId by remember { mutableStateOf("robot1") }
    var manualCommand by remember { mutableStateOf("") }
    var commandLog by remember { mutableStateOf(listOf<String>()) }

    // Init Vosk model on first composition
    LaunchedEffect(Unit) {
        vosk.initModel()
    }

    // Auto-send recognised text via MQTT
    LaunchedEffect(recognisedText) {
        if (recognisedText.isNotEmpty() && isConnected) {
            mqttClient.sendVoiceCommand(recognisedText)
            commandLog = (listOf("VOICE: $recognisedText") + commandLog).take(50)
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Samurai Robot", fontWeight = FontWeight.Bold) },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.primaryContainer
                )
            )
        }
    ) { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(16.dp)
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            // ── MQTT Connection ─────────────────────────────
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("MQTT Connection", fontSize = 18.sp, fontWeight = FontWeight.SemiBold)
                    Spacer(modifier = Modifier.height(8.dp))

                    OutlinedTextField(
                        value = brokerIp,
                        onValueChange = { brokerIp = it },
                        label = { Text("Broker IP") },
                        modifier = Modifier.fillMaxWidth(),
                        singleLine = true,
                    )
                    Spacer(modifier = Modifier.height(4.dp))
                    OutlinedTextField(
                        value = robotId,
                        onValueChange = { robotId = it },
                        label = { Text("Robot ID") },
                        modifier = Modifier.fillMaxWidth(),
                        singleLine = true,
                    )
                    Spacer(modifier = Modifier.height(8.dp))

                    Row(
                        horizontalArrangement = Arrangement.spacedBy(8.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Button(
                            onClick = {
                                if (isConnected) mqttClient.disconnect()
                                else mqttClient.connect(brokerIp, robotId)
                            }
                        ) {
                            Text(if (isConnected) "Отключить" else "Подключить")
                        }
                        Text(
                            text = if (isConnected) "Подключено" else "Отключено",
                            color = if (isConnected) Color(0xFF4CAF50) else Color.Red,
                            fontWeight = FontWeight.Bold,
                        )
                    }
                }
            }

            // ── Voice Recognition ───────────────────────────
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("Голосовое управление", fontSize = 18.sp, fontWeight = FontWeight.SemiBold)
                    Spacer(modifier = Modifier.height(8.dp))

                    Text(
                        text = if (!modelReady) "Загрузка модели Vosk..."
                        else if (isListening) "Слушаю..."
                        else "Готов к распознаванию",
                        color = if (isListening) Color(0xFF4CAF50) else MaterialTheme.colorScheme.onSurface,
                    )
                    Spacer(modifier = Modifier.height(8.dp))

                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(
                            onClick = { vosk.startListening() },
                            enabled = modelReady && !isListening,
                        ) {
                            Text("Слушать")
                        }
                        OutlinedButton(
                            onClick = { vosk.stopListening() },
                            enabled = isListening,
                        ) {
                            Text("Стоп")
                        }
                    }

                    if (recognisedText.isNotEmpty()) {
                        Spacer(modifier = Modifier.height(8.dp))
                        Text(
                            text = "Распознано: \"$recognisedText\"",
                            fontWeight = FontWeight.Medium,
                        )
                    }
                }
            }

            // ── Manual Command ──────────────────────────────
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("Ручная команда", fontSize = 18.sp, fontWeight = FontWeight.SemiBold)
                    Spacer(modifier = Modifier.height(8.dp))

                    OutlinedTextField(
                        value = manualCommand,
                        onValueChange = { manualCommand = it },
                        label = { Text("Введите команду") },
                        modifier = Modifier.fillMaxWidth(),
                        singleLine = true,
                    )
                    Spacer(modifier = Modifier.height(8.dp))

                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(
                            onClick = {
                                if (manualCommand.isNotBlank() && isConnected) {
                                    mqttClient.sendVoiceCommand(manualCommand)
                                    commandLog = (listOf("MANUAL: $manualCommand") + commandLog).take(50)
                                    manualCommand = ""
                                }
                            },
                            enabled = isConnected && manualCommand.isNotBlank(),
                        ) {
                            Text("Отправить")
                        }

                        // Quick buttons
                        FilledTonalButton(onClick = {
                            if (isConnected) {
                                mqttClient.sendVoiceCommand("стоп")
                                commandLog = (listOf("QUICK: стоп") + commandLog).take(50)
                            }
                        }, enabled = isConnected) {
                            Text("Стоп")
                        }

                        FilledTonalButton(onClick = {
                            if (isConnected) {
                                mqttClient.sendVoiceCommand("вызови вторую машину")
                                commandLog = (listOf("QUICK: вызови вторую машину") + commandLog).take(50)
                            }
                        }, enabled = isConnected) {
                            Text("Вызов")
                        }
                    }
                }
            }

            // ── Robot Status ────────────────────────────────
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("Статус робота", fontSize = 18.sp, fontWeight = FontWeight.SemiBold)
                    Spacer(modifier = Modifier.height(8.dp))
                    Text(
                        text = robotStatus.ifEmpty { "Нет данных" },
                        fontSize = 14.sp,
                    )
                }
            }

            // ── Command Log ─────────────────────────────────
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("Лог команд", fontSize = 18.sp, fontWeight = FontWeight.SemiBold)
                    Spacer(modifier = Modifier.height(8.dp))
                    if (commandLog.isEmpty()) {
                        Text("Пусто", fontSize = 14.sp, color = Color.Gray)
                    } else {
                        commandLog.forEach { entry ->
                            Text(text = entry, fontSize = 13.sp)
                        }
                    }
                }
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
