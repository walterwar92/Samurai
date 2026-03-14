package com.samurai.robotcontrol.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.samurai.robotcontrol.api.RobotFullState
import com.samurai.robotcontrol.ui.ConnectionMode
import com.samurai.robotcontrol.voice.VoskRecognizer
import kotlin.math.abs
import kotlin.math.sqrt

private val FSM_STATES = listOf(
    "IDLE", "SEARCHING", "TARGETING", "APPROACHING",
    "GRABBING", "BURNING", "CALLING", "RETURNING"
)

private val FSM_COLORS = mapOf(
    "IDLE"       to Color(0xFF607D8B),
    "SEARCHING"  to Color(0xFF4CAF50),
    "TARGETING"  to Color(0xFFFF9800),
    "APPROACHING" to Color(0xFFE65100),
    "GRABBING"   to Color(0xFF9C27B0),
    "BURNING"    to Color(0xFFF44336),
    "CALLING"    to Color(0xFF2196F3),
    "RETURNING"  to Color(0xFF8BC34A),
)

private val COLOUR_MAP = mapOf(
    "red"    to Color(0xFFEF5350), "blue"   to Color(0xFF42A5F5),
    "green"  to Color(0xFF66BB6A), "yellow" to Color(0xFFFFEB3B),
    "orange" to Color(0xFFFF9800),
)

private val COLOUR_RU = mapOf(
    "red"    to "красный", "blue"   to "синий",
    "green"  to "зелёный","yellow" to "жёлтый",
    "orange" to "оранжевый",
)

@OptIn(ExperimentalLayoutApi::class, ExperimentalMaterial3Api::class)
@Composable
fun ControlScreen(
    robotState: RobotFullState,
    robotStatus: String,
    connectionMode: ConnectionMode,
    isConnected: Boolean,
    isListening: Boolean,
    modelReady: Boolean,
    recognisedText: String,
    commandLog: List<String>,
    vosk: VoskRecognizer,
    sendCommand: (String) -> Unit,
    onStop: () -> Unit,
    onReset: () -> Unit,
    onSetVelocity: (Double, Double) -> Unit,
    onSetClaw: (Boolean) -> Unit,
    onSetLaser: (Boolean) -> Unit,
    onForceState: (String) -> Unit,
) {
    var manualCommand by remember { mutableStateOf("") }
    val state = robotState.fsm.state
    val color = FSM_COLORS[state] ?: Color.Gray

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {

        // ── Emergency Stop ────────────────────────────────────────
        Button(
            onClick = onStop,
            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFD32F2F)),
            shape = RoundedCornerShape(8.dp),
            enabled = isConnected
        ) {
            Icon(Icons.Default.Emergency, contentDescription = null,
                modifier = Modifier.size(20.dp))
            Spacer(Modifier.width(8.dp))
            Text("ЭКСТРЕННАЯ ОСТАНОВКА", fontSize = 15.sp, fontWeight = FontWeight.Bold,
                letterSpacing = 1.sp)
        }

        // ── FSM State ─────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Состояние FSM", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))

                Surface(
                    color = color.copy(alpha = 0.2f),
                    shape = RoundedCornerShape(6.dp),
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Text(
                        text = state,
                        color = color,
                        fontWeight = FontWeight.Bold,
                        fontSize = 18.sp,
                        modifier = Modifier.padding(8.dp),
                        textAlign = TextAlign.Center
                    )
                }

                Spacer(Modifier.height(6.dp))
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Column {
                        Text("Цель", fontSize = 11.sp, color = Color.Gray)
                        val tc = robotState.fsm.target_colour
                        Text(
                            COLOUR_RU[tc] ?: tc.ifEmpty { "—" },
                            color = COLOUR_MAP[tc] ?: MaterialTheme.colorScheme.onSurface,
                            fontWeight = FontWeight.Medium
                        )
                    }
                    Column {
                        Text("Действие", fontSize = 11.sp, color = Color.Gray)
                        val act = robotState.fsm.target_action
                        Text(
                            when (act) { "grab" -> "захват"; "burn" -> "лазер"; else -> act.ifEmpty { "—" } },
                            fontWeight = FontWeight.Medium
                        )
                    }
                    Column {
                        Text("Позиция", fontSize = 11.sp, color = Color.Gray)
                        Text(
                            "%.2f, %.2f".format(robotState.pose.x, robotState.pose.y),
                            fontWeight = FontWeight.Medium
                        )
                    }
                }

                // Force transition (simulator)
                if (connectionMode == ConnectionMode.SIMULATOR) {
                    Spacer(Modifier.height(8.dp))
                    Text("Принудительный переход", fontSize = 10.sp, color = Color.Gray)
                    Spacer(Modifier.height(4.dp))
                    FlowRow(
                        horizontalArrangement = Arrangement.spacedBy(4.dp),
                        verticalArrangement = Arrangement.spacedBy(4.dp),
                    ) {
                        FSM_STATES.forEach { s ->
                            FilterChip(
                                selected = s == state,
                                onClick = { onForceState(s) },
                                label = { Text(s, fontSize = 10.sp) },
                            )
                        }
                    }
                }
            }
        }

        // ── Quick Commands ────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Быстрые команды", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))

                FlowRow(
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                    verticalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    COLOUR_RU.forEach { (en, ru) ->
                        Button(
                            onClick = { sendCommand("найди $ru мяч") },
                            enabled = isConnected,
                            colors = ButtonDefaults.buttonColors(
                                containerColor = (COLOUR_MAP[en] ?: Color.Gray).copy(alpha = 0.2f),
                                contentColor = COLOUR_MAP[en] ?: Color.Gray
                            ),
                            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
                            modifier = Modifier.height(36.dp)
                        ) {
                            Text(ru.replaceFirstChar { it.uppercaseChar() }, fontSize = 12.sp)
                        }
                    }
                }

                Spacer(Modifier.height(6.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    OutlinedButton(
                        onClick = { sendCommand("домой") },
                        enabled = isConnected,
                        modifier = Modifier.weight(1f)
                    ) { Text("Домой") }
                    OutlinedButton(
                        onClick = { sendCommand("сожги лазером") },
                        enabled = isConnected,
                        modifier = Modifier.weight(1f)
                    ) { Text("Лазер") }
                    OutlinedButton(
                        onClick = { sendCommand("вызови вторую машину") },
                        enabled = isConnected,
                        modifier = Modifier.weight(1f)
                    ) { Text("Вызов") }
                }

                if (connectionMode == ConnectionMode.SIMULATOR) {
                    Spacer(Modifier.height(4.dp))
                    OutlinedButton(
                        onClick = onReset,
                        modifier = Modifier.fillMaxWidth(),
                        colors = ButtonDefaults.outlinedButtonColors(contentColor = Color(0xFFFF9800))
                    ) { Text("Сброс симуляции") }
                }
            }
        }

        // ── Actuators (available in both modes) ───────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Актуаторы", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    val clawOpen = robotState.actuators.isClawOpen
                    Button(
                        onClick = { onSetClaw(!clawOpen) },
                        enabled = isConnected,
                        colors = ButtonDefaults.buttonColors(
                            containerColor = if (clawOpen) Color(0xFF4CAF50).copy(alpha = 0.2f)
                                            else MaterialTheme.colorScheme.surfaceVariant,
                            contentColor   = if (clawOpen) Color(0xFF4CAF50)
                                            else MaterialTheme.colorScheme.onSurface
                        ),
                        modifier = Modifier.weight(1f)
                    ) {
                        Icon(Icons.Default.PanTool, null, Modifier.size(14.dp))
                        Spacer(Modifier.width(4.dp))
                        Text("Клешня: ${if (clawOpen) "ОТКР" else "ЗАКР"}", fontSize = 12.sp)
                    }

                    val laserOn = robotState.actuators.isLaserOn
                    Button(
                        onClick = { onSetLaser(!laserOn) },
                        enabled = isConnected,
                        colors = ButtonDefaults.buttonColors(
                            containerColor = if (laserOn) Color(0xFFF44336).copy(alpha = 0.2f)
                                            else MaterialTheme.colorScheme.surfaceVariant,
                            contentColor   = if (laserOn) Color(0xFFF44336)
                                            else MaterialTheme.colorScheme.onSurface
                        ),
                        modifier = Modifier.weight(1f)
                    ) {
                        Icon(Icons.Default.FlashOn, null, Modifier.size(14.dp))
                        Spacer(Modifier.width(4.dp))
                        Text("Лазер: ${if (laserOn) "ВКЛ" else "ВЫКЛ"}", fontSize = 12.sp)
                    }
                }
            }
        }

        // ── Joystick (available in both modes) ────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(
                modifier = Modifier.padding(12.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Text("Управление скоростью", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))
                JoystickControl(
                    enabled = isConnected,
                    onSetVelocity = onSetVelocity,
                    onStop = onStop
                )
            }
        }

        // ── Voice Recognition ─────────────────────────────────────
        val modelError       by vosk.modelError.collectAsState()
        val downloadProgress by vosk.downloadProgress.collectAsState()
        val isDownloading = downloadProgress in 0..100
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Голосовое управление", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))
                Text(
                    text = when {
                        modelError != null -> "Ошибка: $modelError"
                        isDownloading      -> "Скачивание модели... $downloadProgress%"
                        !modelReady        -> "Инициализация..."
                        isListening        -> "Слушаю..."
                        else               -> "Готов"
                    },
                    color = when {
                        modelError != null -> Color(0xFFEF5350)
                        isListening        -> Color(0xFF4CAF50)
                        else               -> Color.Gray
                    },
                    fontSize = 13.sp
                )
                if (isDownloading) {
                    Spacer(Modifier.height(4.dp))
                    LinearProgressIndicator(
                        progress = downloadProgress / 100f,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
                Spacer(Modifier.height(6.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = { vosk.startListening() },
                        enabled = modelReady && !isListening,
                    ) { Text("Слушать") }
                    OutlinedButton(
                        onClick = { vosk.stopListening() },
                        enabled = isListening,
                    ) { Text("Стоп") }
                }
                if (recognisedText.isNotEmpty()) {
                    Spacer(Modifier.height(4.dp))
                    Text("\"$recognisedText\"", fontWeight = FontWeight.Medium, fontSize = 13.sp)
                }
            }
        }

        // ── Manual Command ────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Ручная команда", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedTextField(
                        value = manualCommand,
                        onValueChange = { manualCommand = it },
                        label = { Text("Команда", fontSize = 12.sp) },
                        modifier = Modifier.weight(1f),
                        singleLine = true,
                        textStyle = LocalTextStyle.current.copy(fontSize = 13.sp)
                    )
                    Button(
                        onClick = {
                            if (manualCommand.isNotBlank()) {
                                sendCommand(manualCommand)
                                manualCommand = ""
                            }
                        },
                        enabled = isConnected && manualCommand.isNotBlank(),
                    ) { Text("OK") }
                }
            }
        }

        // ── Command Log ───────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Лог команд (${commandLog.size})", fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(4.dp))
                if (commandLog.isEmpty()) {
                    Text("Пусто", fontSize = 12.sp, color = Color.Gray)
                } else {
                    commandLog.take(20).forEach { entry ->
                        Text(entry, fontSize = 11.sp)
                    }
                }
            }
        }

        Spacer(Modifier.height(20.dp))
    }
}

@Composable
private fun JoystickControl(
    enabled: Boolean,
    onSetVelocity: (Double, Double) -> Unit,
    onStop: () -> Unit,
) {
    var offsetX by remember { mutableFloatStateOf(0f) }
    var offsetY by remember { mutableFloatStateOf(0f) }
    val maxRadius = 180f

    Box(modifier = Modifier.size(200.dp), contentAlignment = Alignment.Center) {
        Box(
            modifier = Modifier
                .size(200.dp)
                .clip(CircleShape)
                .background(
                    if (enabled) MaterialTheme.colorScheme.surfaceVariant
                    else MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.4f)
                )
                .pointerInput(enabled) {
                    if (!enabled) return@pointerInput
                    detectDragGestures(
                        onDragStart = { },
                        onDrag = { change, dragAmount ->
                            change.consume()
                            offsetX += dragAmount.x
                            offsetY += dragAmount.y
                            val dist = sqrt(offsetX * offsetX + offsetY * offsetY)
                            if (dist > maxRadius) {
                                offsetX = offsetX / dist * maxRadius
                                offsetY = offsetY / dist * maxRadius
                            }
                            val lin = (-offsetY / maxRadius * 0.3).coerceIn(-0.3, 0.3)
                            val ang = (offsetX / maxRadius * 2.0).coerceIn(-2.0, 2.0)
                            onSetVelocity(lin, ang)
                        },
                        onDragEnd = { offsetX = 0f; offsetY = 0f; onStop() },
                        onDragCancel = { offsetX = 0f; offsetY = 0f; onStop() },
                    )
                }
        ) {
            Box(
                modifier = Modifier.size(8.dp).clip(CircleShape)
                    .background(Color.White.copy(alpha = 0.3f))
                    .align(Alignment.Center)
            )
        }

        Box(
            modifier = Modifier
                .size(48.dp)
                .offset(x = (offsetX / 3).dp, y = (offsetY / 3).dp)
                .clip(CircleShape)
                .background(
                    if (enabled) MaterialTheme.colorScheme.primary.copy(alpha = 0.8f)
                    else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.2f)
                )
        )
    }

    Spacer(Modifier.height(4.dp))
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
        val lin = if (abs(offsetY) > 0.01f) (-offsetY / maxRadius * 0.3) else 0.0
        val ang = if (abs(offsetX) > 0.01f) (offsetX / maxRadius * 2.0) else 0.0
        Text("Lin: ${"%.2f".format(lin)} м/с", fontSize = 11.sp, color = Color.Gray)
        Text("Ang: ${"%.2f".format(ang)} рад/с", fontSize = 11.sp, color = Color.Gray)
    }
}
