package com.samurai.robotcontrol.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.samurai.robotcontrol.api.RobotFullState

@OptIn(ExperimentalMaterial3Api::class, ExperimentalLayoutApi::class)
@Composable
fun AutoScreen(
    robotState: RobotFullState,
    isConnected: Boolean,
    onSpeedProfile: (String) -> Unit,
    onPatrolCommand: (String) -> Unit,
    onPatrolWaypoints: (List<List<Double>>) -> Unit,
    onFollowMe: (String) -> Unit,
    onPathRecorderCommand: (String, String?) -> Unit,
    onSaveMap: (String) -> Unit,
    onLoadMap: (String) -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {

        // ── Speed Profile ─────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Speed, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Профиль скорости", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                val current = robotState.speedProfile
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    listOf(
                        Triple("slow",   "Медленно", "0.10 м/с"),
                        Triple("normal", "Норма",    "0.20 м/с"),
                        Triple("fast",   "Быстро",   "0.30 м/с"),
                    ).forEach { (key, label, hint) ->
                        val selected = current == key
                        val tint = when (key) {
                            "slow"  -> Color(0xFF4FC3F7)
                            "fast"  -> Color(0xFFEF5350)
                            else    -> Color(0xFF4CAF50)
                        }
                        OutlinedButton(
                            onClick = { onSpeedProfile(key) },
                            enabled = isConnected,
                            modifier = Modifier.weight(1f),
                            colors = ButtonDefaults.outlinedButtonColors(
                                containerColor = if (selected) tint.copy(alpha = 0.15f) else Color.Transparent,
                                contentColor   = if (selected) tint else MaterialTheme.colorScheme.onSurface,
                            )
                        ) {
                            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                                Text(label, fontSize = 12.sp, fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal)
                                Text(hint, fontSize = 9.sp, color = Color.Gray)
                            }
                        }
                    }
                }
            }
        }

        // ── Patrol Mode ───────────────────────────────────────────
        PatrolSection(
            patrol = robotState.patrol,
            isConnected = isConnected,
            onPatrolCommand = onPatrolCommand,
            onPatrolWaypoints = onPatrolWaypoints,
        )

        // ── Follow Me ─────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.DirectionsWalk, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Следование за человеком", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                val fm = robotState.followMe
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    // Status indicator
                    Surface(
                        color = when {
                            fm.tracking -> Color(0xFF4CAF50).copy(alpha = 0.15f)
                            fm.active   -> Color(0xFFFF9800).copy(alpha = 0.15f)
                            else        -> MaterialTheme.colorScheme.surfaceVariant
                        },
                        shape = MaterialTheme.shapes.small,
                        modifier = Modifier.weight(1f)
                    ) {
                        Column(modifier = Modifier.padding(8.dp)) {
                            Text(
                                when {
                                    fm.tracking -> "Слежение активно"
                                    fm.active   -> "Ожидание цели..."
                                    else        -> "Выключено"
                                },
                                fontSize = 12.sp,
                                color = when {
                                    fm.tracking -> Color(0xFF4CAF50)
                                    fm.active   -> Color(0xFFFF9800)
                                    else        -> Color.Gray
                                }
                            )
                            if (fm.tracking && fm.distance > 0) {
                                Text("Дистанция: ${"%.2f".format(fm.distance)} м",
                                    fontSize = 11.sp, color = Color.Gray)
                            }
                        }
                    }

                    Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Button(
                            onClick = { onFollowMe("start") },
                            enabled = isConnected && !fm.active,
                            modifier = Modifier.width(96.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF4CAF50))
                        ) { Text("Старт", fontSize = 12.sp) }
                        OutlinedButton(
                            onClick = { onFollowMe("stop") },
                            enabled = isConnected && fm.active,
                            modifier = Modifier.width(96.dp)
                        ) { Text("Стоп", fontSize = 12.sp) }
                    }
                }
            }
        }

        // ── Path Recorder ─────────────────────────────────────────
        PathRecorderSection(
            pathRecorder = robotState.pathRecorder,
            pathList = robotState.pathList,
            isConnected = isConnected,
            onCommand = onPathRecorderCommand,
        )

        // ── Map Manager ───────────────────────────────────────────
        MapManagerSection(
            mapList = robotState.mapList,
            isConnected = isConnected,
            onSave = onSaveMap,
            onLoad = onLoadMap,
        )

        Spacer(Modifier.height(20.dp))
    }
}

// ─── Patrol section ───────────────────────────────────────────────

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun PatrolSection(
    patrol: com.samurai.robotcontrol.api.PatrolStatus,
    isConnected: Boolean,
    onPatrolCommand: (String) -> Unit,
    onPatrolWaypoints: (List<List<Double>>) -> Unit,
) {
    var showWaypointDialog by remember { mutableStateOf(false) }
    var waypoints by remember { mutableStateOf(listOf<List<Double>>()) }

    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Route, contentDescription = null,
                    modifier = Modifier.size(18.dp),
                    tint = MaterialTheme.colorScheme.primary)
                Spacer(Modifier.width(6.dp))
                Text("Патрулирование", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
            }
            Spacer(Modifier.height(8.dp))

            // Status
            if (patrol.total_waypoints > 0) {
                LinearProgressIndicator(
                    progress = if (patrol.total_waypoints > 0)
                        patrol.current_waypoint.toFloat() / patrol.total_waypoints else 0f,
                    modifier = Modifier.fillMaxWidth().height(4.dp)
                )
                Spacer(Modifier.height(4.dp))
                Text(
                    "Точка ${patrol.current_waypoint} / ${patrol.total_waypoints}",
                    fontSize = 11.sp, color = Color.Gray
                )
            } else {
                Text(
                    "Точки патруля не заданы",
                    fontSize = 11.sp, color = Color.Gray
                )
            }

            Spacer(Modifier.height(8.dp))

            // Waypoints list preview
            if (waypoints.isNotEmpty()) {
                Text("Заданные точки (${waypoints.size}):", fontSize = 11.sp, color = Color.Gray)
                waypoints.forEachIndexed { i, wp ->
                    Text(
                        "${i + 1}. X=${"%.2f".format(wp[0])}, Y=${"%.2f".format(wp[1])}",
                        fontSize = 11.sp
                    )
                }
                Spacer(Modifier.height(6.dp))
            }

            // Controls
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                Button(
                    onClick = { onPatrolCommand("start") },
                    enabled = isConnected && !patrol.active,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF4CAF50))
                ) { Text("Старт", fontSize = 12.sp) }

                OutlinedButton(
                    onClick = { onPatrolCommand("stop") },
                    enabled = isConnected && patrol.active,
                    modifier = Modifier.weight(1f)
                ) { Text("Стоп", fontSize = 12.sp) }

                OutlinedButton(
                    onClick = { showWaypointDialog = true },
                    modifier = Modifier.weight(1f)
                ) {
                    Icon(Icons.Default.AddLocation, contentDescription = null,
                        modifier = Modifier.size(16.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Точки", fontSize = 12.sp)
                }
            }
        }
    }

    // Waypoint input dialog
    if (showWaypointDialog) {
        WaypointDialog(
            current = waypoints,
            onDismiss = { showWaypointDialog = false },
            onConfirm = { newWps ->
                waypoints = newWps
                if (newWps.isNotEmpty()) onPatrolWaypoints(newWps)
                showWaypointDialog = false
            }
        )
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun WaypointDialog(
    current: List<List<Double>>,
    onDismiss: () -> Unit,
    onConfirm: (List<List<Double>>) -> Unit,
) {
    var xText by remember { mutableStateOf("") }
    var yText by remember { mutableStateOf("") }
    val waypoints = remember { mutableStateListOf(*current.toTypedArray()) }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Точки патруля", fontSize = 16.sp) },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                if (waypoints.isNotEmpty()) {
                    Text("Добавлено: ${waypoints.size} точек", fontSize = 12.sp, color = Color.Gray)
                    waypoints.forEachIndexed { i, wp ->
                        Row(
                            Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("${i + 1}. (${"%.2f".format(wp[0])}, ${"%.2f".format(wp[1])})",
                                fontSize = 12.sp)
                            IconButton(onClick = { waypoints.removeAt(i) },
                                modifier = Modifier.size(24.dp)) {
                                Icon(Icons.Default.Close, contentDescription = null,
                                    tint = Color(0xFFEF5350), modifier = Modifier.size(14.dp))
                            }
                        }
                    }
                    Divider()
                }
                Text("Добавить точку:", fontSize = 12.sp)
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedTextField(
                        value = xText,
                        onValueChange = { xText = it },
                        label = { Text("X (м)", fontSize = 11.sp) },
                        modifier = Modifier.weight(1f),
                        singleLine = true,
                    )
                    OutlinedTextField(
                        value = yText,
                        onValueChange = { yText = it },
                        label = { Text("Y (м)", fontSize = 11.sp) },
                        modifier = Modifier.weight(1f),
                        singleLine = true,
                    )
                }
                Button(
                    onClick = {
                        val x = xText.toDoubleOrNull()
                        val y = yText.toDoubleOrNull()
                        if (x != null && y != null) {
                            waypoints.add(listOf(x, y))
                            xText = ""; yText = ""
                        }
                    },
                    enabled = xText.toDoubleOrNull() != null && yText.toDoubleOrNull() != null,
                    modifier = Modifier.fillMaxWidth()
                ) { Text("+ Добавить") }
            }
        },
        confirmButton = {
            Button(onClick = { onConfirm(waypoints.toList()) },
                enabled = waypoints.isNotEmpty()) { Text("Применить") }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) { Text("Отмена") }
        }
    )
}

// ─── Path recorder section ────────────────────────────────────────

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun PathRecorderSection(
    pathRecorder: com.samurai.robotcontrol.api.PathRecorderStatus,
    pathList: List<String>,
    isConnected: Boolean,
    onCommand: (String, String?) -> Unit,
) {
    var recordName by remember { mutableStateOf("") }
    var showPlayDialog by remember { mutableStateOf(false) }

    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.FiberManualRecord, contentDescription = null,
                    modifier = Modifier.size(18.dp),
                    tint = if (pathRecorder.recording) Color(0xFFEF5350) else MaterialTheme.colorScheme.primary)
                Spacer(Modifier.width(6.dp))
                Text("Запись маршрута", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                if (pathRecorder.recording) {
                    Spacer(Modifier.width(8.dp))
                    Surface(
                        color = Color(0xFFEF5350).copy(alpha = 0.15f),
                        shape = MaterialTheme.shapes.small
                    ) {
                        Text(" ЗАПИСЬ ", fontSize = 10.sp, color = Color(0xFFEF5350),
                            modifier = Modifier.padding(horizontal = 4.dp, vertical = 2.dp))
                    }
                }
                if (pathRecorder.playing) {
                    Spacer(Modifier.width(8.dp))
                    Surface(
                        color = Color(0xFF4CAF50).copy(alpha = 0.15f),
                        shape = MaterialTheme.shapes.small
                    ) {
                        Text(" ВОСПР. ", fontSize = 10.sp, color = Color(0xFF4CAF50),
                            modifier = Modifier.padding(horizontal = 4.dp, vertical = 2.dp))
                    }
                }
            }
            Spacer(Modifier.height(8.dp))

            // Name input
            OutlinedTextField(
                value = recordName,
                onValueChange = { recordName = it },
                label = { Text("Имя маршрута", fontSize = 12.sp) },
                modifier = Modifier.fillMaxWidth(),
                singleLine = true,
                enabled = !pathRecorder.recording,
            )
            Spacer(Modifier.height(8.dp))

            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                Button(
                    onClick = { onCommand("start", recordName.ifBlank { "route_${System.currentTimeMillis() / 1000}" }) },
                    enabled = isConnected && !pathRecorder.recording && !pathRecorder.playing,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFEF5350))
                ) {
                    Icon(Icons.Default.FiberManualRecord, null, Modifier.size(14.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Запись", fontSize = 12.sp)
                }

                OutlinedButton(
                    onClick = { onCommand("stop", null) },
                    enabled = isConnected && (pathRecorder.recording || pathRecorder.playing),
                    modifier = Modifier.weight(1f)
                ) {
                    Icon(Icons.Default.Stop, null, Modifier.size(14.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Стоп", fontSize = 12.sp)
                }

                Button(
                    onClick = { showPlayDialog = true },
                    enabled = isConnected && pathList.isNotEmpty() && !pathRecorder.recording,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF4CAF50))
                ) {
                    Icon(Icons.Default.PlayArrow, null, Modifier.size(14.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Играть", fontSize = 12.sp)
                }
            }

            if (pathList.isNotEmpty()) {
                Spacer(Modifier.height(6.dp))
                Text("Сохранённые маршруты: ${pathList.joinToString(", ")}",
                    fontSize = 10.sp, color = Color.Gray)
            }
        }
    }

    // Play selection dialog
    if (showPlayDialog) {
        AlertDialog(
            onDismissRequest = { showPlayDialog = false },
            title = { Text("Выбрать маршрут", fontSize = 16.sp) },
            text = {
                Column {
                    if (pathList.isEmpty()) {
                        Text("Нет сохранённых маршрутов", color = Color.Gray, fontSize = 13.sp)
                    } else {
                        pathList.forEach { name ->
                            TextButton(
                                onClick = {
                                    onCommand("play", name)
                                    showPlayDialog = false
                                },
                                modifier = Modifier.fillMaxWidth()
                            ) {
                                Icon(Icons.Default.PlayCircle, null,
                                    modifier = Modifier.size(16.dp),
                                    tint = Color(0xFF4CAF50))
                                Spacer(Modifier.width(8.dp))
                                Text(name, fontSize = 13.sp)
                            }
                        }
                    }
                }
            },
            confirmButton = {},
            dismissButton = {
                TextButton(onClick = { showPlayDialog = false }) { Text("Закрыть") }
            }
        )
    }
}

// ─── Map manager section ──────────────────────────────────────────

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun MapManagerSection(
    mapList: List<String>,
    isConnected: Boolean,
    onSave: (String) -> Unit,
    onLoad: (String) -> Unit,
) {
    var mapName by remember { mutableStateOf("") }
    var showLoadDialog by remember { mutableStateOf(false) }

    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Map, contentDescription = null,
                    modifier = Modifier.size(18.dp),
                    tint = MaterialTheme.colorScheme.primary)
                Spacer(Modifier.width(6.dp))
                Text("Управление картами", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
            }
            Spacer(Modifier.height(8.dp))

            OutlinedTextField(
                value = mapName,
                onValueChange = { mapName = it },
                label = { Text("Имя карты", fontSize = 12.sp) },
                modifier = Modifier.fillMaxWidth(),
                singleLine = true,
            )
            Spacer(Modifier.height(8.dp))

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = { onSave(mapName.ifBlank { "map" }) },
                    enabled = isConnected,
                    modifier = Modifier.weight(1f),
                ) {
                    Icon(Icons.Default.Save, null, Modifier.size(14.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Сохранить", fontSize = 12.sp)
                }
                OutlinedButton(
                    onClick = { showLoadDialog = true },
                    enabled = isConnected && mapList.isNotEmpty(),
                    modifier = Modifier.weight(1f),
                ) {
                    Icon(Icons.Default.FolderOpen, null, Modifier.size(14.dp))
                    Spacer(Modifier.width(4.dp))
                    Text("Загрузить", fontSize = 12.sp)
                }
            }

            if (mapList.isNotEmpty()) {
                Spacer(Modifier.height(4.dp))
                Text("Доступных карт: ${mapList.size}", fontSize = 10.sp, color = Color.Gray)
            }
        }
    }

    if (showLoadDialog) {
        AlertDialog(
            onDismissRequest = { showLoadDialog = false },
            title = { Text("Загрузить карту", fontSize = 16.sp) },
            text = {
                Column {
                    mapList.forEach { name ->
                        TextButton(
                            onClick = {
                                onLoad(name)
                                showLoadDialog = false
                            },
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            Icon(Icons.Default.Map, null,
                                modifier = Modifier.size(16.dp),
                                tint = MaterialTheme.colorScheme.primary)
                            Spacer(Modifier.width(8.dp))
                            Text(name, fontSize = 13.sp)
                        }
                    }
                }
            },
            confirmButton = {},
            dismissButton = {
                TextButton(onClick = { showLoadDialog = false }) { Text("Закрыть") }
            }
        )
    }
}
