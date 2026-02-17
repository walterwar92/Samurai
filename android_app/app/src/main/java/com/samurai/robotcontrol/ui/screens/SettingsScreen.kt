package com.samurai.robotcontrol.ui.screens

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
import com.samurai.robotcontrol.ui.ConnectionMode

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(
    connectionMode: ConnectionMode,
    serverIp: String,
    serverPort: String,
    robotId: String,
    isConnected: Boolean,
    mqttConnected: Boolean,
    apiConnected: Boolean,
    onModeChange: (ConnectionMode) -> Unit,
    onServerIpChange: (String) -> Unit,
    onServerPortChange: (String) -> Unit,
    onRobotIdChange: (String) -> Unit,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        // ── Mode Selection ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Режим подключения", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    FilterChip(
                        selected = connectionMode == ConnectionMode.SIMULATOR,
                        onClick = { onModeChange(ConnectionMode.SIMULATOR) },
                        label = { Text("Симулятор") },
                        modifier = Modifier.weight(1f)
                    )
                    FilterChip(
                        selected = connectionMode == ConnectionMode.ROBOT,
                        onClick = { onModeChange(ConnectionMode.ROBOT) },
                        label = { Text("Робот") },
                        modifier = Modifier.weight(1f)
                    )
                }

                Spacer(Modifier.height(4.dp))
                Text(
                    when (connectionMode) {
                        ConnectionMode.SIMULATOR ->
                            "REST API подключение к симулятору.\nДанные: камера, карта, датчики, FSM через HTTP.\nКоманды: через REST API."
                        ConnectionMode.ROBOT ->
                            "MQTT подключение к реальному роботу.\nКоманды: через MQTT topic.\nДанные: статус через MQTT."
                    },
                    fontSize = 11.sp,
                    color = Color.Gray,
                    lineHeight = 15.sp
                )
            }
        }

        // ── Connection Settings ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Параметры подключения", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                OutlinedTextField(
                    value = serverIp,
                    onValueChange = onServerIpChange,
                    label = { Text("IP-адрес сервера") },
                    modifier = Modifier.fillMaxWidth(),
                    singleLine = true,
                    textStyle = LocalTextStyle.current.copy(fontSize = 14.sp)
                )

                if (connectionMode == ConnectionMode.SIMULATOR) {
                    Spacer(Modifier.height(6.dp))
                    OutlinedTextField(
                        value = serverPort,
                        onValueChange = onServerPortChange,
                        label = { Text("Порт") },
                        modifier = Modifier.fillMaxWidth(),
                        singleLine = true,
                        textStyle = LocalTextStyle.current.copy(fontSize = 14.sp)
                    )
                }

                Spacer(Modifier.height(6.dp))
                OutlinedTextField(
                    value = robotId,
                    onValueChange = onRobotIdChange,
                    label = { Text("Robot ID") },
                    modifier = Modifier.fillMaxWidth(),
                    singleLine = true,
                    textStyle = LocalTextStyle.current.copy(fontSize = 14.sp)
                )

                Spacer(Modifier.height(12.dp))

                Row(
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Button(
                        onClick = { if (isConnected) onDisconnect() else onConnect() },
                        modifier = Modifier.weight(1f),
                        colors = if (isConnected)
                            ButtonDefaults.buttonColors(containerColor = Color(0xFFE53935))
                        else
                            ButtonDefaults.buttonColors()
                    ) {
                        Text(if (isConnected) "Отключить" else "Подключить")
                    }

                    Surface(
                        color = if (isConnected) Color(0xFF4CAF50) else Color(0xFFE53935),
                        shape = androidx.compose.foundation.shape.RoundedCornerShape(12.dp),
                        modifier = Modifier.height(36.dp)
                    ) {
                        Text(
                            text = if (isConnected) "Подключено" else "Отключено",
                            color = Color.White,
                            modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp),
                            fontSize = 13.sp,
                            fontWeight = FontWeight.Medium
                        )
                    }
                }
            }
        }

        // ── Connection Status ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Статус подключений", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                StatusRow("REST API (симулятор)", apiConnected)
                StatusRow("MQTT (робот)", mqttConnected)

                Spacer(Modifier.height(8.dp))
                Text(
                    when (connectionMode) {
                        ConnectionMode.SIMULATOR ->
                            "URL: http://$serverIp:$serverPort"
                        ConnectionMode.ROBOT ->
                            "Broker: tcp://$serverIp:1883\nRobot ID: $robotId"
                    },
                    fontSize = 11.sp,
                    color = Color.Gray,
                    lineHeight = 15.sp
                )
            }
        }

        // ── Info ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("О приложении", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))
                Text("Samurai Robot Control", fontSize = 13.sp)
                Text("Управление роботом через REST API и MQTT", fontSize = 11.sp, color = Color.Gray)
                Spacer(Modifier.height(4.dp))
                Text("Функционал:", fontSize = 11.sp, color = Color.Gray)
                Text("  - Голосовое управление (Vosk, офлайн)", fontSize = 11.sp, color = Color.Gray)
                Text("  - Камера + YOLO детекция", fontSize = 11.sp, color = Color.Gray)
                Text("  - Карта арены + запретные зоны", fontSize = 11.sp, color = Color.Gray)
                Text("  - Датчики: ультразвук, IMU", fontSize = 11.sp, color = Color.Gray)
                Text("  - Управление: FSM, актуаторы, скорость", fontSize = 11.sp, color = Color.Gray)
                Text("  - Маршруты A* с обходом зон", fontSize = 11.sp, color = Color.Gray)
            }
        }

        Spacer(Modifier.height(20.dp))
    }
}

@Composable
private fun StatusRow(label: String, connected: Boolean) {
    Row(
        Modifier.fillMaxWidth().padding(vertical = 3.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, fontSize = 12.sp)
        Surface(
            color = if (connected) Color(0xFF4CAF50).copy(alpha = 0.15f)
            else Color(0xFFE53935).copy(alpha = 0.15f),
            shape = androidx.compose.foundation.shape.RoundedCornerShape(8.dp)
        ) {
            Text(
                if (connected) "Подключено" else "Отключено",
                color = if (connected) Color(0xFF4CAF50) else Color(0xFFE53935),
                fontSize = 11.sp,
                fontWeight = FontWeight.Medium,
                modifier = Modifier.padding(horizontal = 8.dp, vertical = 2.dp)
            )
        }
    }
}
