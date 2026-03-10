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
import com.samurai.robotcontrol.ui.ConnectionMode

@Composable
fun SensorsScreen(
    robotState: RobotFullState,
    connectionMode: ConnectionMode,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {

        // ── Battery ───────────────────────────────────────────────
        val bat = robotState.battery
        if (bat.voltage > 0 || bat.percentage > 0) {
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(12.dp)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        val battColor = when {
                            bat.percentage in 0..20  -> Color(0xFFEF5350)
                            bat.percentage in 21..50 -> Color(0xFFFF9800)
                            else                     -> Color(0xFF4CAF50)
                        }
                        val icon = when {
                            bat.percentage in 0..20  -> Icons.Default.BatteryAlert
                            bat.percentage in 21..50 -> Icons.Default.Battery3Bar
                            else                     -> Icons.Default.BatteryFull
                        }
                        Icon(icon, contentDescription = null,
                            modifier = Modifier.size(20.dp), tint = battColor)
                        Spacer(Modifier.width(6.dp))
                        Text("Батарея", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                        Spacer(Modifier.weight(1f))
                        if (bat.percentage >= 0) {
                            Text("${bat.percentage}%", fontSize = 18.sp,
                                fontWeight = FontWeight.Bold, color = battColor)
                        }
                    }
                    Spacer(Modifier.height(6.dp))
                    if (bat.percentage >= 0) {
                        LinearProgressIndicator(
                            progress = (bat.percentage / 100f).coerceIn(0f, 1f),
                            modifier = Modifier.fillMaxWidth().height(8.dp),
                            color = when {
                                bat.percentage <= 20 -> Color(0xFFEF5350)
                                bat.percentage <= 50 -> Color(0xFFFF9800)
                                else                 -> Color(0xFF4CAF50)
                            }
                        )
                        Spacer(Modifier.height(4.dp))
                    }
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        if (bat.voltage > 0)
                            Text("${"%.2f".format(bat.voltage)} В", fontSize = 13.sp,
                                fontWeight = FontWeight.Medium)
                        Text(bat.status, fontSize = 12.sp, color = Color.Gray)
                    }
                }
            }
        }

        // ── Temperature ───────────────────────────────────────────
        val temp = robotState.temperature
        if (temp.value > 0) {
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(12.dp)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        val tempColor = when {
                            temp.value > 80 -> Color(0xFFEF5350)
                            temp.value > 60 -> Color(0xFFFF9800)
                            else            -> Color(0xFF4CAF50)
                        }
                        Icon(Icons.Default.Thermostat, contentDescription = null,
                            modifier = Modifier.size(20.dp), tint = tempColor)
                        Spacer(Modifier.width(6.dp))
                        Text("Температура CPU", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                        Spacer(Modifier.weight(1f))
                        Text(
                            "${"%.1f".format(temp.value)}°${temp.unit}",
                            fontSize = 18.sp, fontWeight = FontWeight.Bold, color = tempColor
                        )
                    }
                    Spacer(Modifier.height(6.dp))
                    LinearProgressIndicator(
                        progress = (temp.value.toFloat() / 100f).coerceIn(0f, 1f),
                        modifier = Modifier.fillMaxWidth().height(6.dp),
                        color = when {
                            temp.value > 80 -> Color(0xFFEF5350)
                            temp.value > 60 -> Color(0xFFFF9800)
                            else            -> Color(0xFF4CAF50)
                        }
                    )
                }
            }
        }

        // ── Watchdog (node health) ────────────────────────────────
        if (robotState.watchdog.isNotEmpty()) {
            Card(modifier = Modifier.fillMaxWidth()) {
                Column(modifier = Modifier.padding(12.dp)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(Icons.Default.MonitorHeart, contentDescription = null,
                            modifier = Modifier.size(18.dp),
                            tint = MaterialTheme.colorScheme.primary)
                        Spacer(Modifier.width(6.dp))
                        Text("Состояние нод", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                    }
                    Spacer(Modifier.height(8.dp))
                    robotState.watchdog.entries.forEach { (name, node) ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Icon(
                                    if (node.alive) Icons.Default.CheckCircle
                                    else Icons.Default.Error,
                                    contentDescription = null,
                                    modifier = Modifier.size(14.dp),
                                    tint = if (node.alive) Color(0xFF4CAF50) else Color(0xFFEF5350)
                                )
                                Spacer(Modifier.width(6.dp))
                                Text(name, fontSize = 12.sp)
                            }
                            Text(
                                if (node.alive) node.last_seen else "МЁРТВ",
                                fontSize = 11.sp,
                                color = if (node.alive) Color.Gray else Color(0xFFEF5350)
                            )
                        }
                    }
                }
            }
        }

        // ── Ultrasonic ────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.SensorsOff, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Ультразвук", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                val range = robotState.sensors.ultrasonic.range_m
                val pct = (range / 2.0).coerceIn(0.0, 1.0).toFloat()
                val rangeColor = when {
                    range < 0.15 -> Color(0xFFEF5350)
                    range < 0.5  -> Color(0xFFFF9800)
                    else         -> Color(0xFF4FC3F7)
                }
                Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                    Text("${"%.3f".format(range)} м", fontSize = 24.sp,
                        fontWeight = FontWeight.Bold, color = rangeColor)
                    LinearProgressIndicator(
                        progress = pct, modifier = Modifier.weight(1f).height(8.dp),
                        color = rangeColor,
                    )
                }
                Spacer(Modifier.height(4.dp))
                Text(
                    "Диапазон: ${robotState.sensors.ultrasonic.min_range} — " +
                    "${robotState.sensors.ultrasonic.max_range} м",
                    fontSize = 10.sp, color = Color.Gray
                )
            }
        }

        // ── IMU ───────────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Explore, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("IMU (инерциальный датчик)", fontSize = 14.sp,
                        fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                val imu = robotState.sensors.imu
                SensorRow("Yaw (курс)",    "${"%.1f".format(imu.yawDeg)}°")
                SensorRow("Pitch (наклон)","${"%.1f".format(imu.pitchDeg)}°")
                SensorRow("Roll (крен)",   "${"%.1f".format(imu.rollDeg)}°")
                Divider(Modifier.padding(vertical = 4.dp))
                SensorRow("Gyro X", "${"%.4f".format(imu.gyro.x)} рад/с")
                SensorRow("Gyro Y", "${"%.4f".format(imu.gyro.y)} рад/с")
                SensorRow("Gyro Z", "${"%.4f".format(imu.gyroZ)} рад/с")
                Divider(Modifier.padding(vertical = 4.dp))
                SensorRow("Accel X", "${"%.4f".format(imu.accelX)} м/с²")
                SensorRow("Accel Y", "${"%.4f".format(imu.accel.y)} м/с²")
                SensorRow("Accel Z", "${"%.4f".format(imu.accel.z)} м/с²")
            }
        }

        // ── Velocity ──────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Speed, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Скорости", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                val vel = robotState.velocity
                SensorRow("Линейная",   "${"%.4f".format(vel.linearSpeed)} м/с")
                SensorRow("Угловая",    "${"%.4f".format(vel.angularSpeed)} рад/с")
                SensorRow("Профиль",    robotState.speedProfile)
                Divider(Modifier.padding(vertical = 4.dp))
                SensorRow("Макс. лин.", "${"%.1f".format(vel.max_linear)} м/с")
                SensorRow("Макс. угл.", "${"%.1f".format(vel.max_angular)} рад/с")
            }
        }

        // ── Pose ──────────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.MyLocation, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Позиция и ориентация", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))

                SensorRow("X",     "${"%.3f".format(robotState.pose.x)} м")
                SensorRow("Y",     "${"%.3f".format(robotState.pose.y)} м")
                SensorRow("Курс",  "${"%.1f".format(robotState.pose.headingDeg)}°")
                SensorRow("Yaw (рад)", "${"%.3f".format(robotState.pose.headingRad)}")
            }
        }

        // ── Actuators ─────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Build, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Актуаторы", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(8.dp))
                SensorRow(
                    "Клешня",
                    if (robotState.actuators.isClawOpen) "ОТКРЫТА" else "ЗАКРЫТА",
                    if (robotState.actuators.isClawOpen) Color(0xFF4CAF50) else Color.Gray
                )
                SensorRow(
                    "Лазер",
                    if (robotState.actuators.isLaserOn) "ВКЛЮЧЁН" else "ВЫКЛЮЧЕН",
                    if (robotState.actuators.isLaserOn) Color(0xFFF44336) else Color.Gray
                )
            }
        }

        // ── Event Log ─────────────────────────────────────────────
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.List, contentDescription = null,
                        modifier = Modifier.size(18.dp),
                        tint = MaterialTheme.colorScheme.primary)
                    Spacer(Modifier.width(6.dp))
                    Text("Лог событий (${robotState.log.size})", fontSize = 14.sp,
                        fontWeight = FontWeight.SemiBold)
                }
                Spacer(Modifier.height(6.dp))

                if (robotState.log.isEmpty()) {
                    Text("Событий пока нет", fontSize = 12.sp, color = Color.Gray)
                } else {
                    robotState.log.reversed().take(30).forEach { entry ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            horizontalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            Text(entry.time, fontSize = 10.sp, color = Color.Gray,
                                modifier = Modifier.width(56.dp))
                            if (entry.type.isNotEmpty()) {
                                Text("[${entry.type}]", fontSize = 10.sp,
                                    color = when (entry.type) {
                                        "api_command" -> Color(0xFF42A5F5)
                                        "voice"       -> Color(0xFF4CAF50)
                                        else          -> Color.Gray
                                    })
                                Spacer(Modifier.width(4.dp))
                            }
                            Text(entry.text, fontSize = 11.sp, modifier = Modifier.weight(1f))
                        }
                    }
                }
            }
        }

        Spacer(Modifier.height(20.dp))
    }
}

@Composable
private fun SensorRow(label: String, value: String, valueColor: Color? = null) {
    Row(
        Modifier.fillMaxWidth().padding(vertical = 3.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, fontSize = 12.sp, color = Color.Gray)
        Text(value, fontSize = 13.sp, fontWeight = FontWeight.Medium,
            color = valueColor ?: Color.Unspecified)
    }
}
