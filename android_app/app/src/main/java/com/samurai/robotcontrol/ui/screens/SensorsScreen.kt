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
        // ── Ultrasonic ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Ультразвук", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                val range = robotState.sensors.ultrasonic.range_m
                val pct = (range / 2.0).coerceIn(0.0, 1.0).toFloat()
                val rangeColor = when {
                    range < 0.15 -> Color(0xFFEF5350)
                    range < 0.5 -> Color(0xFFFF9800)
                    else -> Color(0xFF4FC3F7)
                }

                Row(
                    Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(12.dp)
                ) {
                    Text(
                        "${"%.3f".format(range)} м",
                        fontSize = 24.sp,
                        fontWeight = FontWeight.Bold,
                        color = rangeColor
                    )
                    LinearProgressIndicator(
                        progress = pct,
                        modifier = Modifier
                            .weight(1f)
                            .height(8.dp),
                        color = rangeColor,
                    )
                }
                Spacer(Modifier.height(4.dp))
                Text(
                    "Диапазон: ${robotState.sensors.ultrasonic.min_range} — ${robotState.sensors.ultrasonic.max_range} м",
                    fontSize = 10.sp,
                    color = Color.Gray
                )
            }
        }

        // ── IMU ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("IMU (инерциальный датчик)", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                val imu = robotState.sensors.imu

                SensorRow("Yaw (курс)", "${"%.1f".format(imu.yaw_deg)}°")
                SensorRow("Pitch (наклон)", "${"%.1f".format(imu.pitch_deg)}°")
                SensorRow("Roll (крен)", "${"%.1f".format(imu.roll_deg)}°")
                Divider(Modifier.padding(vertical = 4.dp))
                SensorRow("Gyro Z", "${"%.3f".format(imu.gyro_z_rad_s)} рад/с")
                SensorRow("Accel X", "${"%.3f".format(imu.accel_x_m_s2)} м/с²")
            }
        }

        // ── Velocity ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Скорости", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                val vel = robotState.velocity
                SensorRow("Линейная", "${"%.4f".format(vel.linear)} м/с")
                SensorRow("Угловая", "${"%.4f".format(vel.angular)} рад/с")
                Divider(Modifier.padding(vertical = 4.dp))
                SensorRow("Макс. лин.", "${"%.1f".format(vel.max_linear)} м/с")
                SensorRow("Макс. угл.", "${"%.1f".format(vel.max_angular)} рад/с")
            }
        }

        // ── Pose ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Позиция и ориентация", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                SensorRow("X", "${"%.3f".format(robotState.pose.x)} м")
                SensorRow("Y", "${"%.3f".format(robotState.pose.y)} м")
                SensorRow("Theta", "${"%.3f".format(robotState.pose.theta)} рад")
                SensorRow("Theta (град)", "${"%.1f".format(robotState.pose.theta_deg)}°")
            }
        }

        // ── Actuators ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Актуаторы", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(8.dp))

                SensorRow(
                    "Клешня",
                    if (robotState.actuators.claw_open) "ОТКРЫТА" else "ЗАКРЫТА",
                    if (robotState.actuators.claw_open) Color(0xFF4CAF50) else Color.Gray
                )
                SensorRow(
                    "Лазер",
                    if (robotState.actuators.laser_on) "ВКЛЮЧЁН" else "ВЫКЛЮЧЕН",
                    if (robotState.actuators.laser_on) Color(0xFFF44336) else Color.Gray
                )
            }
        }

        // ── Event Log ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text(
                    "Лог событий (${robotState.log.size})",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold
                )
                Spacer(Modifier.height(6.dp))

                if (robotState.log.isEmpty()) {
                    Text("Событий пока нет", fontSize = 12.sp, color = Color.Gray)
                } else {
                    robotState.log.reversed().take(30).forEach { entry ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            horizontalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            Text(entry.time, fontSize = 10.sp, color = Color.Gray)
                            Text(entry.text, fontSize = 11.sp)
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
        Text(
            value,
            fontSize = 13.sp,
            fontWeight = FontWeight.Medium,
            color = valueColor ?: Color.Unspecified
        )
    }
}
