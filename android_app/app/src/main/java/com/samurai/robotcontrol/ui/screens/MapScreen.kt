package com.samurai.robotcontrol.ui.screens

import android.graphics.Bitmap
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.samurai.robotcontrol.api.BallInfo
import com.samurai.robotcontrol.api.ForbiddenZone
import com.samurai.robotcontrol.api.RobotPose
import com.samurai.robotcontrol.ui.ConnectionMode

private val COLOUR_MAP = mapOf(
    "red" to Color(0xFFEF5350), "blue" to Color(0xFF42A5F5),
    "green" to Color(0xFF66BB6A), "yellow" to Color(0xFFFFEB3B),
    "orange" to Color(0xFFFF9800),
)

private val COLOUR_RU = mapOf(
    "red" to "красный", "blue" to "синий", "green" to "зелёный",
    "yellow" to "жёлтый", "orange" to "оранжевый",
)

@Composable
fun MapScreen(
    mapImage: Bitmap?,
    zones: List<ForbiddenZone>,
    balls: List<BallInfo>,
    pose: RobotPose,
    path: List<List<Double>>,
    connectionMode: ConnectionMode,
    onAddZone: (Double, Double, Double, Double) -> Unit,
    onDeleteZone: (Int) -> Unit,
    onClearZones: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        // ── Map Image ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column {
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(12.dp, 8.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Карта арены", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                    if (connectionMode == ConnectionMode.SIMULATOR) {
                        TextButton(
                            onClick = onClearZones,
                            contentPadding = PaddingValues(horizontal = 8.dp)
                        ) {
                            Text("Очистить зоны", fontSize = 11.sp, color = Color(0xFFEF5350))
                        }
                    }
                }

                if (mapImage != null) {
                    Image(
                        bitmap = mapImage.asImageBitmap(),
                        contentDescription = "Map",
                        modifier = Modifier
                            .fillMaxWidth()
                            .aspectRatio(1f),
                        contentScale = ContentScale.FillWidth
                    )
                } else {
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .aspectRatio(1f),
                        contentAlignment = Alignment.Center
                    ) {
                        if (connectionMode == ConnectionMode.SIMULATOR) {
                            CircularProgressIndicator(modifier = Modifier.size(32.dp))
                        } else {
                            Text(
                                "Карта доступна только\nв режиме симулятора",
                                fontSize = 13.sp,
                                color = Color.Gray,
                                textAlign = androidx.compose.ui.text.style.TextAlign.Center
                            )
                        }
                    }
                }
            }
        }

        // ── Robot Position ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text("Позиция робота", fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.height(6.dp))
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("X", fontSize = 10.sp, color = Color.Gray)
                        Text("${"%.3f".format(pose.x)} м", fontWeight = FontWeight.Medium)
                    }
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("Y", fontSize = 10.sp, color = Color.Gray)
                        Text("${"%.3f".format(pose.y)} м", fontWeight = FontWeight.Medium)
                    }
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("Курс", fontSize = 10.sp, color = Color.Gray)
                        Text("${"%.1f".format(pose.theta_deg)}°", fontWeight = FontWeight.Medium)
                    }
                }
                if (path.isNotEmpty()) {
                    Spacer(Modifier.height(4.dp))
                    Text(
                        "Маршрут A*: ${path.size} точек",
                        fontSize = 11.sp,
                        color = Color(0xFF4CAF50)
                    )
                }
            }
        }

        // ── Balls ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                val active = balls.count { !it.grabbed }
                val grabbed = balls.count { it.grabbed }
                Text(
                    "Мячи ($active активных, $grabbed захвачено)",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold
                )
                Spacer(Modifier.height(6.dp))

                if (balls.isEmpty()) {
                    Text("Нет данных", fontSize = 12.sp, color = Color.Gray)
                } else {
                    // Header
                    Row(Modifier.fillMaxWidth()) {
                        Text("ID", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(0.5f))
                        Text("Цвет", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1.5f))
                        Text("X", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1f))
                        Text("Y", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1f))
                        Text("Статус", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1f))
                    }
                    Divider(modifier = Modifier.padding(vertical = 2.dp))

                    balls.forEach { ball ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("${ball.id}", fontSize = 11.sp, modifier = Modifier.weight(0.5f))
                            Row(Modifier.weight(1.5f), verticalAlignment = Alignment.CenterVertically) {
                                Surface(
                                    modifier = Modifier.size(8.dp),
                                    shape = CircleShape,
                                    color = COLOUR_MAP[ball.colour] ?: Color.Gray
                                ) {}
                                Spacer(Modifier.width(4.dp))
                                Text(
                                    COLOUR_RU[ball.colour] ?: ball.colour,
                                    fontSize = 11.sp
                                )
                            }
                            Text("${"%.3f".format(ball.x)}", fontSize = 11.sp, modifier = Modifier.weight(1f))
                            Text("${"%.3f".format(ball.y)}", fontSize = 11.sp, modifier = Modifier.weight(1f))
                            Text(
                                if (ball.grabbed) "Захвачен" else "Активен",
                                fontSize = 11.sp,
                                color = if (ball.grabbed) Color(0xFFEF5350) else Color(0xFF4CAF50),
                                modifier = Modifier.weight(1f)
                            )
                        }
                    }
                }
            }
        }

        // ── Forbidden Zones ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text(
                    "Запретные зоны (${zones.size})",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold
                )
                Spacer(Modifier.height(6.dp))

                if (zones.isEmpty()) {
                    Text("Зон нет", fontSize = 12.sp, color = Color.Gray)
                } else {
                    zones.forEach { zone ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text(
                                "#${zone.id}: (${"%.2f".format(zone.x1)}, ${"%.2f".format(zone.y1)}) — (${"%.2f".format(zone.x2)}, ${"%.2f".format(zone.y2)})",
                                fontSize = 11.sp
                            )
                            if (connectionMode == ConnectionMode.SIMULATOR) {
                                TextButton(
                                    onClick = { onDeleteZone(zone.id) },
                                    contentPadding = PaddingValues(horizontal = 4.dp)
                                ) {
                                    Text("X", color = Color(0xFFEF5350), fontSize = 12.sp)
                                }
                            }
                        }
                    }
                }
            }
        }

        Spacer(Modifier.height(20.dp))
    }
}
