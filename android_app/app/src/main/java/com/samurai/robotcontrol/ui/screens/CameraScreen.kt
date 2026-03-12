package com.samurai.robotcontrol.ui.screens

import android.graphics.Bitmap
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
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
import com.samurai.robotcontrol.api.Detection
import com.samurai.robotcontrol.api.DetectionResult
import com.samurai.robotcontrol.ui.ConnectionMode

private val COLOUR_MAP = mapOf(
    "red" to Color(0xFFEF5350), "blue" to Color(0xFF42A5F5),
    "green" to Color(0xFF66BB6A), "yellow" to Color(0xFFFFEB3B),
    "orange" to Color(0xFFFF9800), "white" to Color(0xFFFAFAFA),
    "black" to Color(0xFF424242),
)

private val COLOUR_RU = mapOf(
    "red" to "красный", "blue" to "синий", "green" to "зелёный",
    "yellow" to "жёлтый", "orange" to "оранжевый", "white" to "белый",
    "black" to "чёрный",
)

@Composable
fun CameraScreen(
    cameraFrame: Bitmap?,
    detections: DetectionResult,
    closestDetection: Detection?,
    connectionMode: ConnectionMode,
    streamUrl: String,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        // ── Camera Feed ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column {
                Text(
                    "Камера + YOLO",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold,
                    modifier = Modifier.padding(12.dp, 8.dp)
                )

                if (cameraFrame != null) {
                    Image(
                        bitmap = cameraFrame.asImageBitmap(),
                        contentDescription = "Camera Feed",
                        modifier = Modifier
                            .fillMaxWidth()
                            .aspectRatio(640f / 480f),
                        contentScale = ContentScale.FillWidth
                    )
                } else {
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .aspectRatio(640f / 480f),
                        contentAlignment = Alignment.Center
                    ) {
                        CircularProgressIndicator(modifier = Modifier.size(32.dp))
                        Text(
                            "Подключение к камере...",
                            fontSize = 12.sp,
                            color = Color.Gray,
                            modifier = Modifier.padding(top = 48.dp)
                        )
                    }
                }
            }
        }

        // ── Closest Detection ──
        if (closestDetection != null && closestDetection.colour.isNotEmpty()) {
            Card(
                modifier = Modifier.fillMaxWidth(),
                colors = CardDefaults.cardColors(
                    containerColor = (COLOUR_MAP[closestDetection.colour] ?: Color.Gray).copy(alpha = 0.1f)
                )
            ) {
                Row(
                    modifier = Modifier.padding(12.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(12.dp)
                ) {
                    Surface(
                        modifier = Modifier.size(20.dp),
                        shape = androidx.compose.foundation.shape.CircleShape,
                        color = COLOUR_MAP[closestDetection.colour] ?: Color.Gray
                    ) {}
                    Column {
                        Text("Ближайший объект", fontSize = 11.sp, color = Color.Gray)
                        Text(
                            "${COLOUR_RU[closestDetection.colour] ?: closestDetection.colour} — " +
                            "${"%.2f".format(closestDetection.distance)} м " +
                            "(${(closestDetection.conf * 100).toInt()}%)",
                            fontWeight = FontWeight.Medium,
                            fontSize = 14.sp
                        )
                    }
                }
            }
        }

        // ── All Detections ──
        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(12.dp)) {
                Text(
                    "Детекции (${detections.count})",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.SemiBold
                )
                Spacer(Modifier.height(6.dp))

                if (detections.detections.isEmpty()) {
                    Text("Нет детекций", fontSize = 12.sp, color = Color.Gray)
                } else {
                    // Header
                    Row(Modifier.fillMaxWidth()) {
                        Text("Цвет", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1.5f))
                        Text("Conf", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1f))
                        Text("Дист.", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(1f))
                        Text("X", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(0.7f))
                        Text("Y", fontSize = 10.sp, color = Color.Gray, modifier = Modifier.weight(0.7f))
                    }
                    Divider(modifier = Modifier.padding(vertical = 2.dp))

                    detections.detections.forEach { det ->
                        Row(
                            Modifier.fillMaxWidth().padding(vertical = 2.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Row(Modifier.weight(1.5f), verticalAlignment = Alignment.CenterVertically) {
                                Surface(
                                    modifier = Modifier.size(8.dp),
                                    shape = androidx.compose.foundation.shape.CircleShape,
                                    color = COLOUR_MAP[det.colour] ?: Color.Gray
                                ) {}
                                Spacer(Modifier.width(4.dp))
                                Text(
                                    COLOUR_RU[det.colour] ?: det.colour,
                                    fontSize = 11.sp
                                )
                            }
                            Text("${(det.conf * 100).toInt()}%", fontSize = 11.sp, modifier = Modifier.weight(1f))
                            Text("${"%.2f".format(det.distance)}м", fontSize = 11.sp, modifier = Modifier.weight(1f))
                            Text("${det.x}", fontSize = 11.sp, modifier = Modifier.weight(0.7f))
                            Text("${det.y}", fontSize = 11.sp, modifier = Modifier.weight(0.7f))
                        }
                    }
                }
            }
        }

        Spacer(Modifier.height(20.dp))
    }
}
