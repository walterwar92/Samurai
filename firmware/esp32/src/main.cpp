// =============================================================================
// Samurai Robot — ESP32-WROOM-32 firmware
// =============================================================================
// Auto-start: setup() runs once on power-on, loop() runs forever.
// Replaces Pi-side hardware nodes: motor, imu, ultrasonic, battery,
// head, arm, led.
//
// Startup sequence:
//   1. I2C bus init
//   2. Hardware modules init (PCA9685, MPU6050, ADS7830, HC-SR04, WS2812B)
//   3. WiFi connect
//   4. MQTT connect + subscribe
//   5. OTA init
//   6. Hardware watchdog enable
//   7. Main loop: timed tasks at 50/20/10/1 Hz
//
// MQTT topics are identical to Python pi_nodes — FSM, dashboard, YOLO
// continue working without changes.

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "imu_module.h"
#include "motor_module.h"
#include "ultrasonic_module.h"
#include "battery_module.h"
#include "servo_module.h"
#include "led_module.h"

// ── Globals ──────────────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

ImuModule        imu;
MotorModule      motor;
UltrasonicModule ultrasonic;
BatteryModule    battery;
ServoModule      servos;
LedModule        leds;

// ── Timing (micros) ──────────────────────────────────────────────
static unsigned long tIMU        = 0;
static unsigned long tMotor      = 0;
static unsigned long tUltrasonic = 0;
static unsigned long tServo      = 0;
static unsigned long tServoState = 0;
static unsigned long tBattery    = 0;
static unsigned long tProfile    = 0;
static unsigned long tLed        = 0;

// ── Publish buffer ───────────────────────────────────────────────
static char pubBuf[MQTT_BUF_SIZE];

// Forward declarations
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttReconnect();
void wifiConnect();
void publishJson(const char* suffix, JsonDocument &doc,
                 bool retain = false, int qos = 0);

// =====================================================================
//  SETUP — runs once on power-on (= auto-start)
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Samurai ESP32 firmware starting...");
    Serial.println("========================================");

    // ── I2C bus ──
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Serial.printf("[I2C] SDA=%d SCL=%d freq=%d\n", I2C_SDA, I2C_SCL, I2C_FREQ);

    // ── Hardware modules init ──
    // Motor/PCA9685 first (shared bus)
    motor.begin();
    // Servos share the PCA9685 from motor module
    servos.begin(&motor.pca());
    // IMU
    bool imu_ok = imu.begin();
    if (!imu_ok) Serial.println("[WARN] IMU init failed — will retry in loop");
    // Ultrasonic
    ultrasonic.begin();
    // Battery
    battery.begin();
    // LED
    leds.begin();

    // ── WiFi ──
    wifiConnect();

    // ── MQTT ──
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setBufferSize(MQTT_BUF_SIZE);
    mqtt.setKeepAlive(MQTT_KEEPALIVE);
    mqtt.setCallback(mqttCallback);
    mqttReconnect();

    // ── OTA ──
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        leds.off();
        motor.stop();
        Serial.println("[OTA] Update starting...");
    });
    ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] Done! Rebooting..."); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] %u%%\r", progress * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]\n", error);
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready (%s)\n", OTA_HOSTNAME);

    // ── Hardware watchdog ──
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);
    Serial.printf("[WDT] Watchdog enabled (%ds)\n", WDT_TIMEOUT_S);

    // Publish online status
    String topic = String(TOPIC_PREFIX) + "esp32/status";
    mqtt.publish(topic.c_str(), "online", true);

    Serial.println("========================================");
    Serial.println("  Startup complete. Entering main loop.");
    Serial.println("========================================");
}

// =====================================================================
//  LOOP — runs forever (= replaces all Pi-side hardware nodes)
// =====================================================================
void loop() {
    // Feed the watchdog
    esp_task_wdt_reset();

    // WiFi/MQTT keep-alive
    if (WiFi.status() != WL_CONNECTED) wifiConnect();
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();

    // OTA check
    ArduinoOTA.handle();

    unsigned long now = micros();

    // ── IMU @ 50 Hz ──────────────────────────────────────────────
    if (now - tIMU >= INTERVAL_IMU_US) {
        float dt = (now - tIMU) / 1e6f;
        tIMU = now;

        if (imu.update(dt)) {
            // Feed IMU data to motor module (for heading + push detection)
            motor.feedIMU(imu.ax, imu.ay, imu.gz,
                          imu.ekf.yaw, imu.calibrated, imu.gravity_body);

            // Publish IMU
            JsonDocument doc;
            float ts = millis() / 1000.0f;
            imu.buildPayload(doc, ts);
            publishJson("imu", doc);
        }
    }

    // ── Motor control + Odom @ 20 Hz ─────────────────────────────
    if (now - tMotor >= INTERVAL_MOTOR_US) {
        float dt = (now - tMotor) / 1e6f;
        tMotor = now;

        motor.controlLoop(dt);

        // Publish odom
        JsonDocument doc;
        float ts = millis() / 1000.0f;
        motor.buildOdom(doc, ts);
        publishJson("odom", doc);
    }

    // ── Ultrasonic @ 20 Hz ───────────────────────────────────────
    if (now - tUltrasonic >= INTERVAL_ULTRASONIC_US) {
        tUltrasonic = now;

        ultrasonic.update();

        // Feed range to motor (collision guard)
        motor.setRange(ultrasonic.range_m);

        if (ultrasonic.shouldPublish) {
            JsonDocument doc;
            doc["range"] = roundf(ultrasonic.range_m * 10000) / 10000;
            doc["ts"] = millis() / 1000.0f;
            publishJson("range", doc);
        }
    }

    // ── Servo smooth movement @ 50 Hz ────────────────────────────
    if (now - tServo >= INTERVAL_SERVO_US) {
        tServo = now;
        servos.headUpdate();
    }

    // ── Servo state publish @ 10 Hz ──────────────────────────────
    if (now - tServoState >= INTERVAL_SERVO_STATE_US) {
        tServoState = now;
        {
            JsonDocument doc;
            servos.buildHeadState(doc);
            publishJson("head/state", doc);
        }
        {
            JsonDocument doc;
            servos.buildArmState(doc);
            publishJson("arm/state", doc);
        }
    }

    // ── Battery @ 1 Hz ───────────────────────────────────────────
    if (now - tBattery >= INTERVAL_BATTERY_US) {
        tBattery = now;
        battery.update();

        JsonDocument doc;
        doc["voltage"] = roundf(battery.voltage * 100) / 100;
        doc["percent"] = roundf(battery.percent * 10) / 10;
        publishJson("battery", doc, true);
    }

    // ── Speed profile + active source @ 1 Hz ────────────────────
    if (now - tProfile >= INTERVAL_PROFILE_US) {
        tProfile = now;

        // Profile name (string, not JSON)
        String topic = String(TOPIC_PREFIX) + "speed_profile/active";
        mqtt.publish(topic.c_str(), motor.profileName());

        // Active source (retained)
        topic = String(TOPIC_PREFIX) + "cmd_vel/active_source";
        mqtt.publish(topic.c_str(), motor.active_source, true);
    }

    // ── LED animation frames @ 50 Hz ────────────────────────────
    if (now - tLed >= INTERVAL_LED_US) {
        tLed = now;
        leds.update();

        if (leds.stateChanged) {
            JsonDocument doc;
            leds.buildState(doc);
            publishJson("led/state", doc, true);
        }
    }
}

// =====================================================================
//  MQTT callback — incoming commands from Pi / dashboard / FSM
// =====================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Null-terminate payload
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';

    // Strip prefix to get suffix
    const char* prefix = TOPIC_PREFIX;
    size_t plen = strlen(prefix);
    if (strncmp(topic, prefix, plen) != 0) return;
    const char* suffix = topic + plen;

    // Parse JSON (or treat as string)
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, msg);

    // ── cmd_vel (autonomous) ──
    if (strcmp(suffix, "cmd_vel") == 0) {
        if (!err) {
            motor.setCmdVel(doc["linear_x"] | 0.0f,
                            doc["angular_z"] | 0.0f);
        }
        return;
    }

    // ── cmd_vel/manual ──
    if (strcmp(suffix, "cmd_vel/manual") == 0) {
        if (!err) {
            motor.setManualVel(doc["linear_x"] | 0.0f,
                               doc["angular_z"] | 0.0f);
        }
        return;
    }

    // ── speed_profile ──
    if (strcmp(suffix, "speed_profile") == 0) {
        motor.setProfile(msg);
        return;
    }

    // ── reset_position ──
    if (strcmp(suffix, "reset_position") == 0) {
        motor.resetPosition();
        return;
    }

    // ── collision_guard/enable ──
    if (strcmp(suffix, "collision_guard/enable") == 0) {
        bool on = (strcasecmp(msg, "on") == 0 ||
                   strcasecmp(msg, "true") == 0 ||
                   strcmp(msg, "1") == 0);
        motor.setCollisionGuard(on);
        return;
    }

    // ── head/command ──
    if (strcmp(suffix, "head/command") == 0) {
        if (err) {
            // Plain string command
            doc.set(msg);
        }
        servos.headCommand(doc);
        return;
    }

    // ── arm/command ──
    if (strcmp(suffix, "arm/command") == 0) {
        if (err) {
            doc.set(msg);
        }
        servos.armCommand(doc);
        return;
    }

    // ── led/command ──
    if (strcmp(suffix, "led/command") == 0) {
        if (err) {
            doc.set(msg);
        }
        leds.command(doc);
        return;
    }
}

// =====================================================================
//  WiFi connect (blocking on boot, non-blocking on reconnect)
// =====================================================================
void wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) return;

    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset();
        if (millis() - start > WIFI_TIMEOUT_MS) {
            Serial.println(" TIMEOUT");
            Serial.println("[WiFi] Will retry in loop");
            return;
        }
    }
    Serial.printf(" OK (%s)\n", WiFi.localIP().toString().c_str());
}

// =====================================================================
//  MQTT reconnect + subscribe
// =====================================================================
void mqttReconnect() {
    if (mqtt.connected()) return;

    String clientId = String("samurai-esp32-") + String(random(0xffff), HEX);
    // LWT: mark offline if connection lost
    String lwt = String(TOPIC_PREFIX) + "esp32/status";

    // Optional auth — раскомментируй MQTT_USER / MQTT_PASS в config.h
#if defined(MQTT_USER) && defined(MQTT_PASS)
    const char* mqttUser = MQTT_USER;
    const char* mqttPass = MQTT_PASS;
    Serial.printf("[MQTT] Connecting to %s:%d as user=%s...",
                  MQTT_BROKER, MQTT_PORT, mqttUser);
#else
    const char* mqttUser = NULL;
    const char* mqttPass = NULL;
    Serial.printf("[MQTT] Connecting to %s:%d (anonymous)...",
                  MQTT_BROKER, MQTT_PORT);
#endif

    if (mqtt.connect(clientId.c_str(), mqttUser, mqttPass,
                     lwt.c_str(), 1, true, "offline")) {
        Serial.println(" OK");

        // Subscribe to command topics (same as Python node subscriptions)
        String p = String(TOPIC_PREFIX);
        mqtt.subscribe((p + "cmd_vel").c_str(),           1);
        mqtt.subscribe((p + "cmd_vel/manual").c_str(),    1);
        mqtt.subscribe((p + "speed_profile").c_str(),     1);
        mqtt.subscribe((p + "reset_position").c_str(),    1);
        mqtt.subscribe((p + "collision_guard/enable").c_str(), 1);
        mqtt.subscribe((p + "head/command").c_str(),      1);
        mqtt.subscribe((p + "arm/command").c_str(),       1);
        mqtt.subscribe((p + "led/command").c_str(),       1);

        Serial.println("[MQTT] Subscribed to 8 command topics");

        // Publish online
        mqtt.publish(lwt.c_str(), "online", true);
    } else {
        Serial.printf(" FAILED (rc=%d). Retry in 5s\n", mqtt.state());
    }
}

// =====================================================================
//  Publish JSON to MQTT
// =====================================================================
void publishJson(const char* suffix, JsonDocument &doc,
                 bool retain, int qos) {
    if (!mqtt.connected()) return;

    size_t len = serializeJson(doc, pubBuf, sizeof(pubBuf));
    if (len == 0 || len >= sizeof(pubBuf)) return;

    String topic = String(TOPIC_PREFIX) + suffix;
    mqtt.publish(topic.c_str(), pubBuf, retain);
}
