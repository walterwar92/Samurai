#pragma once
// =============================================================================
// Samurai Robot — ESP32 firmware config
// =============================================================================
// All tunable constants in one place (mirrors config.yaml on Pi).
// Edit WiFi/MQTT credentials below before first flash.

// ── WiFi ─────────────────────────────────────────────────────────
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASS       "YOUR_PASSWORD"
#define WIFI_TIMEOUT_MS 15000   // max wait for connect on boot

// ── MQTT ─────────────────────────────────────────────────────────
#define MQTT_BROKER     "192.168.1.100"   // IP of Pi (Mosquitto)
#define MQTT_PORT       1883
#define MQTT_ROBOT_ID   "robot1"
#define MQTT_KEEPALIVE  15        // seconds
#define MQTT_BUF_SIZE   1024      // PubSubClient buffer (bytes)

// Topic prefix: samurai/robot1/
#define TOPIC_PREFIX    "samurai/" MQTT_ROBOT_ID "/"

// ── I2C ──────────────────────────────────────────────────────────
#define I2C_SDA         21
#define I2C_SCL         22
#define I2C_FREQ        400000    // 400 kHz fast mode

// ── I2C device addresses ─────────────────────────────────────────
#define MPU6050_ADDR    0x68
#define PCA9685_ADDR    0x5F      // Adeept HAT V3.1
#define ADS7830_ADDR    0x48

// ── GPIO pins ────────────────────────────────────────────────────
#define PIN_US_TRIG     25        // HC-SR04 trigger
#define PIN_US_ECHO     26        // HC-SR04 echo
#define PIN_LED_DATA    13        // WS2812B data line

// ── Motor channels on PCA9685 ────────────────────────────────────
// M1 (left-rear):  IN1=ch11, IN2=ch10
// M2 (right-rear): IN1=ch8,  IN2=ch9
#define M1_CH_IN1       11
#define M1_CH_IN2       10
#define M2_CH_IN1       8
#define M2_CH_IN2       9

// ── Servo channels on PCA9685 ────────────────────────────────────
#define SERVO_HEAD_CH   4         // camera pan
#define SERVO_ARM_CH0   0         // base
#define SERVO_ARM_CH1   1         // joint 1
#define SERVO_ARM_CH2   2         // joint 2
#define SERVO_ARM_CH3   3         // claw
#define NUM_ARM_JOINTS  4

// Servo pulse range (microseconds) — standard 180-degree servo
#define SERVO_PULSE_MIN 500
#define SERVO_PULSE_MAX 2400

// ── Head servo config ────────────────────────────────────────────
#define HEAD_HOME       90
#define HEAD_MIN        0
#define HEAD_MAX        180
#define HEAD_SPEED      2.0f      // degrees per tick (50 Hz)
#define HEAD_LOCKED     true      // locked on startup

// ── Arm servo config ─────────────────────────────────────────────
static const float ARM_HOME[]  = {0, 120, 0, 0};
static const float ARM_MIN[]   = {0,   0, 0, 0};
static const float ARM_MAX[]   = {120, 145, 180, 180};
#define ARM_LOCKED      true

// ── Motor / Odometry ─────────────────────────────────────────────
#define WHEEL_BASE          0.17f     // metres
#define PHYS_MAX_LIN        0.30f     // m/s (= fast profile)
#define PHYS_MAX_ANG        2.00f     // rad/s
#define DEADZONE_LINEAR     0.01f     // m/s
#define DEADZONE_ANGULAR    0.05f     // rad/s
#define CMD_VEL_TIMEOUT_S   0.50f     // auto-stop timeout
#define MANUAL_TIMEOUT_S    0.50f     // manual override timeout

// Speed profiles: {max_linear, max_angular}
#define PROFILE_SLOW_LIN    0.10f
#define PROFILE_SLOW_ANG    0.80f
#define PROFILE_NORMAL_LIN  0.20f
#define PROFILE_NORMAL_ANG  1.50f
#define PROFILE_FAST_LIN    0.30f
#define PROFILE_FAST_ANG    2.00f

// Wheel calibration (from calibration_profiles.yaml)
#define CAL_SCALE_FWD       1.235f
#define CAL_SCALE_BWD       0.988f
#define CAL_MOTOR_TRIM      -12.003f  // percent

// Collision guard
#define COLLISION_STOP_M    0.20f
#define COLLISION_SLOW_M    0.40f
#define COLLISION_AVOID_ANG 0.60f     // rad/s steering

// ── IMU (MPU6050) ────────────────────────────────────────────────
#define IMU_ACCEL_SCALE     16384.0f  // LSB/g  (±2g)
#define IMU_GYRO_SCALE      131.0f    // LSB/(deg/s) (±250 deg/s)
#define IMU_CAL_SAMPLES     200       // calibration samples (200 @ 50Hz = 4s)
#define IMU_EMA_ACCEL       0.2f      // EMA alpha for accelerometer
#define IMU_EMA_GYRO        0.3f      // EMA alpha for gyroscope

// EKF parameters
#define EKF_Q_ANGLE         0.001f
#define EKF_Q_BIAS          0.0001f
#define EKF_R_ACCEL         0.5f
#define EKF_ACCEL_GATE      0.3f

// IMU push detector (passive movement when motors OFF)
#define PUSH_SPEED          0.12f     // m/s assumed push speed
#define PUSH_DEV_THR        0.20f     // m/s² deviation threshold
#define PUSH_CONFIRM        2         // consecutive samples
#define PUSH_MAX_TICKS      30        // 1.5s at 20 Hz
#define PUSH_BIAS_SLOW      0.02f
#define PUSH_BIAS_FAST      0.15f
#define PUSH_FAST_TICKS     20

// ── Ultrasonic (HC-SR04) ─────────────────────────────────────────
#define US_MAX_RANGE        2.0f      // metres
#define US_MIN_RANGE        0.02f     // metres
#define US_TIMEOUT_US       12000     // echo timeout (microseconds) ≈ 2m
#define US_PUBLISH_DELTA    0.01f     // m — publish only on significant change
#define US_FORCE_INTERVAL   1.0f      // s — force publish at least this often

// ── Battery (ADS7830) ────────────────────────────────────────────
#define VBAT_MIN            3.0f      // V — cutoff
#define VBAT_MAX            4.2f      // V — full charge
#define VDIV_RATIO          3.0f      // voltage divider ratio

// ── LED (WS2812B) ────────────────────────────────────────────────
#define LED_COUNT           12        // 4 panels x 3 diodes
#define LED_BRIGHTNESS      76        // 0-255 (≈0.3 of 255)
#define LED_TYPE            WS2812B
#define LED_COLOR_ORDER     GRB

// ── Loop intervals (microseconds) ───────────────────────────────
#define INTERVAL_IMU_US         20000     // 50 Hz
#define INTERVAL_MOTOR_US       50000     // 20 Hz
#define INTERVAL_ULTRASONIC_US  50000     // 20 Hz
#define INTERVAL_SERVO_US       20000     // 50 Hz (smooth movement)
#define INTERVAL_SERVO_STATE_US 100000    // 10 Hz (state publish)
#define INTERVAL_BATTERY_US     1000000   // 1 Hz
#define INTERVAL_PROFILE_US     1000000   // 1 Hz
#define INTERVAL_LED_US         20000     // 50 Hz (animation frames)

// ── Watchdog ─────────────────────────────────────────────────────
#define WDT_TIMEOUT_S       8         // hardware watchdog (seconds)

// ── OTA ──────────────────────────────────────────────────────────
#define OTA_HOSTNAME        "samurai-esp32"
#define OTA_PASSWORD        ""        // empty = no password
