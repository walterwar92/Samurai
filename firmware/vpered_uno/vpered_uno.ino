/*
 * Vpered — Arduino Uno + shift-register motor driver (74HC595)
 *
 * Базовое поведение: едет вперёд, останавливается перед препятствием (< STOP_DISTANCE см).
 *
 * Железо:
 *   - 2 мотора (задние ведущие колёса), PWM1 (левый) / PWM2 (правый),
 *     направление задаётся через 74HC595 (DATA/SHCP/STCP) + EN
 *   - MPU-6050 (гироскоп по Z для ПИД коррекции курса)
 *   - HC-SR04 ультразвук (TRIG/ECHO)
 *   - 3 серво: CLAW (клешня), ARM (плечо), BASE (база)
 *
 * Исправление проблемы "левые колёса стартуют позже и медленнее":
 *   1. Kick-start — короткий импульс PWM=255 на оба мотора, чтобы преодолеть
 *      статическое трение одновременно. Без этого мотор с большим трением
 *      (обычно левый) не трогается, пока правый уже крутится.
 *   2. MIN_PWM = 120 — ниже этой зоны мотор не крутится стабильно.
 *      constrain() в ПИД-цикле держит обе стороны выше порога.
 *   3. LEFT_TRIM = 1.12 — множитель PWM для левого мотора. Компенсирует
 *      механическую разницу (трение, посадка, редуктор). Калибруется опытно:
 *      если робот уводит вправо — увеличить, если влево — уменьшить.
 *   4. ПИД по гироскопу продолжает держать курс в движении.
 */

#include <Wire.h>
#include <Servo.h>

// ========== ПИНЫ ==========
#define PWM1_PIN    5    // левый PWM
#define PWM2_PIN    6    // правый PWM
#define SHCP_PIN    2    // 74HC595 shift clock
#define EN_PIN      7    // motor driver enable (LOW = enabled)
#define DATA_PIN    8    // 74HC595 data
#define STCP_PIN    4    // 74HC595 storage clock
#define CLAW_PIN    11
#define ARM_PIN     10
#define BASE_PIN    9
#define TRIG_PIN    12
#define ECHO_PIN    13

// ========== НАПРАВЛЕНИЯ (биты 74HC595) ==========
const uint8_t DIR_FORWARD = 92;   // 0b01011100
const uint8_t DIR_STOP    = 0;

// ========== ПАРАМЕТРЫ ДВИЖЕНИЯ ==========
const float STOP_DISTANCE_CM = 10.0;
const int   BASE_PWM         = 200;
const int   MIN_PWM          = 120;   // ниже моторы не крутятся уверенно
const int   MAX_PWM          = 255;
const int   KICK_PWM         = 255;   // kick-start для срыва статического трения
const int   KICK_MS          = 180;   // длительность kick-start

// Механическая компенсация: левый мотор слабее → умножаем его PWM
// Подбирается опытно. 1.0 = без компенсации.
const float LEFT_TRIM  = 1.12f;
const float RIGHT_TRIM = 1.00f;

// ПИД коррекция курса по гироскопу (угол Z)
float Kp = 1.80f;   // пропорциональный
float Kd = 0.85f;   // дифференциальный

// ========== ГИРОСКОП MPU-6050 ==========
#define MPU6050_ADDR 0x68
float theta     = 0.0f;    // угол (рад)
float theta_deg = 0.0f;    // угол (град)
float omega     = 0.0f;    // угловая скорость (рад/с)
unsigned long lastTime = 0;

Servo clawServo, armServo, baseServo;

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========
// Применяет trim-коэффициенты и clamp к [MIN_PWM, MAX_PWM].
// При speed <= 0 — мотор останавливается полностью (обход trim для штатного стопа).
static inline int applyTrim(int speed, float trim) {
    if (speed <= 0) return 0;
    int v = (int)(speed * trim + 0.5f);
    return constrain(v, MIN_PWM, MAX_PWM);
}

void motorRaw(uint8_t dir, int pwmLeft, int pwmRight) {
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, constrain(pwmLeft,  0, 255));
    analogWrite(PWM2_PIN, constrain(pwmRight, 0, 255));
    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, dir);
    digitalWrite(STCP_PIN, HIGH);
    delayMicroseconds(100);
}

void motorDrive(uint8_t dir, int speedLeft, int speedRight) {
    motorRaw(dir, applyTrim(speedLeft, LEFT_TRIM), applyTrim(speedRight, RIGHT_TRIM));
}

void motorStop() {
    motorRaw(DIR_STOP, 0, 0);
    delay(50);
    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, 0);
    digitalWrite(STCP_PIN, HIGH);
}

// Kick-start: одновременно поднимает оба мотора на KICK_PWM, чтобы
// они тронулись вместе, затем плавно снижает до BASE_PWM.
void kickStart() {
    Serial.println("Kick-start");
    motorRaw(DIR_FORWARD, KICK_PWM, KICK_PWM);
    delay(KICK_MS);
    // Плавный спуск до BASE_PWM — чтобы моторы не "провалились" ниже MIN_PWM
    for (int pwm = KICK_PWM; pwm >= BASE_PWM; pwm -= 10) {
        motorDrive(DIR_FORWARD, pwm, pwm);
        delay(15);
    }
}

// ========== УЛЬТРАЗВУК ==========
// Медианный фильтр из 3 измерений — отсекает выбросы от эха/шумов
float readDistanceRaw() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 999.0f;
    return duration * 0.0343f / 2.0f;
}

float getDistance() {
    float a = readDistanceRaw(); delay(6);
    float b = readDistanceRaw(); delay(6);
    float c = readDistanceRaw();
    // median of 3
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

// ========== ГИРОСКОП ==========
void readGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    for (int i = 0; i < 4; i++) { (void)(Wire.read() << 8 | Wire.read()); }
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
}

// Калибровка bias при старте (робот должен стоять неподвижно)
float gyroBiasZ = 0.0f;

void calibrateGyro() {
    const int N = 200;
    long sum = 0;
    int16_t gx, gy, gz;
    for (int i = 0; i < N; i++) {
        readGyroRaw(&gx, &gy, &gz);
        sum += gz;
        delay(3);
    }
    gyroBiasZ = (float)sum / N;
    Serial.print("Gyro bias Z: ");
    Serial.println(gyroBiasZ);
}

void updateGyro() {
    int16_t gx, gy, gz;
    readGyroRaw(&gx, &gy, &gz);
    float omega_deg = (gz - gyroBiasZ) / 131.0f;   // MPU-6050 ±250°/s → 131 LSB/°/s
    omega = omega_deg * PI / 180.0f;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    theta += omega * dt;
    theta_deg = theta * 180.0f / PI;
}

// ========== СЕРВО ==========
void armSafePose() {
    clawServo.write(135);   // клешня — чуть приоткрыта
    armServo.write(90);     // плечо — нейтраль
    baseServo.write(90);    // база — нейтраль
    delay(500);
}

// ========== SETUP ==========
void setup() {
    Serial.begin(9600);

    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN,   OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    motorStop();

    clawServo.attach(CLAW_PIN);
    armServo.attach(ARM_PIN);
    baseServo.attach(BASE_PIN);
    armSafePose();

    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);   // PWR_MGMT_1
    Wire.write(0x00);   // wake up
    Wire.endTransmission();
    delay(100);

    Serial.println("=== Vpered Uno ===");
    Serial.println("Calibrating gyro (hold still)...");
    calibrateGyro();

    lastTime = micros();
    Serial.println("Ready. Going forward in 1s...");
    delay(1000);
}

// ========== MAIN ==========
void loop() {
    // сброс интеграла угла в момент старта заезда
    theta = 0.0f;
    lastTime = micros();

    // Kick-start — раскручиваем оба мотора одновременно
    kickStart();

    unsigned long loopStart = millis();

    while (true) {
        float obstacle = getDistance();
        updateGyro();

        // ПИД: ошибка по углу (0 = идти прямо) + демпфирование по скорости
        float correction = Kp * theta_deg + Kd * (omega * 180.0f / PI);
        int leftCmd  = BASE_PWM - (int)correction;
        int rightCmd = BASE_PWM + (int)correction;

        motorDrive(DIR_FORWARD, leftCmd, rightCmd);

        // Лог — раз в ~100 мс хватит, полный лог каждый цикл забивает Serial
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 100) {
            lastLog = millis();
            Serial.print("t=");   Serial.print((millis() - loopStart) / 1000.0f, 1);
            Serial.print("s  theta="); Serial.print(theta_deg, 1);
            Serial.print("  L="); Serial.print(leftCmd);
            Serial.print("  R="); Serial.print(rightCmd);
            Serial.print("  dist="); Serial.println(obstacle, 1);
        }

        if (obstacle < STOP_DISTANCE_CM) {
            motorStop();
            Serial.print("STOP — obstacle at ");
            Serial.print(obstacle, 1);
            Serial.println(" cm");
            break;
        }

        delay(20);
    }

    // После остановки — висим. Reset или power-cycle чтобы повторить.
    while (true) { delay(1000); }
}
