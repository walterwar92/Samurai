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
 * ДВА ПРАКТИЧЕСКИХ ПЕРЕКЛЮЧАТЕЛЯ (см. ниже):
 *   - OPEN_LOOP  — разомкнутый цикл (ПИД выключен). Включи и проверь:
 *                  если в open-loop едет ПРЯМО — значит проблема была в знаке ПИД
 *                  (см. GYRO_SIGN). Если крутится — значит проблема физическая
 *                  (подбирай LEFT_TRIM).
 *   - GYRO_SIGN  — знак угловой скорости по Z. Если в закрытом цикле робот
 *                  только УСИЛИВАЕТ поворот (едет по кругу), поменяй +1 на -1.
 */

#include <Wire.h>
#include <Servo.h>

// ========== РЕЖИМ ОТЛАДКИ ==========
// Раскомментируй для разомкнутого цикла (без ПИД) — диагностика
// #define OPEN_LOOP

// Знак гироскопа по оси Z.
//   Если робот отклоняется вправо, а theta растёт в «не ту» сторону и
//   ПИД усиливает поворот — поменяй на -1.
const int GYRO_SIGN = +1;

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
const float STOP_DISTANCE_CM = 10.0f;
const int   BASE_PWM         = 180;  // крейсер
const int   MIN_PWM          = 70;   // не душим ПИД clamp'ом
const int   MAX_PWM          = 255;
const int   KICK_PWM         = 255;  // kick-start для срыва статического трения
const int   KICK_MS          = 150;  // длительность kick-start

// Механическая компенсация (1.0 = без компенсации).
// ВАЖНО: оставь 1.00/1.00 для первого теста, иначе навяжешь асимметрию, которую
// ПИД будет пытаться отработать. Калибруй только если в OPEN_LOOP робот уводит.
const float LEFT_TRIM  = 1.00f;
const float RIGHT_TRIM = 1.00f;

// ПИД коррекция курса по гироскопу (угол Z).
// Слабее, чем было: слишком агрессивный ПИД сам катает робота при скачках
// гироскопа от вибраций.
float Kp = 0.8f;    // пропорциональный (по deg)
float Kd = 0.25f;   // дифференциальный (по deg/s)

// ========== ГИРОСКОП MPU-6050 ==========
#define MPU6050_ADDR 0x68
float theta     = 0.0f;    // угол (рад)
float theta_deg = 0.0f;    // угол (град)
float omega     = 0.0f;    // угловая скорость (рад/с)
unsigned long lastTime = 0;

Servo clawServo, armServo, baseServo;

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========
// Применяет trim и clamp. speed<=0 полностью останавливает мотор
// (обход MIN_PWM для штатного стопа).
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

// Kick-start: одинаковый импульс 255 → оба мотора срывают трение вместе.
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
// Медианный фильтр из 3 измерений — отсекает выбросы от эха/шумов.
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
// Явный каст параметров requestFrom — снимает ambiguity warning Uno.
void readGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission((uint8_t)MPU6050_ADDR);
    Wire.write((uint8_t)0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    for (int i = 0; i < 4; i++) { (void)(Wire.read() << 8 | Wire.read()); }
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
}

// Калибровка bias (робот стоит неподвижно).
float gyroBiasZ = 0.0f;

void calibrateGyro() {
    const int N = 300;
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
    // MPU-6050 ±250°/s → 131 LSB/°/s. GYRO_SIGN переворачивает знак при
    // необходимости (зависит от ориентации чипа на корпусе).
    float omega_deg = GYRO_SIGN * (gz - gyroBiasZ) / 131.0f;
    omega = omega_deg * PI / 180.0f;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    // Защита от глюка micros() при первом вызове / overflow: слишком большой dt
    // мгновенно испортит theta. Разумный потолок 0.1 с.
    if (dt > 0.1f) dt = 0.0f;
    lastTime = now;
    theta += omega * dt;
    theta_deg = theta * 180.0f / PI;
}

void resetGyroIntegral() {
    theta = 0.0f;
    lastTime = micros();
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
    Wire.beginTransmission((uint8_t)MPU6050_ADDR);
    Wire.write((uint8_t)0x6B);   // PWR_MGMT_1
    Wire.write((uint8_t)0x00);   // wake up
    Wire.endTransmission();
    delay(100);

    Serial.println("=== Vpered Uno ===");
#ifdef OPEN_LOOP
    Serial.println("MODE: OPEN_LOOP (no gyro correction)");
#else
    Serial.println("MODE: PID (closed-loop)");
    Serial.print("GYRO_SIGN="); Serial.println(GYRO_SIGN);
#endif
    Serial.println("Calibrating gyro (hold still)...");
    calibrateGyro();

    Serial.println("Ready. Going forward in 1s...");
    delay(1000);
}

// ========== MAIN ==========
void loop() {
    // Kick-start раскручивает моторы; только ПОСЛЕ него сбрасываем интеграл —
    // иначе dt первого updateGyro() включает длительность kick-start.
    kickStart();
    resetGyroIntegral();

    unsigned long loopStart = millis();

    while (true) {
        float obstacle = getDistance();
        updateGyro();

#ifdef OPEN_LOOP
        // Равные PWM на оба борта. Используй для диагностики: если едет прямо
        // — ПИД рулил в обратную сторону; если крутится — подбирай LEFT_TRIM.
        int leftCmd  = BASE_PWM;
        int rightCmd = BASE_PWM;
#else
        // ПИД: ошибка по углу + демпфирование по скорости (оба в градусах).
        float correction = Kp * theta_deg + Kd * (omega * 180.0f / PI);
        // Clamp коррекции — чтобы одиночный спайк не перекосил моторы на весь диапазон
        if (correction >  60.0f) correction =  60.0f;
        if (correction < -60.0f) correction = -60.0f;
        int leftCmd  = BASE_PWM - (int)correction;
        int rightCmd = BASE_PWM + (int)correction;
#endif

        motorDrive(DIR_FORWARD, leftCmd, rightCmd);

        // Лог ~10 Гц — удобно для калибровки без мусора в Serial.
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 100) {
            lastLog = millis();
            Serial.print("t=");   Serial.print((millis() - loopStart) / 1000.0f, 1);
            Serial.print("s  theta="); Serial.print(theta_deg, 1);
            Serial.print("  omega="); Serial.print(omega * 180.0f / PI, 1);
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
