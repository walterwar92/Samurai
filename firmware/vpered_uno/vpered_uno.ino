/*
 * Vpered — Arduino Uno + shift-register motor driver (74HC595)
 *
 * Базовое поведение: едет вперёд, держит курс по гироскопу (target = 0°),
 * останавливается перед препятствием (< STOP_DISTANCE см).
 *
 * Контроллер курса — полноценный PI+D:
 *   correction = Kp*err + Ki*∫err dt + Kd*omega_filt
 *   err     = targetTheta - theta  (target = 0 → возврат на изначальный курс)
 *   ∫err    — интегральный член с anti-windup (устраняет стационарную ошибку
 *             от асимметрии моторов; без него робот стабилизируется не на 0,
 *             а на каком-то смещении).
 *   omega   — отфильтрованная угловая скорость (LPF EMA), для демпфирования.
 *   slew    — лимит изменения PWM за цикл, плавный возврат без рывков.
 *   deadband — мёртвая зона ±0.3° чтобы не дёргаться от шума гироскопа.
 *
 * Железо:
 *   - 2 мотора (задние ведущие колёса), PWM1 (левый) / PWM2 (правый),
 *     направление через 74HC595 (DATA/SHCP/STCP) + EN
 *   - MPU-6050 (гироскоп по Z для ПИД коррекции курса)
 *   - HC-SR04 ультразвук (TRIG/ECHO)
 *   - 3 серво: CLAW, ARM, BASE
 *
 * Переключатели для отладки:
 *   - OPEN_LOOP — ПИД выключен, оба мотора равны. Для диагностики физики.
 *   - GYRO_SIGN — переворот знака гироскопа, если ось Z на корпусе перевёрнута.
 */

#include <Wire.h>
#include <Servo.h>

// ========== РЕЖИМ ОТЛАДКИ ==========
// Раскомментируй для разомкнутого цикла (без ПИД) — диагностика
// #define OPEN_LOOP

// Знак гироскопа по оси Z. Если ПИД усиливает поворот → -1.
const int GYRO_SIGN = +1;

// ========== ПИНЫ ==========
#define PWM1_PIN    5    // левый PWM
#define PWM2_PIN    6    // правый PWM
#define SHCP_PIN    2
#define EN_PIN      7
#define DATA_PIN    8
#define STCP_PIN    4
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
const int   BASE_PWM         = 180;
const int   MIN_PWM          = 70;
const int   MAX_PWM          = 255;
const int   KICK_PWM         = 255;
const int   KICK_MS          = 150;

// Механическая компенсация (1.0 = без компенсации). С PI это чаще не нужно —
// интеграл сам поднимет постоянное смещение, чтобы устранить уход.
const float LEFT_TRIM  = 1.00f;
const float RIGHT_TRIM = 1.00f;

// ========== ПИД-КОНТРОЛЛЕР КУРСА (PI + D на угловой скорости) ==========
// Цель — целевой курс (0 = идти прямо «как стартанули»).
const float TARGET_THETA_DEG = 0.0f;

// Коэффициенты. Тюнинг (в этом порядке):
//   1) Поставь Ki=0, Kd=0. Подбирай Kp пока робот не начнёт чуть-чуть
//      колебаться около 0. Возьми ~70% от этого Kp.
//   2) Подними Kd пока колебания не исчезнут (обычно Kd ≈ 0.2..0.5 от Kp).
//   3) Если устойчивое смещение (стабильно едет под углом) — поднимай Ki
//      малыми шагами. Слишком большой Ki → раскачка с большим периодом.
float Kp = 2.2f;     // на градус ошибки → единиц PWM (более агрессивно)
float Ki = 0.12f;    // на (градус·с) накопленной ошибки
float Kd = 0.40f;    // на (градус/с) угловой скорости

// Anti-windup: ограничение интеграла чтобы он не накапливался бесконечно при
// насыщении исполнительных механизмов.
const float INTEGRAL_MAX_DEG_S = 60.0f;

// Deadband: ошибки меньше этого не интегрируются и не корректируются —
// чтобы шум гироскопа не дёргал моторы.
const float HEADING_DEADBAND_DEG = 0.3f;

// LPF для угловой скорости (EMA). 0 < α ≤ 1; меньше = больше сглаживания.
const float OMEGA_LPF_ALPHA = 0.35f;

// Slew rate: максимальное изменение PWM за один цикл, единиц.
// Плавный возврат без рывков. 50 — компромисс между плавностью и
// скоростью реакции.
const int SLEW_MAX_PWM = 50;

// Clamp итоговой коррекции — защита от выбросов.
const float CORRECTION_CLAMP = 100.0f;

// Агрессивный режим: при |err| > AGGRESSIVE_DEG включается ускоренный
// возврат — Kp умножается на BOOST. Так робот возвращается к курсу
// БЛИЖАЙШИМ путём, а не «дрифтит» долго при сильных отклонениях.
const float AGGRESSIVE_DEG = 12.0f;
const float AGGRESSIVE_BOOST = 2.5f;

// Watchdog: если |err| > 30° непрерывно дольше WATCHDOG_MS → STOP.
// Знак ПИД скорее всего перевёрнут.
const float WATCHDOG_ERR_DEG = 30.0f;
const unsigned long WATCHDOG_MS = 1500;

// Знак контроля. Авто-определяется на старте — НЕ const.
// +1 = стандартная схема (correction>0 → правый быстрее → поворот в −θ)
// −1 = инвертированная (нестандартное шасси или GYRO_SIGN не тот)
int controlSign = +1;

// ========== ГИРОСКОП MPU-6050 ==========
#define MPU6050_ADDR 0x68
float theta         = 0.0f;    // угол (рад)
float theta_deg     = 0.0f;    // угол (град)
float omega         = 0.0f;    // угловая скорость (рад/с)
float omega_deg     = 0.0f;    // угловая скорость (град/с)
float omega_filt    = 0.0f;    // отфильтрованная (для D-члена)
float dt_sec        = 0.0f;    // dt последнего цикла updateGyro
unsigned long lastTime = 0;

// Состояние ПИД
float pidIntegral = 0.0f;
int prevLeftCmd  = 0;
int prevRightCmd = 0;

Servo clawServo, armServo, baseServo;

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========
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

void kickStart() {
    Serial.println("Kick-start");
    motorRaw(DIR_FORWARD, KICK_PWM, KICK_PWM);
    delay(KICK_MS);
    for (int pwm = KICK_PWM; pwm >= BASE_PWM; pwm -= 10) {
        motorDrive(DIR_FORWARD, pwm, pwm);
        delay(15);
    }
    prevLeftCmd  = BASE_PWM;
    prevRightCmd = BASE_PWM;
}

// ========== УЛЬТРАЗВУК ==========
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
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

// ========== ГИРОСКОП ==========
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
    omega_deg = GYRO_SIGN * (gz - gyroBiasZ) / 131.0f;   // ±250°/s, 131 LSB/(°/s)
    omega = omega_deg * PI / 180.0f;

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.1f) dt = 0.0f;     // защита от мусорного первого dt
    lastTime = now;
    dt_sec = dt;

    theta += omega * dt;
    theta_deg = theta * 180.0f / PI;

    // EMA фильтр угловой скорости (для D-члена)
    omega_filt = OMEGA_LPF_ALPHA * omega_deg + (1.0f - OMEGA_LPF_ALPHA) * omega_filt;
}

void resetGyroIntegral() {
    theta = 0.0f;
    pidIntegral = 0.0f;
    omega_filt = 0.0f;
    lastTime = micros();
}

// ========== АВТО-КАЛИБРОВКА ЗНАКА КОНТРОЛЯ ==========
// Подаёт на 250 мс асимметричную команду «правый быстрее на 60 PWM»
// и смотрит, в какую сторону по theta ушёл робот.
//
// Стандартная схема (controlSign=+1):
//   correction>0 → leftCmd = BASE-corr, rightCmd = BASE+corr → правый быстрее
//   → робот поворачивает физически в одну сторону → ИЗМЕРЕНИЕ θ должно
//   уменьшаться (drift < 0).
//
// Если drift положительный — измерение θ растёт когда робот делает физический
// «правый-быстрее»-поворот. Тогда при θ>0 наша коррекция correction>0 даст
// тот же физический поворот → θ растёт ещё → робот в кругу.
// Решение: инвертируем controlSign — тогда при θ>0 correction<0,
// leftCmd>BASE, rightCmd<BASE → левый быстрее → робот в обратную сторону.
void autoCalibrateControlSign() {
    Serial.println("Auto-calibrating control polarity...");
    delay(120);                      // дать моторам стабилизироваться после kickStart

    // Чистый сброс перед измерением
    theta = 0.0f;
    lastTime = micros();
    updateGyro();
    float thetaBefore = theta_deg;

    // Тестовая асимметрия: «правый быстрее» (как при correction=+60)
    motorRaw(DIR_FORWARD, BASE_PWM - 60, BASE_PWM + 60);

    unsigned long t0 = millis();
    while (millis() - t0 < 250) {
        updateGyro();
        delay(10);
    }
    // Снять асимметрию, дать гироскопу осесть
    motorRaw(DIR_FORWARD, BASE_PWM, BASE_PWM);
    delay(60);
    updateGyro();
    float drift = theta_deg - thetaBefore;

    Serial.print("  drift=");  Serial.print(drift, 2);  Serial.print("° → ");
    if (drift > 1.5f) {
        controlSign = -1;
        Serial.println("INVERTED (controlSign=-1)");
    } else if (drift < -1.5f) {
        controlSign = +1;
        Serial.println("STANDARD (controlSign=+1)");
    } else {
        // Слишком слабая реакция — оставляем дефолт.
        Serial.println("WEAK response, keeping +1");
    }

    // Полный reset перед основным циклом — чтобы накопленная за тест θ
    // не сбила ПИД.
    resetGyroIntegral();
    prevLeftCmd  = BASE_PWM;
    prevRightCmd = BASE_PWM;
}

// ========== СЕРВО ==========
void armSafePose() {
    clawServo.write(135);
    armServo.write(90);
    baseServo.write(90);
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
    Serial.println("MODE: PI+D (closed-loop heading hold)");
    Serial.print("GYRO_SIGN="); Serial.println(GYRO_SIGN);
    Serial.print("Kp="); Serial.print(Kp); Serial.print(" Ki="); Serial.print(Ki); Serial.print(" Kd="); Serial.println(Kd);
#endif
    Serial.println("Calibrating gyro (hold still)...");
    calibrateGyro();

    Serial.println("Ready. Going forward in 1s...");
    delay(1000);
}

// ========== MAIN ==========
void loop() {
    // Kick-start раскручивает моторы; только ПОСЛЕ него сбрасываем интегралы.
    kickStart();
    resetGyroIntegral();

#ifndef OPEN_LOOP
    // Авто-определение знака контроля — устраняет проблему «робот едет
    // по кругу из-за перевёрнутой ориентации MPU или нестандартного шасси».
    autoCalibrateControlSign();
#endif

    unsigned long loopStart = millis();
    unsigned long watchdogSince = 0;

    while (true) {
        float obstacle = getDistance();
        updateGyro();

#ifdef OPEN_LOOP
        int leftCmd  = BASE_PWM;
        int rightCmd = BASE_PWM;
#else
        // === PI+D heading hold ===
        // Ошибка курса (+ значит робот «ушёл» вправо относительно стартового).
        // target=0 → робот всегда стремится к стартовому курсу.
        float err = theta_deg - TARGET_THETA_DEG;

        // Deadband: малые ошибки игнорируем (не интегрируем, не корректируем).
        // Даёт «спокойствие» на прямой и убирает дребезг моторов от шума.
        float errEffective = err;
        if (err > -HEADING_DEADBAND_DEG && err < HEADING_DEADBAND_DEG) {
            errEffective = 0.0f;
        }

        // Интеграл — устраняет стационарную ошибку от любой асимметрии моторов.
        // Только когда есть РЕАЛЬНАЯ ошибка (за пределами deadband).
        pidIntegral += errEffective * dt_sec;
        if (pidIntegral >  INTEGRAL_MAX_DEG_S) pidIntegral =  INTEGRAL_MAX_DEG_S;
        if (pidIntegral < -INTEGRAL_MAX_DEG_S) pidIntegral = -INTEGRAL_MAX_DEG_S;

        // Kp_eff: при больших отклонениях усиливаем P-член → быстрый
        // возврат «ближайшим путём» (робот активно докручивается обратно,
        // а не дрейфит по широкой дуге).
        float kpEff = Kp;
        if (err > AGGRESSIVE_DEG || err < -AGGRESSIVE_DEG) {
            kpEff = Kp * AGGRESSIVE_BOOST;
        }

        // Полная коррекция (PI+D), затем умножение на controlSign
        // (определён автокалибровкой в начале loop).
        float correction = controlSign * (kpEff * errEffective + Ki * pidIntegral + Kd * omega_filt);
        if (correction >  CORRECTION_CLAMP) correction =  CORRECTION_CLAMP;
        if (correction < -CORRECTION_CLAMP) correction = -CORRECTION_CLAMP;

        // err>0 → correction>0 → leftCmd уменьшается, rightCmd
        // увеличивается → правый быстрее → робот поворачивает в сторону
        // уменьшения theta. ControlSign уже учтён в correction выше.
        int leftCmd  = BASE_PWM - (int)correction;
        int rightCmd = BASE_PWM + (int)correction;

        // Watchdog: если ошибка гигантская и не уменьшается полторы секунды
        // → знак точно перевёрнут, надо остановиться.
        if (err > WATCHDOG_ERR_DEG || err < -WATCHDOG_ERR_DEG) {
            if (watchdogSince == 0) watchdogSince = millis();
            if (millis() - watchdogSince > WATCHDOG_MS) {
                motorStop();
                Serial.print("STOP — watchdog: |err|>");
                Serial.print(WATCHDOG_ERR_DEG, 0);
                Serial.print("° for ");
                Serial.print(WATCHDOG_MS);
                Serial.println("ms. Check GYRO_SIGN / wiring.");
                break;
            }
        } else {
            watchdogSince = 0;
        }

        // Slew-rate limit: плавный возврат, без рывков (защита моторов и
        // драйвера, плюс уменьшает «качание» во время больших корректировок).
        int dL = leftCmd - prevLeftCmd;
        if (dL >  SLEW_MAX_PWM) leftCmd = prevLeftCmd + SLEW_MAX_PWM;
        if (dL < -SLEW_MAX_PWM) leftCmd = prevLeftCmd - SLEW_MAX_PWM;
        int dR = rightCmd - prevRightCmd;
        if (dR >  SLEW_MAX_PWM) rightCmd = prevRightCmd + SLEW_MAX_PWM;
        if (dR < -SLEW_MAX_PWM) rightCmd = prevRightCmd - SLEW_MAX_PWM;
        prevLeftCmd  = leftCmd;
        prevRightCmd = rightCmd;
#endif

        motorDrive(DIR_FORWARD, leftCmd, rightCmd);

        // Лог ~10 Гц
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 100) {
            lastLog = millis();
            Serial.print("t=");     Serial.print((millis() - loopStart) / 1000.0f, 1);
            Serial.print("s θ=");   Serial.print(theta_deg, 2);
            Serial.print(" ω=");    Serial.print(omega_filt, 1);
#ifndef OPEN_LOOP
            Serial.print(" I=");    Serial.print(pidIntegral, 2);
            Serial.print(" s=");    Serial.print(controlSign);
            if (theta_deg > AGGRESSIVE_DEG || theta_deg < -AGGRESSIVE_DEG) {
                Serial.print("[!]");
            }
#endif
            Serial.print(" L=");    Serial.print(leftCmd);
            Serial.print(" R=");    Serial.print(rightCmd);
            Serial.print(" d=");    Serial.println(obstacle, 1);
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

    while (true) { delay(1000); }
}
