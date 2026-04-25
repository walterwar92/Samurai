/*
 * Vpered — Arduino Uno controller (USB Serial, 9600 baud)
 *
 * Объединённая прошивка: моторы + ПИД-курсодержание + ультразвук +
 * клешня (3 серво) + Serial-протокол управления с компьютера.
 *
 * Главное отличие от автономной версии — НЕблокирующий main loop:
 * команда из Serial меняет state, действия выполняются в tick'е.
 *
 * === ПРОТОКОЛ (текстовые команды, разделитель \n или \r) ===
 *
 *   F          — drive forward (с ПИД-удержанием курса)
 *   B          — drive backward
 *   L          — pivot left  (правое колесо вперёд, левое стоит)
 *   R          — pivot right (левое колесо вперёд, правое стоит)
 *   S          — stop
 *
 *   O          — open claw
 *   X          — close claw
 *   G          — grab sequence (open → arm forward → close → arm up)
 *   P          — park arm
 *   M<deg>     — move ARM to angle (0..180), e.g. "M45"
 *   N<deg>     — move BASE to angle (0..180)
 *   D          — detach all servos NOW
 *
 *   K          — manual kick-start
 *   C          — calibrate gyro + control sign (~3 сек, робот на полу!)
 *   T          — toggle telemetry stream
 *   Z          — zero heading (target = current θ)
 *   H          — print help
 *
 * === ТЕЛЕМЕТРИЯ (раз в 200 мс если включена) ===
 *
 *   T,θ=2.30,ω=0.5,d=120.4,L=180,R=180,m=FWD,obs=0,arm=90,claw=70
 */

#include <Wire.h>
#include <Servo.h>

// ========== ПИНЫ ==========
#define PWM1_PIN  5    // левый PWM
#define PWM2_PIN  6    // правый PWM
#define SHCP_PIN  2
#define EN_PIN    7
#define DATA_PIN  8
#define STCP_PIN  4
#define CLAW_PIN  11
#define ARM_PIN   10
#define BASE_PIN  9
#define TRIG_PIN  12
#define ECHO_PIN  13

// ========== НАПРАВЛЕНИЯ (биты 74HC595) ==========
// Эти биты подаются на 8 выходов 74HC595, которые управляют входами H-моста.
// Точная распиновка моста на этом шасси неизвестна, поэтому DIR_FORWARD
// был подобран опытно (=92), а DIR_BACKWARD нужно подобрать командой Y<N>.
//
// КАК ПОДОБРАТЬ DIR_BACKWARD:
//   1. Подними робота (или поставь на подставку чтобы колёса крутились свободно).
//   2. Из UI на странице Vpered → панель "Подбор reverse" → набирай байты,
//      или из Serial шли команды вида "Y163", "Y172", "Y228" и т.д.
//   3. Каждая команда Y запускает моторы на 1 сек с этим DIR-байтом и BASE PWM.
//   4. Когда оба колеса крутятся НАЗАД (одинаково и вместе) — впиши значение
//      сюда в DIR_BACKWARD и перепрошей, кнопка "Назад" заработает.
//
// Типичные кандидаты для классического 74HC595+L298 шасси (если FWD=92):
//   163 (~92 ^ 0xFF), 172, 228, 35, 76, 240, 95, 80
const uint8_t DIR_FORWARD  = 92;    // 0b01011100  (подобрано в оригинале)
const uint8_t DIR_STOP     = 0;     // 0b00000000  (все H-мосты в high-Z)
const uint8_t DIR_BACKWARD = 163;   // 0b10100011  (FORWARD XOR 0xFF — на этом
                                    //  шасси полная инверсия битов даёт reverse)

// ========== ПАРАМЕТРЫ ==========
const float STOP_DISTANCE_CM = 10.0f;
const int   BASE_PWM         = 180;
const int   MIN_PWM          = 70;
const int   MAX_PWM          = 255;
const int   KICK_PWM         = 255;
const int   KICK_MS          = 150;

const float LEFT_TRIM  = 1.00f;
const float RIGHT_TRIM = 1.00f;

// PI+D
float Kp = 2.2f;
float Ki = 0.12f;
float Kd = 0.40f;
const float INTEGRAL_MAX_DEG_S   = 60.0f;
const float HEADING_DEADBAND_DEG = 0.3f;
const float OMEGA_LPF_ALPHA      = 0.35f;
const int   SLEW_MAX_PWM         = 50;
const float CORRECTION_CLAMP     = 100.0f;
const float AGGRESSIVE_DEG       = 12.0f;
const float AGGRESSIVE_BOOST     = 2.5f;

const int GYRO_SIGN = +1;
int controlSign = +1;

// ========== СОСТОЯНИЕ ==========
enum RobotMode {
    MODE_IDLE,
    MODE_FWD,
    MODE_BWD,
    MODE_LEFT,
    MODE_RIGHT
};
RobotMode mode = MODE_IDLE;
float targetTheta = 0.0f;

// ========== СЕРВО ПРЕСЕТЫ ==========
int BASE_PARK    = 90;
int ARM_PARK     = 90;
int CLAW_PARK    = 135;
int BASE_FORWARD = 90;
int ARM_FORWARD  = 40;
int CLAW_OPEN    = 70;
int CLAW_CLOSED  = 150;

const int SERVO_SETTLE_MS = 400;
const int GRAB_HOLD_MS    = 300;

const unsigned long DETACH_IDLE_MS = 600;
bool autoDetach = true;

Servo clawServo, armServo, baseServo;
int baseAngle = 90, armAngle = 90, clawAngle = 135;
unsigned long lastBaseMs = 0, lastArmMs = 0, lastClawMs = 0;

// ========== ГИРОСКОП ==========
#define MPU6050_ADDR 0x68
float theta = 0, theta_deg = 0, omega = 0, omega_deg = 0, omega_filt = 0;
float dt_sec = 0;
unsigned long lastTime = 0;
float gyroBiasZ = 0;

float pidIntegral = 0;
int prevLeftCmd = 0, prevRightCmd = 0;
unsigned long watchdogSince = 0;

// ========== ТЕЛЕМЕТРИЯ ==========
bool telemetryEnabled = true;
float lastDistance = 999.0f;
bool obstacleStop = false;

// ============================================================
// MOTOR
// ============================================================
static inline int applyTrim(int speed, float trim) {
    if (speed <= 0) return 0;
    int v = (int)(speed * trim + 0.5f);
    return constrain(v, MIN_PWM, MAX_PWM);
}

void motorRaw(uint8_t dir, int pwmL, int pwmR) {
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, constrain(pwmL, 0, 255));
    analogWrite(PWM2_PIN, constrain(pwmR, 0, 255));
    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, dir);
    digitalWrite(STCP_PIN, HIGH);
    delayMicroseconds(100);
}

void motorDrive(uint8_t dir, int sL, int sR) {
    motorRaw(dir, applyTrim(sL, LEFT_TRIM), applyTrim(sR, RIGHT_TRIM));
}

void motorStop() {
    motorRaw(DIR_STOP, 0, 0);
    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, 0);
    digitalWrite(STCP_PIN, HIGH);
    prevLeftCmd = 0;
    prevRightCmd = 0;
}

void kickStart() {
    motorRaw(DIR_FORWARD, KICK_PWM, KICK_PWM);
    delay(KICK_MS);
    for (int p = KICK_PWM; p >= BASE_PWM; p -= 10) {
        motorDrive(DIR_FORWARD, p, p);
        delay(15);
    }
    prevLeftCmd  = BASE_PWM;
    prevRightCmd = BASE_PWM;
}

// ============================================================
// ULTRASONIC
// ============================================================
float readDistanceRaw() {
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long d = pulseIn(ECHO_PIN, HIGH, 25000);
    if (d == 0) return 999.0f;
    return d * 0.0343f / 2.0f;
}

// ============================================================
// GYRO
// ============================================================
void readGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission((uint8_t)MPU6050_ADDR);
    Wire.write((uint8_t)0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    for (int i = 0; i < 4; i++) (void)(Wire.read() << 8 | Wire.read());
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
    Serial.println(F("Calibrating gyro..."));
    const int N = 300;
    long sum = 0;
    int16_t gx, gy, gz;
    for (int i = 0; i < N; i++) {
        readGyroRaw(&gx, &gy, &gz);
        sum += gz;
        delay(3);
    }
    gyroBiasZ = (float)sum / N;
    Serial.print(F("bias=")); Serial.println(gyroBiasZ);
}

void updateGyro() {
    int16_t gx, gy, gz;
    readGyroRaw(&gx, &gy, &gz);
    omega_deg = GYRO_SIGN * (gz - gyroBiasZ) / 131.0f;
    omega = omega_deg * PI / 180.0f;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.1f) dt = 0.0f;
    lastTime = now;
    dt_sec = dt;
    theta += omega * dt;
    theta_deg = theta * 180.0f / PI;
    omega_filt = OMEGA_LPF_ALPHA * omega_deg + (1.0f - OMEGA_LPF_ALPHA) * omega_filt;
}

void resetGyroIntegral() {
    theta = 0;
    pidIntegral = 0;
    omega_filt = 0;
    lastTime = micros();
    targetTheta = 0;
    watchdogSince = 0;
}

void autoCalibrateControlSign() {
    Serial.println(F("Auto-cal sign..."));
    delay(120);
    theta = 0;
    lastTime = micros();
    updateGyro();
    float thetaBefore = theta_deg;
    motorRaw(DIR_FORWARD, BASE_PWM - 60, BASE_PWM + 60);
    unsigned long t0 = millis();
    while (millis() - t0 < 250) { updateGyro(); delay(10); }
    motorRaw(DIR_FORWARD, BASE_PWM, BASE_PWM);
    delay(60);
    updateGyro();
    float drift = theta_deg - thetaBefore;
    Serial.print(F("drift=")); Serial.print(drift); Serial.print(F(" sign="));
    if (drift > 1.5f)       { controlSign = -1; Serial.println(-1); }
    else if (drift < -1.5f) { controlSign = +1; Serial.println(+1); }
    else                    { Serial.println(F("WEAK")); }
    resetGyroIntegral();
    prevLeftCmd  = BASE_PWM;
    prevRightCmd = BASE_PWM;
}

// ============================================================
// SERVOS
// ============================================================
void ensureBase() { if (!baseServo.attached()) baseServo.attach(BASE_PIN); lastBaseMs = millis(); }
void ensureArm()  { if (!armServo.attached())  armServo.attach(ARM_PIN);   lastArmMs  = millis(); }
void ensureClaw() { if (!clawServo.attached()) clawServo.attach(CLAW_PIN); lastClawMs = millis(); }

void checkAutoDetach() {
    if (!autoDetach) return;
    unsigned long now = millis();
    if (baseServo.attached() && now - lastBaseMs > DETACH_IDLE_MS) baseServo.detach();
    if (armServo.attached()  && now - lastArmMs  > DETACH_IDLE_MS) armServo.detach();
    if (clawServo.attached() && now - lastClawMs > DETACH_IDLE_MS) clawServo.detach();
}

void smoothServo(Servo &s, int &cur, int tgt) {
    tgt = constrain(tgt, 0, 180);
    int step = (tgt > cur) ? 1 : -1;
    while (cur != tgt) { cur += step; s.write(cur); delay(8); }
}

void moveBase(int a) { ensureBase(); smoothServo(baseServo, baseAngle, a); lastBaseMs = millis(); }
void moveArm(int a)  { ensureArm();  smoothServo(armServo,  armAngle,  a); lastArmMs  = millis(); }
void moveClaw(int a) { ensureClaw(); smoothServo(clawServo, clawAngle, a); lastClawMs = millis(); }

void detachAll() {
    baseServo.detach(); armServo.detach(); clawServo.detach();
}

void doGrab() {
    Serial.println(F("GRAB"));
    moveClaw(CLAW_OPEN);   delay(SERVO_SETTLE_MS);
    moveBase(BASE_FORWARD);
    moveArm(ARM_FORWARD);  delay(SERVO_SETTLE_MS);
    delay(GRAB_HOLD_MS);
    moveClaw(CLAW_CLOSED); delay(SERVO_SETTLE_MS);
    moveArm(ARM_PARK);     delay(SERVO_SETTLE_MS);
    Serial.println(F("OK"));
}

void doPark() {
    moveClaw(CLAW_PARK);
    moveArm(ARM_PARK);
    moveBase(BASE_PARK);
}

// ============================================================
// MOTION TICK
// ============================================================
void tickForward() {
    updateGyro();
    float err = theta_deg - targetTheta;
    float errEff = (err > -HEADING_DEADBAND_DEG && err < HEADING_DEADBAND_DEG) ? 0.0f : err;
    pidIntegral += errEff * dt_sec;
    if (pidIntegral >  INTEGRAL_MAX_DEG_S) pidIntegral =  INTEGRAL_MAX_DEG_S;
    if (pidIntegral < -INTEGRAL_MAX_DEG_S) pidIntegral = -INTEGRAL_MAX_DEG_S;

    float kpEff = (err > AGGRESSIVE_DEG || err < -AGGRESSIVE_DEG) ? Kp * AGGRESSIVE_BOOST : Kp;
    float corr = controlSign * (kpEff * errEff + Ki * pidIntegral + Kd * omega_filt);
    if (corr >  CORRECTION_CLAMP) corr =  CORRECTION_CLAMP;
    if (corr < -CORRECTION_CLAMP) corr = -CORRECTION_CLAMP;

    int leftCmd  = BASE_PWM - (int)corr;
    int rightCmd = BASE_PWM + (int)corr;

    int dL = leftCmd - prevLeftCmd;
    if (dL >  SLEW_MAX_PWM) leftCmd = prevLeftCmd + SLEW_MAX_PWM;
    if (dL < -SLEW_MAX_PWM) leftCmd = prevLeftCmd - SLEW_MAX_PWM;
    int dR = rightCmd - prevRightCmd;
    if (dR >  SLEW_MAX_PWM) rightCmd = prevRightCmd + SLEW_MAX_PWM;
    if (dR < -SLEW_MAX_PWM) rightCmd = prevRightCmd - SLEW_MAX_PWM;
    prevLeftCmd  = leftCmd;
    prevRightCmd = rightCmd;

    motorDrive(DIR_FORWARD, leftCmd, rightCmd);
}

void tickPivot(bool leftDir) {
    // Левый поворот: правый колесо вперёд, левое стоит → робот разворачивается влево
    // (физическая полярность зависит от шасси, при необходимости — поменяй).
    updateGyro();
    if (leftDir) {
        motorDrive(DIR_FORWARD, 0, BASE_PWM);
    } else {
        motorDrive(DIR_FORWARD, BASE_PWM, 0);
    }
}

void tickBackward() {
    // Простой open-loop задний ход (без ПИД — гироскоп правильнее интегрируется
    // только при движении вперёд). Если DIR_BACKWARD не задан — стоп.
    if (DIR_BACKWARD == 0) {
        motorStop();
        return;
    }
    updateGyro();
    motorDrive(DIR_BACKWARD, BASE_PWM, BASE_PWM);
}

// Тестовая прокрутка моторов произвольным DIR-байтом (для подбора DIR_BACKWARD)
unsigned long dirTestUntilMs = 0;
uint8_t dirTestByte = 0;

void startDirTest(uint8_t dirByte, unsigned long ms = 1000) {
    dirTestByte = dirByte;
    dirTestUntilMs = millis() + ms;
    // MAX_PWM чтобы точно проявить даже «слабое» направление при
    // неправильной комбинации битов. И motorRaw напрямую — без trim/clamp
    // которые могут мешать.
    motorRaw(dirByte, MAX_PWM, MAX_PWM);
    Serial.print(F("DIR-TEST byte=")); Serial.print(dirByte);
    Serial.print(F(" (0b"));
    for (int i = 7; i >= 0; i--) Serial.print((dirByte >> i) & 1);
    Serial.print(F(") for ")); Serial.print(ms); Serial.println(F("ms"));
}

void tickDirTest() {
    if (millis() >= dirTestUntilMs) {
        motorStop();
        dirTestUntilMs = 0;
        Serial.println(F("DIR-TEST done"));
    }
}

// ============================================================
// SERIAL HANDLER
// ============================================================
char cmdBuf[24];
uint8_t cmdLen = 0;

void enterMode(RobotMode m) {
    if (m == mode) return;
    mode = m;
    if (m == MODE_FWD) {
        // полноценный старт с kick-start и сбросом интегралов
        kickStart();
        resetGyroIntegral();
    } else if (m == MODE_LEFT || m == MODE_RIGHT) {
        // мягкий старт без kick (поворот не нужен kick)
        prevLeftCmd = prevRightCmd = 0;
    } else if (m == MODE_IDLE) {
        motorStop();
    }
}

void printHelp() {
    Serial.println(F("=== VPERED CMDS ==="));
    Serial.println(F("F B L R S    : drive / stop"));
    Serial.println(F("Y<0..255>    : test DIR byte 1s (для подбора DIR_BACKWARD)"));
    Serial.println(F("O X G P      : open / close / grab / park"));
    Serial.println(F("M<deg>       : ARM angle"));
    Serial.println(F("N<deg>       : BASE angle"));
    Serial.println(F("D            : detach servos"));
    Serial.println(F("K C T Z H    : kick / cal / tlm / zero / help"));
}

void executeCommand() {
    if (cmdLen == 0) return;
    char c = cmdBuf[0];
    int arg = (cmdLen > 1) ? atoi(cmdBuf + 1) : 0;

    switch (c) {
        case 'F': enterMode(MODE_FWD);   Serial.println(F("FWD"));  break;
        case 'B':
            if (DIR_BACKWARD == 0) {
                Serial.println(F("BWD disabled — DIR_BACKWARD=0, подбери через Y<N>"));
                enterMode(MODE_IDLE);
            } else {
                enterMode(MODE_BWD);
                Serial.println(F("BWD"));
            }
            break;
        case 'L': enterMode(MODE_LEFT);  Serial.println(F("LEFT")); break;
        case 'R': enterMode(MODE_RIGHT); Serial.println(F("RIGHT")); break;
        case 'S': enterMode(MODE_IDLE);  Serial.println(F("STOP")); break;
        case 'Y': {
            // Y<num> — тест произвольного DIR-байта на 1 сек.
            // Используется для подбора DIR_BACKWARD.
            int dir = arg;
            if (dir < 0 || dir > 255) {
                Serial.println(F("Y: byte 0..255"));
            } else {
                enterMode(MODE_IDLE);
                startDirTest((uint8_t)dir, 1000);
            }
            break;
        }
        case 'O': moveClaw(CLAW_OPEN);   Serial.println(F("OPEN")); break;
        case 'X': moveClaw(CLAW_CLOSED); Serial.println(F("CLOSE")); break;
        case 'G': enterMode(MODE_IDLE); doGrab(); break;
        case 'P': enterMode(MODE_IDLE); doPark(); Serial.println(F("PARK")); break;
        case 'M':
            if (arg >= 0 && arg <= 180) { moveArm(arg);  Serial.print(F("ARM=")); Serial.println(arg); }
            break;
        case 'N':
            if (arg >= 0 && arg <= 180) { moveBase(arg); Serial.print(F("BASE=")); Serial.println(arg); }
            break;
        case 'D': detachAll(); Serial.println(F("DETACH")); break;
        case 'K': enterMode(MODE_IDLE); kickStart(); enterMode(MODE_IDLE); Serial.println(F("KICK")); break;
        case 'C':
            enterMode(MODE_IDLE);
            calibrateGyro();
            kickStart();
            autoCalibrateControlSign();
            enterMode(MODE_IDLE);
            break;
        case 'T':
            telemetryEnabled = !telemetryEnabled;
            Serial.print(F("TLM=")); Serial.println(telemetryEnabled ? F("ON") : F("OFF"));
            break;
        case 'Z':
            targetTheta = theta_deg;
            pidIntegral = 0;
            Serial.print(F("ZERO θ=")); Serial.println(targetTheta);
            break;
        case 'H': printHelp(); break;
        default:
            Serial.print(F("?")); Serial.println(c);
    }
    cmdLen = 0;
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmdBuf[cmdLen] = '\0';
            executeCommand();
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// ============================================================
// TELEMETRY
// ============================================================
const __FlashStringHelper* modeName() {
    switch (mode) {
        case MODE_IDLE:  return F("IDLE");
        case MODE_FWD:   return F("FWD");
        case MODE_BWD:   return F("BWD");
        case MODE_LEFT:  return F("LEFT");
        case MODE_RIGHT: return F("RIGHT");
    }
    return F("?");
}

void sendTelemetry() {
    Serial.print(F("T,th=")); Serial.print(theta_deg, 2);
    Serial.print(F(",om="));  Serial.print(omega_filt, 1);
    Serial.print(F(",d="));   Serial.print(lastDistance, 1);
    Serial.print(F(",L="));   Serial.print(prevLeftCmd);
    Serial.print(F(",R="));   Serial.print(prevRightCmd);
    Serial.print(F(",m="));   Serial.print(modeName());
    Serial.print(F(",ob="));  Serial.print(obstacleStop ? 1 : 0);
    Serial.print(F(",a="));   Serial.print(armAngle);
    Serial.print(F(",b="));   Serial.print(baseAngle);
    Serial.print(F(",c="));   Serial.println(clawAngle);
}

// ============================================================
// SETUP / LOOP
// ============================================================
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

    // park position (без долгого attach — серво будут attach по требованию)
    baseServo.attach(BASE_PIN); baseServo.write(BASE_PARK); baseAngle = BASE_PARK;
    armServo.attach(ARM_PIN);   armServo.write(ARM_PARK);   armAngle  = ARM_PARK;
    clawServo.attach(CLAW_PIN); clawServo.write(CLAW_PARK); clawAngle = CLAW_PARK;
    delay(500);

    Wire.begin();
    Wire.beginTransmission((uint8_t)MPU6050_ADDR);
    Wire.write((uint8_t)0x6B);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
    delay(100);

    Serial.println(F("=== VPERED v2 ==="));
    calibrateGyro();
    Serial.println(F("READY (send 'H' for help)"));

    unsigned long now = millis();
    lastBaseMs = lastArmMs = lastClawMs = now;
}

void loop() {
    handleSerial();
    checkAutoDetach();

    // DIR-test (tested байт работает 1 сек, потом стоп)
    if (dirTestUntilMs != 0) tickDirTest();

    // Active state tick (~50 Hz)
    static unsigned long lastTick = 0;
    unsigned long now = millis();
    if (now - lastTick >= 20) {
        lastTick = now;
        if (mode == MODE_FWD)        tickForward();
        else if (mode == MODE_BWD)   tickBackward();
        else if (mode == MODE_LEFT)  tickPivot(true);
        else if (mode == MODE_RIGHT) tickPivot(false);
    }

    // Ультразвук — раз в 100 мс (медленный pulseIn до 25 мс)
    static unsigned long lastSonar = 0;
    if (now - lastSonar >= 100) {
        lastSonar = now;
        lastDistance = readDistanceRaw();
        // Авто-стоп при езде вперёд
        if (mode == MODE_FWD && lastDistance < STOP_DISTANCE_CM) {
            obstacleStop = true;
            enterMode(MODE_IDLE);
            Serial.print(F("OBS stop d=")); Serial.println(lastDistance, 1);
        } else if (lastDistance > STOP_DISTANCE_CM + 5.0f) {
            obstacleStop = false;
        }
    }

    // Watchdog: курс уехал безнадёжно при FWD → STOP
    if (mode == MODE_FWD) {
        float err = theta_deg - targetTheta;
        if (err > 30.0f || err < -30.0f) {
            if (watchdogSince == 0) watchdogSince = now;
            if (now - watchdogSince > 1500) {
                enterMode(MODE_IDLE);
                Serial.println(F("WATCHDOG stop"));
                watchdogSince = 0;
            }
        } else {
            watchdogSince = 0;
        }
    }

    // Telemetry (~5 Hz)
    static unsigned long lastTelem = 0;
    if (telemetryEnabled && now - lastTelem >= 200) {
        lastTelem = now;
        sendTelemetry();
    }
}
