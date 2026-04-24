/*
 * Vpered — калибровка руки и клешни
 *
 * Отдельный скетч для настройки трёх серв:
 *   BASE (D9)  — поворот базы вокруг вертикальной оси
 *   ARM  (D10) — подъём/опускание плеча
 *   CLAW (D11) — открытие/закрытие клешни
 *
 * Что делает: даёт ручное управление через Serial (9600 бод) + готовую
 * последовательность захвата «переведи руку вперёд → открой → закрой → подними».
 *
 * Serial-команды (1 символ, без перевода строки):
 *   b / B  — BASE -2° / +2°
 *   a / A  — ARM  -2° / +2°
 *   c / C  — CLAW -2° / +2°
 *   n      — маленький шаг 1° (следующее b/a/c будет на 1°) [не реализовано, см. 1/5/0]
 *   1 / 5 / 0  — переключить шаг: 1° / 5° / 10° (по умолчанию 2°)
 *   o      — OPEN_CLAW (быстро открыть клешню на CLAW_OPEN)
 *   x      — CLOSE_CLAW (быстро закрыть на CLAW_CLOSED)
 *   f      — ARM_FORWARD (рука вперёд, для захвата)
 *   p      — ARM_PARK (рука в парк, нейтраль)
 *   g      — полная последовательность GRAB (f → o → задержка → x → p)
 *   r      — RESET в стартовую позицию
 *   ?      — показать текущие углы и пресеты
 *
 * Как настраивать:
 *   1. Поставь робота перед маленьким предметом на нужном расстоянии.
 *   2. Нажми 'f' — рука пойдёт в позу для захвата. Клешня должна оказаться
 *      ВОКРУГ объекта. Подбирай ARM_FORWARD (a/A по 2°) пока не совпадёт.
 *      Если клешня слишком высоко/низко — ARM. Если смещена вбок — BASE.
 *   3. Нажми 'o' — клешня открывается. Подбирай CLAW_OPEN пока не будет
 *      достаточно широкая чтобы объект помещался свободно.
 *   4. Нажми 'x' — закрывается. Подбирай CLAW_CLOSED пока объект не
 *      зажимается уверенно, но без перегрузки серво.
 *   5. Нажми 'g' — проверь всю последовательность целиком.
 *   6. Перенеси подобранные значения в основную прошивку vpered_uno.ino
 *      (константы ниже).
 */

#include <Servo.h>

#define CLAW_PIN 11
#define ARM_PIN  10
#define BASE_PIN 9

// ========== ПРЕСЕТЫ (КАЛИБРОВАТЬ!) ==========
// Стартовая (парк) поза — безопасная для включения.
int BASE_PARK    = 90;
int ARM_PARK     = 90;
int CLAW_PARK    = 135;

// Поза «рука вперёд для захвата». Обычно ARM опускается (число меньше 90),
// BASE смотрит вперёд (90). Подбирай под свой робот.
int BASE_FORWARD = 90;
int ARM_FORWARD  = 40;

// Открытая / закрытая клешня. Точные значения зависят от конструкции.
// Типично: открытая = больше, закрытая = меньше. Если наоборот — поменяй.
int CLAW_OPEN    = 160;
int CLAW_CLOSED  = 70;

// Задержки между шагами последовательности (мс)
const int SERVO_SETTLE_MS = 400;   // даём серве доехать
const int GRAB_HOLD_MS    = 300;   // пауза после открытия, перед закрытием

// ========== СОСТОЯНИЕ ==========
Servo clawServo, armServo, baseServo;

int baseAngle = 90;
int armAngle  = 90;
int clawAngle = 135;
int stepDeg   = 2;    // шаг изменения при b/a/c

// Плавная подача угла на серво — чтобы не дёргалось рывком
void smoothWriteServo(Servo &s, int &currentAngle, int targetAngle) {
    targetAngle = constrain(targetAngle, 0, 180);
    int step = (targetAngle > currentAngle) ? 1 : -1;
    while (currentAngle != targetAngle) {
        currentAngle += step;
        s.write(currentAngle);
        delay(8);
    }
}

void moveBase(int a) { smoothWriteServo(baseServo, baseAngle, a); }
void moveArm(int a)  { smoothWriteServo(armServo,  armAngle,  a); }
void moveClaw(int a) { smoothWriteServo(clawServo, clawAngle, a); }

void goPark() {
    Serial.println("→ PARK");
    moveClaw(CLAW_PARK);
    moveArm(ARM_PARK);
    moveBase(BASE_PARK);
}

void armForward() {
    Serial.println("→ ARM FORWARD");
    moveBase(BASE_FORWARD);
    moveArm(ARM_FORWARD);
}

void openClaw() {
    Serial.println("→ OPEN CLAW");
    moveClaw(CLAW_OPEN);
}

void closeClaw() {
    Serial.println("→ CLOSE CLAW");
    moveClaw(CLAW_CLOSED);
}

void grabSequence() {
    Serial.println("=== GRAB SEQUENCE ===");

    // 1. Убедиться что клешня открыта (чтобы не врезаться в объект)
    openClaw();
    delay(SERVO_SETTLE_MS);

    // 2. Перевести руку вперёд над объектом
    armForward();
    delay(SERVO_SETTLE_MS);

    // 3. Короткая пауза перед закрытием (объект должен быть между пальцев)
    delay(GRAB_HOLD_MS);

    // 4. Закрыть клешню — захват
    closeClaw();
    delay(SERVO_SETTLE_MS);

    // 5. Поднять руку в парк с объектом
    moveArm(ARM_PARK);
    delay(SERVO_SETTLE_MS);

    Serial.println("=== DONE ===");
    printStatus();
}

void printStatus() {
    Serial.println();
    Serial.println("---- STATUS ----");
    Serial.print("BASE = "); Serial.print(baseAngle);
    Serial.print("°   ARM = "); Serial.print(armAngle);
    Serial.print("°   CLAW = "); Serial.print(clawAngle); Serial.println("°");
    Serial.print("step = "); Serial.print(stepDeg); Serial.println("°");
    Serial.println();
    Serial.println("PRESETS:");
    Serial.print("  PARK     → B="); Serial.print(BASE_PARK);    Serial.print(" A="); Serial.print(ARM_PARK);    Serial.print(" C="); Serial.println(CLAW_PARK);
    Serial.print("  FORWARD  → B="); Serial.print(BASE_FORWARD); Serial.print(" A="); Serial.println(ARM_FORWARD);
    Serial.print("  CLAW     → open="); Serial.print(CLAW_OPEN); Serial.print("  closed="); Serial.println(CLAW_CLOSED);
    Serial.println("----------------");
}

void printHelp() {
    Serial.println();
    Serial.println("=== GRAB TEST ===");
    Serial.println("b/B  : BASE -step/+step");
    Serial.println("a/A  : ARM  -step/+step");
    Serial.println("c/C  : CLAW -step/+step");
    Serial.println("1/5/0: шаг 1° / 5° / 10°  (default 2°)");
    Serial.println("o / x: открыть / закрыть клешню");
    Serial.println("f    : рука вперёд");
    Serial.println("p    : парк");
    Serial.println("g    : полная последовательность захвата");
    Serial.println("r    : reset в парк");
    Serial.println("?    : статус");
    Serial.println();
    Serial.println("Совет: меняй текущий угол (b/a/c), проверяй 'f','o','x'.");
    Serial.println("      Найдя нужный угол, ОБНОВИ пресеты в коде и перезалей.");
    printStatus();
}

void setup() {
    Serial.begin(9600);
    delay(200);

    clawServo.attach(CLAW_PIN);
    armServo.attach(ARM_PIN);
    baseServo.attach(BASE_PIN);

    // Аккуратно выставляем серво в парк-позу с плавным переходом:
    // сначала кажется текущим, сразу пишем парк, затем обновляем состояние.
    baseServo.write(BASE_PARK); baseAngle = BASE_PARK;
    armServo.write(ARM_PARK);   armAngle  = ARM_PARK;
    clawServo.write(CLAW_PARK); clawAngle = CLAW_PARK;
    delay(600);

    printHelp();
}

void loop() {
    if (!Serial.available()) return;
    char ch = Serial.read();

    switch (ch) {
        // base
        case 'b': moveBase(baseAngle - stepDeg); printStatus(); break;
        case 'B': moveBase(baseAngle + stepDeg); printStatus(); break;

        // arm
        case 'a': moveArm(armAngle - stepDeg); printStatus(); break;
        case 'A': moveArm(armAngle + stepDeg); printStatus(); break;

        // claw
        case 'c': moveClaw(clawAngle - stepDeg); printStatus(); break;
        case 'C': moveClaw(clawAngle + stepDeg); printStatus(); break;

        // step size
        case '1': stepDeg = 1;  Serial.println("step=1°");  break;
        case '5': stepDeg = 5;  Serial.println("step=5°");  break;
        case '0': stepDeg = 10; Serial.println("step=10°"); break;

        // презеты
        case 'o': openClaw();  printStatus(); break;
        case 'x': closeClaw(); printStatus(); break;
        case 'f': armForward(); printStatus(); break;
        case 'p': goPark();     printStatus(); break;

        // последовательность
        case 'g': grabSequence(); break;

        // reset
        case 'r': goPark(); printStatus(); break;

        // help / status
        case '?': printStatus(); break;
        case 'h': case 'H': printHelp(); break;

        // игнорируем \r \n пробел
        case '\r': case '\n': case ' ': break;

        default:
            Serial.print("? unknown: '");
            Serial.print(ch);
            Serial.println("' — нажми 'h' для справки");
            break;
    }
}
