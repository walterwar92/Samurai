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
 * Serial-команды (без Enter, 1 символ):
 *   b / B     — BASE -step / +step
 *   a / A     — ARM  -step / +step
 *   c / C     — CLAW -step / +step
 *   1 / 5 / 0 — шаг 1° / 5° / 10° (по умолчанию 2°)
 *   o / x     — открыть / закрыть клешню (пресетами CLAW_OPEN/CLAW_CLOSED)
 *   ~         — swap CLAW_OPEN <-> CLAW_CLOSED (если полярность серв обратная)
 *   f / p     — ARM_FORWARD / ARM_PARK
 *   g         — полная последовательность захвата
 *   r / ? / h — reset / статус / справка
 *
 * Сохранение текущего положения как пресет (2 символа):
 *   s o — текущий CLAW → CLAW_OPEN
 *   s x — текущий CLAW → CLAW_CLOSED
 *   s f — текущие BASE+ARM → FORWARD
 *   s p — текущие BASE+ARM+CLAW → PARK
 *   e   — экспортировать все пресеты готовым copy-paste блоком
 *
 * Workflow:
 *   1. Крути угол вручную (b/a/c) — ищешь подходящее положение для шага.
 *   2. Когда нашёл — сохрани (so/sx/sf/sp) чтобы эта позиция стала пресетом.
 *   3. Проверь целиком командой 'g' — последовательность работает с новыми пресетами.
 *   4. 'e' — напечатает готовый блок констант для копирования в vpered_uno.ino.
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

// Открытая / закрытая клешня. Полярность зависит от конструкции:
// на этом роботе МЕНЬШЕ = открыто. Если на твоём наоборот — нажми '~'
// в Serial чтобы swap'нуть, или поменяй значения здесь.
int CLAW_OPEN    = 70;
int CLAW_CLOSED  = 160;

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
    Serial.println("o / x: открыть / закрыть клешню (пресетами)");
    Serial.println("~    : swap CLAW_OPEN <-> CLAW_CLOSED (если 'o' и 'x' перепутаны)");
    Serial.println("f    : рука вперёд");
    Serial.println("p    : парк");
    Serial.println("g    : полная последовательность захвата");
    Serial.println("r    : reset в парк");
    Serial.println("?    : статус");
    Serial.println("e    : экспорт пресетов (copy-paste в основную прошивку)");
    Serial.println();
    Serial.println("СОХРАНЕНИЕ текущего положения как пресет:");
    Serial.println("  so : текущий CLAW -> CLAW_OPEN");
    Serial.println("  sx : текущий CLAW -> CLAW_CLOSED");
    Serial.println("  sf : текущие BASE+ARM -> FORWARD");
    Serial.println("  sp : текущие BASE+ARM+CLAW -> PARK");
    Serial.println();
    Serial.println("Рабочий цикл: крути b/a/c руками, найди хорошее положение,");
    Serial.println("  сохрани через so/sx/sf/sp, проверь 'g', потом 'e' и скопируй.");
    printStatus();
}

// ========== СОХРАНЕНИЕ ПРЕСЕТОВ ==========
void saveAsOpen() {
    CLAW_OPEN = clawAngle;
    Serial.print("SAVED CLAW_OPEN = "); Serial.println(CLAW_OPEN);
}
void saveAsClosed() {
    CLAW_CLOSED = clawAngle;
    Serial.print("SAVED CLAW_CLOSED = "); Serial.println(CLAW_CLOSED);
}
void saveAsForward() {
    BASE_FORWARD = baseAngle;
    ARM_FORWARD  = armAngle;
    Serial.print("SAVED FORWARD: BASE="); Serial.print(BASE_FORWARD);
    Serial.print(" ARM="); Serial.println(ARM_FORWARD);
}
void saveAsPark() {
    BASE_PARK = baseAngle;
    ARM_PARK  = armAngle;
    CLAW_PARK = clawAngle;
    Serial.print("SAVED PARK: BASE="); Serial.print(BASE_PARK);
    Serial.print(" ARM="); Serial.print(ARM_PARK);
    Serial.print(" CLAW="); Serial.println(CLAW_PARK);
}

void swapClawPolarity() {
    int tmp = CLAW_OPEN;
    CLAW_OPEN = CLAW_CLOSED;
    CLAW_CLOSED = tmp;
    Serial.print("SWAPPED: CLAW_OPEN="); Serial.print(CLAW_OPEN);
    Serial.print(" CLAW_CLOSED="); Serial.println(CLAW_CLOSED);
}

void exportPresets() {
    Serial.println();
    Serial.println("// ======== COPY-PASTE В vpered_uno.ino ========");
    Serial.print("const int BASE_PARK    = "); Serial.print(BASE_PARK);    Serial.println(";");
    Serial.print("const int ARM_PARK     = "); Serial.print(ARM_PARK);     Serial.println(";");
    Serial.print("const int CLAW_PARK    = "); Serial.print(CLAW_PARK);    Serial.println(";");
    Serial.print("const int BASE_FORWARD = "); Serial.print(BASE_FORWARD); Serial.println(";");
    Serial.print("const int ARM_FORWARD  = "); Serial.print(ARM_FORWARD);  Serial.println(";");
    Serial.print("const int CLAW_OPEN    = "); Serial.print(CLAW_OPEN);    Serial.println(";");
    Serial.print("const int CLAW_CLOSED  = "); Serial.print(CLAW_CLOSED);  Serial.println(";");
    Serial.println("// =============================================");
    Serial.println();
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

// Двухсимвольная команда 's<X>' (save-as): при получении 's' ждём следующий символ.
bool savePending = false;

void loop() {
    if (!Serial.available()) return;
    char ch = Serial.read();

    // Обработка save-as: 'so'/'sx'/'sf'/'sp'
    if (savePending) {
        savePending = false;
        switch (ch) {
            case 'o': saveAsOpen();    return;
            case 'x': saveAsClosed();  return;
            case 'f': saveAsForward(); return;
            case 'p': saveAsPark();    return;
            case '\r': case '\n': case ' ':
                savePending = true;   // игнор whitespace, продолжаем ждать
                return;
            default:
                Serial.print("? save-as: ожидал o/x/f/p, получил '");
                Serial.print(ch); Serial.println("'");
                return;
        }
    }

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

        // swap claw полярности
        case '~': swapClawPolarity(); break;

        // сохранение current -> preset (2-символьная команда)
        case 's':
            savePending = true;
            Serial.println("save-as: нажми o (open) / x (closed) / f (forward) / p (park)");
            break;

        // экспорт
        case 'e': exportPresets(); break;

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
