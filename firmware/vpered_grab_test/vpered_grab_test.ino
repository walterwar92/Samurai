/*
 * Vpered — grab/arm calibration sketch (Arduino Uno, 2KB SRAM!)
 *
 * All strings wrapped in F() to keep SRAM free.
 *
 * Servos: BASE D9, ARM D10, CLAW D11.
 * Robot does NOT drive in this sketch.
 *
 * Serial (9600, one-char commands unless noted):
 *   b/B a/A c/C  : BASE/ARM/CLAW -step/+step
 *   1 5 0        : step = 1 / 5 / 10 deg (default 2)
 *   o x          : open/close claw via presets
 *   ~            : swap CLAW_OPEN <-> CLAW_CLOSED (if 'o'/'x' are inverted)
 *   f            : arm forward
 *   p            : park
 *   g            : full grab sequence (open -> forward -> close -> up)
 *   r            : reset to park
 *   ? h          : status / help
 *   s<X>         : save current angles as preset
 *                  so = CLAW_OPEN, sx = CLAW_CLOSED,
 *                  sf = FORWARD,   sp = PARK
 *   e            : export all presets as copy-paste C++ block
 */

#include <Servo.h>

#define CLAW_PIN 11
#define ARM_PIN  10
#define BASE_PIN 9

// ===== PRESETS (edit me) =====
int BASE_PARK    = 90;
int ARM_PARK     = 90;
int CLAW_PARK    = 135;

int BASE_FORWARD = 90;
int ARM_FORWARD  = 40;

// Less = open on this robot. Press '~' if inverted.
int CLAW_OPEN    = 70;
int CLAW_CLOSED  = 160;

const int SERVO_SETTLE_MS = 400;
const int GRAB_HOLD_MS    = 300;

// ===== STATE =====
Servo clawServo, armServo, baseServo;

int baseAngle = 90;
int armAngle  = 90;
int clawAngle = 135;
int stepDeg   = 2;
bool savePending = false;

// ===== SMOOTH SERVO =====
void smoothWriteServo(Servo &s, int &cur, int tgt) {
    tgt = constrain(tgt, 0, 180);
    int step = (tgt > cur) ? 1 : -1;
    while (cur != tgt) {
        cur += step;
        s.write(cur);
        delay(8);
    }
}

void moveBase(int a) { smoothWriteServo(baseServo, baseAngle, a); }
void moveArm(int a)  { smoothWriteServo(armServo,  armAngle,  a); }
void moveClaw(int a) { smoothWriteServo(clawServo, clawAngle, a); }

// ===== ACTIONS =====
void goPark() {
    Serial.println(F("-> PARK"));
    moveClaw(CLAW_PARK);
    moveArm(ARM_PARK);
    moveBase(BASE_PARK);
}

void armForward() {
    Serial.println(F("-> FORWARD"));
    moveBase(BASE_FORWARD);
    moveArm(ARM_FORWARD);
}

void openClaw() {
    Serial.println(F("-> OPEN"));
    moveClaw(CLAW_OPEN);
}

void closeClaw() {
    Serial.println(F("-> CLOSE"));
    moveClaw(CLAW_CLOSED);
}

void printStatus();   // fwd decl

void grabSequence() {
    Serial.println(F("=== GRAB ==="));
    openClaw();      delay(SERVO_SETTLE_MS);
    armForward();    delay(SERVO_SETTLE_MS);
    delay(GRAB_HOLD_MS);
    closeClaw();     delay(SERVO_SETTLE_MS);
    moveArm(ARM_PARK); delay(SERVO_SETTLE_MS);
    Serial.println(F("=== DONE ==="));
    printStatus();
}

void printStatus() {
    Serial.println();
    Serial.print(F("B="));    Serial.print(baseAngle);
    Serial.print(F(" A="));   Serial.print(armAngle);
    Serial.print(F(" C="));   Serial.print(clawAngle);
    Serial.print(F(" step=")); Serial.println(stepDeg);

    Serial.print(F("PARK:    B=")); Serial.print(BASE_PARK);
    Serial.print(F(" A="));         Serial.print(ARM_PARK);
    Serial.print(F(" C="));         Serial.println(CLAW_PARK);

    Serial.print(F("FORWARD: B=")); Serial.print(BASE_FORWARD);
    Serial.print(F(" A="));         Serial.println(ARM_FORWARD);

    Serial.print(F("CLAW:    open=")); Serial.print(CLAW_OPEN);
    Serial.print(F(" closed="));       Serial.println(CLAW_CLOSED);
}

void printHelp() {
    Serial.println(F("=== GRAB TEST ==="));
    Serial.println(F("b/B a/A c/C : -/+ step for BASE/ARM/CLAW"));
    Serial.println(F("1 5 0       : step 1 / 5 / 10 deg"));
    Serial.println(F("o x ~       : open / close / swap polarity"));
    Serial.println(F("f p         : forward / park"));
    Serial.println(F("g r         : grab sequence / reset"));
    Serial.println(F("so sx sf sp : save current as preset"));
    Serial.println(F("e           : export presets"));
    Serial.println(F("? h         : status / help"));
    printStatus();
}

// ===== SAVE PRESETS =====
void saveAsOpen() {
    CLAW_OPEN = clawAngle;
    Serial.print(F("SAVE CLAW_OPEN=")); Serial.println(CLAW_OPEN);
}
void saveAsClosed() {
    CLAW_CLOSED = clawAngle;
    Serial.print(F("SAVE CLAW_CLOSED=")); Serial.println(CLAW_CLOSED);
}
void saveAsForward() {
    BASE_FORWARD = baseAngle;
    ARM_FORWARD  = armAngle;
    Serial.print(F("SAVE FORWARD B=")); Serial.print(BASE_FORWARD);
    Serial.print(F(" A="));              Serial.println(ARM_FORWARD);
}
void saveAsPark() {
    BASE_PARK = baseAngle;
    ARM_PARK  = armAngle;
    CLAW_PARK = clawAngle;
    Serial.print(F("SAVE PARK B=")); Serial.print(BASE_PARK);
    Serial.print(F(" A="));           Serial.print(ARM_PARK);
    Serial.print(F(" C="));           Serial.println(CLAW_PARK);
}

void swapClawPolarity() {
    int t = CLAW_OPEN;
    CLAW_OPEN = CLAW_CLOSED;
    CLAW_CLOSED = t;
    Serial.print(F("SWAP open="));  Serial.print(CLAW_OPEN);
    Serial.print(F(" closed="));    Serial.println(CLAW_CLOSED);
}

void exportPresets() {
    Serial.println(F("// ---- paste into vpered_uno.ino ----"));
    Serial.print(F("const int BASE_PARK    = ")); Serial.print(BASE_PARK);    Serial.println(';');
    Serial.print(F("const int ARM_PARK     = ")); Serial.print(ARM_PARK);     Serial.println(';');
    Serial.print(F("const int CLAW_PARK    = ")); Serial.print(CLAW_PARK);    Serial.println(';');
    Serial.print(F("const int BASE_FORWARD = ")); Serial.print(BASE_FORWARD); Serial.println(';');
    Serial.print(F("const int ARM_FORWARD  = ")); Serial.print(ARM_FORWARD);  Serial.println(';');
    Serial.print(F("const int CLAW_OPEN    = ")); Serial.print(CLAW_OPEN);    Serial.println(';');
    Serial.print(F("const int CLAW_CLOSED  = ")); Serial.print(CLAW_CLOSED);  Serial.println(';');
    Serial.println(F("// -----------------------------------"));
}

// ===== SETUP =====
void setup() {
    Serial.begin(9600);
    delay(200);

    clawServo.attach(CLAW_PIN);
    armServo.attach(ARM_PIN);
    baseServo.attach(BASE_PIN);

    baseServo.write(BASE_PARK); baseAngle = BASE_PARK;
    armServo.write(ARM_PARK);   armAngle  = ARM_PARK;
    clawServo.write(CLAW_PARK); clawAngle = CLAW_PARK;
    delay(600);

    printHelp();
}

// ===== LOOP =====
void loop() {
    if (!Serial.available()) return;
    char ch = Serial.read();

    // two-char command: s<X>
    if (savePending) {
        savePending = false;
        switch (ch) {
            case 'o': saveAsOpen();    return;
            case 'x': saveAsClosed();  return;
            case 'f': saveAsForward(); return;
            case 'p': saveAsPark();    return;
            case '\r': case '\n': case ' ':
                savePending = true;
                return;
            default:
                Serial.print(F("? save: need o/x/f/p, got '"));
                Serial.print(ch); Serial.println('\'');
                return;
        }
    }

    switch (ch) {
        case 'b': moveBase(baseAngle - stepDeg); printStatus(); break;
        case 'B': moveBase(baseAngle + stepDeg); printStatus(); break;
        case 'a': moveArm(armAngle  - stepDeg); printStatus(); break;
        case 'A': moveArm(armAngle  + stepDeg); printStatus(); break;
        case 'c': moveClaw(clawAngle - stepDeg); printStatus(); break;
        case 'C': moveClaw(clawAngle + stepDeg); printStatus(); break;

        case '1': stepDeg = 1;  Serial.println(F("step=1"));  break;
        case '5': stepDeg = 5;  Serial.println(F("step=5"));  break;
        case '0': stepDeg = 10; Serial.println(F("step=10")); break;

        case 'o': openClaw();   printStatus(); break;
        case 'x': closeClaw();  printStatus(); break;
        case 'f': armForward(); printStatus(); break;
        case 'p': goPark();     printStatus(); break;

        case '~': swapClawPolarity(); break;

        case 's':
            savePending = true;
            Serial.println(F("save: press o/x/f/p"));
            break;

        case 'e': exportPresets(); break;
        case 'g': grabSequence();  break;
        case 'r': goPark(); printStatus(); break;

        case '?': printStatus(); break;
        case 'h': case 'H': printHelp(); break;

        case '\r': case '\n': case ' ': break;

        default:
            Serial.print(F("? '"));
            Serial.print(ch);
            Serial.println(F("' (h=help)"));
            break;
    }
}
