#pragma once
// =============================================================================
// Servo Module — Head pan + 4-joint Arm via PCA9685
// =============================================================================
// Ported from pi_nodes/nodes/head_node.py + arm_node.py
// Uses shared PCA9685 instance from MotorModule.
//
// Publishes:  head/state @ 10 Hz, arm/state @ 10 Hz
// Subscribes: head/command, arm/command

#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include "config.h"

class ServoModule {
public:
    // Head state
    float head_angle;
    bool  head_locked;
    bool  head_frozen;

    // Arm state
    float arm_angles[NUM_ARM_JOINTS];
    bool  arm_locked;
    bool  arm_frozen[NUM_ARM_JOINTS];

    ServoModule()
        : head_angle(HEAD_HOME), head_locked(HEAD_LOCKED), head_frozen(false),
          _head_target(HEAD_HOME), _head_init(false),
          arm_locked(ARM_LOCKED),
          _pca(nullptr) {
        for (int i = 0; i < NUM_ARM_JOINTS; i++) {
            arm_angles[i] = ARM_HOME[i];
            arm_frozen[i] = false;
            _arm_targets[i] = ARM_HOME[i];
        }
    }

    // Must call after MotorModule::begin() — shares PCA9685
    void begin(Adafruit_PWMServoDriver *pca) {
        _pca = pca;
        // Don't move servos on boot (locked=true by default)
        Serial.printf("[SERVO] Head ch%d (home=%d, locked=%s), "
                      "Arm ch[%d,%d,%d,%d] (locked=%s)\n",
                      SERVO_HEAD_CH, HEAD_HOME,
                      head_locked ? "true" : "false",
                      SERVO_ARM_CH0, SERVO_ARM_CH1,
                      SERVO_ARM_CH2, SERVO_ARM_CH3,
                      arm_locked ? "true" : "false");
    }

    // ── Head commands ────────────────────────────────────────────

    void headCommand(JsonDocument &doc) {
        // String command
        if (doc.is<const char*>()) {
            _headStringCmd(doc.as<const char*>());
            return;
        }
        if (!doc.is<JsonObject>()) return;

        const char* cmd = doc["command"] | "";
        if (strlen(cmd) > 0) {
            _headStringCmd(cmd);
            // angle in same message
        }
        if (doc["angle"].is<float>()) {
            _headUnlock();
            float a = doc["angle"].as<float>();
            _head_target = _clamp(a, (float)HEAD_MIN, (float)HEAD_MAX);
        }
    }

    // Call at 50 Hz for smooth movement
    void headUpdate() {
        if (head_locked || head_frozen) return;
        float diff = _head_target - head_angle;
        if (fabsf(diff) < 0.5f) {
            head_angle = _head_target;
        } else {
            float step = fminf(fabsf(diff), HEAD_SPEED);
            head_angle += (diff > 0) ? step : -step;
        }
        _setServo(SERVO_HEAD_CH, head_angle);
    }

    void buildHeadState(JsonDocument &doc) {
        doc["angle"]  = _round1(head_angle);
        doc["frozen"] = head_frozen;
        doc["locked"] = head_locked;
    }

    // ── Arm commands ─────────────────────────────────────────────

    void armCommand(JsonDocument &doc) {
        if (doc.is<const char*>()) {
            _armStringCmd(doc.as<const char*>());
            return;
        }
        if (!doc.is<JsonObject>()) return;

        const char* cmd = doc["command"] | "";
        if (strcmp(cmd, "home") == 0) {
            _armUnlock();
            for (int i = 0; i < NUM_ARM_JOINTS; i++)
                _setArmJoint(i, ARM_HOME[i]);
            return;
        }
        if (strcmp(cmd, "unlock") == 0) { _armUnlock(); return; }
        if (strcmp(cmd, "freeze") == 0) {
            _armUnlock();
            if (doc["joint"].is<int>()) {
                int j = doc["joint"].as<int>() - 1;
                if (j >= 0 && j < NUM_ARM_JOINTS) arm_frozen[j] = true;
            } else {
                for (int i = 0; i < NUM_ARM_JOINTS; i++) arm_frozen[i] = true;
            }
            return;
        }
        if (strcmp(cmd, "unfreeze") == 0) {
            if (doc["joint"].is<int>()) {
                int j = doc["joint"].as<int>() - 1;
                if (j >= 0 && j < NUM_ARM_JOINTS) arm_frozen[j] = false;
            } else {
                for (int i = 0; i < NUM_ARM_JOINTS; i++) arm_frozen[i] = false;
            }
            return;
        }

        // Single joint: {"joint": 1, "angle": 90}
        if (doc["joint"].is<int>() && doc["angle"].is<float>()) {
            _armUnlock();
            int idx = doc["joint"].as<int>() - 1;
            _setArmJoint(idx, doc["angle"].as<float>());
            return;
        }

        // All joints: {"joints": [90, 90, 90, 90]}
        JsonArray arr = doc["joints"];
        if (!arr.isNull()) {
            _armUnlock();
            for (int i = 0; i < NUM_ARM_JOINTS && i < (int)arr.size(); i++)
                _setArmJoint(i, arr[i].as<float>());
        }
    }

    void buildArmState(JsonDocument &doc) {
        for (int i = 0; i < NUM_ARM_JOINTS; i++) {
            char key[4];
            snprintf(key, sizeof(key), "j%d", i + 1);
            doc[key] = _round1(arm_angles[i]);
        }
        JsonArray f = doc["frozen"].to<JsonArray>();
        for (int i = 0; i < NUM_ARM_JOINTS; i++) f.add(arm_frozen[i]);
        doc["locked"] = arm_locked;
    }

private:
    float _head_target;
    bool  _head_init;
    float _arm_targets[NUM_ARM_JOINTS];
    Adafruit_PWMServoDriver *_pca;

    static const uint8_t _arm_channels[NUM_ARM_JOINTS];

    void _headStringCmd(const char* cmd) {
        if (strcasecmp(cmd, "center") == 0 || strcasecmp(cmd, "home") == 0) {
            _headUnlock(); _head_target = HEAD_HOME;
        } else if (strcasecmp(cmd, "unlock") == 0) {
            _headUnlock();
        } else if (strcasecmp(cmd, "lock") == 0) {
            head_locked = true;
        } else if (strcasecmp(cmd, "freeze") == 0) {
            _headUnlock(); head_frozen = true;
        } else if (strcasecmp(cmd, "unfreeze") == 0) {
            head_frozen = false;
        }
    }

    void _headUnlock() {
        head_locked = false;
        if (!_head_init) {
            _setServo(SERVO_HEAD_CH, head_angle);
            _head_init = true;
        }
    }

    void _armStringCmd(const char* cmd) {
        if (strcasecmp(cmd, "home") == 0) {
            _armUnlock();
            for (int i = 0; i < NUM_ARM_JOINTS; i++)
                _setArmJoint(i, ARM_HOME[i]);
        } else if (strcasecmp(cmd, "unlock") == 0) {
            _armUnlock();
        } else if (strcasecmp(cmd, "freeze") == 0) {
            _armUnlock();
            for (int i = 0; i < NUM_ARM_JOINTS; i++) arm_frozen[i] = true;
        } else if (strcasecmp(cmd, "unfreeze") == 0) {
            for (int i = 0; i < NUM_ARM_JOINTS; i++) arm_frozen[i] = false;
        }
    }

    void _armUnlock() {
        if (!arm_locked) return;
        arm_locked = false;
        for (int i = 0; i < NUM_ARM_JOINTS; i++) {
            _setServo(_arm_channels[i], arm_angles[i]);
        }
    }

    void _setArmJoint(int idx, float angle) {
        if (idx < 0 || idx >= NUM_ARM_JOINTS) return;
        angle = _clamp(angle, ARM_MIN[idx], ARM_MAX[idx]);
        if (!arm_frozen[idx]) {
            arm_angles[idx] = angle;
            _setServo(_arm_channels[idx], angle);
        }
    }

    // Convert angle (0-180) to PCA9685 PWM pulse
    void _setServo(uint8_t channel, float angle) {
        if (!_pca) return;
        angle = _clamp(angle, 0, 180);
        // Map 0-180 degrees to SERVO_PULSE_MIN-SERVO_PULSE_MAX microseconds
        float pulse_us = SERVO_PULSE_MIN +
                         (angle / 180.0f) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);
        // PCA9685 @ 50Hz: 1 tick = 1/50/4096 = 4.883 us
        uint16_t tick = (uint16_t)(pulse_us / 4.883f);
        _pca->setPWM(channel, 0, tick);
    }

    static float _clamp(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }
    static float _round1(float v) { return roundf(v * 10.0f) / 10.0f; }
};

// Arm channel map
const uint8_t ServoModule::_arm_channels[NUM_ARM_JOINTS] = {
    SERVO_ARM_CH0, SERVO_ARM_CH1, SERVO_ARM_CH2, SERVO_ARM_CH3
};
