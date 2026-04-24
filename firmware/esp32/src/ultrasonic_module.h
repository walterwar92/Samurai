#pragma once
// =============================================================================
// Ultrasonic Module — HC-SR04 distance sensor
// =============================================================================
// Ported from pi_nodes/nodes/ultrasonic_node.py
// Publishes: samurai/{robot_id}/range @ 20 Hz (with delta filter)

#include <Arduino.h>
#include "config.h"

class UltrasonicModule {
public:
    float range_m;       // latest reading (metres)
    bool  shouldPublish; // true when delta or time threshold exceeded

    UltrasonicModule()
        : range_m(US_MAX_RANGE), shouldPublish(false),
          _last_published(-1.0f), _last_pub_ms(0),
          _last_valid(US_MAX_RANGE), _fail_count(0) {}

    void begin() {
        pinMode(PIN_US_TRIG, OUTPUT);
        pinMode(PIN_US_ECHO, INPUT);
        digitalWrite(PIN_US_TRIG, LOW);
        Serial.printf("[US] HC-SR04 ready (trig=%d echo=%d)\n",
                      PIN_US_TRIG, PIN_US_ECHO);
    }

    // Call at 20 Hz
    void update() {
        shouldPublish = false;

        // Trigger pulse
        digitalWrite(PIN_US_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_US_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_US_TRIG, LOW);

        // Measure echo (with timeout)
        unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, US_TIMEOUT_US);

        if (dur == 0) {
            _fail_count++;
            range_m = _last_valid;  // use last valid
            if (_fail_count <= 3 || _fail_count % 50 == 0) {
                Serial.printf("[US] read error (%d)\n", _fail_count);
            }
        } else {
            // Speed of sound ≈ 343 m/s → dist = dur * 0.000343 / 2
            float dist = dur * 0.0001715f;
            if (dist < US_MIN_RANGE) dist = US_MIN_RANGE;
            if (dist > US_MAX_RANGE) dist = US_MAX_RANGE;
            range_m = dist;
            _last_valid = dist;
            _fail_count = 0;
        }

        // Publish only on significant change or timeout
        unsigned long now = millis();
        if (fabsf(range_m - _last_published) >= US_PUBLISH_DELTA ||
            (now - _last_pub_ms) >= (unsigned long)(US_FORCE_INTERVAL * 1000)) {
            shouldPublish = true;
            _last_published = range_m;
            _last_pub_ms = now;
        }
    }

private:
    float _last_published;
    unsigned long _last_pub_ms;
    float _last_valid;
    int   _fail_count;
};
