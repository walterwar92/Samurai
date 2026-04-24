#pragma once
// =============================================================================
// Battery Module — ADS7830 ADC voltage monitoring
// =============================================================================
// Ported from pi_nodes/nodes/battery_node.py
// Publishes: samurai/{robot_id}/battery @ 1 Hz (retained)

#include <Wire.h>
#include "config.h"

// ADS7830 channel commands (single-ended)
static const uint8_t ADS_CH_CMD[] = {
    0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4
};

class BatteryModule {
public:
    float voltage;
    float percent;

    BatteryModule()
        : voltage(0), percent(0), _channel(0), _ok(false),
          _i2c_errors(0), _recover_ms(0) {}

    bool begin() {
        _ok = _probe();
        if (_ok) {
            Serial.printf("[BAT] ADS7830 found @ 0x%02X, ch %d\n",
                          ADS7830_ADDR, _channel);
        } else {
            Serial.println("[BAT] ADS7830 not found — simulated");
        }
        return _ok;
    }

    // Call at 1 Hz
    void update() {
        if (!_ok) {
            // Attempt recovery
            if (millis() >= _recover_ms) {
                _ok = _probe();
                if (_ok) {
                    Serial.println("[BAT] ADS7830 recovered");
                } else {
                    _recover_ms = millis() + 5000;
                }
            }
            if (!_ok) {
                voltage = 0;
                percent = 0;
                return;
            }
        }

        Wire.beginTransmission(ADS7830_ADDR);
        Wire.write(ADS_CH_CMD[_channel]);
        if (Wire.endTransmission() != 0) {
            _handleError();
            return;
        }

        Wire.requestFrom((uint8_t)ADS7830_ADDR, (uint8_t)1);
        if (Wire.available() < 1) {
            _handleError();
            return;
        }

        uint8_t raw = Wire.read();
        _i2c_errors = 0;

        float adc_v = raw / 255.0f * 3.3f;
        voltage = adc_v * VDIV_RATIO;

        percent = (voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0f;
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;

        if (percent < 10.0f) {
            Serial.printf("[BAT] CRITICAL: %.1fV (%.0f%%)\n", voltage, percent);
        } else if (percent < 20.0f) {
            Serial.printf("[BAT] Low: %.1fV (%.0f%%)\n", voltage, percent);
        }
    }

private:
    uint8_t _channel;
    bool    _ok;
    int     _i2c_errors;
    unsigned long _recover_ms;

    bool _probe() {
        Wire.beginTransmission(ADS7830_ADDR);
        Wire.write(ADS_CH_CMD[0]);
        if (Wire.endTransmission() != 0) return false;
        Wire.requestFrom((uint8_t)ADS7830_ADDR, (uint8_t)1);
        return Wire.available() >= 1;
    }

    void _handleError() {
        _i2c_errors++;
        if (_i2c_errors >= 3) {
            _ok = false;
            _recover_ms = millis() + 5000;
            Serial.printf("[BAT] %d I2C errors — recovery in 5s\n",
                          _i2c_errors);
        }
    }
};
