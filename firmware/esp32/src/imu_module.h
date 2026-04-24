#pragma once
// =============================================================================
// IMU Module — MPU6050 reading + EMA filter + calibration
// =============================================================================
// Ported from pi_nodes/nodes/imu_node.py
// Publishes: samurai/{robot_id}/imu @ 50 Hz

#include <Wire.h>
#include <ArduinoJson.h>
#include "config.h"
#include "ekf_yaw.h"

// MPU6050 registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_CONFIG  0x1C
#define REG_GYRO_CONFIG   0x1B
#define REG_DLPF_CFG      0x1A
#define REG_SMPLRT_DIV    0x19

class ImuModule {
public:
    // Filtered + bias-corrected values (read by motor_module)
    float ax, ay, az;           // m/s^2
    float gx, gy, gz;           // rad/s (bias-corrected after calibration)
    bool  calibrated;
    float gravity_body[3];      // gravity vector in body frame at rest
    EkfYaw ekf;

    ImuModule() : ax(0), ay(0), az(9.81f), gx(0), gy(0), gz(0),
                  calibrated(false), _cal_idx(0),
                  _ema_ax(0), _ema_ay(0), _ema_az(0),
                  _ema_gx(0), _ema_gy(0), _ema_gz(0),
                  _ema_init(false),
                  _gyro_off_x(0), _gyro_off_y(0), _gyro_off_z(0),
                  _home_roll(0), _home_pitch(0) {
        gravity_body[0] = 0;
        gravity_body[1] = 0;
        gravity_body[2] = 9.81f;
    }

    bool begin() {
        // Wake up MPU6050
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(REG_PWR_MGMT_1);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) {
            Serial.println("[IMU] MPU6050 not found at 0x68");
            return false;
        }
        delay(100);

        // DLPF = 3 (bandwidth ~44Hz accel, ~42Hz gyro)
        _writeReg(REG_DLPF_CFG, 0x03);
        // Sample rate divider = 9 → 1000/(1+9) = 100 Hz internal
        _writeReg(REG_SMPLRT_DIV, 9);
        // Accel: ±2g
        _writeReg(REG_ACCEL_CONFIG, 0x00);
        // Gyro: ±250 deg/s
        _writeReg(REG_GYRO_CONFIG, 0x00);

        Serial.printf("[IMU] MPU6050 init OK @ 0x%02X (DLPF=3, SR=100Hz)\n",
                      MPU6050_ADDR);
        Serial.printf("[IMU] Calibrating (%d samples, ~%.1f sec)...\n",
                      IMU_CAL_SAMPLES, IMU_CAL_SAMPLES * 0.02f);
        return true;
    }

    // Call at 50 Hz. Returns true if new data ready to publish.
    bool update(float dt) {
        float raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        if (!_readRaw(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz))
            return false;

        // EMA low-pass filter
        _applyEMA(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);
        float fax = _ema_ax, fay = _ema_ay, faz = _ema_az;
        float fgx = _ema_gx, fgy = _ema_gy, fgz = _ema_gz;

        // Calibration phase
        if (!calibrated) {
            _cal_sum_ax += fax; _cal_sum_ay += fay; _cal_sum_az += faz;
            _cal_sum_gx += fgx; _cal_sum_gy += fgy; _cal_sum_gz += fgz;
            _cal_idx++;
            if (_cal_idx >= IMU_CAL_SAMPLES) {
                _finishCalibration();
            }
            // Publish raw during calibration
            ax = fax; ay = fay; az = faz;
            gx = fgx; gy = fgy; gz = fgz;
            return true;
        }

        // Apply gyro offset
        fgx -= _gyro_off_x;
        fgy -= _gyro_off_y;
        fgz -= _gyro_off_z;

        ax = fax; ay = fay; az = faz;
        gx = fgx; gy = fgy; gz = fgz;

        // EKF predict + update
        ekf.predict(fgx, fgy, fgz, dt);
        ekf.update(fax, fay, faz);

        return true;
    }

    // Build JSON payload (same format as Python imu_node)
    void buildPayload(JsonDocument &doc, float ts) {
        doc["ax"] = _round4(ax);
        doc["ay"] = _round4(ay);
        doc["az"] = _round4(az);
        doc["gx"] = _round5(gx);
        doc["gy"] = _round5(gy);
        doc["gz"] = _round5(gz);
        doc["calibrated"] = calibrated;
        doc["ts"] = ts;

        if (calibrated) {
            JsonObject e = doc["ekf"].to<JsonObject>();
            float rd, pd, yd;
            ekf.getEulerDeg(rd, pd, yd);
            e["roll"]      = _round2(rd);
            e["pitch"]     = _round2(pd);
            e["yaw"]       = _round2(yd);
            e["roll_rad"]  = _round5(ekf.roll);
            e["pitch_rad"] = _round5(ekf.pitch);
            e["yaw_rad"]   = _round5(ekf.yaw);
            e["bias_gx"]   = 0;
            e["bias_gy"]   = 0;
            e["bias_gz"]   = _round5(ekf.gz_bias);

            JsonArray gb = doc["gravity_body"].to<JsonArray>();
            gb.add(_round4(gravity_body[0]));
            gb.add(_round4(gravity_body[1]));
            gb.add(_round4(gravity_body[2]));
        }
    }

private:
    int   _cal_idx;
    float _cal_sum_ax = 0, _cal_sum_ay = 0, _cal_sum_az = 0;
    float _cal_sum_gx = 0, _cal_sum_gy = 0, _cal_sum_gz = 0;
    float _gyro_off_x, _gyro_off_y, _gyro_off_z;
    float _home_roll, _home_pitch;

    // EMA state
    bool  _ema_init;
    float _ema_ax, _ema_ay, _ema_az;
    float _ema_gx, _ema_gy, _ema_gz;

    void _writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    bool _readRaw(float &oax, float &oay, float &oaz,
                  float &ogx, float &ogy, float &ogz) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        if (Wire.endTransmission(false) != 0) return false;

        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
        if (Wire.available() < 14) return false;

        int16_t raw[7];
        for (int i = 0; i < 7; i++) {
            uint8_t hi = Wire.read();
            uint8_t lo = Wire.read();
            raw[i] = (int16_t)((hi << 8) | lo);
        }
        // raw[0..2] = accel XYZ, raw[3] = temp, raw[4..6] = gyro XYZ
        oax = raw[0] / IMU_ACCEL_SCALE * 9.81f;
        oay = raw[1] / IMU_ACCEL_SCALE * 9.81f;
        oaz = raw[2] / IMU_ACCEL_SCALE * 9.81f;
        ogx = raw[4] / IMU_GYRO_SCALE * (M_PI / 180.0f);
        ogy = raw[5] / IMU_GYRO_SCALE * (M_PI / 180.0f);
        ogz = raw[6] / IMU_GYRO_SCALE * (M_PI / 180.0f);
        return true;
    }

    void _applyEMA(float a_x, float a_y, float a_z,
                   float g_x, float g_y, float g_z) {
        if (!_ema_init) {
            _ema_ax = a_x; _ema_ay = a_y; _ema_az = a_z;
            _ema_gx = g_x; _ema_gy = g_y; _ema_gz = g_z;
            _ema_init = true;
            return;
        }
        const float aa = IMU_EMA_ACCEL, a1 = 1.0f - aa;
        const float ga = IMU_EMA_GYRO,  g1 = 1.0f - ga;
        _ema_ax = aa * a_x + a1 * _ema_ax;
        _ema_ay = aa * a_y + a1 * _ema_ay;
        _ema_az = aa * a_z + a1 * _ema_az;
        _ema_gx = ga * g_x + g1 * _ema_gx;
        _ema_gy = ga * g_y + g1 * _ema_gy;
        _ema_gz = ga * g_z + g1 * _ema_gz;
    }

    void _finishCalibration() {
        float n = (float)_cal_idx;
        _gyro_off_x = _cal_sum_gx / n;
        _gyro_off_y = _cal_sum_gy / n;
        _gyro_off_z = _cal_sum_gz / n;

        float avg_ax = _cal_sum_ax / n;
        float avg_ay = _cal_sum_ay / n;
        float avg_az = _cal_sum_az / n;
        gravity_body[0] = avg_ax;
        gravity_body[1] = avg_ay;
        gravity_body[2] = avg_az;

        _home_roll  = atan2f(avg_ay, avg_az);
        _home_pitch = atan2f(-avg_ax, sqrtf(avg_ay*avg_ay + avg_az*avg_az));

        ekf.initFromCalibration(_home_roll, _home_pitch, 0, 0);
        calibrated = true;

        float g_mag = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);
        Serial.printf("[IMU] Calibration done: gyro_off=(%.5f,%.5f,%.5f) "
                      "gravity=(%.3f,%.3f,%.3f) |g|=%.3f "
                      "home roll=%.1f pitch=%.1f deg\n",
                      _gyro_off_x, _gyro_off_y, _gyro_off_z,
                      avg_ax, avg_ay, avg_az, g_mag,
                      _home_roll * 180.0f / M_PI,
                      _home_pitch * 180.0f / M_PI);
    }

    static float _round2(float v) { return roundf(v * 100.0f) / 100.0f; }
    static float _round4(float v) { return roundf(v * 10000.0f) / 10000.0f; }
    static float _round5(float v) { return roundf(v * 100000.0f) / 100000.0f; }
};
