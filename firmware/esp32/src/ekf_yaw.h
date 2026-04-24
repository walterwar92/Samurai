#pragma once
// =============================================================================
// 1D Yaw Kalman Filter — ported from pi_nodes/filters/ekf_imu.py
// =============================================================================
// Ground robot: yaw from gyro Z integration + ZUPT bias correction.
// Roll/pitch from accelerometer (informational, no filtering).

#include <math.h>
#include "config.h"

class EkfYaw {
public:
    float q_angle;
    float q_bias;
    float r_accel;
    float accel_gate;
    float g;

    // Public state — read by other modules
    float roll;       // radians, relative to home
    float pitch;      // radians, relative to home
    float yaw;        // radians, relative to startup
    float gz_bias;    // estimated gyro Z bias (rad/s)
    bool  stationary; // ZUPT detected

    EkfYaw()
        : q_angle(EKF_Q_ANGLE), q_bias(EKF_Q_BIAS),
          r_accel(EKF_R_ACCEL), accel_gate(EKF_ACCEL_GATE), g(9.81f),
          roll(0), pitch(0), yaw(0), gz_bias(0), stationary(false),
          _P_yaw(0.1f), _P_bias(0.01f), _P_cross(0),
          _home_roll(0), _home_pitch(0),
          _zupt_enter(0.015f), _zupt_exit(0.04f),
          _zupt_accel_enter(0.12f), _zupt_accel_exit(0.30f),
          _zupt_confirm(5), _zupt_count(0),
          _r_zupt(0.0005f) {}

    void reset() {
        yaw = 0; gz_bias = 0;
        _P_yaw = 0.1f; _P_bias = 0.01f; _P_cross = 0;
        roll = 0; pitch = 0;
        stationary = false; _zupt_count = 0;
    }

    void initFromCalibration(float home_roll, float home_pitch,
                             float init_yaw, float init_gz_bias) {
        _home_roll = home_roll;
        _home_pitch = home_pitch;
        yaw = init_yaw;
        gz_bias = 0;  // imu_node already subtracts static offset
        _P_yaw = 0.001f;
        _P_bias = 0.0001f;
        _P_cross = 0;
        stationary = false;
        _zupt_count = 0;
    }

    // Predict: integrate gyro Z for yaw (gx/gy ignored — ground robot)
    void predict(float gx, float gy, float gz, float dt) {
        if (dt <= 0) return;

        float wz = gz - gz_bias;

        // ── ZUPT hysteresis ──
        float abs_wz = fabsf(wz);
        if (stationary) {
            if (abs_wz > _zupt_exit) {
                stationary = false;
                _zupt_count = 0;
            }
        } else {
            if (abs_wz < _zupt_enter) {
                _zupt_count++;
                if (_zupt_count >= _zupt_confirm) stationary = true;
            } else {
                _zupt_count = 0;
            }
        }

        if (stationary) {
            // ZUPT measurement update: gz should be 0
            float S = _P_bias + _r_zupt;
            if (S > 1e-12f) {
                float K_yaw  = _P_cross / S;
                float K_bias = _P_bias / S;
                yaw     += K_yaw  * wz;
                gz_bias += K_bias * wz;

                float p11 = _P_yaw, p12 = _P_cross, p22 = _P_bias;
                _P_yaw   = p11 - K_yaw * p12;
                _P_cross = p12 - K_yaw * p22;
                _P_bias  = p22 - K_bias * p22;
                if (_P_yaw  < 1e-10f) _P_yaw  = 1e-10f;
                if (_P_bias < 1e-10f) _P_bias = 1e-10f;
            }
            _P_yaw  += q_angle * dt * 0.01f;
            _P_bias += q_bias * dt;
            yaw = _normalize(yaw);
            return;
        }

        // Normal predict: integrate gyro
        yaw += wz * dt;
        yaw = _normalize(yaw);

        // Covariance: F=[[1,-dt],[0,1]], Q=diag(q_angle*dt, q_bias*dt)
        float p11 = _P_yaw, p12 = _P_cross, p22 = _P_bias;
        _P_yaw   = p11 + (-dt)*p12 + (-dt)*(p12 + (-dt)*p22) + q_angle*dt;
        _P_cross = p12 + (-dt)*p22;
        _P_bias  = p22 + q_bias*dt;
    }

    // Update: roll/pitch from accelerometer + ZUPT accel confirmation
    void update(float ax, float ay, float az) {
        float a_mag = sqrtf(ax*ax + ay*ay + az*az);
        float a_dev = fabsf(a_mag - g);

        // Accel ZUPT reinforcement
        if (stationary && a_dev > _zupt_accel_exit) {
            stationary = false;
            _zupt_count = 0;
        } else if (!stationary && a_dev < _zupt_accel_enter) {
            if (_zupt_count < _zupt_confirm) _zupt_count++;
        }

        if (a_dev > accel_gate * g) return;

        float raw_roll  = atan2f(ay, az);
        float raw_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        roll  = _normalize(raw_roll  - _home_roll);
        pitch = _normalize(raw_pitch - _home_pitch);
    }

    void getEulerDeg(float &r, float &p, float &y) const {
        r = roll  * 180.0f / M_PI;
        p = pitch * 180.0f / M_PI;
        y = yaw   * 180.0f / M_PI;
    }

private:
    float _P_yaw, _P_bias, _P_cross;
    float _home_roll, _home_pitch;
    float _zupt_enter, _zupt_exit;
    float _zupt_accel_enter, _zupt_accel_exit;
    int   _zupt_confirm, _zupt_count;
    float _r_zupt;

    static float _normalize(float a) {
        while (a >  M_PI) a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }
};
