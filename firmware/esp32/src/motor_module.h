#pragma once
// =============================================================================
// Motor Module — PCA9685 DC motors + differential mixer + dead-reckoning odom
// =============================================================================
// Ported from pi_nodes/nodes/motor_node.py + pi_nodes/hardware/motor_driver.py
//
// Publishes:  samurai/{robot_id}/odom @ 20 Hz
// Subscribes: cmd_vel, cmd_vel/manual, speed_profile

#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include "config.h"

// Speed profile enum
enum SpeedProfile { PROF_SLOW, PROF_NORMAL, PROF_FAST };

class MotorModule {
public:
    // Odometry state (read by other modules)
    float odom_x;        // metres
    float odom_y;        // metres
    float odom_theta;    // radians
    float odom_vx;       // m/s (published forward velocity)
    float odom_vz;       // rad/s (published angular velocity)
    float odom_speed;    // scalar velocity magnitude

    // Active source for UI
    const char* active_source;

    MotorModule()
        : odom_x(0), odom_y(0), odom_theta(0),
          odom_vx(0), odom_vz(0), odom_speed(0),
          active_source("none"),
          _pca(PCA9685_ADDR),
          _profile(PROF_NORMAL),
          _max_lin(PROFILE_NORMAL_LIN), _max_ang(PROFILE_NORMAL_ANG),
          _scale_fwd(CAL_SCALE_FWD), _scale_bwd(CAL_SCALE_BWD),
          _motor_trim(CAL_MOTOR_TRIM),
          _linear(0), _angular(0), _last_cmd_time(0),
          _man_linear(0), _man_angular(0), _last_man_time(0),
          _range_m(999.0f), _collision_guard(false),
          _collision_avoid_dir(1.0f), _collision_avoid_active(false),
          // IMU push detector state
          _imu_yaw_rad(0), _imu_calibrated(false), _imu_gz(0),
          _imu_gravity_body{0,0,9.81f},
          _imu_raw_lin_bx(0), _imu_raw_lin_by(0),
          _push_bias_x(0), _push_bias_y(0),
          _push_bias_ready(false), _push_bias_samples(0),
          _push_dev_count(0), _push_fast_remaining(0),
          _push_cont_ticks(0), _push_active(false),
          _push_dir_x(0), _push_dir_y(0),
          _prev_source("") {}

    bool begin() {
        _pca.begin();
        _pca.setPWMFreq(50);  // 50 Hz for motor/servo
        delay(10);
        // Stop motors
        _setMotor(M1_CH_IN1, M1_CH_IN2, 0);
        _setMotor(M2_CH_IN1, M2_CH_IN2, 0);
        Serial.printf("[MOTOR] PCA9685 init @ 0x%02X, PWM=50Hz\n", PCA9685_ADDR);
        return true;
    }

    // Feed IMU data (called from main when IMU publishes)
    void feedIMU(float imu_ax, float imu_ay, float imu_gz_val,
                 float yaw_rad, bool cal, const float grav_body[3]) {
        _imu_gz = imu_gz_val;
        if (cal && !_imu_calibrated) {
            _imu_calibrated = true;
            _imu_gravity_body[0] = grav_body[0];
            _imu_gravity_body[1] = grav_body[1];
            _imu_gravity_body[2] = grav_body[2];
            Serial.println("[MOTOR] IMU calibrated -> odometry active");
        }
        if (_imu_calibrated) {
            _imu_yaw_rad = yaw_rad;
            // Raw gravity-free body accel for push detector
            _imu_raw_lin_bx = imu_ax - _imu_gravity_body[0];
            _imu_raw_lin_by = imu_ay - _imu_gravity_body[1];
        }
    }

    // cmd_vel from autonomous (FSM)
    void setCmdVel(float lin, float ang) {
        _linear = lin;
        _angular = ang;
        _last_cmd_time = millis();
    }

    // cmd_vel from manual (joystick/voice)
    void setManualVel(float lin, float ang) {
        _man_linear = lin;
        _man_angular = ang;
        _last_man_time = millis();
    }

    void setRange(float range_m) { _range_m = range_m; }

    void setCollisionGuard(bool on) {
        if (on != _collision_guard) {
            _collision_guard = on;
            Serial.printf("[MOTOR] Collision guard: %s\n", on ? "ON" : "OFF");
        }
    }

    void setProfile(const char* name) {
        if (strcmp(name, "slow") == 0) {
            _profile = PROF_SLOW;
            _max_lin = PROFILE_SLOW_LIN;
            _max_ang = PROFILE_SLOW_ANG;
        } else if (strcmp(name, "fast") == 0) {
            _profile = PROF_FAST;
            _max_lin = PROFILE_FAST_LIN;
            _max_ang = PROFILE_FAST_ANG;
        } else {
            _profile = PROF_NORMAL;
            _max_lin = PROFILE_NORMAL_LIN;
            _max_ang = PROFILE_NORMAL_ANG;
        }
        Serial.printf("[MOTOR] Profile: %s (lin=%.2f ang=%.2f)\n",
                      name, _max_lin, _max_ang);
    }

    const char* profileName() const {
        switch (_profile) {
            case PROF_SLOW: return "slow";
            case PROF_FAST: return "fast";
            default:        return "normal";
        }
    }

    void resetPosition() {
        odom_x = odom_y = odom_theta = 0;
        odom_vx = odom_vz = odom_speed = 0;
        _push_bias_x = _push_bias_y = 0;
        _push_bias_ready = false;
        _push_bias_samples = 0;
        _push_dev_count = 0;
        _push_fast_remaining = 0;
        _push_cont_ticks = 0;
        _push_active = false;
        Serial.println("[MOTOR] Position reset to (0,0,0)");
    }

    // Call at 20 Hz — main control loop
    void controlLoop(float dt) {
        unsigned long now = millis();

        // ── Priority mux ──
        bool manual_active = (_last_man_time > 0 &&
                              (now - _last_man_time) <= (unsigned long)(MANUAL_TIMEOUT_S * 1000));
        // Auto-stop on autonomous timeout
        if (_last_cmd_time > 0 && (now - _last_cmd_time) > (unsigned long)(CMD_VEL_TIMEOUT_S * 1000)) {
            _linear = 0; _angular = 0;
        }

        float lin_cmd, ang_cmd;
        if (manual_active) {
            lin_cmd = _man_linear;
            ang_cmd = _man_angular;
            active_source = "manual";
        } else {
            lin_cmd = _linear;
            ang_cmd = _angular;
            active_source = (fabsf(lin_cmd) > 0 || fabsf(ang_cmd) > 0) ? "autonomous" : "none";
        }

        // Dead zone
        if (fabsf(lin_cmd) < DEADZONE_LINEAR) lin_cmd = 0;
        if (fabsf(ang_cmd) < DEADZONE_ANGULAR) ang_cmd = 0;

        // ── Collision guard ──
        if (_collision_guard && lin_cmd > 0) {
            float r = _range_m;
            if (r < COLLISION_STOP_M) {
                lin_cmd = 0;
                if (!manual_active) {
                    ang_cmd = COLLISION_AVOID_ANG * _collision_avoid_dir;
                    active_source = "collision";
                    if (!_collision_avoid_active) {
                        _collision_avoid_active = true;
                    }
                }
            } else if (r < COLLISION_SLOW_M) {
                float factor = (r - COLLISION_STOP_M) / (COLLISION_SLOW_M - COLLISION_STOP_M);
                lin_cmd *= fmaxf(0, factor);
                if (!manual_active) active_source = "collision";
            } else {
                if (_collision_avoid_active) {
                    _collision_avoid_active = false;
                    _collision_avoid_dir *= -1.0f;
                }
            }
        }

        // Clamp to profile limits
        float lin = _clamp(lin_cmd, -_max_lin, _max_lin);
        float ang = _clamp(ang_cmd, -_max_ang, _max_ang);

        // Convert to percentage for PCA9685
        float lin_pct = lin / PHYS_MAX_LIN * 100.0f;
        float ang_pct = ang / PHYS_MAX_ANG * 100.0f;

        // Motor trim (compensate asymmetry)
        if (fabsf(lin_pct) > 1.0f) {
            float trim = (lin_pct >= 0) ? _motor_trim : -_motor_trim;
            ang_pct += trim;
        }

        // Differential mixer
        _driveMotors(lin_pct, ang_pct);

        // ── Heading ──
        if (_imu_calibrated) {
            odom_theta = _imu_yaw_rad;
        } else {
            odom_theta += ang_cmd * dt;
        }
        float cy = cosf(odom_theta);
        float sy = sinf(odom_theta);

        // ── Dead-reckoning position ──
        float v_actual = 0;
        if (fabsf(lin) > DEADZONE_LINEAR) {
            float scale = (lin >= 0) ? _scale_fwd : _scale_bwd;
            v_actual = lin * scale;
        }
        odom_x += v_actual * dt * cy;
        odom_y += v_actual * dt * sy;

        odom_vx = v_actual;
        odom_vz = _imu_calibrated ? _imu_gz : ang;
        odom_speed = fabsf(v_actual);
        bool motors_idle = (fabsf(v_actual) < 0.001f && fabsf(ang) < DEADZONE_ANGULAR);

        // ── IMU push detection (motors OFF) ──
        float push_vx = 0, push_vy = 0;
        if (motors_idle && _imu_calibrated) {
            _pushDetect(cy, sy, dt, push_vx, push_vy);
        } else {
            _push_dev_count = 0;
            _push_cont_ticks = 0;
            _push_active = false;
        }

        _is_stationary = motors_idle && push_vx == 0 && push_vy == 0;
    }

    // Build odom JSON (same format as Python motor_node)
    void buildOdom(JsonDocument &doc, float ts) {
        doc["x"]     = _round2(odom_x * 100.0f);  // cm
        doc["y"]     = _round2(odom_y * 100.0f);   // cm
        doc["theta"] = _round4(odom_theta);
        doc["vx"]    = _round3(odom_vx);
        doc["vz"]    = _round3(odom_vz);
        doc["speed"] = _round3(odom_speed);
        doc["accel_x"] = 0;
        doc["accel_y"] = 0;
        doc["stationary"] = _is_stationary;
        doc["ts"]    = ts;
    }

    void stop() {
        _setMotor(M1_CH_IN1, M1_CH_IN2, 0);
        _setMotor(M2_CH_IN1, M2_CH_IN2, 0);
    }

    // Access PCA9685 for servo use
    Adafruit_PWMServoDriver& pca() { return _pca; }

private:
    Adafruit_PWMServoDriver _pca;
    SpeedProfile _profile;
    float _max_lin, _max_ang;
    float _scale_fwd, _scale_bwd, _motor_trim;

    float _linear, _angular;
    unsigned long _last_cmd_time;
    float _man_linear, _man_angular;
    unsigned long _last_man_time;

    float _range_m;
    bool  _collision_guard;
    float _collision_avoid_dir;
    bool  _collision_avoid_active;

    // IMU data (fed externally)
    float _imu_yaw_rad;
    bool  _imu_calibrated;
    float _imu_gz;
    float _imu_gravity_body[3];
    float _imu_raw_lin_bx, _imu_raw_lin_by;

    // Push detector state
    float _push_bias_x, _push_bias_y;
    bool  _push_bias_ready;
    int   _push_bias_samples;
    int   _push_dev_count;
    int   _push_fast_remaining;
    int   _push_cont_ticks;
    bool  _push_active;
    float _push_dir_x, _push_dir_y;

    bool  _is_stationary = false;
    const char* _prev_source;

    void _driveMotors(float lin_pct, float ang_pct) {
        float left  = lin_pct + ang_pct;
        float right = lin_pct - ang_pct;

        // Normalize if exceeds ±100
        float max_val = fmaxf(fabsf(left), fmaxf(fabsf(right), 100.0f));
        if (max_val > 100.0f) {
            left  = left  / max_val * 100.0f;
            right = right / max_val * 100.0f;
        }

        _setMotor(M1_CH_IN1, M1_CH_IN2, left);   // M1 left
        _setMotor(M2_CH_IN1, M2_CH_IN2, right);   // M2 right
    }

    // Set single motor via PCA9685 (slow decay: both channels active)
    void _setMotor(uint8_t ch_in1, uint8_t ch_in2, float speed_pct) {
        speed_pct = _clamp(speed_pct, -100.0f, 100.0f);
        uint16_t pwm = (uint16_t)(fabsf(speed_pct) / 100.0f * 4095);
        if (speed_pct > 0.5f) {
            // Forward: IN1=PWM, IN2=0
            _pca.setPWM(ch_in1, 0, pwm);
            _pca.setPWM(ch_in2, 0, 0);
        } else if (speed_pct < -0.5f) {
            // Reverse: IN1=0, IN2=PWM
            _pca.setPWM(ch_in1, 0, 0);
            _pca.setPWM(ch_in2, 0, pwm);
        } else {
            // Stop (slow decay: both low)
            _pca.setPWM(ch_in1, 0, 0);
            _pca.setPWM(ch_in2, 0, 0);
        }
    }

    void _pushDetect(float cy, float sy, float dt,
                     float &push_vx, float &push_vy) {
        // Body -> world rotation
        float bx = _imu_raw_lin_bx;
        float by = _imu_raw_lin_by;
        float wx = bx * cy - by * sy;
        float wy = bx * sy + by * cy;

        float dx = wx - _push_bias_x;
        float dy = wy - _push_bias_y;
        float dev = sqrtf(dx*dx + dy*dy);

        bool is_push = false;
        if (dev > PUSH_DEV_THR) {
            _push_dev_count++;
            if (_push_dev_count >= PUSH_CONFIRM) is_push = true;
        } else {
            _push_dev_count = 0;
        }

        if (is_push) {
            _push_cont_ticks++;
            if (_push_cont_ticks > PUSH_MAX_TICKS) {
                // GRAV recovery
                _push_active = false;
                float alpha = PUSH_BIAS_FAST;
                _push_bias_x += alpha * (wx - _push_bias_x);
                _push_bias_y += alpha * (wy - _push_bias_y);
            } else {
                if (!_push_active) {
                    _push_dir_x = dx / dev;
                    _push_dir_y = dy / dev;
                    _push_active = true;
                }
                push_vx = PUSH_SPEED * _push_dir_x;
                push_vy = PUSH_SPEED * _push_dir_y;
                _push_fast_remaining = PUSH_FAST_TICKS;
            }
        } else {
            _push_cont_ticks = 0;
            _push_active = false;
            float alpha;
            if (_push_bias_samples < 40) {
                alpha = 0.1f;
                _push_bias_samples++;
                if (_push_bias_samples >= 40) _push_bias_ready = true;
            } else if (_push_fast_remaining > 0) {
                alpha = PUSH_BIAS_FAST;
                _push_fast_remaining--;
            } else {
                alpha = PUSH_BIAS_SLOW;
            }
            _push_bias_x += alpha * (wx - _push_bias_x);
            _push_bias_y += alpha * (wy - _push_bias_y);
        }

        odom_x += push_vx * dt;
        odom_y += push_vy * dt;
    }

    static float _clamp(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }
    static float _round2(float v) { return roundf(v * 100.0f) / 100.0f; }
    static float _round3(float v) { return roundf(v * 1000.0f) / 1000.0f; }
    static float _round4(float v) { return roundf(v * 10000.0f) / 10000.0f; }
};
