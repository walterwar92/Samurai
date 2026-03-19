#!/usr/bin/env python3
"""
imu_node — MPU6050 6-axis IMU on I2C bus.

Publishes:
    samurai/{robot_id}/imu  — {ax,ay,az, gx,gy,gz, ts, ekf?, calibrated} @ 50 Hz

Startup calibration:
    Collects CALIBRATION_SAMPLES readings while robot is stationary,
    computes gyro bias offsets and initial orientation from accelerometer.
    All subsequent angles are relative to this "home" position.

When EKF is enabled (config imu.ekf.enabled), the payload includes
an 'ekf' sub-dict with filtered roll/pitch/yaw and gyro bias estimates.
"""

import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    from config_loader import cfg
except ImportError:
    def cfg(key, default=None):
        return default

try:
    import smbus2
    _HW = True
except ImportError:
    _HW = False

# IMU filter — no longer requires numpy (pure Python 2D filter)
_FILTER_AVAILABLE = False
try:
    from pi_nodes.filters.ekf_imu import EkfImu
    _FILTER_AVAILABLE = True
except ImportError:
    pass

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
DLPF_CFG = 0x1A         # Digital Low Pass Filter config register
SMPLRT_DIV = 0x19        # Sample rate divider register
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0
DEG2RAD = math.pi / 180.0

# Calibration: 100 samples @ 50 Hz = 2 seconds
CALIBRATION_SAMPLES = 100

# Exponential Moving Average alpha for accelerometer (0..1)
# Lower = smoother but more lag.  0.2 @ 50Hz ≈ 8Hz cutoff — good for vibrations.
ACCEL_EMA_ALPHA = 0.2
# EMA for gyroscope — less aggressive, gyro is less noisy
GYRO_EMA_ALPHA = 0.5


class IMUNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('imu_node', **kwargs)

        # EMA filter state (initialized on first reading)
        self._ema_ax = None
        self._ema_ay = None
        self._ema_az = None
        self._ema_gx = None
        self._ema_gy = None
        self._ema_gz = None

        self._bus = None
        if _HW:
            try:
                bus = smbus2.SMBus(1)
                # Wake up MPU6050
                bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
                time.sleep(0.1)
                # Set DLPF to mode 3 (accel 44Hz, gyro 42Hz) — filters vibrations
                bus.write_byte_data(MPU6050_ADDR, DLPF_CFG, 0x03)
                # Sample rate = 1kHz / (1 + 9) = 100 Hz
                bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 9)
                # Gyro: ±250°/s (most sensitive), Accel: ±2g (most sensitive)
                bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
                bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
                self._bus = bus
                self.log_info('MPU6050 initialised @ 0x%02X (DLPF=3, SR=100Hz)',
                              MPU6050_ADDR)
            except Exception as e:
                self.log_warn('MPU6050 not found (0x%02X): %s — IMU simulated',
                              MPU6050_ADDR, e)
        else:
            self.log_warn('smbus2 unavailable — IMU simulated')

        # Calibration state
        self._calibrated = False
        self._cal_samples = []
        self._gyro_offset = (0.0, 0.0, 0.0)  # raw gyro bias from calibration
        self._gravity_body = (0.0, 0.0, 9.81)  # gravity vector in body frame at rest
        self._home_roll = 0.0
        self._home_pitch = 0.0

        # EKF setup
        self._ekf = None
        self._ekf_enabled = cfg('imu.ekf.enabled', True)
        if self._ekf_enabled and _FILTER_AVAILABLE:
            self._ekf = EkfImu(
                q_angle=cfg('imu.ekf.q_angle', 0.001),
                q_bias=cfg('imu.ekf.q_bias', 0.0001),
                r_accel=cfg('imu.ekf.r_accel', 0.5),
                accel_gate=cfg('imu.ekf.accel_gate', 0.5),
            )
            self.log_info('EKF enabled (q_angle=%.4f, q_bias=%.5f, r_accel=%.2f)',
                          self._ekf.q_angle, self._ekf.q_bias, self._ekf.r_accel)
        elif self._ekf_enabled and not _FILTER_AVAILABLE:
            self.log_warn('EKF requested but numpy/ekf_imu not available — disabled')
        else:
            self.log_info('EKF disabled by config')

        self._last_time = time.monotonic()

        self.log_info('Starting calibration (%d samples, ~%.1f sec)...',
                      CALIBRATION_SAMPLES, CALIBRATION_SAMPLES * 0.02)
        self.create_timer(0.02, self._read_and_publish)  # 50 Hz

    def _read_raw(self):
        """Read raw sensor values (no offset correction)."""
        if not self._bus:
            return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
        try:
            raw = self._bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
            ax = struct.unpack('>h', bytes(raw[0:2]))[0] / ACCEL_SCALE * 9.81
            ay = struct.unpack('>h', bytes(raw[2:4]))[0] / ACCEL_SCALE * 9.81
            az = struct.unpack('>h', bytes(raw[4:6]))[0] / ACCEL_SCALE * 9.81
            gx = struct.unpack('>h', bytes(raw[8:10]))[0] / GYRO_SCALE * DEG2RAD
            gy = struct.unpack('>h', bytes(raw[10:12]))[0] / GYRO_SCALE * DEG2RAD
            gz = struct.unpack('>h', bytes(raw[12:14]))[0] / GYRO_SCALE * DEG2RAD
            return (ax, ay, az, gx, gy, gz)
        except Exception as e:
            self.log_warn('IMU read error: %s — returning zeros', e)
            self._bus = None  # stop trying until restart
            return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

    def _apply_ema(self, ax, ay, az, gx, gy, gz):
        """Exponential Moving Average — smooths vibration noise."""
        a = ACCEL_EMA_ALPHA
        g = GYRO_EMA_ALPHA
        if self._ema_ax is None:
            # First sample — initialize
            self._ema_ax, self._ema_ay, self._ema_az = ax, ay, az
            self._ema_gx, self._ema_gy, self._ema_gz = gx, gy, gz
        else:
            self._ema_ax = a * ax + (1 - a) * self._ema_ax
            self._ema_ay = a * ay + (1 - a) * self._ema_ay
            self._ema_az = a * az + (1 - a) * self._ema_az
            self._ema_gx = g * gx + (1 - g) * self._ema_gx
            self._ema_gy = g * gy + (1 - g) * self._ema_gy
            self._ema_gz = g * gz + (1 - g) * self._ema_gz
        return (self._ema_ax, self._ema_ay, self._ema_az,
                self._ema_gx, self._ema_gy, self._ema_gz)

    def _finish_calibration(self):
        """Compute offsets from collected samples, init EKF with home position."""
        n = len(self._cal_samples)
        sum_ax = sum_ay = sum_az = 0.0
        sum_gx = sum_gy = sum_gz = 0.0
        for (ax, ay, az, gx, gy, gz) in self._cal_samples:
            sum_ax += ax; sum_ay += ay; sum_az += az
            sum_gx += gx; sum_gy += gy; sum_gz += gz

        # Gyro offset: average at rest should be zero
        self._gyro_offset = (sum_gx / n, sum_gy / n, sum_gz / n)

        # Accel average — gravity vector in body frame at rest
        avg_ax = sum_ax / n
        avg_ay = sum_ay / n
        avg_az = sum_az / n
        self._gravity_body = (avg_ax, avg_ay, avg_az)

        # Initial roll/pitch from gravity vector (home = 0)
        self._home_roll = math.atan2(avg_ay, avg_az)
        self._home_pitch = math.atan2(-avg_ax,
                                      math.sqrt(avg_ay * avg_ay + avg_az * avg_az))

        # Initialize EKF with home orientation and calibrated gyro bias
        if self._ekf is not None:
            self._ekf.init_from_calibration(
                roll=self._home_roll,
                pitch=self._home_pitch,
                yaw=0.0,  # yaw = 0 at startup (no magnetometer)
                gyro_bias=self._gyro_offset,
            )

        self._calibrated = True
        self._cal_samples = []  # free memory

        g_mag = math.sqrt(avg_ax**2 + avg_ay**2 + avg_az**2)
        self.log_info(
            'Calibration done: gyro_offset=(%.5f, %.5f, %.5f) rad/s, '
            'gravity=(%.3f, %.3f, %.3f) |g|=%.3f m/s², '
            'home roll=%.1f° pitch=%.1f°',
            self._gyro_offset[0], self._gyro_offset[1], self._gyro_offset[2],
            avg_ax, avg_ay, avg_az, g_mag,
            math.degrees(self._home_roll), math.degrees(self._home_pitch),
        )

    def _read_and_publish(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        ax, ay, az, gx, gy, gz = self._read_raw()

        # ── EMA low-pass filter (before everything else) ──────────────
        ax, ay, az, gx, gy, gz = self._apply_ema(ax, ay, az, gx, gy, gz)

        # ── Calibration phase ─────────────────────────────────────────
        if not self._calibrated:
            self._cal_samples.append((ax, ay, az, gx, gy, gz))
            if len(self._cal_samples) >= CALIBRATION_SAMPLES:
                self._finish_calibration()
            # During calibration publish raw + calibrated=false
            self.publish('imu', {
                'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
                'gx': round(gx, 5), 'gy': round(gy, 5), 'gz': round(gz, 5),
                'calibrated': False,
                'ts': self.timestamp(),
            })
            return

        # ── Apply gyro offset correction ──────────────────────────────
        gx -= self._gyro_offset[0]
        gy -= self._gyro_offset[1]
        gz -= self._gyro_offset[2]

        payload = {
            'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
            'gx': round(gx, 5), 'gy': round(gy, 5), 'gz': round(gz, 5),
            'calibrated': True,
            'ts': self.timestamp(),
        }

        if self._ekf is not None:
            self._ekf.predict(gx, gy, gz, dt)
            self._ekf.update(ax, ay, az)
            r_deg, p_deg, y_deg = self._ekf.get_euler_deg()
            bx, by, bz = self._ekf.gyro_bias
            payload['ekf'] = {
                'roll':  round(r_deg, 2),
                'pitch': round(p_deg, 2),
                'yaw':   round(y_deg, 2),
                'roll_rad':  round(self._ekf.roll, 5),
                'pitch_rad': round(self._ekf.pitch, 5),
                'yaw_rad':   round(self._ekf.yaw, 5),
                'bias_gx': round(bx, 5),
                'bias_gy': round(by, 5),
                'bias_gz': round(bz, 5),
            }

        # Include calibration data for position estimator (motor_node)
        payload['gravity_body'] = [round(self._gravity_body[0], 4),
                                   round(self._gravity_body[1], 4),
                                   round(self._gravity_body[2], 4)]

        self.publish('imu', payload)

    def on_shutdown(self):
        if self._bus:
            self._bus.close()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = IMUNode(broker=args.broker, port=args.port,
                   robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
