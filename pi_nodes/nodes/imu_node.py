#!/usr/bin/env python3
"""
imu_node — MPU6050 6-axis IMU on I2C bus.

Publishes:
    samurai/{robot_id}/imu  — {ax,ay,az, gx,gy,gz, ts, ekf?} @ 50 Hz

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

# EKF — optional (requires numpy)
_EKF_AVAILABLE = False
try:
    from pi_nodes.filters.ekf_imu import EkfImu
    _EKF_AVAILABLE = True
except ImportError:
    pass

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0
DEG2RAD = math.pi / 180.0


class IMUNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('imu_node', **kwargs)

        self._bus = None
        if _HW:
            try:
                bus = smbus2.SMBus(1)
                bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
                self._bus = bus
                self.log_info('MPU6050 initialised @ 0x%02X', MPU6050_ADDR)
            except Exception as e:
                self.log_warn('MPU6050 not found (0x%02X): %s — IMU simulated',
                              MPU6050_ADDR, e)
        else:
            self.log_warn('smbus2 unavailable — IMU simulated')

        # EKF setup
        self._ekf = None
        ekf_enabled = cfg('imu.ekf.enabled', True)
        if ekf_enabled and _EKF_AVAILABLE:
            self._ekf = EkfImu(
                q_angle=cfg('imu.ekf.q_angle', 0.001),
                q_bias=cfg('imu.ekf.q_bias', 0.0001),
                r_accel=cfg('imu.ekf.r_accel', 0.5),
                accel_gate=cfg('imu.ekf.accel_gate', 0.5),
            )
            self.log_info('EKF enabled (q_angle=%.4f, q_bias=%.5f, r_accel=%.2f)',
                          self._ekf.q_angle, self._ekf.q_bias, self._ekf.r_accel)
        elif ekf_enabled and not _EKF_AVAILABLE:
            self.log_warn('EKF requested but numpy/ekf_imu not available — disabled')
        else:
            self.log_info('EKF disabled by config')

        self._last_time = time.monotonic()

        self.create_timer(0.02, self._read_and_publish)  # 50 Hz

    def _read_raw(self):
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

    def _read_and_publish(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        ax, ay, az, gx, gy, gz = self._read_raw()

        payload = {
            'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
            'gx': round(gx, 5), 'gy': round(gy, 5), 'gz': round(gz, 5),
            'ts': self.timestamp(),
        }

        if self._ekf is not None:
            self._ekf.predict(gx, gy, gz, dt)
            self._ekf.update(ax, ay, az)
            r, p, y = self._ekf.get_euler_deg()
            bx, by, bz = self._ekf.gyro_bias
            payload['ekf'] = {
                'roll':  round(r, 2),
                'pitch': round(p, 2),
                'yaw':   round(y, 2),
                'bias_gx': round(bx, 5),
                'bias_gy': round(by, 5),
                'bias_gz': round(bz, 5),
            }

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
