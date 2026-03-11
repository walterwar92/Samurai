#!/usr/bin/env python3
"""
imu_node — MPU6050 6-axis IMU on I2C bus.

Publishes:
    samurai/{robot_id}/imu  — {ax,ay,az, gx,gy,gz, ts} @ 50 Hz
"""

import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

try:
    import smbus2
    _HW = True
except ImportError:
    _HW = False

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
        ax, ay, az, gx, gy, gz = self._read_raw()
        self.publish('imu', {
            'ax': round(ax, 4), 'ay': round(ay, 4), 'az': round(az, 4),
            'gx': round(gx, 5), 'gy': round(gy, 5), 'gz': round(gz, 5),
            'ts': self.timestamp(),
        })

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
