#!/usr/bin/env python3
"""
imu_node — MPU6050 6-axis IMU on Robot HAT V3.1 I2C bus.

Reads accelerometer + gyroscope via smbus2.
Publishes: /imu/data  (sensor_msgs/Imu)  @ 50 Hz

MPU6050 I2C address: 0x68 (default).
"""

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

try:
    import smbus2
    _HW = True
except ImportError:
    _HW = False

# MPU6050 registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B  # 14 bytes: accel(6) + temp(2) + gyro(6)

ACCEL_SCALE = 16384.0   # ±2 g  → LSB/g
GYRO_SCALE = 131.0      # ±250 °/s → LSB/(°/s)
DEG2RAD = math.pi / 180.0


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        if _HW:
            self._bus = smbus2.SMBus(1)
            # Wake up MPU6050
            self._bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
            self.get_logger().info('MPU6050 initialised on I2C bus 1')
        else:
            self._bus = None
            self.get_logger().warn('smbus2 unavailable — IMU simulated')

        # BEST_EFFORT + depth=1: IMU 50Hz — только последнее значение важно
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.create_timer(0.02, self._read_and_publish)  # 50 Hz

    def _read_raw(self):
        """Read 14 bytes of sensor data, return (ax,ay,az, gx,gy,gz)."""
        if not self._bus:
            return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
        raw = self._bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
        vals = struct.unpack('>hhh_hhh', bytes(raw))
        # vals: ax, ay, az, (temp skipped), gx, gy, gz
        # unpack skips temp with 'h' placeholder — we use manual indices
        ax = struct.unpack('>h', bytes(raw[0:2]))[0] / ACCEL_SCALE * 9.81
        ay = struct.unpack('>h', bytes(raw[2:4]))[0] / ACCEL_SCALE * 9.81
        az = struct.unpack('>h', bytes(raw[4:6]))[0] / ACCEL_SCALE * 9.81
        gx = struct.unpack('>h', bytes(raw[8:10]))[0] / GYRO_SCALE * DEG2RAD
        gy = struct.unpack('>h', bytes(raw[10:12]))[0] / GYRO_SCALE * DEG2RAD
        gz = struct.unpack('>h', bytes(raw[12:14]))[0] / GYRO_SCALE * DEG2RAD
        return (ax, ay, az, gx, gy, gz)

    def _read_and_publish(self):
        ax, ay, az, gx, gy, gz = self._read_raw()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientation unknown — set covariance[0] = -1
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self._pub.publish(msg)

    def destroy_node(self):
        if self._bus:
            self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
