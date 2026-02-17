#!/usr/bin/env python3
"""
battery_node — Battery voltage monitoring via ADS7830 ADC.

Reads battery voltage through I2C ADC (ADS7830) with voltage divider.
2S LiPo: 6.0V = 0%, 8.4V = 100%.

Publishes:
    /battery          (std_msgs/Float32)  — voltage in volts
    /battery_percent  (std_msgs/Float32)  — 0-100 %
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ADS7830 channel commands (single-ended, channels 0-7)
_CHANNEL_CMDS = (0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4)

# 2S LiPo voltage range
VBAT_MIN = 6.0   # 0 %
VBAT_MAX = 8.4   # 100 %

# Voltage divider ratio (ADC reads 0-3.3V, battery is 6-8.4V)
VDIV_RATIO = 3.0  # R1+R2 / R2 ≈ 3.0 (tune for your divider)

try:
    import smbus2
    _HW = True
except ImportError:
    _HW = False


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('i2c_address', 0x48)
        self.declare_parameter('channel', 0)
        self.declare_parameter('vdiv_ratio', VDIV_RATIO)

        self._addr = self.get_parameter('i2c_address').value
        self._channel = self.get_parameter('channel').value
        self._vdiv = self.get_parameter('vdiv_ratio').value

        self._bus = None
        if _HW:
            try:
                self._bus = smbus2.SMBus(1)
                # Test read
                self._bus.write_byte(self._addr, _CHANNEL_CMDS[self._channel])
                self._bus.read_byte(self._addr)
                self.get_logger().info(
                    f'ADS7830 found @ 0x{self._addr:02X}, channel {self._channel}')
            except Exception as e:
                self.get_logger().warn(f'ADS7830 not available: {e} — simulated')
                self._bus = None
        else:
            self.get_logger().warn('smbus2 not available — simulated battery')

        self._voltage_pub = self.create_publisher(Float32, '/battery', 10)
        self._percent_pub = self.create_publisher(Float32, '/battery_percent', 10)

        # 1 Hz reading
        self.create_timer(1.0, self._read_battery)

    def _read_battery(self):
        if self._bus is not None:
            try:
                self._bus.write_byte(self._addr, _CHANNEL_CMDS[self._channel])
                raw = self._bus.read_byte(self._addr)
                adc_voltage = raw / 255.0 * 3.3
                voltage = adc_voltage * self._vdiv
            except Exception:
                voltage = 0.0
        else:
            voltage = 7.8  # simulated

        percent = max(0.0, min(100.0,
            (voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0))

        v_msg = Float32()
        v_msg.data = round(voltage, 2)
        self._voltage_pub.publish(v_msg)

        p_msg = Float32()
        p_msg.data = round(percent, 1)
        self._percent_pub.publish(p_msg)

        if percent < 10.0:
            self.get_logger().error(f'CRITICAL battery: {voltage:.1f}V ({percent:.0f}%)')
        elif percent < 20.0:
            self.get_logger().warn(f'Low battery: {voltage:.1f}V ({percent:.0f}%)')

    def destroy_node(self):
        if self._bus is not None:
            self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
