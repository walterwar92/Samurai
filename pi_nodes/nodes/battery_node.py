#!/usr/bin/env python3
"""
battery_node — Battery voltage monitoring via ADS7830 ADC.

Publishes:
    samurai/{robot_id}/battery  — {voltage, percent} @ 1 Hz
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

_CHANNEL_CMDS = (0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4)
VBAT_MIN = 6.0
VBAT_MAX = 8.4
VDIV_RATIO = 3.0

try:
    import smbus2
    _HW = True
except ImportError:
    _HW = False


class BatteryNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('battery_node', **kwargs)

        self._addr = 0x48
        self._channel = 0
        self._vdiv = VDIV_RATIO
        self._bus = None

        if _HW:
            try:
                self._bus = smbus2.SMBus(1)
                self._bus.write_byte(self._addr, _CHANNEL_CMDS[self._channel])
                self._bus.read_byte(self._addr)
                self.log_info('ADS7830 found @ 0x%02X, channel %d',
                              self._addr, self._channel)
            except Exception as e:
                self.log_warn('ADS7830 not available: %s — simulated', e)
                self._bus = None
        else:
            self.log_warn('smbus2 not available — simulated battery')

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
            voltage = 7.8

        percent = max(0.0, min(100.0,
            (voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0))

        self.publish('battery', {
            'voltage': round(voltage, 2),
            'percent': round(percent, 1),
        }, qos=1, retain=True)

        if percent < 10.0:
            self.log_error('CRITICAL battery: %.1fV (%.0f%%)', voltage, percent)
        elif percent < 20.0:
            self.log_warn('Low battery: %.1fV (%.0f%%)', voltage, percent)

    def on_shutdown(self):
        if self._bus is not None:
            self._bus.close()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = BatteryNode(broker=args.broker, port=args.port,
                       robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
