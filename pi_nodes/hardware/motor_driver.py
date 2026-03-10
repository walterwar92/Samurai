"""
DC motor driver for 4-motor tracked chassis on Adeept Robot HAT V3.1.

Motor channel map on PCA9685 (address 0x5f):
  M1: IN1=ch15, IN2=ch14  (right-front)
  M2: IN1=ch12, IN2=ch13  (left-front)
  M3: IN1=ch11, IN2=ch10  (left-rear)
  M4: IN1=ch8,  IN2=ch9   (right-rear)

Throttle range: -1.0 … +1.0  (negative = reverse).
Based on adeept_rasptank2/web/move.py patterns.
"""

import logging

try:
    from adafruit_motor import motor as adafruit_motor
    _HW_MOTOR = True
except (ImportError, RuntimeError):
    _HW_MOTOR = False

from .pca9685_driver import PCA9685Driver

# (IN1_channel, IN2_channel)
MOTOR_CHANNELS = {
    1: (15, 14),  # M1 right-front
    2: (12, 13),  # M2 left-front
    3: (11, 10),  # M3 left-rear
    4: (8, 9),    # M4 right-rear
}

_log = logging.getLogger(__name__)


class _FakeMotor:
    """Stub DC motor for simulation mode."""
    throttle = 0.0
    decay_mode = 0


def _map_value(x: float, in_min: float, in_max: float,
               out_min: float, out_max: float) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class MotorDriver:
    """High-level API for 4-motor tracked platform."""

    def __init__(self):
        self._pca = PCA9685Driver()
        self._pca.init()
        self._simulated = self._pca.simulated or not _HW_MOTOR
        self._motors: dict = {}

        if self._simulated:
            _log.warning('Motor hardware unavailable — running in simulation mode')
            for mid in MOTOR_CHANNELS:
                self._motors[mid] = _FakeMotor()
        else:
            for mid, (ch_in1, ch_in2) in MOTOR_CHANNELS.items():
                m = adafruit_motor.DCMotor(
                    self._pca.channel(ch_in1),
                    self._pca.channel(ch_in2),
                )
                m.decay_mode = adafruit_motor.SLOW_DECAY
                self._motors[mid] = m

    @property
    def simulated(self) -> bool:
        return self._simulated

    # ------------------------------------------------------------------
    def set_motor(self, motor_id: int, speed_pct: float):
        """Set single motor. *speed_pct* in [-100 … 100]."""
        speed_pct = max(-100.0, min(100.0, speed_pct))
        throttle = _map_value(abs(speed_pct), 0, 100, 0.0, 1.0)
        if speed_pct < 0:
            throttle = -throttle
        self._motors[motor_id].throttle = throttle

    # ------------------------------------------------------------------
    def move(self, linear: float, angular: float):
        """
        Differential-drive mixer.
        *linear*  : -100 … +100  (forward positive)
        *angular* : -100 … +100  (counter-clockwise positive)

        Left  side = M2, M3
        Right side = M1, M4
        """
        left = linear + angular
        right = linear - angular

        # clamp
        max_val = max(abs(left), abs(right), 100.0)
        if max_val > 100.0:
            left = left / max_val * 100.0
            right = right / max_val * 100.0

        self.set_motor(2, left)   # left-front
        self.set_motor(3, left)   # left-rear
        self.set_motor(1, right)  # right-front
        self.set_motor(4, right)  # right-rear

    # ------------------------------------------------------------------
    def stop(self):
        for m in self._motors.values():
            m.throttle = 0

    def shutdown(self):
        self.stop()
        self._pca.deinit()
