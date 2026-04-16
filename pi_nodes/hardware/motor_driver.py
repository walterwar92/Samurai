"""
DC motor driver for 2-motor tracked chassis on Adeept Robot HAT V3.1.

Физически на роботе установлены только 2 задних мотора (передние
каналы HAT'а ch12-ch15 не используются — нет моторов).

Motor channel map on PCA9685 (address 0x5f):
  M1: IN1=ch11, IN2=ch10  (left-rear)
  M2: IN1=ch8,  IN2=ch9   (right-rear)

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
    1: (11, 10),  # M1 left-rear
    2: (8, 9),    # M2 right-rear
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
    """High-level API for 2-motor tracked platform (rear wheels only)."""

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

        Left  side = M1 (left-rear)
        Right side = M2 (right-rear)
        """
        left = linear + angular
        right = linear - angular

        # clamp
        max_val = max(abs(left), abs(right), 100.0)
        if max_val > 100.0:
            left = left / max_val * 100.0
            right = right / max_val * 100.0

        self.set_motor(1, left)   # M1 left-rear
        self.set_motor(2, right)  # M2 right-rear

    # ------------------------------------------------------------------
    def stop(self):
        for m in self._motors.values():
            m.throttle = 0

    def shutdown(self):
        self.stop()
        self._pca.deinit()
