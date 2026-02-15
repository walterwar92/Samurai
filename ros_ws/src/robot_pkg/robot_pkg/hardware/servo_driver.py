"""
Servo driver for Adeept Robot HAT V3.1 — PCA9685 channel 0 (claw).
Pulse range 500-2400 µs, 180° actuation.
Based on adeept_rasptank2/web/RPIservo.py patterns.
"""

from adafruit_motor import servo as adafruit_servo
from .pca9685_driver import PCA9685Driver

CLAW_CHANNEL = 0
MIN_PULSE = 500    # µs
MAX_PULSE = 2400   # µs
ACTUATION_RANGE = 180  # degrees

CLAW_OPEN_ANGLE = 30.0
CLAW_CLOSE_ANGLE = 130.0
CLAW_INIT_ANGLE = 90.0


class ServoDriver:
    """Controls claw servo on PCA9685 channel 0."""

    def __init__(self, channel: int = CLAW_CHANNEL):
        self._pca = PCA9685Driver()
        self._pca.init()
        self._servo = adafruit_servo.Servo(
            self._pca.channel(channel),
            min_pulse=MIN_PULSE,
            max_pulse=MAX_PULSE,
            actuation_range=ACTUATION_RANGE,
        )
        self._angle = CLAW_INIT_ANGLE
        self._servo.angle = self._angle

    @property
    def angle(self) -> float:
        return self._angle

    def set_angle(self, angle: float):
        """Set servo to *angle* degrees (0-180)."""
        angle = max(0.0, min(180.0, angle))
        self._servo.angle = angle
        self._angle = angle

    def open_claw(self):
        self.set_angle(CLAW_OPEN_ANGLE)

    def close_claw(self):
        self.set_angle(CLAW_CLOSE_ANGLE)

    def init_position(self):
        self.set_angle(CLAW_INIT_ANGLE)
