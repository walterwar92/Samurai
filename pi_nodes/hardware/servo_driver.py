"""
Servo driver for Adeept Robot HAT V3.1 — PCA9685 channel 0 (claw).
Pulse range 500-2400 us, 180 degree actuation.
Based on adeept_rasptank2/web/RPIservo.py patterns.
"""

import logging

try:
    from adafruit_motor import servo as adafruit_servo
    _HW_SERVO = True
except (ImportError, RuntimeError):
    _HW_SERVO = False

from .pca9685_driver import PCA9685Driver

CLAW_CHANNEL = 0
MIN_PULSE = 500    # us
MAX_PULSE = 2400   # us
ACTUATION_RANGE = 180  # degrees

CLAW_OPEN_ANGLE = 0.0
CLAW_CLOSE_ANGLE = 180.0
CLAW_INIT_ANGLE = 0.0

_log = logging.getLogger(__name__)


class _FakeServo:
    """Stub servo for simulation mode."""
    angle = 90.0


class ServoDriver:
    """Controls a servo on any PCA9685 channel."""

    def __init__(self, channel: int = CLAW_CHANNEL,
                 init_angle: float = CLAW_INIT_ANGLE,
                 start_disabled: bool = False):
        """
        Args:
            channel: PCA9685 channel number.
            init_angle: Initial angle (degrees). Only applied if start_disabled=False.
            start_disabled: If True, do NOT send any PWM signal on init.
                The servo stays in whatever physical position it was in.
                Call set_angle() later to start controlling it.
        """
        self._pca = PCA9685Driver()
        self._pca.init()
        self._channel = channel
        self._simulated = self._pca.simulated or not _HW_SERVO

        if self._simulated:
            _log.warning('Servo ch%d hardware unavailable — simulation mode', channel)
            self._servo = _FakeServo()
        else:
            self._servo = adafruit_servo.Servo(
                self._pca.channel(channel),
                min_pulse=MIN_PULSE,
                max_pulse=MAX_PULSE,
                actuation_range=ACTUATION_RANGE,
            )

        self._angle = init_angle
        if not start_disabled:
            self._servo.angle = self._angle

    @property
    def simulated(self) -> bool:
        return self._simulated

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
