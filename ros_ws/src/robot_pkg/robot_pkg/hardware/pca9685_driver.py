"""
PCA9685 PWM driver wrapper for Adeept Robot HAT V3.1.
I2C address: 0x5f, 16 channels, 50 Hz base frequency.
Based on adeept_rasptank2 move.py / RPIservo.py patterns.
"""

import board
import busio
from adafruit_pca9685 import PCA9685

PCA9685_ADDRESS = 0x5F
PWM_FREQUENCY = 50  # Hz — standard for servos and DC motors


class PCA9685Driver:
    """Singleton-style wrapper around adafruit PCA9685."""

    _instance: 'PCA9685Driver | None' = None
    _pwm: PCA9685 | None = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    # ------------------------------------------------------------------
    def init(self):
        """Initialise I2C bus and PCA9685 chip (idempotent)."""
        if self._pwm is not None:
            return
        i2c = busio.I2C(board.SCL, board.SDA)
        self._pwm = PCA9685(i2c, address=PCA9685_ADDRESS)
        self._pwm.frequency = PWM_FREQUENCY

    # ------------------------------------------------------------------
    @property
    def pwm(self) -> PCA9685:
        if self._pwm is None:
            self.init()
        return self._pwm

    def channel(self, ch: int):
        """Return raw PWMChannel object for *ch* (0-15)."""
        return self.pwm.channels[ch]

    def deinit(self):
        if self._pwm is not None:
            self._pwm.deinit()
            self._pwm = None
            PCA9685Driver._instance = None
