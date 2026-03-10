"""
PCA9685 PWM driver wrapper for Adeept Robot HAT V3.1.
I2C address: 0x5f, 16 channels, 50 Hz base frequency.
Based on adeept_rasptank2 move.py / RPIservo.py patterns.
"""

import logging
import os
import subprocess

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    _HW_AVAILABLE = True
except (ImportError, NotImplementedError, RuntimeError):
    _HW_AVAILABLE = False

PCA9685_ADDRESS = 0x5F
PWM_FREQUENCY = 50  # Hz — standard for servos and DC motors

_log = logging.getLogger(__name__)


def ensure_i2c_enabled() -> bool:
    """Check that I2C interface is enabled and /dev/i2c-1 exists.

    If I2C kernel module is not loaded, tries ``modprobe i2c-dev``.
    If the interface is disabled in raspi-config, tries to enable it.
    Returns True when /dev/i2c-1 is available, False otherwise.
    """
    if os.path.exists('/dev/i2c-1'):
        _log.debug('I2C: /dev/i2c-1 exists — OK')
        return True

    _log.warning('I2C: /dev/i2c-1 not found — attempting to enable')

    # Try loading kernel module
    try:
        subprocess.run(
            ['modprobe', 'i2c-dev'],
            capture_output=True, timeout=5,
        )
        _log.info('I2C: modprobe i2c-dev executed')
    except Exception as exc:
        _log.warning('I2C: modprobe i2c-dev failed: %s', exc)

    if os.path.exists('/dev/i2c-1'):
        _log.info('I2C: /dev/i2c-1 appeared after modprobe — OK')
        return True

    # Check raspi-config status and enable if disabled
    try:
        result = subprocess.run(
            ['raspi-config', 'nonint', 'get_i2c'],
            capture_output=True, text=True, timeout=5,
        )
        # raspi-config get_i2c returns "1" when DISABLED, "0" when ENABLED
        if result.stdout.strip() == '1':
            _log.warning('I2C: interface is disabled in raspi-config — enabling')
            subprocess.run(
                ['raspi-config', 'nonint', 'do_i2c', '0'],
                capture_output=True, timeout=10,
            )
            # Reload kernel module after enabling
            subprocess.run(
                ['modprobe', 'i2c-dev'],
                capture_output=True, timeout=5,
            )
            if os.path.exists('/dev/i2c-1'):
                _log.info('I2C: enabled successfully via raspi-config')
                return True
            else:
                _log.error('I2C: enabled in raspi-config but /dev/i2c-1 still '
                           'missing — reboot required')
                return False
        else:
            _log.warning('I2C: raspi-config says enabled but /dev/i2c-1 missing '
                         '— reboot may be required')
            return False
    except FileNotFoundError:
        _log.warning('I2C: raspi-config not found (not a Raspberry Pi?)')
        return False
    except Exception as exc:
        _log.error('I2C: raspi-config check failed: %s', exc)
        return False


class _FakeChannel:
    """Stub PWM channel used when real hardware is unavailable."""
    duty_cycle = 0


class _FakePWM:
    """Stub PCA9685 used when real hardware is unavailable."""
    frequency = PWM_FREQUENCY

    def __init__(self):
        self.channels = [_FakeChannel() for _ in range(16)]

    def deinit(self):
        pass


class PCA9685Driver:
    """Singleton-style wrapper around adafruit PCA9685."""

    _instance: 'PCA9685Driver | None' = None
    _pwm = None
    _simulated: bool = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    @property
    def simulated(self) -> bool:
        return self._simulated

    # ------------------------------------------------------------------
    def init(self):
        """Initialise I2C bus and PCA9685 chip (idempotent)."""
        if self._pwm is not None:
            return
        if not _HW_AVAILABLE:
            _log.warning('PCA9685 hardware libs unavailable — simulation mode')
            self._pwm = _FakePWM()
            self._simulated = True
            return

        # Ensure I2C is enabled before attempting to use it
        if not ensure_i2c_enabled():
            _log.error('I2C is not available — falling back to simulation')
            self._pwm = _FakePWM()
            self._simulated = True
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._pwm = PCA9685(i2c, address=PCA9685_ADDRESS)
            self._pwm.frequency = PWM_FREQUENCY
        except Exception as exc:
            _log.error('PCA9685 init failed: %s — falling back to simulation', exc)
            self._pwm = _FakePWM()
            self._simulated = True

    # ------------------------------------------------------------------
    @property
    def pwm(self):
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
            self._simulated = False
