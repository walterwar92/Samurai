"""
Servo driver for Adeept Robot HAT V3.1 — PCA9685 any channel.
Pulse range 500-2400 us, 180 degree actuation.
Supports freeze mode (PWM stays active, resists physical movement).
"""

import logging
import threading

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

# Time (seconds) to keep PWM active after set_angle, then release
HOLD_TIME = 0.5

# Freeze: re-apply PWM at this interval to resist physical displacement
FREEZE_REFRESH_INTERVAL = 0.1  # seconds

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
        self._release_timer: threading.Timer | None = None
        self._frozen = False
        self._freeze_timer: threading.Timer | None = None

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
            self._schedule_release()

    @property
    def simulated(self) -> bool:
        return self._simulated

    @property
    def angle(self) -> float:
        return self._angle

    @property
    def frozen(self) -> bool:
        return self._frozen

    def set_angle(self, angle: float, force: bool = False):
        """Set servo to *angle* degrees (0-180).

        PWM is sent immediately, then automatically released after
        HOLD_TIME seconds so the servo doesn't buzz at rest.
        If frozen and force=False, the command is ignored.
        """
        if self._frozen and not force:
            _log.debug('Servo ch%d frozen — ignoring set_angle(%.1f)',
                        self._channel, angle)
            return
        angle = max(0.0, min(180.0, angle))
        self._servo.angle = angle
        self._angle = angle
        if not self._frozen:
            self._schedule_release()

    def freeze(self):
        """Hold current position — PWM stays active, resists movement."""
        self._frozen = True
        # Cancel pending release
        if self._release_timer is not None:
            self._release_timer.cancel()
            self._release_timer = None
        # Activate PWM at current angle
        self._servo.angle = self._angle
        # Start periodic refresh to maintain position
        self._start_freeze_refresh()
        _log.info('Servo ch%d FROZEN at %.1f°', self._channel, self._angle)

    def unfreeze(self):
        """Return to normal mode with auto-release after movement."""
        self._frozen = False
        self._stop_freeze_refresh()
        self._schedule_release()
        _log.info('Servo ch%d UNFROZEN', self._channel)

    def release(self):
        """Disable PWM signal — servo goes limp, stops buzzing."""
        if self._frozen:
            return  # Never release while frozen
        self._servo.angle = None
        _log.debug('Servo ch%d PWM released', self._channel)

    def _schedule_release(self):
        """Cancel any pending release and schedule a new one."""
        if self._frozen:
            return
        if self._release_timer is not None:
            self._release_timer.cancel()
        self._release_timer = threading.Timer(HOLD_TIME, self.release)
        self._release_timer.daemon = True
        self._release_timer.start()

    def _start_freeze_refresh(self):
        """Periodically re-apply PWM to resist physical displacement."""
        self._stop_freeze_refresh()

        def _refresh():
            if self._frozen:
                self._servo.angle = self._angle
                self._freeze_timer = threading.Timer(
                    FREEZE_REFRESH_INTERVAL, _refresh)
                self._freeze_timer.daemon = True
                self._freeze_timer.start()

        self._freeze_timer = threading.Timer(FREEZE_REFRESH_INTERVAL, _refresh)
        self._freeze_timer.daemon = True
        self._freeze_timer.start()

    def _stop_freeze_refresh(self):
        """Stop freeze refresh timer."""
        if self._freeze_timer is not None:
            self._freeze_timer.cancel()
            self._freeze_timer = None

    def open_claw(self):
        self.set_angle(CLAW_OPEN_ANGLE)

    def close_claw(self):
        self.set_angle(CLAW_CLOSE_ANGLE)

    def init_position(self):
        self.set_angle(CLAW_INIT_ANGLE)
