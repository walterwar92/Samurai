"""
CollisionPolicy — range-based override of forward velocity.

Stateful (tracks last-warning time + avoid direction across calls) but pure:
no IO, no threads. Lets the caller decide how to log warnings.
"""

from dataclasses import dataclass
from typing import Callable, Optional, Tuple


@dataclass
class CollisionDecision:
    linear: float
    angular: float
    source: Optional[str]   # None | 'collision'
    avoidance_active: bool


class CollisionPolicy:
    """Forward-motion collision guard with active steering and stale-data handling.

    Behaviour (matches the original `_control_loop` block):
      r >= slow_m  → no change.
      stop_m <= r < slow_m → linear scaled down linearly.
      r <  stop_m  → full stop. If not manual: also issue an angular command
                     in the current avoid_dir; flip avoid_dir on the next clear.
      Stale range  → guard skipped, warning throttled to once per `warn_period_s`.
    """

    def __init__(self,
                 stop_m: float = 0.20,
                 slow_m: float = 0.40,
                 avoid_angular: float = 0.6,
                 stale_threshold_s: float = 1.0,
                 warn_period_s: float = 5.0):
        if not (stop_m < slow_m):
            raise ValueError('stop_m must be < slow_m')
        self.stop_m = stop_m
        self.slow_m = slow_m
        self.avoid_angular = avoid_angular
        self.stale_threshold_s = stale_threshold_s
        self.warn_period_s = warn_period_s

        # Mutable state
        self.avoid_dir = 1.0          # +1 = left, -1 = right
        self.avoid_active = False
        # -inf so the first stale event always logs, regardless of monotonic
        # clock origin (t=0 boundary case).
        self._stale_warn_t = float('-inf')

    def apply(self,
              linear: float,
              angular: float,
              range_m: float,
              range_age_s: float,
              manual_active: bool,
              now_mono: float,
              log_warn: Optional[Callable[[str], None]] = None
              ) -> CollisionDecision:
        """Compute the post-guard (linear, angular) and decide if 'collision' source applies."""
        # Guard only acts on forward motion. Reverse + zero never trigger.
        if linear <= 0.0:
            # Clear avoid_active when robot is not driving forward — so the
            # direction flip on next forward obstacle still works.
            return CollisionDecision(linear, angular, None, self.avoid_active)

        r = range_m
        stale = (range_age_s >= self.stale_threshold_s) or (range_age_s < 0.0)
        if stale:
            if log_warn and (now_mono - self._stale_warn_t) > self.warn_period_s:
                self._stale_warn_t = now_mono
                log_warn(f'Ultrasonic stale (age={range_age_s:.1f}s) — '
                         f'collision guard skipped')
            r = float('inf')

        source: Optional[str] = None
        if r < self.stop_m:
            linear = 0.0
            if not manual_active:
                angular = self.avoid_angular * self.avoid_dir
                source = 'collision'
                if not self.avoid_active:
                    self.avoid_active = True
                    if log_warn:
                        log_warn('Collision avoidance: steering '
                                 + ('left' if self.avoid_dir > 0 else 'right'))
        elif r < self.slow_m:
            factor = (r - self.stop_m) / (self.slow_m - self.stop_m)
            linear *= max(0.0, factor)
            if not manual_active:
                source = 'collision'
        else:
            if self.avoid_active:
                self.avoid_active = False
                self.avoid_dir *= -1.0    # alternate direction next time

        return CollisionDecision(linear, angular, source, self.avoid_active)
