"""
IMUPushDetector — passive motion estimation when motors are off.

When the chassis is pushed by a human hand or sliding, the wheel encoder
reports zero. The accelerometer sees the body-frame linear acceleration,
which we project to world frame, subtract a slow-EMA bias (gravity residual
plus accelerometer DC offset), and threshold. Confirmed deviations produce
a fixed-magnitude velocity vector — *not* an integral of acceleration,
because integrating noise drifts unboundedly.

Stateful, pure: no IO, no threads.
"""

import math
from dataclasses import dataclass


@dataclass
class PushParams:
    speed: float = 0.12          # m/s — fixed magnitude of the estimated push velocity
    deviation_threshold: float = 0.20  # m/s² — trigger threshold above bias
    confirm_samples: int = 2     # consecutive over-threshold samples before locking
    max_move_ticks: int = 30     # consecutive push ticks → assume gravity drift, fast bias adapt
    bias_alpha_slow: float = 0.02
    bias_alpha_fast: float = 0.15
    bias_init_alpha: float = 0.1
    bias_init_samples: int = 40
    fast_adapt_ticks: int = 20   # ticks after a movement: fast bias adapt to new orientation


class IMUPushDetector:
    """Detects passive motion via accel-bias deviation. Outputs (vx, vy) in world frame."""

    def __init__(self, params: PushParams = PushParams()):
        self.p = params
        self._reset_state()

    def reset(self) -> None:
        """Forget bias and any locked direction. Call after pose reset."""
        self._reset_state()

    def _reset_state(self) -> None:
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_samples = 0
        self.bias_ready = False
        self.dev_count = 0
        self.cont_ticks = 0
        self.fast_remaining = 0
        self.active = False
        self.dir_x = 0.0
        self.dir_y = 0.0

    def update(self,
               raw_lin_bx: float,
               raw_lin_by: float,
               theta: float,
               motors_idle: bool):
        """Return (vx, vy) world-frame velocity estimate for this tick.

        raw_lin_b{x,y} — body-frame linear accel with gravity vector subtracted.
        Caller is responsible for that subtraction (the gravity_body comes
        from IMU calibration and lives outside this class).
        """
        if not motors_idle:
            # Motors driving — vibration would corrupt the bias estimate.
            self.dev_count = 0
            self.cont_ticks = 0
            self.active = False
            return 0.0, 0.0

        cy = math.cos(theta)
        sy = math.sin(theta)
        wx = raw_lin_bx * cy - raw_lin_by * sy
        wy = raw_lin_bx * sy + raw_lin_by * cy

        dx = wx - self.bias_x
        dy = wy - self.bias_y
        dev = math.sqrt(dx * dx + dy * dy)

        is_push = False
        if dev > self.p.deviation_threshold:
            self.dev_count += 1
            if self.dev_count >= self.p.confirm_samples:
                is_push = True
        else:
            self.dev_count = 0

        push_vx = 0.0
        push_vy = 0.0

        if is_push:
            self.cont_ticks += 1
            if self.cont_ticks > self.p.max_move_ticks:
                # Sustained "push" → most likely gravity residual after a tilt.
                # Treat as bias drift: stop emitting velocity and adapt fast.
                self.active = False
                a = self.p.bias_alpha_fast
                self.bias_x += a * (wx - self.bias_x)
                self.bias_y += a * (wy - self.bias_y)
            else:
                if not self.active:
                    self.dir_x = dx / dev
                    self.dir_y = dy / dev
                    self.active = True
                push_vx = self.p.speed * self.dir_x
                push_vy = self.p.speed * self.dir_y
                self.fast_remaining = self.p.fast_adapt_ticks
        else:
            self.cont_ticks = 0
            self.active = False
            if self.bias_samples < self.p.bias_init_samples:
                a = self.p.bias_init_alpha
                self.bias_samples += 1
                if self.bias_samples >= self.p.bias_init_samples:
                    self.bias_ready = True
            elif self.fast_remaining > 0:
                a = self.p.bias_alpha_fast
                self.fast_remaining -= 1
            else:
                a = self.p.bias_alpha_slow
            self.bias_x += a * (wx - self.bias_x)
            self.bias_y += a * (wy - self.bias_y)

        return push_vx, push_vy
