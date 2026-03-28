"""
2D Velocity Extended Kalman Filter for tracked ground robot.

Fuses two velocity sources for optimal position estimation:
    1. Wheel odometry velocity (from motor commands) — stable, no drift,
       but susceptible to wheel slip and inaccurate on rough terrain.
    2. Accelerometer-derived velocity (single integration) — captures
       actual motion including slip, but drifts over time.

State vector: [vx, vy] in world frame.
Prediction: from wheel commands (reliable direction, noisy magnitude).
Update: from accelerometer (reliable short-term, drifts long-term).
ZUPT: zero-velocity measurement when stationary.

Position is obtained by integrating the fused velocity.
This gives better accuracy than complementary filter because:
- Kalman gain automatically weights sources by their noise levels
- Covariance grows naturally when uncertain, shrinks with measurements
- ZUPT update is a proper measurement, not just a hard reset

No numpy required — pure Python 2x2 matrix operations.
"""

from math import cos, sin, sqrt


class VelocityEKF:
    """2D velocity Kalman filter with position integration.

    Usage:
        ekf = VelocityEKF()

        # Prediction: from wheel commands (20 Hz)
        ekf.predict_wheel(vx_cmd, theta, dt)

        # Update: from accelerometer-derived velocity (50 Hz)
        ekf.update_accel(ax_world, ay_world, dt)

        # ZUPT: when stationary
        ekf.update_zupt()

        # Read fused state
        x, y = ekf.x, ekf.y
        vx, vy = ekf.vx, ekf.vy
    """

    def __init__(self,
                 q_wheel: float = 0.01,
                 q_accel: float = 0.05,
                 r_accel: float = 0.1,
                 r_zupt: float = 0.0001):
        """
        Args:
            q_wheel: Process noise for wheel prediction (m/s)² — how much
                     we distrust wheel commands (slip, calibration).
            q_accel: Process noise for accel integration (m/s)² — how fast
                     accel-derived velocity drifts.
            r_accel: Measurement noise for accel velocity update (m/s)² —
                     noise level of accelerometer after gravity removal.
            r_zupt: Measurement noise for ZUPT (m/s)² — very small,
                    we trust that stationary = zero velocity.
        """
        self.q_wheel = q_wheel
        self.q_accel = q_accel
        self.r_accel = r_accel
        self.r_zupt = r_zupt

        # Velocity state [vx, vy] in world frame
        self.vx = 0.0
        self.vy = 0.0

        # 2x2 covariance matrix (stored as 4 scalars)
        # P = [[p00, p01], [p10, p11]]
        self._p00 = 0.01
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = 0.01

        # Position (integrated from fused velocity)
        self.x = 0.0
        self.y = 0.0

        # Accel-integrated velocity (used as measurement source)
        self._accel_vx = 0.0
        self._accel_vy = 0.0

        # Non-holonomic: lateral decay
        self._lateral_decay = 0.3  # aggressively kill sideways velocity

    def predict_wheel(self, vx_cmd: float, theta: float, dt: float):
        """Prediction step using wheel odometry command.

        The wheel command gives us a forward velocity along the heading.
        We trust this as the primary prediction source.

        Args:
            vx_cmd: Commanded forward velocity (m/s).
            theta: Current heading from IMU (radians).
            dt: Time step.
        """
        if dt <= 0:
            return

        # Wheel-predicted velocity in world frame
        vx_wheel = vx_cmd * cos(theta)
        vy_wheel = vx_cmd * sin(theta)

        # Prediction: state transitions toward wheel command
        # Higher alpha = more trust in wheel commands (primary source)
        alpha = 0.65  # prediction blend — wheels are more reliable
        self.vx = (1 - alpha) * self.vx + alpha * vx_wheel
        self.vy = (1 - alpha) * self.vy + alpha * vy_wheel

        # Process noise: covariance grows
        self._p00 += self.q_wheel * dt
        self._p11 += self.q_wheel * dt

        # Integrate position using current fused velocity
        self.x += self.vx * dt
        self.y += self.vy * dt

    def update_accel(self, la_wx: float, la_wy: float, dt: float):
        """Update step using accelerometer-derived velocity.

        Single-integrates linear acceleration to get a velocity measurement,
        then fuses it with the predicted velocity via Kalman update.

        Args:
            la_wx, la_wy: Gravity-free linear acceleration in world frame (m/s²).
            dt: Time step.
        """
        if dt <= 0:
            return

        # Integrate accel to get velocity measurement (single integration)
        self._accel_vx += la_wx * dt
        self._accel_vy += la_wy * dt

        # Decay accel velocity to prevent unbounded drift
        self._accel_vx *= 0.98
        self._accel_vy *= 0.98

        # Measurement: z = [accel_vx, accel_vy]
        # Innovation: y = z - H*x (H = identity for velocity)
        yx = self._accel_vx - self.vx
        yy = self._accel_vy - self.vy

        # S = P + R (measurement covariance)
        r = self.r_accel + self.q_accel * dt
        s00 = self._p00 + r
        s11 = self._p11 + r
        # (off-diagonal S terms are just P off-diagonal since R is diagonal)

        # Kalman gain: K = P * S^-1 (for diagonal S, this is element-wise)
        if s00 > 1e-10:
            k00 = self._p00 / s00
        else:
            k00 = 0.0
        if s11 > 1e-10:
            k11 = self._p11 / s11
        else:
            k11 = 0.0

        # State update
        self.vx += k00 * yx
        self.vy += k11 * yy

        # Covariance update: P = (I - K*H) * P
        self._p00 *= (1 - k00)
        self._p11 *= (1 - k11)
        # Cross terms decay
        self._p01 *= (1 - 0.5 * (k00 + k11))
        self._p10 *= (1 - 0.5 * (k00 + k11))

        # Apply non-holonomic constraint after update
        # (tracked robot can't strafe — reduce lateral velocity)
        # Done outside the Kalman to avoid violating mathematical assumptions,
        # but it's physically correct for this robot type
        # This is applied in the AccelPositionEstimator, so skip here

    def update_zupt(self):
        """Zero-Velocity Update — robot is stationary.

        Measurement: v = [0, 0] with very high confidence.
        """
        r = self.r_zupt

        # Kalman gain
        s00 = self._p00 + r
        s11 = self._p11 + r

        k00 = self._p00 / s00 if s00 > 1e-10 else 0.0
        k11 = self._p11 / s11 if s11 > 1e-10 else 0.0

        # Update: z = [0, 0], so innovation = -[vx, vy]
        self.vx -= k00 * self.vx
        self.vy -= k11 * self.vy

        # Covariance update
        self._p00 *= (1 - k00)
        self._p11 *= (1 - k11)
        self._p01 *= (1 - 0.5 * (k00 + k11))
        self._p10 *= (1 - 0.5 * (k00 + k11))

        # Also zero the accel-integrated velocity
        self._accel_vx = 0.0
        self._accel_vy = 0.0

    def apply_nonholonomic(self, theta: float):
        """Apply non-holonomic constraint for tracked robot.

        Projects velocity onto heading direction, damping lateral component.
        """
        cy, sy = cos(theta), sin(theta)
        fwd = self.vx * cy + self.vy * sy       # forward component
        lat = -self.vx * sy + self.vy * cy       # lateral component

        lat *= self._lateral_decay

        self.vx = fwd * cy - lat * sy
        self.vy = fwd * sy + lat * cy

        # Same for accel velocity
        fwd_a = self._accel_vx * cy + self._accel_vy * sy
        lat_a = -self._accel_vx * sy + self._accel_vy * cy
        lat_a *= self._lateral_decay
        self._accel_vx = fwd_a * cy - lat_a * sy
        self._accel_vy = fwd_a * sy + lat_a * cy

    def reset(self):
        """Reset all state to zero."""
        self.vx = 0.0
        self.vy = 0.0
        self.x = 0.0
        self.y = 0.0
        self._accel_vx = 0.0
        self._accel_vy = 0.0
        self._p00 = 0.01
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = 0.01

    @property
    def velocity_magnitude(self) -> float:
        return sqrt(self.vx * self.vx + self.vy * self.vy)

    @property
    def covariance_trace(self) -> float:
        """Sum of diagonal — overall uncertainty."""
        return self._p00 + self._p11
