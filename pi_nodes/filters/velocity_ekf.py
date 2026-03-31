"""
2D Velocity EKF with accelerometer bias estimation.

Properly fuses wheel-command odometry with accelerometer data for
accurate position estimation on a tracked robot without encoders.

Architecture: two parallel 2-state Kalman filters:
  X-axis: [vx, bias_x]  in world frame
  Y-axis: [vy, bias_y]  in world frame

Key improvements over naive approaches:
  - Motor dynamics model (first-order lag) instead of instant response
  - Bias state absorbs accelerometer integration drift
  - Trapezoidal position integration (O(dt^3) error vs O(dt^2))
  - Proper Kalman gain adapts automatically to noise levels
  - No artificial velocity decay — bias estimation handles drift
  - Position integrated at every call for maximum rate

Prediction: motor command with first-order response model.
Update: accelerometer-integrated velocity (bias-compensated).
ZUPT: zero-velocity update when stationary.

No numpy required — pure Python 2x2 matrix operations.
"""

from math import cos, sin, sqrt, exp


class VelocityEKF:
    """2D velocity Kalman filter with bias estimation and position integration.

    Usage:
        ekf = VelocityEKF()

        # Prediction from wheel commands (20 Hz)
        ekf.predict_wheel(vx_cmd, theta, dt)

        # Update from accelerometer (50 Hz)
        ekf.update_accel(la_wx, la_wy, dt)

        # ZUPT when stationary
        ekf.update_zupt()

        # Read fused state
        x, y = ekf.x, ekf.y
        vx, vy = ekf.vx, ekf.vy
    """

    def __init__(self,
                 motor_tau: float = 0.25,
                 q_velocity: float = 0.08,
                 q_bias: float = 0.003,
                 r_accel: float = 0.12,
                 r_zupt: float = 0.0001):
        """
        Args:
            motor_tau: Motor time constant (seconds). How fast the robot
                       actually reaches commanded speed. Smaller = faster
                       response. 0.2-0.4s typical for tracked robots.
            q_velocity: Velocity process noise (m/s)^2/s. How much we
                        distrust the motor model (slip, friction changes).
            q_bias: Bias random walk noise (m/s)^2/s. How fast the accel
                    bias drifts. Small = bias changes slowly.
            r_accel: Accel velocity measurement noise (m/s)^2. Higher =
                     less trust in accelerometer.
            r_zupt: ZUPT measurement noise (m/s)^2. Very small = high
                    trust that stationary means zero velocity.
        """
        self.motor_tau = max(motor_tau, 0.05)
        self.q_velocity = q_velocity
        self.q_bias = q_bias
        self.r_accel = r_accel
        self.r_zupt = r_zupt

        # Velocity in world frame
        self.vx = 0.0
        self.vy = 0.0

        # Accelerometer bias in world frame (absorbs integration drift)
        self._bx = 0.0
        self._by = 0.0

        # 2x2 covariance per axis: [p_vv, p_vb, p_bv, p_bb]
        self._px = [0.01, 0.0, 0.0, 0.002]
        self._py = [0.01, 0.0, 0.0, 0.002]

        # Position (integrated from fused velocity)
        self.x = 0.0
        self.y = 0.0

        # Accel-integrated velocity (raw measurement source)
        self._accel_vx = 0.0
        self._accel_vy = 0.0

        # Non-holonomic: lateral velocity decay
        self._lateral_decay = 0.15  # aggressively kill sideways velocity

        # Bias reset counter — periodically re-center to prevent float drift
        self._update_count = 0
        self._RECENTER_INTERVAL = 500  # every ~10s at 50 Hz

    def predict_wheel(self, v_cmd: float, theta: float, dt: float):
        """Prediction step using wheel odometry command with motor dynamics.

        Models the motor as a first-order system: the robot doesn't
        instantly reach commanded speed, but converges exponentially
        with time constant motor_tau.

        Args:
            v_cmd: Commanded forward velocity (m/s), already scaled.
            theta: Current heading from IMU (radians).
            dt: Time step (seconds).
        """
        if dt <= 0 or dt > 1.0:
            return

        # -- Trapezoidal position integration (BEFORE velocity change) --
        # Uses average of current and predicted velocity
        ct, st = cos(theta), sin(theta)
        vcx = v_cmd * ct
        vcy = v_cmd * st

        # Motor response: alpha = fraction of gap closed in dt
        alpha = min(dt / self.motor_tau, 1.0)

        # Predicted velocity
        vx_new = self.vx + (vcx - self.vx) * alpha
        vy_new = self.vy + (vcy - self.vy) * alpha

        # Trapezoidal integration: x += 0.5*(v_old + v_new)*dt
        self.x += 0.5 * (self.vx + vx_new) * dt
        self.y += 0.5 * (self.vy + vy_new) * dt

        # Apply velocity prediction
        self.vx = vx_new
        self.vy = vy_new

        # Bias unchanged in prediction (random walk)

        # -- Covariance prediction: P = F*P*F' + Q --
        f_vv = 1.0 - alpha  # d(v_new)/d(v_old) = 1 - alpha
        # f_vb = 0, f_bv = 0, f_bb = 1

        qv = self.q_velocity * dt
        qb = self.q_bias * dt

        f2 = f_vv * f_vv
        p = self._px
        p[0] = f2 * p[0] + qv
        p[1] = f_vv * p[1]
        p[2] = f_vv * p[2]
        p[3] += qb

        p = self._py
        p[0] = f2 * p[0] + qv
        p[1] = f_vv * p[1]
        p[2] = f_vv * p[2]
        p[3] += qb

    def reanchor_accel(self):
        """Re-anchor accel-integrated velocity to current state estimate.

        Call when acceleration signal is below noise floor (no new info).
        Prevents stale accel velocity from creating false innovations
        at the next non-zero acceleration sample.

        After re-anchoring: accel_vx = vx + bx, so innovation = 0.
        When new acceleration arrives, innovation = integral(new_accel*dt),
        giving only the SHORT-WINDOW velocity change — drift-free.
        """
        self._accel_vx = self.vx + self._bx
        self._accel_vy = self.vy + self._by

    def update_accel(self, la_wx: float, la_wy: float, dt: float):
        """Update step using accelerometer-derived velocity measurement.

        Integrates gravity-free linear acceleration to get a velocity
        estimate, then fuses with predicted velocity via Kalman update.
        The bias state absorbs integration drift.

        IMPORTANT: Only call when acceleration is non-zero (above noise
        gate). When acceleration is noise-gated to zero, call
        reanchor_accel() instead to prevent stale measurements.

        Measurement model:
            z = accel_velocity
            h(x) = v + bias  (accel velocity = true velocity + accumulated bias)
            H = [1, 1]

        Args:
            la_wx, la_wy: Gravity-free linear acceleration in world frame (m/s^2).
            dt: Time step (seconds).
        """
        if dt <= 0 or dt > 0.5:
            return

        # Single-integrate acceleration to get velocity measurement
        self._accel_vx += la_wx * dt
        self._accel_vy += la_wy * dt

        # Innovation: z - h(x) = accel_v - (v + bias)
        innov_x = self._accel_vx - self.vx - self._bx
        innov_y = self._accel_vy - self.vy - self._by

        # Update each axis
        self._kalman_update_axis_x(innov_x, self.r_accel, full_h=True)
        self._kalman_update_axis_y(innov_y, self.r_accel, full_h=True)

        # Periodic re-centering: shift accel_v and bias together
        # to prevent float precision loss from large accumulated values
        self._update_count += 1
        if self._update_count >= self._RECENTER_INTERVAL:
            self._update_count = 0
            self._accel_vx -= self._bx
            self._accel_vy -= self._by
            self._bx = 0.0
            self._by = 0.0

    def _kalman_update_axis_x(self, innov: float, r: float,
                               full_h: bool = True):
        """Kalman update for X-axis.

        Args:
            innov: Innovation (measurement - predicted measurement).
            r: Measurement noise variance.
            full_h: True for H=[1,1] (accel), False for H=[1,0] (ZUPT).
        """
        p = self._px

        if full_h:
            # H = [1, 1]: S = p_vv + p_vb + p_bv + p_bb + R
            s = p[0] + p[1] + p[2] + p[3] + r
            if s < 1e-12:
                return
            # K = P * H' / S = [p_vv+p_vb, p_bv+p_bb]' / S
            kv = (p[0] + p[1]) / s
            kb = (p[2] + p[3]) / s
        else:
            # H = [1, 0]: S = p_vv + R
            s = p[0] + r
            if s < 1e-12:
                return
            kv = p[0] / s
            kb = p[2] / s

        # State update
        self.vx += kv * innov
        self._bx += kb * innov

        # Covariance update: P = (I - K*H) * P  — in-place
        pvv, pvb, pbv, pbb = p[0], p[1], p[2], p[3]
        if full_h:
            ikv = 1.0 - kv; ikb = 1.0 - kb
            p[0] = ikv * pvv - kv * pbv
            p[1] = ikv * pvb - kv * pbb
            p[2] = -kb * pvv + ikb * pbv
            p[3] = -kb * pvb + ikb * pbb
        else:
            ikv = 1.0 - kv
            p[0] = ikv * pvv
            p[1] = ikv * pvb
            p[2] = -kb * pvv + pbv
            p[3] = -kb * pvb + pbb

        # Enforce symmetry and positive-definiteness
        p[1] = 0.5 * (p[1] + p[2])
        p[2] = p[1]
        if p[0] < 1e-8:
            p[0] = 1e-8
        if p[3] < 1e-8:
            p[3] = 1e-8

    def _kalman_update_axis_y(self, innov: float, r: float,
                               full_h: bool = True):
        """Kalman update for Y-axis (same math as X)."""
        p = self._py

        if full_h:
            s = p[0] + p[1] + p[2] + p[3] + r
            if s < 1e-12:
                return
            kv = (p[0] + p[1]) / s
            kb = (p[2] + p[3]) / s
        else:
            s = p[0] + r
            if s < 1e-12:
                return
            kv = p[0] / s
            kb = p[2] / s

        self.vy += kv * innov
        self._by += kb * innov

        pvv, pvb, pbv, pbb = p[0], p[1], p[2], p[3]
        if full_h:
            ikv = 1.0 - kv; ikb = 1.0 - kb
            p[0] = ikv * pvv - kv * pbv
            p[1] = ikv * pvb - kv * pbb
            p[2] = -kb * pvv + ikb * pbv
            p[3] = -kb * pvb + ikb * pbb
        else:
            ikv = 1.0 - kv
            p[0] = ikv * pvv
            p[1] = ikv * pvb
            p[2] = -kb * pvv + pbv
            p[3] = -kb * pvb + pbb

        p[1] = 0.5 * (p[1] + p[2])
        p[2] = p[1]
        if p[0] < 1e-8:
            p[0] = 1e-8
        if p[3] < 1e-8:
            p[3] = 1e-8

    def update_zupt(self):
        """Zero-Velocity Update — robot is confirmed stationary.

        Measurement: v = [0, 0] with very high confidence.
        H = [1, 0] — ZUPT measures velocity only, not bias.
        Also resets accel-integrated velocity and bias to zero.
        """
        # Innovation: 0 - vx = -vx
        self._kalman_update_axis_x(-self.vx, self.r_zupt, full_h=False)
        self._kalman_update_axis_y(-self.vy, self.r_zupt, full_h=False)

        # Hard reset accel velocity and bias:
        # We know v=0, accel_v should be 0, so bias = accel_v - v = 0
        self._accel_vx = 0.0
        self._accel_vy = 0.0
        self._bx = 0.0
        self._by = 0.0
        self._update_count = 0

    def apply_nonholonomic(self, theta: float):
        """Apply non-holonomic constraint for tracked robot.

        Tracked robots cannot strafe. Projects velocity onto heading
        direction and aggressively damps the lateral component.
        Applied to both EKF velocity and accel-integrated velocity.
        """
        cy, sy = cos(theta), sin(theta)

        # Decompose into forward and lateral
        fwd = self.vx * cy + self.vy * sy
        lat = -self.vx * sy + self.vy * cy
        lat *= self._lateral_decay

        self.vx = fwd * cy - lat * sy
        self.vy = fwd * sy + lat * cy

        # Same for accel velocity (keeps measurement consistent)
        fwd_a = self._accel_vx * cy + self._accel_vy * sy
        lat_a = -self._accel_vx * sy + self._accel_vy * cy
        lat_a *= self._lateral_decay
        self._accel_vx = fwd_a * cy - lat_a * sy
        self._accel_vy = fwd_a * sy + lat_a * cy

    def reset(self):
        """Reset all state to zero."""
        self.vx = 0.0
        self.vy = 0.0
        self._bx = 0.0
        self._by = 0.0
        self.x = 0.0
        self.y = 0.0
        self._accel_vx = 0.0
        self._accel_vy = 0.0
        self._px = [0.01, 0.0, 0.0, 0.002]
        self._py = [0.01, 0.0, 0.0, 0.002]
        self._update_count = 0

    @property
    def velocity_magnitude(self) -> float:
        return sqrt(self.vx * self.vx + self.vy * self.vy)

    @property
    def covariance_trace(self) -> float:
        """Sum of velocity covariance diagonals — overall uncertainty."""
        return self._px[0] + self._py[0]

    @property
    def bias_magnitude(self) -> float:
        """Current estimated accel bias magnitude."""
        return sqrt(self._bx * self._bx + self._by * self._by)
