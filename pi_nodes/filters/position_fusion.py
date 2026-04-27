"""
PositionFusion — runtime-switchable fusion of wheel and IMU position estimates.

Each tick the motor_node feeds in two independent estimates:

    wheel  : cmd_vel × calibration scale, integrated with IMU yaw
             — believes commanded velocity, ignores blocked motion / slip /
               external pushes
    imu    : AccelPositionEstimator (accel-only, ZUPT, EKF on velocity)
             — believes physical motion, drifts when accel signal is weak

Fusion modes (selectable at runtime via MQTT):

    'wheel'         passthrough wheel (baseline / debug)
    'imu'           passthrough IMU (test isolation)
    'complementary' x = α·wheel + (1-α)·imu  — simple weighted blend
    'ekf'           1-D Kalman per axis, both estimates as measurements

The EKF mode is the principled answer: process model is constant-velocity
random walk, two measurements per axis (wheel position and IMU position +
IMU velocity). Measurement noises encode each source's trustworthiness:
wheel is precise but biased (R_wheel small but mean ≠ truth under slip);
IMU is unbiased but noisy (R_imu larger). Kalman picks the right blend
adaptively as the innovation history evolves.

Usage:

    fusion = PositionFusion(mode='complementary', alpha=0.7)
    fusion.set_mode('ekf')  # switch at runtime, no state loss

    # Each motor tick:
    x_fused, y_fused = fusion.update(
        x_wheel=..., y_wheel=...,
        x_imu=..., y_imu=...,
        vx_imu=..., vy_imu=...,
        dt=...,
    )

    fusion.reset(x=0, y=0)  # also forwards to internal EKF state
"""
from __future__ import annotations

from typing import Tuple

VALID_MODES = ('wheel', 'imu', 'complementary', 'ekf')


class _Kalman1D:
    """Constant-velocity 1-D Kalman filter for one axis (x or y).

    State:        [pos, vel]ᵀ
    Process:      pos' = pos + vel·dt;  vel' = vel  (+ process noise)
    Measurements: z_wheel = pos       (noise R_wheel)
                  z_imu   = pos       (noise R_imu)
                  z_imu_v = vel       (noise R_imu_v)

    Two position measurements per tick are fused sequentially (mathematically
    equivalent to a 2-element joint update for diagonal R).
    """
    __slots__ = ('p', 'v',
                 'P00', 'P01', 'P10', 'P11',
                 'q_pos', 'q_vel',
                 'r_wheel', 'r_imu', 'r_imu_v')

    def __init__(self, q_pos: float, q_vel: float,
                 r_wheel: float, r_imu: float, r_imu_v: float):
        self.p = 0.0
        self.v = 0.0
        # Covariance matrix [[P00, P01], [P10, P11]] — start with moderate
        # uncertainty so the first measurement pulls state strongly.
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_wheel = r_wheel
        self.r_imu = r_imu
        self.r_imu_v = r_imu_v

    def predict(self, dt: float) -> None:
        # x' = F x with F = [[1, dt], [0, 1]]
        self.p += self.v * dt
        # P' = F P Fᵀ + Q
        # F P:
        a = self.P00 + dt * self.P10
        b = self.P01 + dt * self.P11
        # (F P) Fᵀ:
        new_P00 = a + dt * b
        new_P01 = b
        new_P10 = self.P10 + dt * self.P11
        new_P11 = self.P11
        # + Q (diagonal — process noise on pos and vel)
        self.P00 = new_P00 + self.q_pos
        self.P01 = new_P01
        self.P10 = new_P10
        self.P11 = new_P11 + self.q_vel

    def _update_pos(self, z: float, r: float) -> None:
        """Scalar position measurement: H = [1, 0], R = r."""
        # innovation
        y = z - self.p
        # S = H P Hᵀ + R = P00 + r
        s = self.P00 + r
        if s <= 0:
            return
        # K = P Hᵀ / S = [P00, P10]ᵀ / s
        k0 = self.P00 / s
        k1 = self.P10 / s
        # state update
        self.p += k0 * y
        self.v += k1 * y
        # P = (I - K H) P
        new_P00 = (1.0 - k0) * self.P00
        new_P01 = (1.0 - k0) * self.P01
        new_P10 = self.P10 - k1 * self.P00
        new_P11 = self.P11 - k1 * self.P01
        self.P00 = new_P00
        self.P01 = new_P01
        self.P10 = new_P10
        self.P11 = new_P11

    def _update_vel(self, z: float, r: float) -> None:
        """Scalar velocity measurement: H = [0, 1], R = r."""
        y = z - self.v
        s = self.P11 + r
        if s <= 0:
            return
        k0 = self.P01 / s
        k1 = self.P11 / s
        self.p += k0 * y
        self.v += k1 * y
        new_P00 = self.P00 - k0 * self.P10
        new_P01 = self.P01 - k0 * self.P11
        new_P10 = (1.0 - k1) * self.P10
        new_P11 = (1.0 - k1) * self.P11
        self.P00 = new_P00
        self.P01 = new_P01
        self.P10 = new_P10
        self.P11 = new_P11

    def update_wheel(self, z: float) -> None:
        self._update_pos(z, self.r_wheel)

    def update_imu(self, z: float) -> None:
        self._update_pos(z, self.r_imu)

    def update_imu_velocity(self, z: float) -> None:
        self._update_vel(z, self.r_imu_v)

    def reset(self, p: float = 0.0, v: float = 0.0) -> None:
        self.p = p
        self.v = v
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0


class PositionFusion:
    """Runtime-switchable fusion between wheel-odom and IMU-position.

    Holds an EKF per axis even when in 'wheel'/'imu'/'complementary' mode —
    so switching to 'ekf' at runtime doesn't start from zero. The non-EKF
    modes still keep the EKF state in sync via the same measurements, just
    don't use its output.
    """

    __slots__ = ('_mode', '_alpha',
                 '_kf_x', '_kf_y',
                 '_x_fused', '_y_fused')

    def __init__(self, mode: str = 'wheel',
                 alpha: float = 0.7,
                 q_pos: float = 0.05,
                 q_vel: float = 0.10,
                 r_wheel: float = 0.04,
                 r_imu: float = 0.20,
                 r_imu_v: float = 0.10):
        """
        Args:
            mode: One of VALID_MODES. Picks the published x, y.
            alpha: Complementary blend weight for wheel (0..1).
                   1.0 = wheel only, 0.0 = IMU only.
            q_pos, q_vel: EKF process noise (per axis, per tick).
            r_wheel: EKF wheel position measurement noise variance (m²).
            r_imu:   EKF IMU position measurement noise variance (m²).
                     Should be > r_wheel: IMU position drifts, wheel is
                     locally precise (just biased on slip).
            r_imu_v: EKF IMU velocity measurement noise variance ((m/s)²).
        """
        self._mode = self._validate_mode(mode)
        self._alpha = self._clamp_alpha(alpha)
        self._kf_x = _Kalman1D(q_pos, q_vel, r_wheel, r_imu, r_imu_v)
        self._kf_y = _Kalman1D(q_pos, q_vel, r_wheel, r_imu, r_imu_v)
        self._x_fused = 0.0
        self._y_fused = 0.0

    @staticmethod
    def _validate_mode(mode: str) -> str:
        if mode not in VALID_MODES:
            raise ValueError(f'Unknown fusion mode: {mode!r}. '
                             f'Expected one of {VALID_MODES}')
        return mode

    @staticmethod
    def _clamp_alpha(alpha: float) -> float:
        if alpha < 0.0:
            return 0.0
        if alpha > 1.0:
            return 1.0
        return alpha

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def alpha(self) -> float:
        return self._alpha

    def set_mode(self, mode: str) -> bool:
        """Switch fusion mode at runtime. Returns True if mode changed."""
        new_mode = self._validate_mode(mode)
        if new_mode == self._mode:
            return False
        self._mode = new_mode
        return True

    def set_alpha(self, alpha: float) -> None:
        """Adjust complementary alpha at runtime."""
        self._alpha = self._clamp_alpha(alpha)

    def update(self,
               x_wheel: float, y_wheel: float,
               x_imu: float, y_imu: float,
               vx_imu: float, vy_imu: float,
               dt: float) -> Tuple[float, float]:
        """Run one fusion step. Returns (x_fused, y_fused) in metres.

        EKF state is updated in every mode so a runtime switch picks up
        from a sensible state, not from (0, 0).
        """
        if dt > 0:
            self._kf_x.predict(dt)
            self._kf_y.predict(dt)
        # Always feed measurements into the EKF — even in non-EKF modes —
        # so set_mode('ekf') hands over a warm filter, not a cold start.
        self._kf_x.update_wheel(x_wheel)
        self._kf_y.update_wheel(y_wheel)
        self._kf_x.update_imu(x_imu)
        self._kf_y.update_imu(y_imu)
        self._kf_x.update_imu_velocity(vx_imu)
        self._kf_y.update_imu_velocity(vy_imu)

        if self._mode == 'wheel':
            self._x_fused = x_wheel
            self._y_fused = y_wheel
        elif self._mode == 'imu':
            self._x_fused = x_imu
            self._y_fused = y_imu
        elif self._mode == 'complementary':
            a = self._alpha
            self._x_fused = a * x_wheel + (1.0 - a) * x_imu
            self._y_fused = a * y_wheel + (1.0 - a) * y_imu
        else:  # 'ekf'
            self._x_fused = self._kf_x.p
            self._y_fused = self._kf_y.p

        return self._x_fused, self._y_fused

    def reset(self, x: float = 0.0, y: float = 0.0) -> None:
        """Reset fused position. Used when motor_node receives reset_position."""
        self._x_fused = x
        self._y_fused = y
        self._kf_x.reset(x, 0.0)
        self._kf_y.reset(y, 0.0)
