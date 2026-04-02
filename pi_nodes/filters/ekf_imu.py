"""
IMU filter for 2D ground robot (tracked chassis).

Designed for a robot that moves on a flat surface — NOT in 3D.
Replaces the full 3D Euler EKF which was unstable due to IMU mounting
orientation causing roll≈±180° and yaw divergence.

Approach:
    - Yaw:   1D Kalman filter on gz gyro integration + bias tracking.
             ZUPT (Zero-velocity Update): if |gz| < threshold, skip integration.
    - Roll/Pitch: direct from accelerometer (atan2), relative to home position.
             No filtering needed — informational only for a ground robot.

All angles start at 0 on boot (home position).
"""

from math import atan2, sqrt, pi, degrees


class EkfImu:
    """2D ground-robot IMU filter.

    Keeps the same class name/interface for backward compatibility
    with imu_node.py, but internally uses a simple 1D yaw filter
    instead of the fragile 6-state 3D EKF.
    """

    def __init__(self, q_angle: float = 0.001, q_bias: float = 0.0001,
                 r_accel: float = 0.5, accel_gate: float = 0.3,
                 g: float = 9.81):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_accel = r_accel
        self.accel_gate = accel_gate
        self.g = g

        # Yaw 1D Kalman state: [yaw, gyro_z_bias]
        self._yaw = 0.0
        self._gz_bias = 0.0
        # 2x2 covariance for [yaw, bias]
        self._P_yaw = 0.1
        self._P_bias = 0.01
        self._P_cross = 0.0

        # Home orientation (subtracted from accel-derived angles)
        self._home_roll = 0.0
        self._home_pitch = 0.0

        # Latest accel-derived roll/pitch (relative to home)
        self._roll = 0.0
        self._pitch = 0.0

        # ZUPT threshold: ignore gyro below this (rad/s)
        # Prevents drift from noise when stationary
        self._zupt_threshold = 0.01  # ~0.6 °/s

    def reset(self):
        self._yaw = 0.0
        self._gz_bias = 0.0
        self._P_yaw = 0.1
        self._P_bias = 0.01
        self._P_cross = 0.0
        self._roll = 0.0
        self._pitch = 0.0

    def init_from_calibration(self, roll: float, pitch: float, yaw: float,
                              gyro_bias: tuple):
        """Initialize from calibration data.

        Args:
            roll, pitch: Home orientation from accelerometer (radians).
                         Will be subtracted so filtered output starts at 0.
            yaw: Initial yaw (always 0 — no magnetometer).
            gyro_bias: (bx, by, bz) in rad/s from static calibration.
        """
        self._home_roll = roll
        self._home_pitch = pitch
        self._yaw = yaw
        # imu_node already subtracts gyro_offset before calling predict(),
        # so EKF bias starts at 0 — it will track residual drift only.
        self._gz_bias = 0.0
        # Low initial covariance — we trust calibration
        self._P_yaw = 0.001
        self._P_bias = 0.0001
        self._P_cross = 0.0

    def predict(self, gx: float, gy: float, gz: float, dt: float):
        """Predict step: integrate gyro Z for yaw.

        Args:
            gx, gy, gz: Bias-corrected gyro (rad/s). Note: gx/gy ignored
                        for yaw — only gz used. They are in the signature
                        for API compatibility.
            dt: Time step (seconds).
        """
        if dt <= 0:
            return

        # Bias-corrected gz
        wz = gz - self._gz_bias

        # ZUPT: if angular rate is tiny, robot is stationary — don't integrate
        if abs(wz) < self._zupt_threshold:
            # Still update covariance (uncertainty grows slowly)
            self._P_yaw += self.q_angle * dt * 0.1  # much slower growth
            self._P_bias += self.q_bias * dt
            return

        # Yaw prediction: simple integration
        self._yaw += wz * dt

        # Normalize to [-π, π]
        self._yaw = (self._yaw + pi) % (2 * pi) - pi

        # Covariance prediction (2x2 manual, no numpy needed)
        # F = [[1, -dt], [0, 1]]
        # P = F @ P @ F.T + Q
        p11 = self._P_yaw
        p12 = self._P_cross
        p22 = self._P_bias

        self._P_yaw = p11 + (-dt) * p12 + (-dt) * (p12 + (-dt) * p22) + self.q_angle * dt
        self._P_cross = p12 + (-dt) * p22
        self._P_bias = p22 + self.q_bias * dt

    def update(self, ax: float, ay: float, az: float):
        """Update step: compute roll/pitch from accelerometer.

        For a ground robot, roll/pitch are derived directly from gravity.
        Yaw cannot be corrected from accelerometer (no magnetometer).

        Accel gate: skip if acceleration magnitude is far from g
        (robot is accelerating, not just gravity).
        """
        a_mag = sqrt(ax * ax + ay * ay + az * az)
        if abs(a_mag - self.g) > self.accel_gate * self.g:
            return

        # Raw roll/pitch from gravity vector
        raw_roll = atan2(ay, az)
        raw_pitch = atan2(-ax, sqrt(ay * ay + az * az))

        # Subtract home position → angles relative to startup
        self._roll = raw_roll - self._home_roll
        self._pitch = raw_pitch - self._home_pitch

        # Normalize to [-π, π]
        self._roll = (self._roll + pi) % (2 * pi) - pi
        self._pitch = (self._pitch + pi) % (2 * pi) - pi

    # ── Accessors (same interface as old EKF) ─────────────────────

    @property
    def roll(self) -> float:
        return self._roll

    @property
    def pitch(self) -> float:
        return self._pitch

    @property
    def yaw(self) -> float:
        return self._yaw

    @property
    def gyro_bias(self) -> tuple:
        """Return (0, 0, gz_bias) — only Z bias is tracked."""
        return (0.0, 0.0, self._gz_bias)

    def get_euler_deg(self) -> tuple:
        """Return (roll, pitch, yaw) in degrees."""
        return (degrees(self._roll), degrees(self._pitch), degrees(self._yaw))
