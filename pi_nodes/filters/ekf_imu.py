"""
Extended Kalman Filter for 6-axis IMU (accelerometer + gyroscope).

State vector (6×1):
    x = [φ, θ, ψ, b_gx, b_gy, b_gz]ᵀ
    φ = roll,  θ = pitch,  ψ = yaw         (radians)
    b_gx, b_gy, b_gz = gyro bias estimates  (rad/s)

Prediction:  gyroscope integration via Euler rate equations
Correction:  accelerometer gravity reference (roll & pitch only)
Yaw:         gyro integration only (drifts without magnetometer)
"""

import numpy as np
from math import sin, cos, tan, sqrt, atan2, pi


class EkfImu:
    def __init__(self, q_angle: float = 0.001, q_bias: float = 0.0001,
                 r_accel: float = 0.5, accel_gate: float = 0.5,
                 g: float = 9.81):
        """
        Args:
            q_angle:    Process noise variance for angles (rad²).
            q_bias:     Process noise variance for gyro bias random walk.
            r_accel:    Measurement noise variance for accelerometer (m/s²)².
            accel_gate: Skip accel update if |‖a‖ − g| > gate·g.
            g:          Gravitational acceleration (m/s²).
        """
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_accel = r_accel
        self.accel_gate = accel_gate
        self.g = g
        self.reset()

    def reset(self):
        """Reset state to zeros, covariance to initial values."""
        self.x = np.zeros((6, 1))
        self.P = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

    # ── Prediction (gyroscope) ────────────────────────────────────────

    def predict(self, gx: float, gy: float, gz: float, dt: float):
        """Predict step from gyroscope readings.

        Args:
            gx, gy, gz: Raw gyro (rad/s).
            dt:         Time step (seconds).
        """
        if dt <= 0:
            return

        phi   = self.x[0, 0]
        theta = self.x[1, 0]
        bgx   = self.x[3, 0]
        bgy   = self.x[4, 0]
        bgz   = self.x[5, 0]

        # Bias-corrected gyro
        wx = gx - bgx
        wy = gy - bgy
        wz = gz - bgz

        sp, cp = sin(phi), cos(phi)
        ct = cos(theta)

        # Gimbal lock guard (pitch near ±90°)
        if abs(ct) < 1e-6:
            ct = 1e-6 if ct >= 0 else -1e-6
        tt = tan(theta)

        # Euler rate equations  (body rates → Euler angle rates)
        phi_dot   = wx + wy * sp * tt + wz * cp * tt
        theta_dot = wy * cp - wz * sp
        psi_dot   = (wy * sp + wz * cp) / ct

        # State prediction
        self.x[0, 0] += phi_dot   * dt
        self.x[1, 0] += theta_dot * dt
        self.x[2, 0] += psi_dot   * dt
        # biases: random walk → unchanged in prediction

        # Normalize yaw to [−π, π]
        self.x[2, 0] = (self.x[2, 0] + pi) % (2 * pi) - pi

        # ── Jacobian F = I + (∂f/∂x) · dt ─────────────────────────
        F = np.eye(6)

        # ∂(φ̇)/∂φ  = wy·cos(φ)·tan(θ) − wz·sin(φ)·tan(θ)
        F[0, 0] += (wy * cp * tt - wz * sp * tt) * dt
        # ∂(φ̇)/∂θ  = (wy·sin(φ) + wz·cos(φ)) / cos²(θ)
        F[0, 1] += (wy * sp + wz * cp) / (ct * ct) * dt
        # ∂(φ̇)/∂b_gx = −1
        F[0, 3] += -1.0 * dt
        # ∂(φ̇)/∂b_gy = −sin(φ)·tan(θ)
        F[0, 4] += -sp * tt * dt
        # ∂(φ̇)/∂b_gz = −cos(φ)·tan(θ)
        F[0, 5] += -cp * tt * dt

        # ∂(θ̇)/∂φ  = −wy·sin(φ) − wz·cos(φ)
        F[1, 0] += (-wy * sp - wz * cp) * dt
        # ∂(θ̇)/∂b_gy = −cos(φ)
        F[1, 4] += -cp * dt
        # ∂(θ̇)/∂b_gz = sin(φ)
        F[1, 5] += sp * dt

        # ∂(ψ̇)/∂φ  = (wy·cos(φ) − wz·sin(φ)) / cos(θ)
        F[2, 0] += (wy * cp - wz * sp) / ct * dt
        # ∂(ψ̇)/∂θ  = (wy·sin(φ) + wz·cos(φ)) · tan(θ) / cos(θ)
        F[2, 1] += (wy * sp + wz * cp) * tt / ct * dt
        # ∂(ψ̇)/∂b_gy = −sin(φ)/cos(θ)
        F[2, 4] += -sp / ct * dt
        # ∂(ψ̇)/∂b_gz = −cos(φ)/cos(θ)
        F[2, 5] += -cp / ct * dt

        # Process noise Q  (scaled by dt)
        Q = np.diag([
            self.q_angle, self.q_angle, self.q_angle,
            self.q_bias,  self.q_bias,  self.q_bias,
        ]) * dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + Q

    # ── Update (accelerometer) ────────────────────────────────────────

    def update(self, ax: float, ay: float, az: float):
        """Correction step from accelerometer (gravity reference).

        Corrects roll and pitch only. Yaw is unobservable from accel.
        Skipped when acceleration magnitude deviates significantly from g
        (robot is accelerating, not just measuring gravity).

        Args:
            ax, ay, az: Raw accelerometer (m/s²).
        """
        # Accel gate: skip if not near free-fall/gravity
        a_mag = sqrt(ax * ax + ay * ay + az * az)
        if abs(a_mag - self.g) > self.accel_gate * self.g:
            return

        phi   = self.x[0, 0]
        theta = self.x[1, 0]
        sp, cp = sin(phi), cos(phi)
        st, ct = sin(theta), cos(theta)

        # Predicted measurement: h(x) = expected accel from gravity
        h = np.array([
            [-self.g * st],
            [ self.g * sp * ct],
            [ self.g * cp * ct],
        ])

        # Actual measurement
        z = np.array([[ax], [ay], [az]])

        # Innovation
        y = z - h

        # Jacobian H (3×6):  ∂h/∂x
        H = np.zeros((3, 6))
        H[0, 1] = -self.g * ct           # ∂h₀/∂θ
        H[1, 0] =  self.g * cp * ct      # ∂h₁/∂φ
        H[1, 1] = -self.g * sp * st      # ∂h₁/∂θ
        H[2, 0] = -self.g * sp * ct      # ∂h₂/∂φ
        H[2, 1] = -self.g * cp * st      # ∂h₂/∂θ
        # All partials wrt ψ and biases are zero

        # Measurement noise
        R = np.eye(3) * self.r_accel

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        # Normalize yaw
        self.x[2, 0] = (self.x[2, 0] + pi) % (2 * pi) - pi

    # ── Accessors ─────────────────────────────────────────────────────

    @property
    def roll(self) -> float:
        return float(self.x[0, 0])

    @property
    def pitch(self) -> float:
        return float(self.x[1, 0])

    @property
    def yaw(self) -> float:
        return float(self.x[2, 0])

    @property
    def gyro_bias(self) -> tuple:
        return (float(self.x[3, 0]), float(self.x[4, 0]), float(self.x[5, 0]))

    def get_euler_deg(self) -> tuple:
        """Return (roll, pitch, yaw) in degrees."""
        r2d = 180.0 / pi
        return (self.roll * r2d, self.pitch * r2d, self.yaw * r2d)
