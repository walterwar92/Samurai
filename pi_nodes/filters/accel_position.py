"""
Accelerometer-based position estimator for 2D ground robot.

Extracts linear acceleration from IMU by removing:
    1. Gravity vector — DIRECT SUBTRACTION of calibrated gravity_body,
       with delta-rotation correction for orientation changes from home
    2. Earth rotation effects (Coriolis)
    3. Vibration noise (adaptive threshold + median filter)

Then double-integrates to get velocity and position, with:
    - ZUPT (Zero-Velocity Update): resets velocity when stationary
    - Complementary filter: blends accel position (high-freq) with
      wheel odometry (low-freq) to prevent integration drift
    - Trapezoidal integration for accuracy

Why direct subtraction:
    The IMU is physically tilted on the chassis. At rest, it reads e.g.
    ax=0.707, ay=-0.085, az=-9.309 — all gravity, zero real motion.
    The old rotation-based method computed gravity assuming roll=pitch=0
    (home), giving g_body=(0,0,9.81), leaving ax=0.707 as "motion".
    Direct subtraction: 0.707 - 0.707 = 0.  Simple, correct, robust.

Coordinate frames:
    Body:  x=forward, y=left, z=up (as mounted on chassis)
    World: x=forward, y=left (2D ground plane, z ignored)
"""

from math import sin, cos, sqrt, pi
from collections import deque

# ── Earth rotation constants ──────────────────────────────────────
EARTH_OMEGA = 7.2921e-5   # rad/s — Earth's angular velocity
DEFAULT_LATITUDE = 55.75   # Moscow — override in config if needed
DEG2RAD = pi / 180.0

# ── Filter constants ─────────────────────────────────────────────
MEDIAN_WINDOW = 5          # samples for median filter
ACCEL_NOISE_THRESHOLD = 0.15   # m/s² — below this = noise, not motion
ZUPT_GYRO_THRESHOLD = 0.02    # rad/s — below = stationary
ZUPT_ACCEL_THRESHOLD = 0.12   # m/s² — linear accel below = stationary
COMPLEMENTARY_ALPHA = 0.98    # 0.98 = 98% wheel odom, 2% accel (long-term)

# Velocity decay — exponential damping to fight integration drift
# 0.98 per step @ 50Hz → velocity halves in ~1.7s without acceleration
VELOCITY_DECAY = 0.98


def _median_of_3(a, b, c):
    """Fast median of 3 values (no sorting needed)."""
    if a <= b:
        if b <= c:
            return b
        return c if c >= a else a
    if a <= c:
        return a
    return c if c >= b else b


class AccelPositionEstimator:
    """Fuses accelerometer with wheel odometry for accurate 2D position.

    Usage:
        estimator = AccelPositionEstimator()
        estimator.set_calibration(gravity_body, home_roll, home_pitch)

        # Each IMU tick (50 Hz):
        estimator.update_imu(ax, ay, az, gx, gy, gz, roll, pitch, yaw, dt)

        # Each odom tick (20 Hz):
        estimator.update_wheel_odom(vx_wheel, theta, dt)

        # Read fused position:
        x, y = estimator.x, estimator.y
    """

    def __init__(self, latitude_deg: float = DEFAULT_LATITUDE,
                 complementary_alpha: float = COMPLEMENTARY_ALPHA):
        self._lat_rad = latitude_deg * DEG2RAD
        self._alpha = complementary_alpha

        # Earth rotation at this latitude
        self._omega_z = EARTH_OMEGA * sin(self._lat_rad)

        # Calibration: gravity vector in body frame at rest
        # This is DIRECTLY subtracted from each measurement.
        self._g_body = (0.0, 0.0, -9.81)
        self._g_mag = 9.81
        self._home_roll = 0.0
        self._home_pitch = 0.0

        # State
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # Wheel odometry position (for complementary blend)
        self._wheel_x = 0.0
        self._wheel_y = 0.0

        # Accel-only position (before blending)
        self._accel_x = 0.0
        self._accel_y = 0.0

        # Median filter buffers
        self._buf_ax = deque(maxlen=MEDIAN_WINDOW)
        self._buf_ay = deque(maxlen=MEDIAN_WINDOW)

        # Previous linear accel for trapezoidal integration
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0

        # Stationary detection
        self._stationary = True
        self._stationary_count = 0

    def set_calibration(self, gravity_body: tuple,
                        home_roll: float, home_pitch: float):
        """Set calibration from imu_node's startup phase.

        Args:
            gravity_body: (ax, ay, az) average at rest — full gravity+tilt
                          vector in body frame. Will be subtracted directly.
            home_roll, home_pitch: Initial orientation (radians).
        """
        self._g_body = gravity_body
        self._g_mag = sqrt(gravity_body[0]**2 + gravity_body[1]**2 +
                           gravity_body[2]**2)
        self._home_roll = home_roll
        self._home_pitch = home_pitch

    def update_imu(self, ax: float, ay: float, az: float,
                   gx: float, gy: float, gz: float,
                   roll: float, pitch: float, yaw: float,
                   dt: float):
        """Process one IMU sample.

        Args:
            ax, ay, az: Filtered accelerometer (m/s²).
            gx, gy, gz: Filtered gyroscope (rad/s, bias-corrected).
            roll, pitch, yaw: Current orientation from filter (radians,
                              relative to home).
            dt: Time step (seconds).
        """
        if dt <= 0:
            return

        # ── 1. Remove gravity — DIRECT SUBTRACTION ───────────────
        #
        # At calibration the IMU recorded gravity_body = (gx, gy, gz).
        # If orientation hasn't changed from home, measured accel ≈ gravity_body,
        # so subtraction gives ~0.  Perfect.
        #
        # If robot tilted since home (delta_roll, delta_pitch), gravity in body
        # frame changes.  We compute a small correction for that delta.
        #
        # For a ground robot on flat surface, delta_roll ≈ delta_pitch ≈ 0,
        # so the correction is negligible — but we include it for accuracy.

        # Base subtraction (handles static tilt of IMU mounting)
        la_bx = ax - self._g_body[0]
        la_by = ay - self._g_body[1]
        la_bz = az - self._g_body[2]

        # Delta orientation correction (only significant on slopes)
        # roll/pitch are relative to home, so they represent the change
        d_roll = roll    # roll - 0 = roll (home is 0)
        d_pitch = pitch  # pitch - 0 = pitch

        if abs(d_roll) > 0.01 or abs(d_pitch) > 0.01:  # >0.6°
            # How gravity changes in body frame due to tilt delta:
            # Δg_x ≈ -g·sin(d_pitch)  (tilting forward shifts gravity into X)
            # Δg_y ≈  g·sin(d_roll)   (tilting sideways shifts gravity into Y)
            g = self._g_mag
            delta_gx = -g * sin(d_pitch)
            delta_gy = g * sin(d_roll)
            la_bx -= delta_gx
            la_by -= delta_gy

        # ── 2. Body → World frame (rotate by yaw) ────────────────
        cy, sy = cos(yaw), sin(yaw)
        la_wx = la_bx * cy - la_by * sy
        la_wy = la_bx * sy + la_by * cy

        # ── 3. Earth rotation compensation (Coriolis) ─────────────
        # a_cor_x = 2·Ω_z·vy,  a_cor_y = -2·Ω_z·vx
        la_wx -= 2.0 * self._omega_z * self.vy
        la_wy -= -2.0 * self._omega_z * self.vx

        # ── 4. Median filter — reject vibration spikes ────────────
        self._buf_ax.append(la_wx)
        self._buf_ay.append(la_wy)

        if len(self._buf_ax) >= 3:
            la_wx = _median_of_3(
                self._buf_ax[-1], self._buf_ax[-2], self._buf_ax[-3])
            la_wy = _median_of_3(
                self._buf_ay[-1], self._buf_ay[-2], self._buf_ay[-3])

        # ── 5. Noise threshold — zero out tiny accelerations ──────
        if abs(la_wx) < ACCEL_NOISE_THRESHOLD:
            la_wx = 0.0
        if abs(la_wy) < ACCEL_NOISE_THRESHOLD:
            la_wy = 0.0

        # ── 6. ZUPT — Zero-Velocity Update ───────────────────────
        gyro_mag = sqrt(gx * gx + gy * gy + gz * gz)
        accel_mag = sqrt(la_wx * la_wx + la_wy * la_wy)

        if gyro_mag < ZUPT_GYRO_THRESHOLD and accel_mag < ZUPT_ACCEL_THRESHOLD:
            self._stationary_count += 1
        else:
            self._stationary_count = 0

        # 5+ consecutive stationary samples (100ms @ 50Hz) to trigger
        self._stationary = (self._stationary_count >= 5)

        if self._stationary:
            self.vx = 0.0
            self.vy = 0.0
            self._prev_la_x = 0.0
            self._prev_la_y = 0.0
            return

        # ── 7. Trapezoidal integration: accel → velocity ─────────
        self.vx += 0.5 * (la_wx + self._prev_la_x) * dt
        self.vy += 0.5 * (la_wy + self._prev_la_y) * dt

        # Velocity decay — exponential damping to fight drift
        # Without this, tiny accel residuals accumulate into velocity
        self.vx *= VELOCITY_DECAY
        self.vy *= VELOCITY_DECAY

        self._prev_la_x = la_wx
        self._prev_la_y = la_wy

        # ── 8. Trapezoidal integration: velocity → position ──────
        self._accel_x += self.vx * dt
        self._accel_y += self.vy * dt

    def update_wheel_odom(self, vx_wheel: float, theta: float, dt: float):
        """Integrate wheel odometry (for complementary blend).

        Args:
            vx_wheel: Forward velocity from motor commands (m/s).
            theta: Current heading from IMU yaw (radians).
            dt: Time step (seconds).
        """
        if dt <= 0 or abs(vx_wheel) < 0.001:
            return

        self._wheel_x += vx_wheel * cos(theta) * dt
        self._wheel_y += vx_wheel * sin(theta) * dt

    def blend(self):
        """Complementary filter: merge wheel odom + accel position.

        High alpha (0.98) = trust wheel odometry long-term (prevents drift),
        but accel corrects short-term (detects slip, bumps).
        """
        a = self._alpha
        self.x = a * self._wheel_x + (1.0 - a) * self._accel_x
        self.y = a * self._wheel_y + (1.0 - a) * self._accel_y

        # Pull accel estimate toward wheel estimate to prevent drift
        self._accel_x = 0.995 * self._accel_x + 0.005 * self._wheel_x
        self._accel_y = 0.995 * self._accel_y + 0.005 * self._wheel_y

    def reset(self):
        """Reset position to (0, 0) — accept current pose as new home."""
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self._wheel_x = 0.0
        self._wheel_y = 0.0
        self._accel_x = 0.0
        self._accel_y = 0.0
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0
        self._stationary = True
        self._stationary_count = 0
        self._buf_ax.clear()
        self._buf_ay.clear()

    @property
    def is_stationary(self) -> bool:
        return self._stationary

    @property
    def linear_accel_world(self) -> tuple:
        """Latest gravity-free linear acceleration in world frame."""
        return (self._prev_la_x, self._prev_la_y)
