"""
Accelerometer-based position estimator for 2D ground robot.

Extracts linear acceleration from IMU by removing:
    1. Gravity vector (rotated by current orientation)
    2. Earth rotation effects (Coriolis + centripetal)
    3. Vibration noise (adaptive threshold + median filter)

Then double-integrates to get velocity and position, with:
    - ZUPT (Zero-Velocity Update): resets velocity when stationary
    - Complementary filter: blends accel position (high-freq) with
      wheel odometry (low-freq) to prevent integration drift
    - Trapezoidal integration for accuracy

Coordinate frames:
    Body:  x=forward, y=left, z=up (IMU-mounted)
    World: x=forward, y=left (2D ground plane, z ignored)
"""

from math import sin, cos, sqrt, pi, atan2
from collections import deque

# ── Earth rotation constants ──────────────────────────────────────
EARTH_OMEGA = 7.2921e-5   # rad/s — Earth's angular velocity
DEFAULT_LATITUDE = 55.75   # Moscow — override in config if needed
DEG2RAD = pi / 180.0

# ── Filter constants ─────────────────────────────────────────────
MEDIAN_WINDOW = 5          # samples for median filter
ACCEL_NOISE_THRESHOLD = 0.05   # m/s² — below this = noise, not motion
ZUPT_GYRO_THRESHOLD = 0.015    # rad/s — below = stationary
ZUPT_ACCEL_THRESHOLD = 0.08    # m/s² — linear accel below = stationary
COMPLEMENTARY_ALPHA = 0.98     # 0.98 = 98% wheel odom, 2% accel (long-term)


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
        vx, vy = estimator.vx, estimator.vy
    """

    def __init__(self, latitude_deg: float = DEFAULT_LATITUDE,
                 complementary_alpha: float = COMPLEMENTARY_ALPHA):
        """
        Args:
            latitude_deg: Robot's latitude for Earth rotation compensation.
            complementary_alpha: Blend factor (0..1). Higher = more trust
                in wheel odometry (long-term stable), lower = more trust
                in accelerometer (short-term responsive).
        """
        self._lat_rad = latitude_deg * DEG2RAD
        self._alpha = complementary_alpha

        # Earth rotation at this latitude
        self._omega_z = EARTH_OMEGA * sin(self._lat_rad)   # vertical component
        self._omega_y = EARTH_OMEGA * cos(self._lat_rad)   # horizontal (north)

        # Calibration: gravity vector in body frame at rest
        self._g_body = (0.0, 0.0, 9.81)  # default, updated from calibration
        self._g_mag = 9.81

        # State
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0   # velocity in world X (forward)
        self.vy = 0.0   # velocity in world Y (left)

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
            gravity_body: (ax, ay, az) average at rest — full gravity vector
                          in body frame.
            home_roll, home_pitch: Initial orientation (radians).
        """
        self._g_body = gravity_body
        self._g_mag = sqrt(gravity_body[0]**2 + gravity_body[1]**2 +
                           gravity_body[2]**2)

    def update_imu(self, ax: float, ay: float, az: float,
                   gx: float, gy: float, gz: float,
                   roll: float, pitch: float, yaw: float,
                   dt: float):
        """Process one IMU sample.

        All angles in radians. ax/ay/az in m/s². gx/gy/gz in rad/s.
        roll/pitch/yaw from the yaw filter (relative to home).

        Args:
            ax, ay, az: Filtered accelerometer (m/s²).
            gx, gy, gz: Filtered gyroscope (rad/s, bias-corrected).
            roll, pitch, yaw: Current orientation from EKF (radians).
            dt: Time step (seconds).
        """
        if dt <= 0:
            return

        # ── 1. Remove gravity ─────────────────────────────────────
        # Rotate gravity vector from world frame to body frame
        # using current orientation, then subtract
        sr, cr = sin(roll), cos(roll)
        sp, cp = sin(pitch), cos(pitch)
        sy, cy = sin(yaw), cos(yaw)

        # Gravity in world frame = (0, 0, g)
        # Rotate to body frame using ZYX rotation matrix transpose
        g = self._g_mag
        g_body_x = -g * sp
        g_body_y =  g * sr * cp
        g_body_z =  g * cr * cp

        # Linear acceleration in body frame
        la_bx = ax - g_body_x
        la_by = ay - g_body_y
        la_bz = az - g_body_z  # not used for 2D, but computed for completeness

        # ── 2. Earth rotation compensation ────────────────────────
        # Coriolis: a_cor = -2 * Ω × v (in body frame)
        # For ground robot, mainly affects North-South motion
        # Ω in world frame ≈ (0, ω_y, ω_z) where ω_y=Ω·cos(lat), ω_z=Ω·sin(lat)
        #
        # World-frame velocity (approximate from current estimates)
        vx_w = self.vx
        vy_w = self.vy

        # Coriolis acceleration in world frame:
        # a_cor = -2 * (Ω × v) = -2 * |  i    j    k  |
        #                              |  0   ω_y  ω_z |
        #                              | vx   vy    0  |
        # a_cor_x = -2 * (ω_y * 0 - ω_z * vy) = 2 * ω_z * vy
        # a_cor_y = -2 * (ω_z * vx - 0) = -2 * ω_z * vx
        cor_x = 2.0 * self._omega_z * vy_w
        cor_y = -2.0 * self._omega_z * vx_w

        # ── 3. Body → World frame rotation ────────────────────────
        # For 2D: rotate body (x,y) by yaw to get world (x,y)
        la_wx = la_bx * cy - la_by * sy
        la_wy = la_bx * sy + la_by * cy

        # Subtract Earth rotation effects (in world frame)
        la_wx -= cor_x
        la_wy -= cor_y

        # ── 4. Median filter — reject vibration spikes ───────────
        self._buf_ax.append(la_wx)
        self._buf_ay.append(la_wy)

        if len(self._buf_ax) >= 3:
            la_wx = _median_of_3(
                self._buf_ax[-1], self._buf_ax[-2], self._buf_ax[-3])
            la_wy = _median_of_3(
                self._buf_ay[-1], self._buf_ay[-2], self._buf_ay[-3])

        # ── 5. Adaptive noise threshold ───────────────────────────
        # If acceleration is below noise floor, zero it out
        if abs(la_wx) < ACCEL_NOISE_THRESHOLD:
            la_wx = 0.0
        if abs(la_wy) < ACCEL_NOISE_THRESHOLD:
            la_wy = 0.0

        # ── 6. ZUPT — Zero-Velocity Update ───────────────────────
        # Detect stationary state from gyro + accel magnitude
        gyro_mag = sqrt(gx * gx + gy * gy + gz * gz)
        accel_mag = sqrt(la_wx * la_wx + la_wy * la_wy)

        if gyro_mag < ZUPT_GYRO_THRESHOLD and accel_mag < ZUPT_ACCEL_THRESHOLD:
            self._stationary_count += 1
        else:
            self._stationary_count = 0

        # Need 5+ consecutive stationary samples (100ms @ 50Hz) to trigger ZUPT
        self._stationary = (self._stationary_count >= 5)

        if self._stationary:
            # Robot is not moving — reset velocity drift
            self.vx = 0.0
            self.vy = 0.0
            la_wx = 0.0
            la_wy = 0.0
            self._prev_la_x = 0.0
            self._prev_la_y = 0.0
            return

        # ── 7. Trapezoidal integration: accel → velocity ─────────
        self.vx += 0.5 * (la_wx + self._prev_la_x) * dt
        self.vy += 0.5 * (la_wy + self._prev_la_y) * dt

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

        Call this after update_imu and update_wheel_odom in each cycle.

        High alpha (0.98) = trust wheel odometry long-term (prevents drift),
        but accel corrects short-term (detects slip, bumps).
        """
        a = self._alpha
        self.x = a * self._wheel_x + (1.0 - a) * self._accel_x
        self.y = a * self._wheel_y + (1.0 - a) * self._accel_y

        # Slowly pull accel estimate toward wheel estimate
        # to prevent unbounded drift
        self._accel_x = 0.999 * self._accel_x + 0.001 * self._wheel_x
        self._accel_y = 0.999 * self._accel_y + 0.001 * self._wheel_y

    @property
    def is_stationary(self) -> bool:
        return self._stationary

    @property
    def linear_accel_world(self) -> tuple:
        """Latest gravity-free linear acceleration in world frame."""
        return (self._prev_la_x, self._prev_la_y)
