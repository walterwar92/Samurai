"""
Accelerometer-based position estimator for 2D ground robot.

Extracts linear acceleration from IMU by removing:
    1. Gravity vector — DIRECT SUBTRACTION of calibrated gravity_body,
       with delta-rotation correction for orientation changes from home
    2. Earth rotation effects (Coriolis)
    3. Vibration noise (adaptive threshold + median filter)

Then double-integrates to get velocity and position, with:
    - Adaptive ZUPT with hysteresis: harder to exit stationary state
    - Non-holonomic constraint: tracked robot cannot move sideways
    - Adaptive complementary alpha: 100% wheel when stationary
    - Velocity decay: exponential damping fights integration drift
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
COMPLEMENTARY_ALPHA = 0.98    # 0.98 = 98% wheel odom, 2% accel (long-term)

# ── ZUPT (Zero-Velocity Update) with hysteresis ──────────────────
# Two thresholds: ENTER (strict) and EXIT (loose).
# Robot must clearly be moving to exit stationary state.
ZUPT_GYRO_ENTER = 0.015       # rad/s — below = entering stationary
ZUPT_ACCEL_ENTER = 0.10       # m/s² — below = entering stationary
ZUPT_GYRO_EXIT = 0.04         # rad/s — above = exiting stationary (higher!)
ZUPT_ACCEL_EXIT = 0.25        # m/s² — above = exiting stationary (higher!)
ZUPT_ENTER_COUNT = 5          # consecutive samples to enter stationary (100ms @ 50Hz)
ZUPT_EXIT_COUNT = 3           # consecutive samples to exit stationary (60ms @ 50Hz)

# Velocity decay — exponential damping to fight integration drift
# 0.95 per step @ 50Hz → velocity halves in ~0.7s without acceleration
# More aggressive than before (was 0.98) to kill drift faster
VELOCITY_DECAY = 0.95

# Non-holonomic constraint: tracked robot lateral velocity damping
# Lateral (sideways) velocity is physically impossible for tracked robot,
# so we aggressively damp it.  1.0 = no damping, 0.0 = instant kill.
LATERAL_DECAY = 0.5   # kill 50% of lateral velocity per step


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

    Key improvements over naive double-integration:
    - Adaptive ZUPT with hysteresis prevents oscillation at motion boundary
    - Non-holonomic constraint zeros lateral velocity (tracked robot can't strafe)
    - Adaptive alpha: stationary → 100% wheel, moving → configurable blend
    - Position freezing: when stationary, position output is locked

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
        self._alpha_moving = complementary_alpha  # alpha when moving
        self._alpha = 1.0  # current alpha (starts stationary = 100% wheel)

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

        # Frozen position — locked when stationary to prevent micro-drift
        self._frozen_x = 0.0
        self._frozen_y = 0.0

        # Median filter buffers
        self._buf_ax = deque(maxlen=MEDIAN_WINDOW)
        self._buf_ay = deque(maxlen=MEDIAN_WINDOW)

        # Previous linear accel for trapezoidal integration
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0

        # Stationary detection with hysteresis
        self._stationary = True
        self._enter_count = ZUPT_ENTER_COUNT  # start as stationary
        self._exit_count = 0

        # Current yaw for non-holonomic constraint
        self._current_yaw = 0.0

        # Command-based motion detection (from motor_node)
        self._cmd_moving = False

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

    def set_cmd_moving(self, is_moving: bool):
        """Hint from motor_node: whether motor commands are non-zero.

        This provides a reliable "ground truth" for motion detection:
        if no commands are being sent, the robot CANNOT be moving
        (barring external forces).
        """
        self._cmd_moving = is_moving

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

        self._current_yaw = yaw

        # ── 1. Remove gravity — DIRECT SUBTRACTION ───────────────
        la_bx = ax - self._g_body[0]
        la_by = ay - self._g_body[1]
        la_bz = az - self._g_body[2]

        # Delta orientation correction (only significant on slopes)
        d_roll = roll
        d_pitch = pitch

        if abs(d_roll) > 0.01 or abs(d_pitch) > 0.01:  # >0.6°
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
        la_wx -= 2.0 * self._omega_z * self.vy
        la_wy += 2.0 * self._omega_z * self.vx

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

        # ── 6. ZUPT with HYSTERESIS ──────────────────────────────
        gyro_mag = sqrt(gx * gx + gy * gy + gz * gz)
        accel_mag = sqrt(la_wx * la_wx + la_wy * la_wy)

        if self._stationary:
            # Currently stationary — need STRONG evidence of motion to exit.
            # Both gyro AND accel must exceed EXIT thresholds,
            # AND motor commands must be active.
            if (self._cmd_moving and
                (gyro_mag > ZUPT_GYRO_EXIT or accel_mag > ZUPT_ACCEL_EXIT)):
                self._exit_count += 1
                self._enter_count = 0
            else:
                self._exit_count = 0

            if self._exit_count >= ZUPT_EXIT_COUNT:
                self._stationary = False
                self._exit_count = 0
                # Snapshot frozen position at motion start
                self._frozen_x = self.x
                self._frozen_y = self.y
        else:
            # Currently moving — easy to re-enter stationary.
            # If no motor commands OR sensors show stillness → stationary.
            if (not self._cmd_moving or
                (gyro_mag < ZUPT_GYRO_ENTER and accel_mag < ZUPT_ACCEL_ENTER)):
                self._enter_count += 1
                self._exit_count = 0
            else:
                self._enter_count = 0

            if self._enter_count >= ZUPT_ENTER_COUNT:
                self._stationary = True
                self._enter_count = ZUPT_ENTER_COUNT
                # Freeze position — lock it so noise can't move it
                self._frozen_x = self.x
                self._frozen_y = self.y

        if self._stationary:
            # Hard zero — no velocity, no integration, position frozen
            self.vx = 0.0
            self.vy = 0.0
            self._prev_la_x = 0.0
            self._prev_la_y = 0.0
            # Keep position at frozen values
            self.x = self._frozen_x
            self.y = self._frozen_y
            self._accel_x = self._frozen_x
            self._accel_y = self._frozen_y
            self._wheel_x = self._frozen_x
            self._wheel_y = self._frozen_y
            return

        # ── 7. Trapezoidal integration: accel → velocity ─────────
        self.vx += 0.5 * (la_wx + self._prev_la_x) * dt
        self.vy += 0.5 * (la_wy + self._prev_la_y) * dt

        # ── 8. Non-holonomic constraint ───────────────────────────
        # Tracked robot cannot strafe. Decompose velocity into
        # forward (along heading) and lateral (perpendicular).
        # Aggressively damp lateral component.
        fwd = self.vx * cy + self.vy * sy      # forward component
        lat = -self.vx * sy + self.vy * cy      # lateral component

        lat *= LATERAL_DECAY  # kill lateral velocity

        # Recompose world-frame velocity
        self.vx = fwd * cy - lat * sy
        self.vy = fwd * sy + lat * cy

        # ── 9. Velocity decay — exponential damping ──────────────
        self.vx *= VELOCITY_DECAY
        self.vy *= VELOCITY_DECAY

        self._prev_la_x = la_wx
        self._prev_la_y = la_wy

        # ── 10. Trapezoidal integration: velocity → position ─────
        self._accel_x += self.vx * dt
        self._accel_y += self.vy * dt

    def update_wheel_odom(self, vx_wheel: float, theta: float, dt: float):
        """Integrate wheel odometry (for complementary blend).

        Args:
            vx_wheel: Forward velocity from motor commands (m/s).
            theta: Current heading from IMU yaw (radians).
            dt: Time step (seconds).
        """
        if dt <= 0:
            return

        # Update command-based motion hint
        self.set_cmd_moving(abs(vx_wheel) > 0.005)

        if abs(vx_wheel) < 0.001:
            return

        self._wheel_x += vx_wheel * cos(theta) * dt
        self._wheel_y += vx_wheel * sin(theta) * dt

    def blend(self):
        """Complementary filter: merge wheel odom + accel position.

        Adaptive alpha:
        - Stationary: alpha=1.0 (100% wheel — no accel drift)
        - Moving: alpha=configured (e.g. 0.98 = 98% wheel, 2% accel)

        When stationary, position is frozen — blend is a no-op.
        """
        if self._stationary:
            # Position already frozen in update_imu
            self._alpha = 1.0
            return

        # Smoothly transition alpha when motion starts/stops
        # Ramp from 1.0 toward moving alpha over time
        self._alpha = 0.95 * self._alpha + 0.05 * self._alpha_moving

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
        self._frozen_x = 0.0
        self._frozen_y = 0.0
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0
        self._stationary = True
        self._enter_count = ZUPT_ENTER_COUNT
        self._exit_count = 0
        self._cmd_moving = False
        self._alpha = 1.0
        self._buf_ax.clear()
        self._buf_ay.clear()

    @property
    def is_stationary(self) -> bool:
        return self._stationary

    @property
    def linear_accel_world(self) -> tuple:
        """Latest gravity-free linear acceleration in world frame."""
        return (self._prev_la_x, self._prev_la_y)
