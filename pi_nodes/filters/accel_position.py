"""
IMU-only position estimator for 2D ground robot.

Position estimation is based EXCLUSIVELY on IMU sensor data
(accelerometer + gyroscope). Motor commands are NOT used — this
prevents false motion when e.g. tracks spin while robot is lifted.

Extracts linear acceleration from IMU by removing:
    1. Gravity vector — direct subtraction of calibrated gravity_body,
       with full rotation correction for orientation changes
    2. Earth rotation effects (Coriolis)
    3. Vibration noise — cascaded IIR low-pass + median spike rejection

Velocity estimation via EKF:
    - Prediction: friction decay model (velocity → 0, no motor commands)
    - Update: from accelerometer (only source of real motion data)
    - Bias estimation: absorbs accelerometer drift automatically
    - ZUPT: zero velocity when stationary (sensor-based, no cmd dependency)
    - Non-holonomic constraint: tracked robot can't strafe

Coordinate frames:
    Body:  x=forward, y=left, z=up (as mounted on chassis)
    World: x=forward, y=left (2D ground plane, z ignored)
"""

from math import sin, cos, sqrt, pi, atan2
from collections import deque
import time

try:
    from pi_nodes.filters.velocity_ekf import VelocityEKF
    _VEKF_AVAILABLE = True
except ImportError:
    _VEKF_AVAILABLE = False

# ── Earth rotation constants ──────────────────────────────────────
EARTH_OMEGA = 7.2921e-5   # rad/s — Earth's angular velocity
DEFAULT_LATITUDE = 55.75   # Moscow
DEG2RAD = pi / 180.0

# ── Filter constants ─────────────────────────────────────────────
MEDIAN_WINDOW = 5
ACCEL_NOISE_THRESHOLD = 0.12   # m/s² — below = noise
COMPLEMENTARY_ALPHA = 0.98

# ── ZUPT with hysteresis ─────────────────────────────────────────
ZUPT_GYRO_ENTER = 0.015
ZUPT_ACCEL_ENTER = 0.10
ZUPT_GYRO_EXIT = 0.04
ZUPT_ACCEL_EXIT = 0.25
ZUPT_ENTER_COUNT = 3
ZUPT_EXIT_COUNT = 3

# ── Velocity / lateral decay ─────────────────────────────────────
VELOCITY_DECAY = 0.95
LATERAL_DECAY = 0.5


def _median_of_3(a, b, c):
    """Fast median of 3 values."""
    if a <= b:
        if b <= c:
            return b
        return c if c >= a else a
    if a <= c:
        return a
    return c if c >= b else b


class _LowPass2:
    """2nd-order IIR low-pass filter (cascaded 1st-order stages).

    For alpha=0.4 at 50 Hz: each stage cutoff ~3.2 Hz,
    cascaded effective cutoff ~2.2 Hz. Removes motor vibrations
    while preserving intentional motion signals (~0-5 Hz).
    """
    __slots__ = ('_a', '_s1', '_s2')

    def __init__(self, alpha: float = 0.4):
        self._a = alpha
        self._s1 = 0.0
        self._s2 = 0.0

    def update(self, x: float) -> float:
        self._s1 += self._a * (x - self._s1)
        self._s2 += self._a * (self._s1 - self._s2)
        return self._s2

    def reset(self, val: float = 0.0):
        self._s1 = val
        self._s2 = val


class AccelPositionEstimator:
    """IMU-only position estimator for 2D ground robot.

    Position estimation based EXCLUSIVELY on IMU sensor data.
    Motor commands are NOT used — prevents false motion detection
    (e.g. tracks spinning while robot is lifted off the ground).

    Uses VelocityEKF with bias estimation and friction decay model.
    Falls back to pure accelerometer integration if EKF not available.

    Key features:
    - 2nd-order IIR low-pass filter removes vibration noise
    - Median filter rejects impulse spikes
    - Actual timestamp-based dt (no hardcoded values)
    - Bias estimation in EKF absorbs accelerometer drift
    - Friction decay model (velocity → 0 without acceleration)
    - Sensor-only ZUPT (no motor command dependency)

    Usage:
        estimator = AccelPositionEstimator()
        estimator.set_calibration(gravity_body, home_roll, home_pitch)

        # Each IMU tick (50 Hz) — pass actual timestamp:
        estimator.update_imu(ax, ay, az, gx, gy, gz,
                             roll, pitch, yaw, dt, timestamp)

        # Each prediction tick (20 Hz):
        estimator.update_prediction(theta, dt)

        # Finalize:
        estimator.blend()

        # Read:
        x, y = estimator.x, estimator.y
    """

    def __init__(self, latitude_deg: float = DEFAULT_LATITUDE,
                 complementary_alpha: float = COMPLEMENTARY_ALPHA,
                 lpf_alpha: float = 0.4,
                 vekf_params: dict = None):
        """
        Args:
            latitude_deg: Latitude for Coriolis correction.
            complementary_alpha: Fallback blend ratio (without EKF).
            lpf_alpha: IIR low-pass filter alpha.
            vekf_params: Dict of VelocityEKF parameters:
                friction_tau, q_velocity, q_bias, r_accel, r_zupt.
                If None, uses defaults from config or hardcoded.
        """
        self._lat_rad = latitude_deg * DEG2RAD
        self._alpha_moving = complementary_alpha
        self._alpha = 1.0

        # Earth rotation at this latitude
        self._omega_z = EARTH_OMEGA * sin(self._lat_rad)

        # Calibration
        self._g_body = (0.0, 0.0, -9.81)
        self._g_mag = 9.81
        self._home_roll = 0.0
        self._home_pitch = 0.0

        # State
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # Velocity EKF (preferred) — with configurable parameters
        self._vekf = None
        if _VEKF_AVAILABLE:
            kw = vekf_params or {}
            self._vekf = VelocityEKF(**kw)

        # Accel-only position (fallback)
        self._accel_x = 0.0
        self._accel_y = 0.0

        # Frozen position — locked when stationary
        self._frozen_x = 0.0
        self._frozen_y = 0.0

        # -- Filtering pipeline --
        # Stage 1: Median filter for spike rejection
        self._buf_ax = deque(maxlen=MEDIAN_WINDOW)
        self._buf_ay = deque(maxlen=MEDIAN_WINDOW)

        # Stage 2: 2nd-order IIR low-pass for vibration removal
        self._lpf_x = _LowPass2(lpf_alpha)
        self._lpf_y = _LowPass2(lpf_alpha)

        # Previous linear accel for trapezoidal integration (fallback)
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0

        # Latest filtered accel for publishing
        self._last_la_wx = 0.0
        self._last_la_wy = 0.0

        # -- ZUPT with hysteresis --
        self._stationary = True
        self._enter_count = ZUPT_ENTER_COUNT
        self._exit_count = 0

        # Current yaw + cached trig (avoid cos/sin at 50 Hz when yaw changes slowly)
        self._current_yaw = 0.0
        self._cached_yaw = -999.0  # force first computation
        self._cy = 1.0
        self._sy = 0.0

        # Gyro magnitude for ZUPT (stored from latest IMU)
        self._last_gyro_mag = 0.0

        # -- IMU timestamp tracking --
        self._last_imu_ts = None

        # -- Blend dt: last dt from update_prediction for position integration --
        self._last_blend_dt = 0.0

    def set_calibration(self, gravity_body: tuple,
                        home_roll: float, home_pitch: float):
        """Set calibration from imu_node's startup phase."""
        self._g_body = gravity_body
        self._g_mag = sqrt(gravity_body[0]**2 + gravity_body[1]**2 +
                           gravity_body[2]**2)
        self._home_roll = home_roll
        self._home_pitch = home_pitch

    def update_imu(self, ax: float, ay: float, az: float,
                   gx: float, gy: float, gz: float,
                   roll: float, pitch: float, yaw: float,
                   dt: float, timestamp: float = 0.0):
        """Process one IMU sample.

        Args:
            ax, ay, az: Filtered accelerometer (m/s^2).
            gx, gy, gz: Filtered gyroscope (rad/s, bias-corrected).
            roll, pitch, yaw: Current orientation from EKF (radians).
            dt: Time step (seconds). Used as fallback if timestamp=0.
            timestamp: Actual IMU timestamp (seconds). If provided,
                       dt is computed from consecutive timestamps for
                       more accurate integration.
        """
        # -- Compute actual dt from timestamps when available --
        if timestamp > 0:
            if self._last_imu_ts is not None and self._last_imu_ts > 0:
                actual_dt = timestamp - self._last_imu_ts
                # Sanity check: allow 0.5ms to 200ms
                if 0.0005 < actual_dt < 0.2:
                    dt = actual_dt
            self._last_imu_ts = timestamp

        if dt <= 0:
            return

        self._current_yaw = yaw

        # ── 1. Remove gravity — DIRECT SUBTRACTION ───────────────
        la_bx = ax - self._g_body[0]
        la_by = ay - self._g_body[1]
        # la_bz = az - self._g_body[2]  # not used for 2D

        # Orientation correction: compensate gravity projection change
        # when robot tilts from home orientation
        d_roll = roll
        d_pitch = pitch

        if abs(d_roll) > 0.005 or abs(d_pitch) > 0.005:  # >0.3°
            g = self._g_mag
            # Full trigonometric correction (not small-angle)
            delta_gx = -g * sin(d_pitch)
            delta_gy = g * sin(d_roll) * cos(d_pitch)
            la_bx -= delta_gx
            la_by -= delta_gy

        # ── 2. Body → World frame (rotate by yaw) ────────────────
        # Cache cos/sin — yaw changes slowly, saves ~100 trig calls/sec
        if abs(yaw - self._cached_yaw) > 0.001:
            self._cached_yaw = yaw
            self._cy = cos(yaw)
            self._sy = sin(yaw)
        cy, sy = self._cy, self._sy
        la_wx = la_bx * cy - la_by * sy
        la_wy = la_bx * sy + la_by * cy

        # ── 3. Earth rotation compensation (Coriolis) ─────────────
        la_wx -= 2.0 * self._omega_z * self.vy
        la_wy += 2.0 * self._omega_z * self.vx

        # ── 4. Median filter — reject impulse spikes ─────────────
        self._buf_ax.append(la_wx)
        self._buf_ay.append(la_wy)

        if len(self._buf_ax) >= 3:
            la_wx = _median_of_3(
                self._buf_ax[-1], self._buf_ax[-2], self._buf_ax[-3])
            la_wy = _median_of_3(
                self._buf_ay[-1], self._buf_ay[-2], self._buf_ay[-3])

        # ── 5. 2nd-order IIR low-pass — remove vibrations ────────
        la_wx = self._lpf_x.update(la_wx)
        la_wy = self._lpf_y.update(la_wy)

        # ── 6. Noise gate — zero out sub-threshold signals ────────
        accel_mag = sqrt(la_wx * la_wx + la_wy * la_wy)
        if accel_mag < ACCEL_NOISE_THRESHOLD:
            la_wx = 0.0
            la_wy = 0.0
            accel_mag = 0.0

        # Store for publishing
        self._last_la_wx = la_wx
        self._last_la_wy = la_wy

        # ── 7. ZUPT with HYSTERESIS ──────────────────────────────
        gyro_mag = sqrt(gx * gx + gy * gy + gz * gz)
        self._last_gyro_mag = gyro_mag

        if self._stationary:
            # Currently stationary — need SENSOR evidence to exit.
            # Pure IMU-based: only gyro or accel above thresholds.
            # Motor commands have NO influence on this decision.
            sensor_evidence = (gyro_mag > ZUPT_GYRO_EXIT or
                               accel_mag > ZUPT_ACCEL_EXIT)

            if sensor_evidence:
                self._exit_count += 1
                self._enter_count = 0
            else:
                self._exit_count = 0

            if self._exit_count >= ZUPT_EXIT_COUNT:
                self._stationary = False
                self._exit_count = 0
                self._frozen_x = self.x
                self._frozen_y = self.y
        else:
            # Currently moving — check if robot has stopped.
            # Pure sensor-based: gyro + accel below thresholds.
            sensors_quiet = (gyro_mag < ZUPT_GYRO_ENTER and
                             accel_mag < ZUPT_ACCEL_ENTER)

            if sensors_quiet:
                self._enter_count += 1
                self._exit_count = 0
            else:
                # Sensors noisy (vibrations after stop). Soft decrement
                # instead of hard reset — forgive occasional spikes.
                self._enter_count = max(0, self._enter_count - 1)
                self._exit_count = 0

            # Safety: if EKF velocity is negligible and gyro quiet,
            # force ZUPT even if accel has residual vibration noise.
            ekf_nearly_stopped = False
            if self._vekf is not None:
                ekf_nearly_stopped = (self._vekf.velocity_magnitude < 0.01
                                      and gyro_mag < ZUPT_GYRO_ENTER)

            if self._enter_count >= ZUPT_ENTER_COUNT or ekf_nearly_stopped:
                self._stationary = True
                self._enter_count = ZUPT_ENTER_COUNT
                self._frozen_x = self.x
                self._frozen_y = self.y

        if self._stationary:
            # Hard zero — no velocity, position frozen
            self.vx = 0.0
            self.vy = 0.0
            self._prev_la_x = 0.0
            self._prev_la_y = 0.0
            self.x = self._frozen_x
            self.y = self._frozen_y
            self._accel_x = self._frozen_x
            self._accel_y = self._frozen_y
            # ZUPT in velocity EKF
            if self._vekf is not None:
                self._vekf.update_zupt()
                self._vekf.x = self._frozen_x
                self._vekf.y = self._frozen_y
            # Reset LP filter to zero when stopped
            self._lpf_x.reset(0.0)
            self._lpf_y.reset(0.0)
            return

        # ── 8. Feed to Velocity EKF or fallback ──────────────────
        if self._vekf is not None:
            if accel_mag > 0:
                # Meaningful accel signal → Kalman update
                self._vekf.update_accel(la_wx, la_wy, dt)
            else:
                # No accel signal → re-anchor to prevent stale innovations
                # When new acceleration appears, only short-window integral
                # is used as innovation — no long-term drift
                self._vekf.reanchor_accel()
            # Apply non-holonomic constraint
            self._vekf.apply_nonholonomic(yaw)
        else:
            # Fallback: trapezoidal integration + decay
            self.vx += 0.5 * (la_wx + self._prev_la_x) * dt
            self.vy += 0.5 * (la_wy + self._prev_la_y) * dt

            # Non-holonomic constraint
            fwd = self.vx * cy + self.vy * sy
            lat = -self.vx * sy + self.vy * cy
            lat *= LATERAL_DECAY
            self.vx = fwd * cy - lat * sy
            self.vy = fwd * sy + lat * cy

            # Velocity decay
            self.vx *= VELOCITY_DECAY
            self.vy *= VELOCITY_DECAY

            self._prev_la_x = la_wx
            self._prev_la_y = la_wy

            # Position integration (fallback)
            self._accel_x += self.vx * dt
            self._accel_y += self.vy * dt

    def update_prediction(self, theta: float, dt: float):
        """Run EKF prediction step (friction model, no motor commands).

        Position estimation is based solely on IMU sensor data.
        Motor commands are NOT used — this prevents false motion
        detection when e.g. tracks spin while robot is lifted.

        Args:
            theta: Current heading from IMU yaw (radians).
            dt: Time step (seconds).
        """
        if dt <= 0:
            return

        # Store dt for blend() position integration
        self._last_blend_dt = dt

        if self._vekf is not None:
            # Velocity EKF: friction-based prediction.
            # Position is NOT integrated here — it's done in blend()
            # after all updates, using the fused velocity.
            self._vekf.predict_friction(dt)
        else:
            # Fallback: simple velocity decay (friction)
            self.vx *= VELOCITY_DECAY
            self.vy *= VELOCITY_DECAY

    def blend(self):
        """Finalize position estimate for this cycle.

        With VelocityEKF: integrates position from fused velocity AFTER
        all predict+update steps. This ensures position uses the fully
        corrected velocity (IMU-corrected, not just motor prediction).
        Without: pure accelerometer position integration.
        """
        if self._stationary:
            self._alpha = 1.0
            return

        if self._vekf is not None:
            # Integrate position from FUSED velocity (after Kalman correction).
            # dt = last IMU dt (most frequent update source, ~50 Hz).
            # This is the key change: position now reflects accelerometer
            # corrections, not just motor command predictions.
            dt = self._last_blend_dt
            if dt > 0:
                self._vekf.integrate_position(dt)
            self.x = self._vekf.x
            self.y = self._vekf.y
            self.vx = self._vekf.vx
            self.vy = self._vekf.vy
        else:
            # Fallback: pure accelerometer position (no wheel odom)
            self.x = self._accel_x
            self.y = self._accel_y

    def reset(self):
        """Reset position to (0, 0) — accept current pose as new home."""
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self._accel_x = 0.0
        self._accel_y = 0.0
        self._frozen_x = 0.0
        self._frozen_y = 0.0
        self._prev_la_x = 0.0
        self._prev_la_y = 0.0
        self._last_la_wx = 0.0
        self._last_la_wy = 0.0
        self._stationary = True
        self._enter_count = ZUPT_ENTER_COUNT
        self._exit_count = 0
        self._alpha = 1.0
        self._buf_ax.clear()
        self._buf_ay.clear()
        self._lpf_x.reset(0.0)
        self._lpf_y.reset(0.0)
        self._last_imu_ts = None
        self._last_blend_dt = 0.0
        if self._vekf is not None:
            self._vekf.reset()

    @property
    def is_stationary(self) -> bool:
        return self._stationary

    @property
    def linear_accel_world(self) -> tuple:
        """Latest gravity-free linear acceleration in world frame."""
        return (self._last_la_wx, self._last_la_wy)
