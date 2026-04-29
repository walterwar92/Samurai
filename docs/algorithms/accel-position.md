# Accelerometer position estimator

!!! info "Source"
    `pi_nodes/filters/accel_position.py` (~420 lines).
    Combines accelerometer, wheel-odometry, and IMU orientation into
    a single position estimate. Sits downstream of the
    [IMU EKF](imu-ekf.md) and provides the dead-reckoning fallback when
    SLAM Toolbox isn't running.

## Architecture

Three input streams are fused:

1. **Accelerometer** — instantaneous linear acceleration (50 Hz).
2. **Wheel odometry** — motor-command-derived velocity (20 Hz).
3. **IMU orientation** — Euler angles for body↔world frame rotation.

Output: world-frame position $(x, y)$ with velocity $(v_x, v_y)$.

## Physical constants

```python title="accel_position.py — constants"
EARTH_OMEGA = 7.2921e-5   # rad/s — Earth angular velocity
DEFAULT_LATITUDE = 55.75  # Moscow

ZUPT_GYRO_ENTER  = 0.015  # rad/s
ZUPT_ACCEL_ENTER = 0.10   # m/s^2
ZUPT_GYRO_EXIT   = 0.04   # rad/s
ZUPT_ACCEL_EXIT  = 0.25   # m/s^2
ZUPT_ENTER_COUNT = 5      # 100 ms @ 50 Hz
ZUPT_EXIT_COUNT  = 3      # 60 ms

LATERAL_DECAY = 0.5       # kill 50% lateral velocity per step
```

Earth's angular velocity:

$$
\Omega = 7.2921 \times 10^{-5}\,\text{rad/s}
$$

Vertical projection at latitude $\varphi$:

$$
\omega_z = \Omega \sin\varphi
$$

For Moscow ($\varphi = 55.75°$): $\omega_z \approx 6.0 \times 10^{-5}$ rad/s.

## Step 1 — Subtract gravity (calibrated body-frame)

```python title="accel_position.py — update_imu()"
# Remove gravity — DIRECT SUBTRACTION
la_bx = ax - self._g_body[0]
la_by = ay - self._g_body[1]
la_bz = az - self._g_body[2]

# Delta orientation correction (slopes only)
d_roll  = roll
d_pitch = pitch
if abs(d_roll) > 0.01 or abs(d_pitch) > 0.01:
    g = self._g_mag
    delta_gx = -g * sin(d_pitch)
    delta_gy =  g * sin(d_roll)
    la_bx -= delta_gx
    la_by -= delta_gy
```

Direct subtraction of the gravity vector calibrated at boot:

$$
\vect{a}^{\text{lin}}_{\text{body}}
= \vect{a}_{\text{meas}} - \vect{g}_{\text{body}}
$$

!!! note "Why direct subtract, not rotational?"
    The IMU is physically tilted on the chassis. At rest it might read
    $a_x = 0.707\text{ m/s}^2$ — pure gravity through the mounting
    angle. Direct subtract: $0.707 - 0.707 = 0$, no phantom motion.
    A rotational method that assumed roll = pitch = 0 would compute
    $g_{\text{body}} = (0, 0, 9.81)$ and leave that 0.707 as "motion".

Slope correction when actually tilted (>0.6°):

$$
\Delta g_x = -g\sin\theta_{\text{pitch}}, \quad
\Delta g_y =  g\sin\phi_{\text{roll}}
$$

## Step 2 — Body → world frame

```python title="accel_position.py — yaw rotation"
# Body -> World frame (rotate by yaw)
cy, sy = cos(yaw), sin(yaw)
la_wx = la_bx * cy - la_by * sy
la_wy = la_bx * sy + la_by * cy
```

Z-axis rotation only — the robot stays on a flat floor:

$$
\boxed{
\begin{pmatrix} a^{\text{world}}_x \\ a^{\text{world}}_y \end{pmatrix}
= \mat{R}(\psi)\,\begin{pmatrix} a^{\text{body}}_x \\ a^{\text{body}}_y \end{pmatrix},
\quad
\mat{R}(\psi) = \begin{pmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{pmatrix}
}
$$

## Step 3 — Coriolis compensation

```python title="accel_position.py — Coriolis"
# Earth rotation compensation (Coriolis)
la_wx -= 2.0 * self._omega_z * self.vy
la_wy += 2.0 * self._omega_z * self.vx
```

Coriolis acceleration per unit mass:

$$
\vect{a}_{\text{Cor}} = -2\,\vect{\Omega} \times \vect{v}
$$

In 2D with only the vertical Earth-rotation component
$\omega_z = \Omega\sin\varphi$:

$$
\begin{cases}
a_{Cx} = -2\,\omega_z\,v_y \\
a_{Cy} = +2\,\omega_z\,v_x
\end{cases}
$$

The correction is tiny ($10^{-5}$ rad/s × m/s) but essential for
multi-minute drift bounds.

## Step 4 — Adaptive median filter

```python title="accel_position.py — vibration rejection"
# Median filter (reject vibration spikes)
self._buf_ax.append(la_wx)
self._buf_ay.append(la_wy)

if len(self._buf_ax) >= 3:
    la_wx = _median_of_3(
        self._buf_ax[-1], self._buf_ax[-2], self._buf_ax[-3])
    la_wy = _median_of_3(
        self._buf_ay[-1], self._buf_ay[-2], self._buf_ay[-3])

# Noise threshold — zero out tiny accelerations
if abs(la_wx) < ACCEL_NOISE_THRESHOLD:   # 0.15 m/s^2
    la_wx = 0.0
```

Median of 3 (single branch — no sort needed):

$$
\hat a = \mathrm{median}(a_k,\,a_{k-1},\,a_{k-2})
$$

Plus dead-zone threshold: $|a| < 0.15\text{ m/s}^2 \Rightarrow a = 0$.

## Step 5 — ZUPT with hysteresis

```python title="accel_position.py — _update_stationary()"
gyro_mag  = sqrt(gx*gx + gy*gy + gz*gz)
accel_mag = sqrt(la_wx*la_wx + la_wy*la_wy)

if self._stationary:
    # Need STRONG evidence of motion to exit
    if (self._cmd_moving and
        (gyro_mag > ZUPT_GYRO_EXIT or       # 0.04 rad/s
         accel_mag > ZUPT_ACCEL_EXIT)):     # 0.25 m/s^2
        self._exit_count += 1
    if self._exit_count >= ZUPT_EXIT_COUNT:  # 3 samples
        self._stationary = False
else:
    # Easy to re-enter stationary
    if (not self._cmd_moving or
        (gyro_mag < ZUPT_GYRO_ENTER and     # 0.015 rad/s
         accel_mag < ZUPT_ACCEL_ENTER)):    # 0.10 m/s^2
        self._enter_count += 1
    if self._enter_count >= ZUPT_ENTER_COUNT:   # 5 samples
        self._stationary = True
```

Two-threshold state machine:

```
                 |w| > 0.04  OR  |a| > 0.25 m/s²  (3 consecutive samples)
   STATIONARY ────────────────────────────────────────► MOVING

                 |w| < 0.015  AND |a| < 0.10 m/s²  (5 consecutive samples)
   MOVING     ────────────────────────────────────────► STATIONARY
```

High exit thresholds ($> 0.04$, $> 0.25$) and low entry thresholds
($< 0.015$, $< 0.10$) prevent chattering at the boundary.

## Step 6 — Trapezoidal integration (fallback path)

```python title="accel_position.py — fallback velocity update"
# Trapezoidal integration (more accurate than Euler)
self.vx += 0.5 * (la_wx + self._prev_la_x) * dt
self.vy += 0.5 * (la_wy + self._prev_la_y) * dt

# Non-holonomic constraint
fwd = self.vx * cy + self.vy * sy
lat = -self.vx * sy + self.vy * cy
lat *= LATERAL_DECAY
self.vx = fwd * cy - lat * sy
self.vy = fwd * sy + lat * cy

# Position integration
self._accel_x += self.vx * dt
self._accel_y += self.vy * dt
```

Trapezoidal rule (order $O(\Delta t^2)$, vs Euler's $O(\Delta t)$):

$$
v_k = v_{k-1} + \frac{\Delta t}{2}\bigl(a_k + a_{k-1}\bigr)
$$

## Step 7 — Complementary filter

```python title="accel_position.py — blend()"
# Complementary filter: alpha*wheel + (1-alpha)*accel
self._alpha = 0.95 * self._alpha + 0.05 * self._alpha_moving
a = self._alpha
self.x = a * self._wheel_x + (1.0 - a) * self._accel_x
self.y = a * self._wheel_y + (1.0 - a) * self._accel_y

# Slow drift correction of accel toward wheel
self._accel_x = 0.995 * self._accel_x + 0.005 * self._wheel_x
self._accel_y = 0.995 * self._accel_y + 0.005 * self._wheel_y
```

Adaptive complementary filter:

$$
\vect{p} = \alpha\,\vect{p}_{\text{wheel}} + (1 - \alpha)\,\vect{p}_{\text{accel}}
$$

with $\alpha \in [0.95, 1.0]$ (when moving the wheel-odometry weight
grows). Slow drift correction pulls the accelerometer-derived position
toward wheel odometry at 0.5%/tick — bounds the integration drift
without trusting wheels too much during track slippage.

## Why this still exists alongside Velocity EKF

The [Velocity EKF](velocity-ekf.md) is the primary fusion path
post-#51. AccelPosition stays as a separate observer because:

- It runs at 50 Hz (one update per accelerometer sample) vs
  the EKF's 20 Hz.
- It exposes adaptive $\alpha$ and ZUPT-with-hysteresis, useful when
  SLAM Toolbox isn't available and the EKF's bounded velocity isn't
  enough on its own.
- It's the single source of "linear_accel_world" feeding into
  motor_node's IMU-push detector.

## Theory — complementary filter as steady-state Kalman

Rewriting the complementary filter:

$$
\hat x = \alpha\,x_{\text{odom}} + (1 - \alpha)\,x_{\text{accel}}
       = x_{\text{accel}} + \alpha\,(x_{\text{odom}} - x_{\text{accel}})
$$

That's the Kalman update form $\hat x = \hat x^- + K\,y$ with
$\alpha$ playing the role of the gain $K$ and $y = x_{\text{odom}} -
x_{\text{accel}}$ as the innovation. The complementary filter is a
Kalman filter with its gain frozen at $\alpha$.

## Theory — frequency interpretation

In the frequency domain the complementary filter is two filters
running in parallel:

$$
X(p) = \underbrace{\frac{\alpha}{1 + \alpha/p}}_{\text{LPF}}\,X_{\text{odom}}(p)
     + \underbrace{\frac{p/\alpha}{1 + p/\alpha}}_{\text{HPF}}\,X_{\text{accel}}(p)
$$

Cutoff: $f_c = \alpha / (2\pi\,\Delta t) \approx 0.95 / (2\pi \cdot 0.02)
\approx 7.6\text{ Hz}$.

Below 7.6 Hz: trust wheel odometry (drift-stable).
Above 7.6 Hz: trust accelerometer (high-frequency motion sensitive).

## Trapezoidal vs Euler integration

| Method | Formula | Per-step error |
|---|---|---|
| Euler forward | $v_k = v_{k-1} + a_k\,\Delta t$ | $O(\Delta t)$ |
| Trapezoidal   | $v_k = v_{k-1} + \tfrac{a_k + a_{k-1}}{2}\,\Delta t$ | $O(\Delta t^2)$ |

At 50 Hz ($\Delta t = 0.02$ s), trapezoidal is ~100× more accurate
than Euler for smooth signals.

## Discrete state-space form

$$
\begin{pmatrix} x \\ y \\ v_x \\ v_y \end{pmatrix}_k
= \begin{pmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
\begin{pmatrix} x \\ y \\ v_x \\ v_y \end{pmatrix}_{k-1}
+
\begin{pmatrix}
\Delta t^2/2 & 0 \\
0 & \Delta t^2/2 \\
\Delta t & 0 \\
0 & \Delta t
\end{pmatrix}
\begin{pmatrix} a_x \\ a_y \end{pmatrix}
$$

## Pipeline summary

```
accelerometer raw
    │
    ▼
[1] subtract gravity (calibrated body-frame)
    │
    ▼
[2] body → world rotation R(ψ)
    │
    ▼
[3] Coriolis compensation
    │
    ▼
[4] median-of-3 + dead-zone
    │
    ▼
[5] ZUPT with hysteresis
    │
    ▼
[6] trapezoidal integration → velocity
    │
    ▼
[7] complementary filter wheel↔accel → position (x, y)
```
