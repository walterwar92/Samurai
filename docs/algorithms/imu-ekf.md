# IMU EKF — yaw + gyro-bias Kalman filter

!!! info "Source"
    `pi_nodes/filters/ekf_imu.py` (~170 lines).
    Tested in `tests/test_ekf_imu.py` (12 cases — see #51).

A ground robot drives on a flat surface, so we care primarily about three
Euler angles:

- **Yaw $\psi$** — rotation around the vertical axis. Primary nav signal.
- **Roll $\phi$, pitch $\theta$** — informational (slope detection).

Two parallel filters:

- **Yaw**: 1-D Kalman filter that integrates gyro $g_z$ while estimating
  its zero-drift bias $b_z$.
- **Roll/pitch**: closed-form `atan2` from accelerometer when the robot
  isn't accelerating. No filtering needed for a flat floor.

## State vector

$$
\vect{x}_{\text{yaw}} = \begin{pmatrix} \psi \\ b_z \end{pmatrix}
$$

where $\psi$ is heading (rad) and $b_z$ is the gyro-z bias drift (rad/s).

The 2×2 covariance is stored as three scalars (no NumPy):

$$
\mat{P} = \begin{pmatrix} P_\psi & P_{\times} \\ P_{\times} & P_b \end{pmatrix}
$$

## Predict — gyro integration

```python title="ekf_imu.py — predict()"
# Bias-corrected angular rate
wz = gz - self._gz_bias

# ZUPT: tiny rate -> skip integration, slow covariance growth
if abs(wz) < self._zupt_threshold:    # 0.01 rad/s
    self._P_yaw  += self.q_angle * dt * 0.1
    self._P_bias += self.q_bias  * dt
    return

# Yaw prediction: simple integration
self._yaw += wz * dt
self._yaw = (self._yaw + pi) % (2 * pi) - pi   # normalise

# Covariance prediction: F = [[1, -dt], [0, 1]];  P = F P F.T + Q
p11, p12, p22 = self._P_yaw, self._P_cross, self._P_bias
self._P_yaw   = p11 + (-dt)*p12 + (-dt)*(p12 + (-dt)*p22) + self.q_angle*dt
self._P_cross = p12 + (-dt) * p22
self._P_bias  = p22 + self.q_bias * dt
```

### The math

Bias-corrected angular rate:

$$
\omega_z = g_z - b_z
$$

ZUPT: when $|\omega_z| < 0.01\text{ rad/s}$ the integration step is
skipped and covariance grows at 1/10 rate. This freezes the angle
estimate when the robot is stationary, preventing drift accumulation
from noise.

Heading integration:

$$
\psi_k = \psi_{k-1} + \omega_z \cdot \Delta t
$$

Wrap to $(-\pi, \pi]$:

$$
\psi \leftarrow (\psi + \pi) \bmod 2\pi - \pi
$$

State-transition matrix:

$$
\mat{F} = \begin{pmatrix} 1 & -\Delta t \\ 0 & 1 \end{pmatrix}
$$

Covariance prediction (canonical Kalman):

$$
\boxed{\mat{P}^- = \mat{F}\,\mat{P}\,\mat{F}^\top + \mat{Q}}
$$

with process-noise:

$$
\mat{Q} = \begin{pmatrix}
q_{\text{angle}} \cdot \Delta t & 0 \\
0 & q_{\text{bias}} \cdot \Delta t
\end{pmatrix}
$$

Expanded element-wise (matches the manual 2×2 multiply in the code):

$$
\begin{aligned}
P_\psi^-      &= P_\psi - \Delta t\,P_{\times} - \Delta t (P_{\times} - \Delta t\,P_b) + q_{\text{angle}}\,\Delta t \\
P_{\times}^-  &= P_{\times} - \Delta t\,P_b \\
P_b^-         &= P_b + q_{\text{bias}}\,\Delta t
\end{aligned}
$$

## Update — roll/pitch from gravity

```python title="ekf_imu.py — update()"
a_mag = sqrt(ax*ax + ay*ay + az*az)
if abs(a_mag - self.g) > self.accel_gate * self.g:
    return    # robot is accelerating — skip gravity update

raw_roll  = atan2(ay, az)
raw_pitch = atan2(-ax, sqrt(ay*ay + az*az))

# Subtract home pose -> angles relative to startup
self._roll  = raw_roll  - self._home_roll
self._pitch = raw_pitch - self._home_pitch
self._roll  = (self._roll  + pi) % (2*pi) - pi
self._pitch = (self._pitch + pi) % (2*pi) - pi
```

When the robot isn't accelerating, the accelerometer reads pure gravity
$\vect{g} = (0, 0, -g)$ in the world frame, so:

$$
|\vect{a}| = \sqrt{a_x^2 + a_y^2 + a_z^2} \approx g = 9.81\text{ m/s}^2
$$

Skip-rule (accel gate): $\bigl||\vect{a}| - g\bigr| > 0.3\,g$ — drops
samples while the robot is accelerating. Tested in
`tests/test_ekf_imu.py::test_accel_gate_rejects_high_vibration`.

Roll (around X):

$$
\boxed{\phi_{\text{raw}} = \mathrm{atan2}(a_y,\,a_z)}
$$

Pitch (around Y):

$$
\boxed{\theta_{\text{raw}} = \mathrm{atan2}\!\left(-a_x,\;\sqrt{a_y^2 + a_z^2}\right)}
$$

Subtract the home pose recorded at boot so the filter outputs are
relative to startup orientation:

$$
\phi = \phi_{\text{raw}} - \phi_{\text{home}}, \quad
\theta = \theta_{\text{raw}} - \theta_{\text{home}}
$$

!!! warning "No magnetometer ⇒ no yaw observation"
    The accelerometer can correct roll/pitch but **not** yaw — for that
    we'd need a magnetometer. So yaw accumulates integration error,
    which is exactly why we run a Kalman filter with bias tracking:
    the bias state $b_z$ absorbs the slow drift.

## Linearisation & stability

Nonlinear gyro-with-bias model:

$$
\vect{f}(\vect{x}, u) = \begin{pmatrix}
\psi + (g_z - b_z)\,\Delta t \\
b_z
\end{pmatrix}
$$

Jacobian by $\vect{x} = (\psi, b_z)^\top$:

$$
\mat{F} = \frac{\partial \vect{f}}{\partial \vect{x}} =
\begin{pmatrix} 1 & -\Delta t \\ 0 & 1 \end{pmatrix}
$$

Eigenvalues of $\mat{F}$: $\lambda_{1,2} = 1$ (double root). Without
correction the system is *neutrally stable* — error neither grows nor
decays. The Kalman gain in update step provides the asymptotic
stability by shrinking $\mat{P}$.

Observability matrix for $\mat{H} = (1, 0)$ (direct yaw measurement
from any external source):

$$
\mat{S}_o = \begin{pmatrix} \mat{H} \\ \mat{H}\mat{F} \end{pmatrix}
= \begin{pmatrix} 1 & 0 \\ 1 & -\Delta t \end{pmatrix}, \quad
\mathrm{rank}(\mat{S}_o) = 2
$$

Full rank → fully observable: bias $b_z$ can be inferred from yaw
measurements alone.

In Laplace form:

$$
\psi(p) = \frac{1}{p}\bigl(g_z(p) - b_z(p)\bigr)
$$

A pure integrator $W(p) = 1/p$, which is exactly why drift accumulates
without correction.

## Parameter reference

| Parameter | Symbol | Value | Meaning |
|---|---|---|---|
| `q_angle`         | $q_\psi$ | 0.001 | Yaw process noise |
| `q_bias`          | $q_b$    | 0.0001 | Gyro-bias drift noise |
| `r_accel`         | $r_a$    | 0.5 | Accelerometer measurement noise |
| `accel_gate`      | —        | 0.3 | Accel gate threshold (30%) |
| `zupt_threshold`  | —        | 0.01 rad/s | ZUPT entry rate (~0.6°/s) |

## Tests covering this chapter

`tests/test_ekf_imu.py` (12 cases):

- initial state zero,
- `predict(dt=0)` no-op,
- yaw integrates gyro,
- yaw wraps to $(-\pi, \pi]$ across multiple revolutions,
- ZUPT enters after `confirm` ticks,
- ZUPT exits on motion,
- bias correction under sustained ZUPT,
- accel-update computes roll/pitch correctly,
- accel-gate rejects high-vibration samples,
- home-orientation subtraction,
- `reset()` clears state,
- `get_euler_deg()` returns degrees.
