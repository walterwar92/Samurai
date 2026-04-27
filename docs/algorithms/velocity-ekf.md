# Velocity EKF — 2D wheel + accelerometer fusion

!!! info "Source"
    `pi_nodes/filters/velocity_ekf.py` (~250 lines).
    Tested in `tests/test_velocity_ekf.py` (8 cases — see #51).

A tracked robot needs a single, trustworthy velocity estimate that
combines two complementary sources:

- **Wheel odometry** — drift-free but blind to track slippage.
- **Accelerometer (single integration)** — sees real motion (slips
  included) but the integration accumulates bias drift.

A 2-state EKF in world frame fuses them, weighted by their statistical
noise characteristics. Position is integrated from the fused velocity.

## State and covariance

State vector — robot velocity in **world** coordinates:

$$
\vect{x} = \begin{pmatrix} v_x \\ v_y \end{pmatrix} \in \mathbb{R}^2
$$

The 2×2 covariance is stored as four scalars (no NumPy):

$$
\mat{P} = \begin{pmatrix} p_{00} & p_{01} \\ p_{10} & p_{11} \end{pmatrix}, \quad
\mat{P}_0 = 0.01 \cdot \mat{I}
$$

## Predict — wheel odometry

```python title="velocity_ekf.py — predict_wheel()"
# Wheel-predicted velocity in world frame
vx_wheel = vx_cmd * cos(theta)
vy_wheel = vx_cmd * sin(theta)

# Prediction: state transitions toward wheel command
alpha = 0.5
self.vx = (1 - alpha) * self.vx + alpha * vx_wheel
self.vy = (1 - alpha) * self.vy + alpha * vy_wheel

# Process noise: covariance grows
self._p00 += self.q_wheel * dt
self._p11 += self.q_wheel * dt

# Integrate position using current fused velocity
self.x += self.vx * dt
self.y += self.vy * dt
```

Wheel command $v_{\text{cmd}}$ is body-frame forward speed; rotate by
yaw $\theta$ (from IMU) into the world frame:

$$
\begin{pmatrix} v_x^{\text{wheel}} \\ v_y^{\text{wheel}} \end{pmatrix}
= v_{\text{cmd}} \begin{pmatrix} \cos\theta \\ \sin\theta \end{pmatrix}
$$

Blend with the prior estimate with mixing factor $\alpha = 0.5$:

$$
\vect{x}^- = (1 - \alpha)\,\vect{x} + \alpha\,\vect{x}^{\text{wheel}}
$$

Process noise grows the covariance ($q_{\text{wheel}} = 0.01$):

$$
p_{00}^- = p_{00} + q_{\text{wheel}}\,\Delta t, \quad
p_{11}^- = p_{11} + q_{\text{wheel}}\,\Delta t
$$

Position via Euler integration:

$$
\boxed{x \mathrel{+}= v_x \cdot \Delta t, \quad y \mathrel{+}= v_y \cdot \Delta t}
$$

## Update — accelerometer

```python title="velocity_ekf.py — update_accel()"
# Integrate accel to get a velocity measurement
self._accel_vx += la_wx * dt
self._accel_vy += la_wy * dt

# Decay accel velocity to prevent unbounded drift
self._accel_vx *= 0.98
self._accel_vy *= 0.98

# Innovation: y = z - H*x  (H = I)
yx = self._accel_vx - self.vx
yy = self._accel_vy - self.vy

# S = P + R (innovation covariance)
r = self.r_accel + self.q_accel * dt
s00 = self._p00 + r
s11 = self._p11 + r

# Kalman gain: K = P * S^-1
k00 = self._p00 / s00
k11 = self._p11 / s11

# State update
self.vx += k00 * yx
self.vy += k11 * yy

# Covariance update: P = (I - K*H) * P
self._p00 *= (1 - k00)
self._p11 *= (1 - k11)
```

Single-integration of world-frame acceleration:

$$
\vect{v}^{\text{accel}}_k = \vect{v}^{\text{accel}}_{k-1}
                          + \vect{a}^{\text{world}} \cdot \Delta t
$$

Exponential decay holds the accel-derived velocity bounded:

$$
\vect{v}^{\text{accel}} \leftarrow 0.98 \cdot \vect{v}^{\text{accel}}
$$

Innovation (residual) with $\mat{H} = \mat{I}$:

$$
\vect{y} = \vect{z} - \mat{H}\,\vect{x}^- = \vect{v}^{\text{accel}} - \vect{x}^-
$$

Innovation covariance:

$$
\mat{S} = \mat{P}^- + \mat{R}, \quad
R = r_{\text{accel}} + q_{\text{accel}}\,\Delta t
$$

Kalman gain (diagonal $\mat{S}$ → element-wise scalars):

$$
\boxed{K_{ii} = \frac{p_{ii}^-}{S_{ii}}, \quad i \in \{0, 1\}}
$$

State update:

$$
\vect{x} = \vect{x}^- + \mat{K}\,\vect{y}
$$

Covariance update (Joseph form, diagonal):

$$
\boxed{p_{ii} \leftarrow (1 - K_{ii})\,p_{ii}}
$$

## ZUPT — zero-velocity update

```python title="velocity_ekf.py — update_zupt()"
s00 = self._p00 + r
s11 = self._p11 + r
k00 = self._p00 / s00
k11 = self._p11 / s11

# Update: z = [0, 0], innovation = -[vx, vy]
self.vx -= k00 * self.vx
self.vy -= k11 * self.vy

self._p00 *= (1 - k00)
self._p11 *= (1 - k11)
```

When the robot is stationary, the measurement is $\vect{z} = \vect{0}$
with very small noise $r_{\text{zupt}} = 10^{-4}$. Innovation is
$\vect{y} = \vect{0} - \vect{v} = -\vect{v}$, so the standard Kalman
update aggressively drives velocity to zero — pinning the EKF state
when the robot isn't moving. Tested in
`tests/test_velocity_ekf.py::test_zupt_zeros_velocity`.

## Non-holonomic constraint

```python title="velocity_ekf.py — apply_nonholonomic()"
cy, sy = cos(theta), sin(theta)
fwd = self.vx * cy + self.vy * sy     # forward component
lat = -self.vx * sy + self.vy * cy    # lateral component

lat *= self._lateral_decay            # kill 70% of lateral vel

self.vx = fwd * cy - lat * sy
self.vy = fwd * sy + lat * cy
```

A tracked robot **cannot move sideways**. After the Kalman update we
project velocity onto the body frame, scale the lateral component
down, and rotate back:

$$
v_{\text{fwd}} = v_x\cos\theta + v_y\sin\theta, \quad
v_{\text{lat}} = -v_x\sin\theta + v_y\cos\theta
$$

Suppress lateral motion: $v_{\text{lat}} \leftarrow 0.3 \cdot v_{\text{lat}}$.

Rotate back via the inverse rotation $\mat{R}^{-1} = \mat{R}^\top$:

$$
\begin{pmatrix} v_x \\ v_y \end{pmatrix}
= \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}
  \begin{pmatrix} v_{\text{fwd}} \\ v_{\text{lat}} \end{pmatrix}
$$

## Velocity magnitude

$$
|\vect{v}| = \sqrt{v_x^2 + v_y^2}
$$

## Stability and optimality

State-space form:

$$
\underbrace{\begin{pmatrix} v_x \\ v_y \end{pmatrix}_{k}}_{\vect{x}_k}
= \underbrace{\begin{pmatrix} 1-\alpha & 0 \\ 0 & 1-\alpha \end{pmatrix}}_{\mat{F}}
\vect{x}_{k-1}
+ \underbrace{\begin{pmatrix} \alpha\cos\theta & 0 \\ 0 & \alpha\sin\theta \end{pmatrix}}_{\mat{B}}
\begin{pmatrix} v_{\text{cmd}} \\ v_{\text{cmd}} \end{pmatrix}
$$

Eigenvalues of $\mat{F}$: $\lambda_{1,2} = 1 - \alpha = 0.5$. Inside
the unit disc, so the filter is **asymptotically stable** in the
discrete-time sense.

The Kalman gain $K_{ii} = p_{ii}/(p_{ii} + R)$ is the minimiser of
posterior variance:

$$
\frac{\partial}{\partial K}\,E\!\left[(x - \hat{x})^2\right] = 0
\;\Rightarrow\;
K^* = \frac{P^-}{P^- + R}
$$

Limit cases:

- $R \to 0$ (perfect sensor): $K \to 1$, filter trusts the measurement.
- $R \to \infty$ (noisy sensor): $K \to 0$, filter trusts the model.

## Parameter reference

| Parameter | Symbol | Value | Physical meaning |
|---|---|---|---|
| `q_wheel`        | $q_w$ | $0.01\,(m/s)^2$  | Wheel-process noise |
| `q_accel`        | $q_a$ | $0.05\,(m/s)^2$  | Accel-drift growth rate |
| `r_accel`        | $r_a$ | $0.1\,(m/s)^2$   | Accel measurement noise |
| `r_zupt`         | $r_z$ | $10^{-4}\,(m/s)^2$ | ZUPT trust |
| `lateral_decay`  | —     | 0.3              | Side-slip suppression |

## Tests covering this chapter

`tests/test_velocity_ekf.py` (8 cases):

- initial state zero,
- friction decay drives velocity → 0 in absence of input,
- `predict_friction(dt=0)` no-op,
- accel-update grows velocity from sustained accelerometer reading,
- ZUPT drives velocity to zero,
- position integrates from velocity (50 ticks @ 0.5 m/s ≈ 0.5 m),
- `reset()` clears state,
- bias absorption: ZUPT bounds drift even when accel has a constant
  steady-state offset.
