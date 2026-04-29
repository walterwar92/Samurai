# Pure Pursuit — path-recording return-home

!!! info "Source"
    `pi_nodes/nodes/path_recorder_node.py` (~340 lines).
    Records the robot's path as it moves and replays it in reverse to
    bring the robot home. The replay loop is a P-controller for yaw with
    cosine-scaled linear velocity — the canonical Pure Pursuit pattern,
    refined with a final-approach phase for sub-cm precision at the
    home point.

## Problem statement

Two phases:

- **Record**: append a waypoint when the robot has moved at least
  $\Delta d = 0.03\text{ m}$ OR turned at least
  $\Delta\theta = 0.08\text{ rad}$ since the previous waypoint.
- **Replay**: drive the recorded path in reverse using Pure Pursuit
  with simultaneous linear/angular control plus a final-approach
  precision phase.

## Recording waypoints

```python title="path_recorder_node.py — _maybe_record_waypoint()"
dx = self._x - self._last_record_x
dy = self._y - self._last_record_y
dist = math.sqrt(dx * dx + dy * dy)
dtheta = abs(self._angle_diff(self._theta, self._last_record_theta))

if dist >= RECORD_INTERVAL_M or dtheta >= RECORD_INTERVAL_RAD:
    self._path.append((self._x, self._y, self._theta))
```

Waypoints record only when the robot has actually progressed —
prevents an infinite list when the robot is stationary.

$$
d = \sqrt{(x - x_{\text{last}})^2 + (y - y_{\text{last}})^2}
$$

A waypoint is appended when $d \geq 0.03\text{ m}$ OR
$|\Delta\theta| \geq 0.08\text{ rad}$.

## Pure Pursuit core

```python title="path_recorder_node.py — _control_loop()"
bearing = math.atan2(dy, dx)
angle_error = self._angle_diff(bearing, self._theta)

if in_final_approach:
    angular   = FINAL_ANGULAR_KP * angle_error    # Kp = 3.0
    angular   = max(-REPLAY_ANGULAR_SPEED, min(REPLAY_ANGULAR_SPEED, angular))
    cos_factor = max(0.0, math.cos(angle_error))
    dist_ratio = dist / FINAL_APPROACH_RADIUS      # 0..1
    linear     = FINAL_LINEAR_SPEED * cos_factor * dist_ratio
    linear     = max(REPLAY_MIN_LINEAR * 0.6 * cos_factor, linear)
    if abs(angle_error) > 0.8:
        linear = 0.0
else:
    angular    = REPLAY_ANGULAR_KP * angle_error  # Kp = 2.5
    angular    = max(-REPLAY_ANGULAR_SPEED, min(REPLAY_ANGULAR_SPEED, angular))
    cos_factor = max(0.0, math.cos(angle_error))
    dist_factor = min(1.0, dist / 0.15)
    linear     = REPLAY_LINEAR_SPEED * cos_factor * dist_factor
    linear     = max(REPLAY_MIN_LINEAR * cos_factor, linear)
    if abs(angle_error) > 1.2:
        linear = 0.0
```

### Bearing to target

For the current target waypoint $(t_x, t_y)$:

$$
\beta = \mathrm{atan2}(t_y - y,\;t_x - x)
$$

### Angular error (shortest signed)

$$
e_\theta = \mathrm{angle\_diff}(\beta, \psi) \in [-\pi, \pi]
$$

### Proportional yaw control

$$
\omega = K_P \cdot e_\theta, \quad
\omega \in [-\omega_{\max}, +\omega_{\max}]
$$

| Mode | $K_P$ | $\omega_{\max}$ |
|---|---|---|
| Normal Pure Pursuit | 2.5 | $0.8\text{ rad/s}$ |
| Final approach      | 3.0 | $0.8\text{ rad/s}$ |

### Cosine-scaled linear velocity

The Pure Pursuit signature trick: scale forward speed by $\cos$ of the
angular error. Big yaw error → near-zero forward speed (robot rotates in
place); small yaw error → full forward speed.

$$
\cos\text{-factor} = \max\bigl(0,\; \cos(e_\theta)\bigr)
$$

$$
v = v_{\max} \cdot \cos\text{-factor} \cdot d_{\text{factor}}
$$

with the distance saturator:

$$
d_{\text{factor}} = \min\!\left(1,\; \frac{d}{0.15\text{ m}}\right)
$$

When $|e_\theta| > \pi/2$, $\cos(e_\theta) \leq 0$, hence $v = 0$ and the
robot only rotates.

### Final approach

Inside a 0.12 m radius around the home point:

$$
v = v_{\text{final}} \cdot \cos(e_\theta) \cdot \frac{d}{r_{\text{final}}}
$$

Speed is proportional to remaining distance — smooth deceleration into
the home tolerance band (1.5 cm).

| Parameter | Value | Meaning |
|---|---|---|
| `REPLAY_LINEAR_SPEED`   | 0.20 m/s | Cruise speed during replay |
| `FINAL_LINEAR_SPEED`    | 0.08 m/s | Final-approach crawl speed |
| `REPLAY_LOOKAHEAD_M`    | 0.10 m   | Look-ahead distance |
| `REPLAY_HOME_TOLERANCE` | 0.015 m  | Home-point tolerance |
| `REPLAY_GOAL_TOLERANCE` | 0.04 m   | Intermediate-waypoint tolerance |

## Angle normalisation

```python title="_angle_diff() — shortest signed angle"
@staticmethod
def _angle_diff(target, current):
    """Shortest signed angle difference, in [-π, π]."""
    d = target - current
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d
```

Equivalent closed-form:

$$
\Delta\theta = \bigl[(\beta - \psi + \pi) \bmod 2\pi\bigr] - \pi
$$

Guarantees $\Delta\theta \in [-\pi, \pi]$, so the robot always turns the
short way around.

## Closed-loop analysis

The yaw-controller plus robot-as-integrator is a textbook first-order
system. Robot kinematics: $\dot\psi = \omega$. P-controller:
$\omega = K_P\,e_\theta$. Closed-loop transfer function:

$$
W_{\text{cl}}(p) = \frac{K_P / p}{1 + K_P / p} = \frac{K_P}{p + K_P}
$$

Single pole at $\lambda = -K_P$. With $K_P = 2.5$ this gives an
exponential heading-error decay $e_\theta(t) = e_\theta(0)\,e^{-2.5\,t}$
with time constant $\tau = 1 / K_P = 0.4\text{ s}$.

### Lyapunov-style stability

The cosine scaling $v = v_{\max}\cos(e_\theta)$ corresponds to the
candidate Lyapunov function

$$
V(e_\theta) = 1 - \cos(e_\theta) \geq 0
$$

For $|e_\theta| < \pi/2$:
$\dot V = \sin(e_\theta)\,\dot e_\theta < 0$ (with the correct sign of
$K_P$), so the closed loop is asymptotically stable to $e_\theta = 0$.

### Full nonlinear model

$$
\begin{cases}
\dot x = v\cos\psi \cdot \cos(e_\theta) \cdot d_{\text{factor}} \\
\dot y = v\sin\psi \cdot \cos(e_\theta) \cdot d_{\text{factor}} \\
\dot \psi = K_P \cdot e_\theta
\end{cases}
$$

Linearising around $e_\theta = 0$ recovers the linear pole at
$\lambda = -K_P$ derived above; the cosine + distance scaling are
saturating nonlinearities that bound the closed loop's behaviour
without changing local stability.
