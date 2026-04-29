# Odometry — differential-drive kinematics

!!! info "Source"
    `pi_nodes/nodes/motor_node.py` (Pure-Python Pi side, primary today).
    `ros_ws/src/robot_pkg_cpp/src/motor_node.cpp` (legacy C++ ROS2 node).
    Constants config-driven via `config.yaml :: motor.*` after #46.

A tracked robot is steered by two independent tracks. The motor node
takes a `(linear, angular)` velocity command, converts it into per-track
PWM values via a differential-drive mixer, and integrates the same
command into open-loop odometry. The error from track slippage is
later compensated by the Velocity EKF
([Velocity EKF chapter](velocity-ekf.md)).

## Differential-drive mixer

```cpp title="motor_node.cpp — controlLoop()"
// Convert m/s -> throttle in [-1, 1]
double lin_t = (max_lin_ > 0.0) ? (linear_  / max_lin_) : 0.0;
double ang_t = (max_ang_ > 0.0) ? (angular_ / max_ang_) : 0.0;

// Differential-drive mixer
double left  = lin_t + ang_t;
double right = lin_t - ang_t;

// Normalise: keep ratio, clamp to [-1, 1]
double max_val = std::max({std::abs(left), std::abs(right), 1.0});
if (max_val > 1.0) { left /= max_val; right /= max_val; }
```

Mixer (normalised throttles in $[-1, 1]$):

$$
\text{left} = \ell_{\text{lin}} + \ell_{\text{ang}}, \quad
\text{right} = \ell_{\text{lin}} - \ell_{\text{ang}}
$$

Ratio-preserving normalisation:

$$
m = \max\!\bigl(1,\; |\text{left}|,\; |\text{right}|\bigr), \quad
\text{left} \leftarrow \frac{\text{left}}{m}, \quad
\text{right} \leftarrow \frac{\text{right}}{m}
$$

If the sum exceeds the actuator's saturation limit, both tracks scale
down by the same factor — the **rotation is preserved** even when full
linear speed isn't achievable.

## Open-loop odometry integration

```cpp title="motor_node.cpp — integration"
// Open-loop odometry integration
x_     += linear_  * std::cos(theta_) * dt;
y_     += linear_  * std::sin(theta_) * dt;
theta_ += angular_ * dt;
```

Forward-Euler integration of the **unicycle** kinematic model:

$$
\begin{aligned}
x_{k+1}      &= x_k + v\cos(\theta_k)\,\Delta t \\
y_{k+1}      &= y_k + v\sin(\theta_k)\,\Delta t \\
\theta_{k+1} &= \theta_k + \omega\,\Delta t
\end{aligned}
$$

Velocity $v$ is always aligned with the current heading $\theta$ —
that's the **non-holonomic constraint**, see below.

!!! warning "Open-loop only — slip is not modelled"
    This integration does NOT account for track slippage. A robot
    spinning its tracks on smooth flooring will report distance covered
    even when it's not actually moving. The
    [Velocity EKF](velocity-ekf.md) closes the loop using the
    accelerometer.

## Yaw-quaternion encoding (ROS2 publish)

```cpp title="motor_node.cpp — publishOdom()"
double sin_h = std::sin(theta_ / 2.0);
double cos_h = std::cos(theta_ / 2.0);

nav_msgs::msg::Odometry odom;
odom.pose.pose.orientation.z = sin_h;
odom.pose.pose.orientation.w = cos_h;
odom.twist.twist.linear.x    = linear_;
odom.twist.twist.angular.z   = angular_;
odom_pub_->publish(odom);
```

Pure rotation around the vertical axis $Z$ encodes as:

$$
\vect{q} = \bigl(0,\;0,\;\sin(\theta/2),\;\cos(\theta/2)\bigr)
$$

For yaw-only motion, $q_x = q_y = 0$, so only $q_z$ and $q_w$ are
populated in the `Odometry` message.

## Non-holonomic constraint

A tracked robot **cannot translate sideways** — only along its current
heading. In control-theory terms:

$$
\dot{x}\sin\theta - \dot{y}\cos\theta = 0
$$

This Pfaffian constraint is **non-integrable** into a holonomic
relation $f(x, y, \theta) = 0$, so the system is **under-actuated**:
the configuration space $(x, y, \theta)$ has dimension 3 while the
input space $(v, \omega)$ has dimension 2.

## State-space form

The unicycle model is a nonlinear system $\dot{\vect{x}} = \vect{f}(\vect{x}, \vect{u})$:

$$
\begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{pmatrix}
= \begin{pmatrix} \cos\theta & 0 \\ \sin\theta & 0 \\ 0 & 1 \end{pmatrix}
  \begin{pmatrix} v \\ \omega \end{pmatrix}
$$

Jacobian by state (for EKF linearisation):

$$
\mat{F} = \frac{\partial \vect{f}}{\partial \vect{x}} =
\begin{pmatrix}
1 & 0 & -v\sin\theta\,\Delta t \\
0 & 1 & v\cos\theta\,\Delta t \\
0 & 0 & 1
\end{pmatrix}
$$

## Why kinematic, not dynamic, model?

Each track is driven by a DC motor whose full electromechanical
model would be:

$$
T_a^2\ddot{\omega}_{\text{motor}} + T_y\dot{\omega}_{\text{motor}}
+ \omega_{\text{motor}} = k\,u_{\text{PWM}}
$$

We use the **kinematic model** (ignoring inertia) because the
electrical time constant $T_a$ is much smaller than the 50 ms control
period — the motor settles to its commanded steady-state well within
one tick.

## Forward-Euler error analysis

Per-step error is $O(\Delta t^2)$, global error $O(\Delta t)$. For the
default $\Delta t = 0.05\text{ s}$ and $v_{\max} = 0.5\text{ m/s}$:

$$
\epsilon \leq \tfrac{1}{2}|\ddot{x}|\,\Delta t^2
            \approx \tfrac{1}{2} v\,\omega\,\Delta t^2
            \leq \tfrac{0.5 \cdot 2.0 \cdot 0.0025}{2}
            = 1.25\text{ mm}
$$

Bounded enough to be eaten by the EKF's process-noise budget.

## Parameter reference

After #46 these defaults are config-driven through
`config.yaml :: motor.*` and `wheel_calibration.*`:

| Parameter | Default | Description |
|---|---|---|
| `max_lin`              | 0.5 m/s   | Linear-velocity saturation |
| `max_ang`              | 2.0 rad/s | Angular-velocity saturation |
| Control period         | 50 ms     | $\Delta t$ for odometry integration (20 Hz) |
| `wheel_base`           | 0.17 m    | Distance between tracks |
| `scale_linear_fwd/bwd` | 1.235 / 0.988 | Per-direction velocity calibration |
| `motor_trim_pct`       | -12.003 % | Asymmetry compensation between tracks |
