# Theory foundations

A condensed reference for the control-theory mathematics underpinning
the project's filters and controllers. The full academic treatment
(stability proofs, Faddeev-Leverrier algorithm, Jordan form, etc.)
lives in `latex_doc/chapters/00_theory.tex` and the compiled PDF —
this page is a navigable summary for cross-references from the
algorithm chapters.

## State-space form

Any plant under control can be written as a first-order ODE:

$$
\boxed{
\dot{\vect x}(t) = \mat A\,\vect x(t) + \mat B\,\vect u(t), \quad
\vect y(t) = \mat C\,\vect x(t) + \mat D\,\vect u(t)
}
$$

with state $\vect x \in \mathbb{R}^n$, input $\vect u \in \mathbb{R}^r$,
output $\vect y \in \mathbb{R}^m$, and matrices $\mat A, \mat B, \mat C, \mat D$
of appropriate dimensions.

### How SAMURAI uses it

| Module | State $\vect x$ | $n$ |
|---|---|---:|
| [Velocity EKF](velocity-ekf.md)        | $(v_x, v_y)^\top$           | 2 |
| [IMU EKF](imu-ekf.md)                  | $(\psi, b_z)^\top$          | 2 |
| [Accel position](accel-position.md)    | $(x, y, v_x, v_y)^\top$     | 4 |
| [Odometry](odometry.md) (unicycle)     | $(x, y, \theta)^\top$       | 3 |

## Linearisation (Taylor / Jacobian)

Real systems are nonlinear:

$$
\dot{\vect x} = \vect f(\vect x, \vect u)
$$

Linearise around an operating point $(\bar{\vect x}, \bar{\vect u})$
where $\vect f(\bar{\vect x}, \bar{\vect u}) = \vect 0$:

$$
\boxed{
\Delta\dot{\vect x} = \mat A\,\Delta\vect x + \mat B\,\Delta\vect u
}
$$

with the Jacobian matrices

$$
\mat A = \left.\frac{\partial \vect f}{\partial \vect x}\right|_{\bar x, \bar u}, \quad
\mat B = \left.\frac{\partial \vect f}{\partial \vect u}\right|_{\bar x, \bar u}
$$

This is what the Extended Kalman Filter does at every step — see
[IMU EKF](imu-ekf.md) for the worked example.

## Stability — eigenvalue criterion

A continuous LTI system $\dot{\vect x} = \mat A\,\vect x$ is
**asymptotically stable** iff every eigenvalue of $\mat A$ has
negative real part:

$$
\boxed{\mathrm{Re}(\lambda_i) < 0, \quad \forall\,i = 1, \ldots, n}
$$

where $\lambda_i$ are the roots of $\det(\lambda \mat I - \mat A) = 0$.

Discrete analogue: every eigenvalue lies inside the unit disc,
$|\lambda_i| < 1$.

## Transfer function

For a SISO LTI system:

$$
\boxed{W(p) = \mat C(p\mat I - \mat A)^{-1}\mat B + \mat D}
$$

with $p$ the Laplace variable. Specific examples elsewhere in the
docs:

- [IMU EKF](imu-ekf.md): $W(p) = 1/p$ (pure integrator) — explains
  why gyro drift accumulates.
- [Pure pursuit](pure-pursuit.md): $W_{\text{cl}}(p) = K_P / (p + K_P)$
  — first-order closed-loop with time constant $1/K_P$.
- [PWM motor](pwm-motor.md): second-order TF
  $W(p) = K_m / (T_m T_e p^2 + T_m p + 1)$ collapsing to a static
  gain when $T_e \ll T_m \ll T_{\text{control}}$.

## Discrete state-space + Kalman filter

Discretised plant:

$$
\vect x_{k+1} = \mat F\,\vect x_k + \mat B\,\vect u_k + \vect w_k, \quad
\vect z_k = \mat H\,\vect x_k + \vect v_k
$$

with $\vect w \sim \mathcal{N}(\vect 0, \mat Q)$ process noise and
$\vect v \sim \mathcal{N}(\vect 0, \mat R)$ measurement noise.

### Predict

$$
\hat{\vect x}_k^{-} = \mat F\,\hat{\vect x}_{k-1} + \mat B\,\vect u_{k-1}
$$

$$
\mat P_k^{-} = \mat F\,\mat P_{k-1}\,\mat F^\top + \mat Q
$$

### Update

$$
\vect y_k = \vect z_k - \mat H\,\hat{\vect x}_k^{-}    \quad\text{(innovation)}
$$

$$
\mat S_k = \mat H\,\mat P_k^{-}\,\mat H^\top + \mat R   \quad\text{(innovation cov)}
$$

$$
\mat K_k = \mat P_k^{-}\,\mat H^\top\,\mat S_k^{-1}     \quad\text{(Kalman gain)}
$$

$$
\hat{\vect x}_k = \hat{\vect x}_k^{-} + \mat K_k\,\vect y_k
$$

$$
\mat P_k = (\mat I - \mat K_k\,\mat H)\,\mat P_k^{-}
$$

### Optimality

The Kalman filter minimises the posterior variance
$\mathbb{E}\bigl[\|\vect x - \hat{\vect x}\|^2\bigr]$. The gain
automatically weights model vs measurement:

- $\mat R \ll \mat P^{-}$: $\mat K \to \mat H^{-1}$, trust the sensor
- $\mat R \gg \mat P^{-}$: $\mat K \to \mat 0$, trust the model

Worked examples in [Velocity EKF](velocity-ekf.md) (where the gain
explicitly chooses between wheel-odom and accel) and
[IMU EKF](imu-ekf.md) (where the bias state absorbs gyro drift).

### Extended Kalman filter

For nonlinear $\vect f, \vect h$, linearise per-step:

$$
\mat F_k = \left.\frac{\partial \vect f}{\partial \vect x}\right|_{\hat{\vect x}_{k-1}}, \quad
\mat H_k = \left.\frac{\partial \vect h}{\partial \vect x}\right|_{\hat{\vect x}_k^{-}}
$$

Then plug $\mat F_k$, $\mat H_k$ into the linear KF equations above.
This is what every "EKF" in the project does — see
[IMU EKF](imu-ekf.md) for the planar-robot Jacobian, or
[Odometry](odometry.md) for the unicycle 3×3 form.

## DC-motor model

Each motor obeys the second-order electromechanical ODE:

$$
\boxed{T_a^2\ddot\omega + T_y\dot\omega + \omega = k\,u}
$$

with $T_a$ the electrical time constant, $T_y$ the mechanical time
constant, $k$ the static gain. In the SAMURAI regime
$T_a \ll T_y \ll T_{\text{control}}$, so the dynamics collapse to a
static gain — exactly why the [Odometry chapter](odometry.md) ignores
motor inertia and uses the kinematic model directly.

The full derivation including back-EMF braking torque and the
slow-decay PWM equivalence lives in [PWM motor](pwm-motor.md).

## Lyapunov stability — quick reference

For a candidate Lyapunov function $V(\vect x) > 0$ (positive definite):
the system is stable around an equilibrium $\vect x^*$ if
$\dot V \leq 0$ in a neighbourhood. Asymptotic stability requires
strict $\dot V < 0$.

Used implicitly by [Pure pursuit](pure-pursuit.md):
$V(e_\theta) = 1 - \cos(e_\theta)$ shrinks under the proportional
yaw controller, certifying convergence to the target heading.

## Where to find more

- **Full proofs**: `latex_doc/main.pdf` (compiled LaTeX, ~80 pp).
- **Per-algorithm deep dives**: chapters in this MkDocs site under
  *Algorithms* — each has its own state-space / stability /
  optimality discussion specialised to that filter or controller.
- **Code cross-references**: every chapter quotes the relevant
  source file lines so you can step from math to implementation.
