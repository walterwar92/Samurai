# Simulator overview — kinematics, sensors, camera

!!! info "Source"
    `compute_node/simulator.py` and the extracted modules around it
    (#44 phases 1–7a):
    `pathfinding.py`, `sim_arena.py`, `sim_robot.py`, `sim_sensors.py`,
    `sim_renderer.py`, `sim_detector.py`, `sim_fsm_states.py`.

This chapter ties together the simulator pieces that earlier chapters
covered in depth. Each section here is the executive summary; the
deep-dive math lives in the algorithm-specific chapters linked below.

## Robot kinematics

!!! abstract "Detail"
    Full derivation in [Odometry](odometry.md). Code in
    `compute_node/sim_robot.py` (#44 phase 3).

The simulator's `SimRobot.tick(dt)` is the discrete unicycle model:

$$
\boxed{
\begin{pmatrix} x_{k+1} \\ y_{k+1} \\ \theta_{k+1} \end{pmatrix}
= \begin{pmatrix} x_k \\ y_k \\ \theta_k \end{pmatrix}
+ \Delta t \begin{pmatrix} v\cos\theta_k \\ v\sin\theta_k \\ \omega \end{pmatrix}
}
$$

After integration, $\theta$ is wrapped to $(-\pi, \pi]$ via
$\theta \leftarrow \mathrm{atan2}(\sin\theta, \cos\theta)$, the
position is clamped to the arena (Minkowski-shrunk by `ROBOT_RADIUS`),
and any cell that lands inside an inflated forbidden zone snaps the
robot back to its previous position with $v_\text{linear} = 0$.

## Ultrasonic sensor

!!! abstract "Detail"
    Code in `compute_node/sim_sensors.py` (#44 phase 4) — 12 unit
    tests in `tests/test_sim_sensors.py`.

The forward-pointing ultrasonic ray:

$$
\vect p(t) = (r_x, r_y) + t (\cos\theta, \sin\theta), \quad t > 0
$$

Wall hit (analytic ray-plane), e.g. against the east wall $x = W$
when $\cos\theta > 0$:

$$
t_{\text{wall}} = \frac{W - r_x}{\cos\theta}
$$

Ball hit (ray-cone) for each unsifted ball at $(b_x, b_y)$:

$$
t_{\text{proj}} = (b_x - r_x)\cos\theta + (b_y - r_y)\sin\theta
$$

$$
d_{\perp} = \bigl|(b_x - r_x)\sin\theta - (b_y - r_y)\cos\theta\bigr|
$$

Detected if $d_{\perp} < r_{\text{ball}} + 0.05$ (the cone half-width)
and $t_{\text{proj}} > r_{\min}$.

The minimum among all wall and ball hits gives the range, then
Gaussian noise is layered on top:

$$
r_{\text{meas}} = \mathrm{clamp}\bigl(r_{\text{true}} + \mathcal{N}(0,\,0.005^2),\;
                  r_{\min},\;r_{\max}\bigr)
$$

Toggleable via `inject_noise=False` for deterministic tests.

## IMU simulation

```python title="sim_sensors.py — _update_imu()"
self.imu_yaw    = math.degrees(robot.theta) + noise
self.imu_pitch  = random.gauss(0, 0.2)
self.imu_roll   = random.gauss(0, 0.2)
self.imu_gyro_z = robot.v_angular
self.accel_x    = (robot.v_linear - robot._prev_v_linear) / SIM_DT
```

Linear acceleration via finite-difference of the previous tick's
velocity:

$$
a_x = \frac{v_k - v_{k-1}}{\Delta t}
$$

`_prev_v_linear` is updated by `SimRobot.tick()` before integration,
so the contract is: tick the robot, *then* update sensors.

## Camera (pinhole projection)

!!! abstract "Detail"
    Code in `compute_node/sim_detector.py` (#44 phase 6) — 12 unit
    tests including FOV culling, grabbed-ball invisibility, and
    confidence-vs-distance monotonicity.

Angle from robot heading to ball:

$$
\alpha = \mathrm{atan2}(b_y - r_y,\;b_x - r_x) - \theta_{\text{robot}}
$$

Ball is in frame iff $|\alpha| \leq \text{FOV}/2 = 30°$.

Horizontal screen projection (linear in angle):

$$
\boxed{s_x = \frac{W}{2} + \frac{\alpha}{\text{FOV}/2} \cdot \frac{W}{2}}
$$

Apparent pixel size — pinhole camera model:

$$
\boxed{s_{\text{px}} = \frac{f \cdot D_{\text{real}}}{d}}
$$

with $f = 500\text{ px}$ and $D = 0.04\text{ m}$ for the balls.

The same pinhole formula drives [Follow-me](follow-me.md)'s distance
estimation from a person bounding-box height.

Vertical position uses a depth-cue (closer = lower on screen):

$$
s_y = h_{\text{horizon}}
    + \left(1 - \frac{0.03}{\max(0.1, d)}\right) \cdot 0.7 \cdot (H - h_{\text{horizon}})
$$

Confidence is a clamped distance fall-off:

$$
\text{conf} = \mathrm{clamp}\!\left(1 - \frac{d}{3.0},\;0.5,\;0.99\right)
$$

## FSM states

!!! abstract "Detail"
    Code in `compute_node/sim_fsm_states.py` (#44 phase 7a) — 8 unit
    tests including the contract test that locks the seven expected
    state names.

The simulated robot's high-level behaviour is a 7-state FSM:

$$
\text{IDLE} \to \text{SEARCHING} \to \text{TARGETING} \to \text{APPROACHING}
            \to \text{GRABBING} \to \text{CALLING} \to \text{RETURNING} \to \text{IDLE}
$$

Implemented as a `str, Enum` so `state == 'IDLE'` keeps working
alongside the proper enum-typed comparisons.

## Pathfinding

!!! abstract "Detail"
    Full chapter: [A* pathfinding & Bresenham](pathfinding.md).

The simulator's planning uses A* on an occupancy grid built from the
arena walls and forbidden zones, smoothed via Bresenham line-of-sight
pruning. Lives in `compute_node/pathfinding.py` (#44 phase 1) with 10
unit tests.

## Map renderer

!!! abstract "Detail"
    Code in `compute_node/sim_renderer.py` (#44 phase 5) — 9 unit
    tests covering PNG signature, dimensions, zone overlay, planned
    path, scan overlay, grabbed-ball invisibility.

A top-down PNG drawn with OpenCV: walls + grid + forbidden zones
(semi-transparent red) + balls + planned path + scan points + robot
body & FOV cone.

## Parameter reference

| Parameter | Value | Description |
|---|---|---|
| `ARENA_W, H`       | 3.0 × 3.0 m | Arena dimensions |
| `SIM_DT`           | 0.05 s     | 20 Hz integrator step |
| `MAX_LINEAR`       | 0.30 m/s   | Velocity saturation |
| `MAX_ANGULAR`      | 2.0 rad/s  | Yaw saturation |
| `ROBOT_RADIUS`     | 0.12 m     | Used for wall clamp + zone reject |
| `BALL_DIAMETER_M`  | 0.04 m     | For pinhole projection |
| `FOCAL_LENGTH_PX`  | 500 px     | Camera focal length |
| `CAM_FOV`          | 60°        | Camera horizontal FOV |
| `CAM_W, H`         | 640 × 480  | Camera resolution |
| `ULTRASONIC_MAX`   | 2.0 m      | HC-SR04 spec range |
