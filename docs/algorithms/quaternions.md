# Quaternions, projections & orientation

!!! info "Source"
    Used across `compute_node/dashboard/*` (yaw extraction from ROS2
    Odometry quaternions, polar→Cartesian for laser scans, raw
    pitch/roll from accelerometer for IMU widgets).

A reference for the small but recurring orientation-math kernels in
the dashboard. None of these are heavy — they're worth documenting
because they cross between sensor frames and the consumers always
need a quick refresher on the conventions.

## Raw Euler angles from accelerometer

When the robot isn't accelerating, the accelerometer reads pure
gravity and we can recover pitch and roll directly:

```python title="dashboard — IMU snapshot"
# Raw pitch/roll from accel (no yaw)
pitch = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
roll  = math.degrees(math.atan2(-ax, az))
```

Pitch (forward / backward tilt):

$$
\theta_{\text{pitch}} = \mathrm{atan2}\!\left(a_y,\;\sqrt{a_x^2 + a_z^2}\right)
$$

Roll (left / right tilt):

$$
\phi_{\text{roll}} = \mathrm{atan2}(-a_x,\;a_z)
$$

!!! note "Why `atan2`, not `atan`"
    `atan2(y, x)` returns the angle in $(-\pi, \pi]$ and handles all
    four quadrants correctly, where `atan(y / x)` collapses the sign
    of the denominator and only spans $(-\pi/2, \pi/2)$.

## Quaternion → yaw (from ROS2 Odometry)

```python title="dashboard — _odom_cb()"
q = msg.pose.pose.orientation
siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
yaw = math.atan2(siny_cosp, cosy_cosp)
```

A unit quaternion $\vect{q} = (q_w, q_x, q_y, q_z)$, $|\vect{q}| = 1$,
encodes a 3D rotation as

$$
\vect{q} = \cos\frac{\alpha}{2} + \hat{\vect{n}}\sin\frac{\alpha}{2}
$$

where $\hat{\vect{n}}$ is the rotation axis and $\alpha$ the rotation
angle.

The aerospace ZYX convention for Euler decomposition picks two
elements of the rotation matrix:

$$
R_{32} = 2(q_w q_z + q_x q_y), \quad
R_{33} = 1 - 2(q_y^2 + q_z^2)
$$

Then yaw is

$$
\boxed{\psi = \mathrm{atan2}\!\bigl(2(q_w q_z + q_x q_y),\;
                1 - 2(q_y^2 + q_z^2)\bigr)}
$$

For a planar robot (yaw-only motion) $q_x = q_y = 0$, so this reduces
to $\psi = \mathrm{atan2}(2 q_w q_z,\;1 - 2 q_z^2) = 2\arctan(q_z / q_w)$.

## Full quaternion → rotation matrix

For completeness:

$$
\mat{R} = \begin{pmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{pmatrix}
$$

For pure yaw ($q_x = q_y = 0$, $q_z = \sin(\psi/2)$, $q_w = \cos(\psi/2)$)
this collapses to the familiar planar rotation matrix:

$$
\mat{R}_z(\psi) = \begin{pmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi &  \cos\psi & 0 \\
0 & 0 & 1
\end{pmatrix}
$$

## Polar → Cartesian (laser scan)

```python title="dashboard — _scan_cb()"
def _scan_cb(self, msg: LaserScan):
    points, angle = [], msg.angle_min
    for r in msg.ranges:
        if msg.range_min < r < msg.range_max:
            points.append([
                round(r * math.cos(angle), 3),
                round(r * math.sin(angle), 3)
            ])
        angle += msg.angle_increment
```

For each ray with measured range $r$ at scanner angle $\alpha$:

$$
\begin{pmatrix} x \\ y \end{pmatrix}
= r \begin{pmatrix} \cos\alpha \\ \sin\alpha \end{pmatrix}
$$

Out-of-range readings (less than `range_min` or greater than
`range_max`) are dropped.

## When to reach for which form

| Representation | Pros | Cons | Where used |
|---|---|---|---|
| Euler (yaw, pitch, roll) | Human-readable | Gimbal lock, three competing conventions | UI display, joystick UX |
| Quaternion              | No singularities, smooth interpolation | Less intuitive | ROS2 Odometry, IMU EKF intermediate |
| Rotation matrix         | Direct vector application | 9 params, redundant | Per-frame transforms in renderers |

The dashboard's data flow is:

```
ROS2 Odometry (quaternion) → atan2 extract yaw → cache as float
IMU pitch/roll             → atan2 from accel  → cache as floats
Laser scan (polar)         → r·(cos α, sin α)  → cache as Cartesian list
```

Everything downstream (map renderer, 3D viz) consumes the cached
post-extraction values, never the raw quaternion or polar form.
