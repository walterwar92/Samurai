# Geofence, sensor projections & reactive nav

A consolidated chapter covering three short modules that share a
theme: geometric work between sensor inputs and a navigable map.

## Geofence — point-in-polygon

!!! info "Source"
    `compute_node/geofence_map_publisher.py` (~110 lines).
    Builds a Nav2-friendly OccupancyGrid from a user-defined polygon.

```python title="geofence_map_publisher.py — _point_in_polygon()"
@staticmethod
def _point_in_polygon(x, y, poly):
    """Ray-casting algorithm."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside
```

### Ray-casting algorithm

For each test point $(x, y)$ shoot a horizontal ray to the right and
count how many polygon edges it crosses. Odd count → inside.

$$
\text{inside} = (\text{crossings}) \bmod 2 = 1
$$

For each edge $(x_i, y_i) \to (x_j, y_j)$:

1. Check the edge actually straddles horizontal line $y$:

$$
(y_i > y) \neq (y_j > y)
$$

2. Compute the x-coordinate where the edge crosses that line via
linear interpolation:

$$
x_{\text{cross}} = x_i + \frac{(x_j - x_i)(y - y_i)}{y_j - y_i}
$$

3. If $x < x_{\text{cross}}$, the ray crosses this edge — flip the
flag.

### Building the OccupancyGrid

```python title="geofence_map_publisher.py — grid build"
poly = np.array(self._polygon, dtype=np.float64)
x_min = poly[:, 0].min() - self._padding   # 0.5 m padding
x_max = poly[:, 0].max() + self._padding
width  = int((x_max - x_min) / self._resolution)  # 0.05 m/cell
height = int((y_max - y_min) / self._resolution)

# 0=free inside, 100=lethal outside
grid = np.full((height, width), 100, dtype=np.int8)
for iy in range(height):
    for ix in range(width):
        px = x_min + ix * self._resolution
        py = y_min + iy * self._resolution
        if self._point_in_polygon(px, py, poly):
            grid[iy, ix] = 0
```

ROS2 `nav_msgs/OccupancyGrid` semantics:

- `0` — free cell (inside polygon)
- `100` — lethal obstacle (outside polygon — Nav2 plans around)

Padding ensures the polygon's bounding box has space for the planner
to find smooth approaches at the boundary.

## Ultrasonic → LaserScan adapter

!!! info "Source"
    `compute_node/depth_to_scan_node.py` (~80 lines).
    Spreads a single HC-SR04 reading over enough beams to look like a
    `sensor_msgs/LaserScan` to SLAM Toolbox / Nav2.

```python title="depth_to_scan_node.py — _publish_scan()"
def _publish_scan(self):
    scan = LaserScan()
    scan.angle_min = -math.pi / 2.0      # -90°
    scan.angle_max =  math.pi / 2.0      # +90°
    scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / NUM_BEAMS  # 180 beams
    scan.range_min = 0.02
    scan.range_max = 3.0

    ranges = [MAX_RANGE] * NUM_BEAMS     # all unknown
    centre = NUM_BEAMS // 2
    # Spread ultrasonic over ~15° cone
    cone_beams = int(0.26 / scan.angle_increment)
    for i in range(max(0, centre - cone_beams),
                   min(NUM_BEAMS, centre + cone_beams + 1)):
        ranges[i] = min(self._latest_range, MAX_RANGE)
    scan.ranges = ranges
```

### Cone geometry

The HC-SR04 has a single distance reading and a ~30° cone of view.
For SLAM Toolbox compatibility we spread that one reading across the
beams that fall inside the cone.

Per-beam angular increment:

$$
\Delta\alpha = \frac{\alpha_{\max} - \alpha_{\min}}{N_{\text{beams}}}
            = \frac{\pi}{180} \approx 0.0175\text{ rad/beam}
$$

Beams covered by the half-cone:

$$
N_{\text{cone}} = \left\lfloor \frac{0.26}{\Delta\alpha} \right\rfloor
                \approx 15
$$

So ~30 central beams carry the real range; the remaining 150 stay at
`MAX_RANGE` (= "unknown").

## Reactive (fallback) nav

!!! info "Source"
    `pi_nodes/nodes/fallback_nav_node.py` (~175 lines).
    A 3-state finite-state machine that drives the robot purely from
    the latest ultrasonic reading. Used when Nav2 isn't available.

```python title="fallback_nav_node.py — _run_fallback_drive()"
if r > SAFE_M:                             # r > 0.40 m
    cmd['linear_x'] = FWD_SPEED            # 0.10 m/s
    self._state = STATE_FORWARD
elif r > STOP_M:                           # 0.20 < r < 0.40
    scale = (r - STOP_M) / (SAFE_M - STOP_M)
    cmd['linear_x'] = SLOW_SPEED + scale * (FWD_SPEED - SLOW_SPEED)
    self._state = STATE_CAUTION
else:                                      # r < 0.20 m
    self._state = STATE_ROTATE
    cmd['angular_z'] = ROT_SPEED * self._rotate_dir  # ±0.8 rad/s
```

### State machine

```
   r > 0.40 m  ┌─────────┐  r < 0.40 m  ┌─────────┐  r < 0.20 m  ┌────────┐
   ───────────►│ FORWARD │─────────────►│ CAUTION │─────────────►│ ROTATE │
               └─────────┘              └─────────┘              └────────┘
                                                                      │
                                                  r > 0.40 m          │
                ◄─────────────────────────────────────────────────────┘
```

### Linear interpolation in the CAUTION zone

$$
\text{scale} = \frac{r - r_{\text{stop}}}{r_{\text{safe}} - r_{\text{stop}}}
             = \frac{r - 0.20}{0.40 - 0.20} \in [0, 1]
$$

$$
\boxed{v = v_{\text{slow}} + \text{scale} \cdot (v_{\text{fwd}} - v_{\text{slow}})}
$$

A linear blend between $v_{\text{slow}} = 0.05\text{ m/s}$ and
$v_{\text{fwd}} = 0.10\text{ m/s}$. Continuous at the boundaries —
no velocity jump when the state machine flips from FORWARD to CAUTION.

### Theory — switched system

This is a **hybrid system**: continuous dynamics (the velocity ramp)
combined with discrete switching (state transitions). Stability holds
if every sub-system is stable AND switching satisfies a *dwell-time*
condition (no thrashing). The linear-interpolated CAUTION zone is
what enforces continuity at switch boundaries — without it, the
discrete velocity jump would put the system on the edge of dwell-time
violation.

## Parameter reference

| Module | Parameter | Value | Description |
|---|---|---|---|
| Geofence | resolution | 0.05 m | Cell size of generated grid |
| Geofence | padding    | 0.5 m  | Bounding-box margin around polygon |
| LaserScan | N_beams   | 180    | Beam count |
| LaserScan | FOV       | 180°   | Total field of view |
| Fallback | $r_{\text{safe}}$ | 0.40 m | Above this — full speed |
| Fallback | $r_{\text{stop}}$ | 0.20 m | Below this — rotate-in-place |
| Fallback | $\omega_{\text{rot}}$ | 0.8 rad/s | Rotate speed |
