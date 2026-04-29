# SLAM map — log-odds occupancy grid

!!! info "Source"
    `pi_nodes/nodes/slam_map_node.py` (Pi side, ultrasonic-driven).
    Updates a 200×200 cell grid (10 m × 10 m at 5 cm resolution)
    using HC-SR04 cone scans and a Bayesian log-odds update.
    Exposed via `samurai/{robot_id}/slam_map` MQTT topic.

The Pi can't run SLAM Toolbox — too heavy. Instead it maintains a
lightweight occupancy grid from the ultrasonic ranges, suitable for a
top-down debug overlay and basic obstacle persistence between
detections. The compute laptop's full SLAM stack (Cartographer / SLAM
Toolbox over the camera laser-scan) is the canonical map source when
that pipeline is available; the Pi-side grid is the fallback.

## Probabilistic occupancy

Each grid cell is a Bernoulli random variable: free (`F`) or
occupied (`O`).

### Log-odds representation

Storing $P(\mathtt{O})$ directly makes Bayesian updates a chain of
multiplications that quickly hit floating-point underflow. Log-odds —
$\log\frac{P(\mathtt{O})}{1 - P(\mathtt{O})}$ — converts those into
additions, with no underflow risk:

$$
l = \log\frac{P(\mathtt{O})}{1 - P(\mathtt{O})}
$$

Inverse:

$$
P(\mathtt{O}) = \frac{e^l}{1 + e^l} = \sigma(l)
$$

### Constants

```python title="slam_map_node.py — log-odds bounds"
L_OCC  =  0.85   # log-odds increment for occupied cell
L_FREE = -0.40   # log-odds increment for free cell
L_MIN  = -2.0    # minimum log-odds (very confident free)
L_MAX  =  3.5    # maximum log-odds (very confident occupied)
L_THRESH_OCC  =  0.6   # above → occupied (output)
L_THRESH_FREE = -0.5   # below → free (output)
```

| log-odds $l$ | $P(\mathtt{O})$ | Meaning |
|:---:|:---:|---|
| −2.0 | 11.9 % | Confident free |
| −0.5 | 37.8 % | Probably free |
|  0.0 | 50.0 % | Unknown |
| +0.6 | 64.6 % | Probably occupied |
| +3.5 | 97.1 % | Confident occupied |

The asymmetric increments (`L_FREE = -0.4` vs `L_OCC = +0.85`) reflect
the HC-SR04's bias: a free reading carries less information than a
hit, because the cone is wide.

## Ray tracing

```python title="slam_map_node.py — _trace_ray()"
def _trace_ray(self, ox, oy, angle, range_m):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    step = CELL_SIZE_M * 0.7   # 3.5 cm step
    dist = 0.0
    hit_range = min(range_m, SENSOR_MAX_RANGE)

    # Mark cells along ray as FREE
    while dist < hit_range - OBSTACLE_THICKNESS:
        wx = ox + cos_a * dist
        wy = oy + sin_a * dist
        ci, cj = self._world_to_cell(wx, wy)
        if 0 <= ci < MAP_CELLS and 0 <= cj < MAP_CELLS:
            idx = cj * MAP_CELLS + ci
            self._grid[idx] = max(L_MIN, self._grid[idx] + L_FREE)
        dist += step

    # Mark obstacle cells as OCCUPIED
    if range_m < SENSOR_MAX_RANGE - 0.05:
        for d in range(3):
            dd = hit_range + d * CELL_SIZE_M * 0.5
            wx = ox + cos_a * dd
            wy = oy + sin_a * dd
            ci, cj = self._world_to_cell(wx, wy)
            if 0 <= ci < MAP_CELLS and 0 <= cj < MAP_CELLS:
                idx = cj * MAP_CELLS + ci
                self._grid[idx] = min(L_MAX, self._grid[idx] + L_OCC)
```

Ray parametric form:

$$
\begin{pmatrix} w_x(t) \\ w_y(t) \end{pmatrix}
= \begin{pmatrix} o_x \\ o_y \end{pmatrix}
+ t \begin{pmatrix} \cos\alpha \\ \sin\alpha \end{pmatrix},
\quad t \in [0, r]
$$

Bayesian update — clamped to keep log-odds bounded:

$$
\boxed{l_{\text{new}} = \mathrm{clamp}\!\bigl(l + \Delta l,\; L_{\min},\; L_{\max}\bigr)}
$$

with

$$
\Delta l = \begin{cases}
L_{\text{FREE}} = -0.40 & \text{ray passed through cell} \\
L_{\text{OCC}} =  +0.85 & \text{ray reached an obstacle here}
\end{cases}
$$

A cell is only marked occupied if the measurement was strictly less
than `SENSOR_MAX_RANGE - 0.05`; an at-max reading is more often
"saw nothing" than "wall at exactly 2 m".

## Ultrasonic cone

The HC-SR04 has a ~15° half-cone, modelled as a fan of rays:

```python title="slam_map_node.py — cone scan"
# Ultrasonic cone: trace multiple rays within the FOV
angle_start = robot_theta - SENSOR_FOV_RAD   # θ - 15°
angle_end   = robot_theta + SENSOR_FOV_RAD   # θ + 15°

angle = angle_start
while angle <= angle_end:
    self._trace_ray(robot_x, robot_y, angle, range_m)
    angle += ANGULAR_RESOLUTION              # 0.05 rad ≈ 3°
```

Number of rays per measurement:

$$
N_{\text{rays}} = \left\lfloor \frac{2 \cdot \text{SENSOR\_FOV\_RAD}}{\text{ANGULAR\_RESOLUTION}} \right\rfloor
                = \left\lfloor \frac{2 \times 0.26}{0.05} \right\rfloor = 10
$$

Ten rays per US ping at ~20 Hz = ~200 ray updates per second, each
touching a handful of cells — the grid converges quickly even with
the HC-SR04's noise budget.

## Coordinate conversion

```python title="slam_map_node.py — world ↔ grid"
def _world_to_cell(self, wx, wy):
    ci = int((wx - self._origin_x) / CELL_SIZE_M)
    cj = int((wy - self._origin_y) / CELL_SIZE_M)
    return ci, cj

def _cell_to_world(self, ci, cj):
    wx = self._origin_x + (ci + 0.5) * CELL_SIZE_M
    wy = self._origin_y + (cj + 0.5) * CELL_SIZE_M
    return wx, wy
```

World → grid:

$$
c_i = \left\lfloor \frac{w_x - o_x}{\delta} \right\rfloor, \quad
c_j = \left\lfloor \frac{w_y - o_y}{\delta} \right\rfloor
$$

Grid → world (cell centre):

$$
w_x = o_x + (c_i + 0.5)\,\delta, \quad
w_y = o_y + (c_j + 0.5)\,\delta
$$

with $o_x = o_y = -5\text{ m}$ (map origin at the south-west corner of
the 10 m × 10 m world) and $\delta = 0.05\text{ m/cell}$.

## Object clustering (Euclidean)

YOLO detections are deduplicated per `(class, colour)` pair using a
30 cm radius:

```python title="slam_map_node.py — _detections_cb()"
# Search for nearest existing object (same class + colour)
for obj_id, obj in self._objects.items():
    dx = obj['x'] - wx
    dy = obj['y'] - wy
    if math.sqrt(dx * dx + dy * dy) < OBJ_CLUSTER_RADIUS_M:  # 0.30 m
        matched_id = obj_id
        break

# Update position: sliding average
if matched_id:
    alpha = 0.3
    obj['x'] = round(alpha * wx + (1 - alpha) * obj['x'], 3)
    obj['y'] = round(alpha * wy + (1 - alpha) * obj['y'], 3)
```

Cluster check:

$$
d_{\text{Euclid}} = \sqrt{(x_{\text{new}} - x_{\text{obj}})^2
                          + (y_{\text{new}} - y_{\text{obj}})^2}
                  < 0.30\text{ m}
$$

Sliding-average position update (EMA, $\alpha = 0.3$):

$$
\vect{p}_{\text{obj}} \leftarrow 0.3\,\vect{p}_{\text{new}}
                                + 0.7\,\vect{p}_{\text{obj}}
$$

This means a stable detection over several frames pulls the cluster
position toward the latest observation while smoothing transient
jitter.

## Parameter reference

| Parameter | Value | Description |
|---|---|---|
| `MAP_SIZE_M`       | 10 m  | Map covers ±5 m around origin |
| `CELL_SIZE_M`      | 0.05 m | 5 cm cell |
| `MAP_CELLS`        | 200 × 200 | Grid dimensions |
| `SENSOR_FOV_RAD`   | 0.26 rad | Cone half-angle (~15°) |
| `SENSOR_MAX_RANGE` | 2.0 m | HC-SR04 spec range |
| `OBJ_CLUSTER_RADIUS_M` | 0.30 m | YOLO dedupe distance |

## Map vs SLAM Toolbox

The compute laptop runs SLAM Toolbox over the camera laser-scan when
ROS2 is available — that's the canonical occupancy map shown on
`/map`. When ROS2 isn't running (Pure-Python Pi mode, dev sim), the
dashboard falls back to this Pi-side log-odds grid. They produce
similar-looking outputs at very different cost:

| Source | Resolution | Update rate | CPU |
|---|---|---|---|
| SLAM Toolbox (laptop)      | 5 cm | 1 Hz | Heavy (loop closure) |
| Pi log-odds grid (this)    | 5 cm | 0.5 Hz publish | <1% on Pi 4 |

The dashboard's `state.map.ros2_map_active` flag picks the active
source.
