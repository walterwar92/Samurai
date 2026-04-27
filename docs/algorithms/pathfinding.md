# A* Pathfinding & Bresenham Line-of-Sight

!!! info "Source"
    `compute_node/pathfinding.py` — extracted from `simulator.py` in #44, with
    unit tests in `tests/test_pathfinding.py`.

The robot needs to find an optimal path from its current position to a goal,
avoiding both arena walls and user-drawn forbidden zones. We use **A\*** —
the canonical informed graph-search algorithm — on a discretised occupancy
grid, with two refinements: **Minkowski safety margin** for obstacle
inflation and **Bresenham line-of-sight pruning** for path smoothing.

## Occupancy grid

The arena is discretised into cells of side $\delta = 5\text{ cm}$:

$$
\text{cols} = \left\lfloor \frac{W}{\delta} \right\rfloor, \quad
\text{rows} = \left\lfloor \frac{H}{\delta} \right\rfloor
$$

A cell is marked **blocked** if its centre lies inside any inflated
obstacle. Inflation uses a Minkowski-sum margin equal to the robot's
radius plus a fixed safety buffer:

$$
\text{margin} = r_{\text{robot}} + r_{\text{safety}} = 0.12 + 0.10 = 0.22\,\text{м}
$$

```python title="compute_node/pathfinding.py — _build_grid()"
def _build_grid(arena, zones, robot_radius,
                grid_res=DEFAULT_GRID_RES,
                safety_margin=DEFAULT_SAFETY_MARGIN):
    cols = int(arena.width / grid_res)
    rows = int(arena.height / grid_res)
    grid = [[False] * cols for _ in range(rows)]
    margin = robot_radius + safety_margin
    for r in range(rows):
        for c in range(cols):
            wx = (c + 0.5) * grid_res
            wy = (r + 0.5) * grid_res
            # Block near arena walls
            if (wx < margin or wx > arena.width - margin or
                    wy < margin or wy > arena.height - margin):
                grid[r][c] = True
                continue
            # Block inside forbidden zones (with safety margin)
            for z in zones:
                zx1, zy1 = z['x1'] - margin, z['y1'] - margin
                zx2, zy2 = z['x2'] + margin, z['y2'] + margin
                if zx1 <= wx <= zx2 and zy1 <= wy <= zy2:
                    grid[r][c] = True
                    break
    return grid, rows, cols
```

## Coordinate conversion

World metres ↔ grid cell:

$$
c = \left\lfloor \frac{w_x}{\delta} \right\rfloor, \quad
r = \left\lfloor \frac{w_y}{\delta} \right\rfloor
$$

$$
w_x = (c + 0.5)\,\delta, \quad w_y = (r + 0.5)\,\delta
$$

## A* search

A* expands the open-set vertex with minimum $f$:

$$
\boxed{f(n) = g(n) + h(n)}
$$

where $g(n)$ is the cost from the start to $n$, and $h(n)$ is the heuristic
estimate of the cost from $n$ to the goal.

### Octile heuristic

For an 8-connected grid (with diagonal moves) the optimal admissible
heuristic is the **octile distance**:

$$
\boxed{h(n) = \max(\Delta r, \Delta c) + (\sqrt{2} - 1)\,\min(\Delta r, \Delta c)}
$$

where $\Delta r = |r_n - r_g|,\;\Delta c = |c_n - c_g|$. In code,
$\sqrt{2}-1 \approx 0.414$.

!!! note "Properties"
    - **Admissible**: $h(n) \leq h^*(n)$ — never overestimates.
    - **Consistent**: $h(n) \leq c(n,n') + h(n')$ — guarantees optimality
      without re-expansion.

### Move costs

$$
c(n, n') = \begin{cases}
1.0 & \text{cardinal direction (4 sides)} \\
\sqrt{2} \approx 1.414 & \text{diagonal direction (4 corners)}
\end{cases}
$$

```python title="compute_node/pathfinding.py — find_path() core loop"
DIRS  = ((-1, 0), (1, 0), (0, -1), (0, 1),
         (-1, -1), (-1, 1), (1, -1), (1, 1))
COSTS = (1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414)

def heuristic(r1, c1, r2, c2):
    dr = abs(r1 - r2)
    dc = abs(c1 - c2)
    return max(dr, dc) + 0.414 * min(dr, dc)

open_set = [(heuristic(sr, sc, gr, gc), 0.0, sr, sc)]
g_cost = {(sr, sc): 0.0}
came_from = {}

while open_set:
    _f, g, r, c = heapq.heappop(open_set)
    if r == gr and c == gc:
        # ... reconstruct path
        break
    for (dr, dc), cost in zip(DIRS, COSTS):
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
            ng = g + cost
            if ng < g_cost.get((nr, nc), math.inf):
                g_cost[(nr, nc)] = ng
                f = ng + heuristic(nr, nc, gr, gc)
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_set, (f, ng, nr, nc))
```

## Bresenham line-of-sight

To check whether two points have a clear straight-line corridor between
them, we rasterise the segment and probe each grid cell:

$$
\text{err} = \Delta c - \Delta r, \quad e_2 = 2\,\text{err}
$$

At each step:

- If $e_2 > -\Delta r$: $\text{err} \leftarrow \text{err} - \Delta r$, advance $c$.
- If $e_2 < \Delta c$:  $\text{err} \leftarrow \text{err} + \Delta c$, advance $r$.

```python title="compute_node/pathfinding.py — _line_of_sight()"
def _line_of_sight(x0, y0, x1, y1, grid, rows, cols, grid_res):
    c0, r0 = _world_to_grid(x0, y0, grid_res)
    c1, r1 = _world_to_grid(x1, y1, grid_res)
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr
    while True:
        if 0 <= r0 < rows and 0 <= c0 < cols:
            if grid[r0][c0]:
                return False
        else:
            return False
        if r0 == r1 and c0 == c1:
            return True
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c0 += sc
        if e2 < dc:
            err += dc
            r0 += sr
```

The algorithm runs in $O(\max(\Delta c, \Delta r))$ integer operations — no
division and no floating-point.

## Path smoothing (line-of-sight pruning)

A* on an 8-connected grid produces zig-zag paths along the cell lattice. We
post-process: walk the waypoint list, and at each waypoint jump to the
farthest later waypoint reachable in a straight line.

```python title="compute_node/pathfinding.py — _smooth_path()"
def _smooth_path(path, grid, rows, cols, grid_res):
    if len(path) <= 2 or grid is None:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        best = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if _line_of_sight(path[i][0], path[i][1],
                              path[j][0], path[j][1],
                              grid, rows, cols, grid_res):
                best = j
                break
        smoothed.append(path[best])
        i = best
    return smoothed
```

Empirically reduces the waypoint count from $O(N)$ to roughly $O(\sqrt{N})$
for paths through open arenas.

## Parameter reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_res` | $0.05\,\text{м}$ | Grid cell size |
| `robot_radius` | $0.12\,\text{м}$ | Used for obstacle inflation |
| `safety_margin` | $0.10\,\text{м}$ | Extra clearance around obstacles |
| 8-direction cardinal cost | 1.0 | A\* edge weight |
| 8-direction diagonal cost | $\sqrt{2} \approx 1.414$ | A\* edge weight |

All of these are overridable via `find_path()` keyword arguments — no
monkey-patching required for tests or alternate arenas.

## Test coverage

`tests/test_pathfinding.py` covers:

- Open arena → short smoothed path between endpoints
- Wall-spanning zone → no path (correctly empty)
- Partial zone → path detours around it
- World ↔ grid round-trip
- Wall-margin blocking
- Nearest-free-cell BFS recovery when start/goal lands in a margin
- Line-of-sight true/false in open / blocked configurations
- Smooth-path reduction in open arena

10 tests, all green in CI.
