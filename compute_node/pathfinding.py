"""
A* pathfinding for the Flask simulator (#44).

Extracted from compute_node/simulator.py — pure utility functions with no
Flask coupling, so they can be unit-tested in isolation and reused by
other components (e.g. compute_node/path_planner/, the dashboard's
planning preview, etc.).

Public API:
    find_path(arena, zones, start_xy, goal_xy, robot_radius)
        → list of (x, y) world-coordinate waypoints, or [] if no path.

The remaining ~1900 lines of simulator.py (SimArena, SimRobot, SimSensors,
SimDetector, SimFSM, MapRenderer + Flask routes) are tracked for
follow-up extraction in memory/improvements_backlog.md (#44).
"""
from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Iterable

# These constants mirror simulator.py's defaults; any caller that passes
# different physics should override via parameters rather than monkey-patching.
DEFAULT_ROBOT_RADIUS = 0.12      # metres
DEFAULT_GRID_RES = 0.05          # 5 cm per A* cell
DEFAULT_SAFETY_MARGIN = 0.10     # 10 cm extra clearance around obstacles


def _build_grid(arena, zones, robot_radius: float,
                grid_res: float = DEFAULT_GRID_RES,
                safety_margin: float = DEFAULT_SAFETY_MARGIN):
    """Build occupancy grid: True = blocked, False = free.

    `arena` is duck-typed: only needs `.width` and `.height` (metres).
    `zones` is iterable of dicts with `x1`, `y1`, `x2`, `y2` keys.
    """
    cols = int(arena.width / grid_res)
    rows = int(arena.height / grid_res)
    grid = [[False] * cols for _ in range(rows)]
    margin = robot_radius + safety_margin

    for r in range(rows):
        for c in range(cols):
            wx = (c + 0.5) * grid_res
            wy = (r + 0.5) * grid_res

            # Block cells near arena walls
            if (wx < margin or wx > arena.width - margin or
                    wy < margin or wy > arena.height - margin):
                grid[r][c] = True
                continue

            # Block cells inside forbidden zones (with robot radius margin)
            for z in zones:
                zx1 = z['x1'] - margin
                zy1 = z['y1'] - margin
                zx2 = z['x2'] + margin
                zy2 = z['y2'] + margin
                if zx1 <= wx <= zx2 and zy1 <= wy <= zy2:
                    grid[r][c] = True
                    break

    return grid, rows, cols


def _world_to_grid(wx: float, wy: float, grid_res: float = DEFAULT_GRID_RES):
    """World metres → grid cell (col, row)."""
    return int(wx / grid_res), int(wy / grid_res)


def _grid_to_world(c: int, r: int, grid_res: float = DEFAULT_GRID_RES):
    """Grid cell → world centre metres."""
    return (c + 0.5) * grid_res, (r + 0.5) * grid_res


def _nearest_free(grid, r: int, c: int, rows: int, cols: int):
    """BFS for the nearest free cell from (r, c)."""
    visited: set[tuple[int, int]] = set()
    queue = deque([(r, c)])
    visited.add((r, c))
    while queue:
        cr, cc = queue.popleft()
        if not grid[cr][cc]:
            return cr, cc
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))
    return None, None


def _line_of_sight(x0: float, y0: float, x1: float, y1: float,
                   grid, rows: int, cols: int,
                   grid_res: float = DEFAULT_GRID_RES) -> bool:
    """Bresenham check: True iff a straight line between two world points
    is free of any blocked grid cell."""
    c0, r0 = _world_to_grid(x0, y0, grid_res)
    c1, r1 = _world_to_grid(x1, y1, grid_res)
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
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


def _smooth_path(path: list[tuple[float, float]], grid, rows: int, cols: int,
                 grid_res: float = DEFAULT_GRID_RES):
    """Reduce path points by line-of-sight pruning against the grid.

    Walks forward through `path` and, at each waypoint, jumps to the
    farthest later waypoint reachable in a straight line. Produces a
    much shorter list of waypoints with the same navigational meaning.
    """
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


def find_path(arena, zones: Iterable[dict],
              start_xy: tuple[float, float],
              goal_xy: tuple[float, float],
              robot_radius: float = DEFAULT_ROBOT_RADIUS,
              grid_res: float = DEFAULT_GRID_RES,
              safety_margin: float = DEFAULT_SAFETY_MARGIN
              ) -> list[tuple[float, float]]:
    """A* pathfinding from start to goal.

    Returns smoothed list of (x, y) world-coordinate waypoints, or `[]`
    if no path exists. 8-directional movement; octile heuristic.
    """
    grid, rows, cols = _build_grid(arena, list(zones), robot_radius,
                                   grid_res, safety_margin)

    sc, sr = _world_to_grid(*start_xy, grid_res)
    gc, gr = _world_to_grid(*goal_xy, grid_res)

    sc = max(0, min(cols - 1, sc))
    sr = max(0, min(rows - 1, sr))
    gc = max(0, min(cols - 1, gc))
    gr = max(0, min(rows - 1, gr))

    if grid[sr][sc]:
        sr, sc = _nearest_free(grid, sr, sc, rows, cols)
    if grid[gr][gc]:
        gr, gc = _nearest_free(grid, gr, gc, rows, cols)

    if sr is None or gr is None:
        return []

    DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1))
    COSTS = (1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414)

    def heuristic(r1: int, c1: int, r2: int, c2: int) -> float:
        dr = abs(r1 - r2)
        dc = abs(c1 - c2)
        return max(dr, dc) + 0.414 * min(dr, dc)   # octile distance

    open_set = [(heuristic(sr, sc, gr, gc), 0.0, sr, sc)]
    g_cost: dict[tuple[int, int], float] = {(sr, sc): 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    while open_set:
        _f, g, r, c = heapq.heappop(open_set)

        if r == gr and c == gc:
            path: list[tuple[float, float]] = []
            while (r, c) in came_from:
                path.append(_grid_to_world(c, r, grid_res))
                r, c = came_from[(r, c)]
            path.append(_grid_to_world(sc, sr, grid_res))
            path.reverse()
            return _smooth_path(path, grid, rows, cols, grid_res)

        if g > g_cost.get((r, c), math.inf):
            continue

        for (dr, dc), cost in zip(DIRS, COSTS):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                ng = g + cost
                if ng < g_cost.get((nr, nc), math.inf):
                    g_cost[(nr, nc)] = ng
                    f = ng + heuristic(nr, nc, gr, gc)
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_set, (f, ng, nr, nc))

    return []   # No path found
