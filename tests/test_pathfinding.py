"""Unit tests for compute_node.pathfinding (#44)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compute_node.pathfinding import (
    find_path, _build_grid, _world_to_grid, _grid_to_world,
    _nearest_free, _line_of_sight, _smooth_path,
)


class _Arena:
    """Minimal arena duck-type for tests."""
    def __init__(self, w: float = 3.0, h: float = 3.0):
        self.width = w
        self.height = h


def test_open_arena_returns_short_path():
    """Empty arena: path is direct, smoothed to 2 endpoints."""
    path = find_path(_Arena(), [], (0.3, 0.3), (2.7, 2.7), robot_radius=0.12)
    assert len(path) >= 2
    assert path[0][0] == pytest.approx(0.3, abs=0.1)
    assert path[-1][0] == pytest.approx(2.7, abs=0.1)


def test_zone_blocks_diagonal():
    """A wall-like zone spanning full height yields no path."""
    blocking = [{'x1': 1.0, 'y1': 0.0, 'x2': 2.0, 'y2': 3.0}]
    path = find_path(_Arena(), blocking, (0.5, 1.5), (2.5, 1.5),
                     robot_radius=0.12)
    assert path == []


def test_zone_routes_around():
    """A zone in the lower half of the arena is bypassable via the upper half."""
    partial = [{'x1': 0.5, 'y1': 0.0, 'x2': 2.5, 'y2': 1.0}]
    path = find_path(_Arena(), partial, (0.3, 0.3), (2.7, 0.3),
                     robot_radius=0.05)
    # Should find SOME path — going up around the zone
    assert path != []
    # Some waypoint should be above the zone
    assert any(y > 1.0 for _, y in path), \
        f'path stays inside zone — should detour: {path}'


def test_world_grid_round_trip():
    c, r = _world_to_grid(1.5, 2.0)
    wx, wy = _grid_to_world(c, r)
    # Should land within half a cell
    assert abs(wx - 1.5) < 0.05
    assert abs(wy - 2.0) < 0.05


def test_build_grid_walls_blocked():
    grid, rows, cols = _build_grid(_Arena(), [], robot_radius=0.12)
    # Margin = 0.12 + 0.10 = 0.22 m → first ~4 cells are walled.
    # Centre cells should be free.
    cr, cc = rows // 2, cols // 2
    assert grid[cr][cc] is False
    # Edges blocked
    assert grid[0][0] is True


def test_nearest_free_finds_free_cell():
    grid, rows, cols = _build_grid(_Arena(), [], robot_radius=0.12)
    # Start position is a wall (0, 0)
    fr, fc = _nearest_free(grid, 0, 0, rows, cols)
    assert fr is not None and fc is not None
    assert grid[fr][fc] is False


def test_line_of_sight_open_arena_clear():
    grid, rows, cols = _build_grid(_Arena(), [], robot_radius=0.05)
    # Two centre points should see each other
    assert _line_of_sight(1.0, 1.5, 2.0, 1.5, grid, rows, cols) is True


def test_line_of_sight_through_zone_blocked():
    blocking = [{'x1': 1.0, 'y1': 0.5, 'x2': 2.0, 'y2': 2.5}]
    grid, rows, cols = _build_grid(_Arena(), blocking, robot_radius=0.05)
    assert _line_of_sight(0.5, 1.5, 2.5, 1.5, grid, rows, cols) is False


def test_smooth_path_reduces_to_endpoints_in_open_arena():
    """In an open arena every intermediate is line-of-sight reachable."""
    grid, rows, cols = _build_grid(_Arena(), [], robot_radius=0.05)
    raw = [(0.3, 0.3), (0.5, 0.5), (1.0, 1.0), (1.5, 1.5), (2.0, 2.0)]
    smoothed = _smooth_path(raw, grid, rows, cols)
    # Should jump from start to end (or near it)
    assert len(smoothed) <= 2 + 1


def test_invalid_start_recovers_to_nearest_free():
    """Start point inside a wall — pathfinder should still produce a path
    starting from a nearby free cell."""
    path = find_path(_Arena(), [], (0.0, 0.0), (2.5, 2.5), robot_radius=0.12)
    # Even though (0,0) is in the wall margin, BFS finds a free cell and
    # we get a valid path.
    assert path != []
