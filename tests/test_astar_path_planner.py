"""
Юнит-тесты для compute_node/path_planner/astar.py (#3).

Запуск:
    pytest tests/test_astar_path_planner.py -v
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, REPO_ROOT)

from compute_node.path_planner.astar import (
    find_path,
    inflate_obstacles,
    simplify_path,
)


# ── Helpers ─────────────────────────────────────────────────────────────
def make_grid(rows: list[str]) -> np.ndarray:
    """Создать grid из ASCII: '.'/'0' = free, '#'/'1' = occupied."""
    h = len(rows)
    w = len(rows[0])
    g = np.zeros((h, w), dtype=np.int8)
    for j, row in enumerate(rows):
        for i, ch in enumerate(row):
            if ch in ('#', '1'):
                g[j, i] = 1
    return g


# ── find_path ──────────────────────────────────────────────────────────
def test_path_in_empty_grid():
    grid = np.zeros((10, 10), dtype=np.int8)
    path = find_path(grid, (0, 0), (9, 9))
    assert len(path) > 0
    assert path[0] == (0, 0)
    assert path[-1] == (9, 9)


def test_path_around_wall():
    """Стена посередине — путь должен обойти её."""
    grid = make_grid([
        '..........',
        '..........',
        '..........',
        '..........',
        '#########.',
        '..........',
        '..........',
        '..........',
        '..........',
        '..........',
    ])
    path = find_path(grid, (0, 0), (0, 9))
    assert len(path) > 0
    assert path[0] == (0, 0)
    assert path[-1] == (0, 9)
    # Каждая точка пути должна быть free
    for i, j in path:
        assert grid[j, i] == 0


def test_no_path_when_blocked():
    """Стена замыкает start — пути нет."""
    grid = make_grid([
        '##########',
        '#........#',
        '#........#',
        '##########',
    ])
    # Start внутри коробки
    path = find_path(grid, (5, 1), (5, 0))  # goal внутри стены
    assert path == []


def test_start_blocked_returns_empty():
    grid = np.ones((5, 5), dtype=np.int8)
    grid[2, 2] = 0
    path = find_path(grid, (2, 2), (0, 0))
    assert path == []


def test_goal_blocked_returns_empty():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[3, 3] = 1
    path = find_path(grid, (0, 0), (3, 3))
    assert path == []


def test_start_equals_goal():
    grid = np.zeros((5, 5), dtype=np.int8)
    path = find_path(grid, (2, 2), (2, 2))
    assert path == [(2, 2)]


def test_path_no_corner_cutting():
    """A* не должен резать угол через две заблокированные cardinal-ячейки."""
    grid = make_grid([
        '.....',
        '.....',
        '..#..',
        '.....',
        '.....',
    ])
    # Зальём дополнительно: между (1,2) и (2,1) формируем замкнутый угол.
    grid[2, 1] = 1  # блок слева от (2, 2)
    grid[1, 2] = 1  # блок снизу от (2, 2)... теперь чтобы пройти из (1,1) в (2,2) нужно обойти
    path = find_path(grid, (1, 1), (3, 3))
    # Проверяем, что переходов через corner нет: для каждой пары соседей
    # path[k]/path[k+1] если они diagonal — обе cardinal-ячейки между
    # ними должны быть free.
    for k in range(len(path) - 1):
        di = path[k + 1][0] - path[k][0]
        dj = path[k + 1][1] - path[k][1]
        if di != 0 and dj != 0:
            assert grid[path[k][1], path[k + 1][0]] == 0
            assert grid[path[k + 1][1], path[k][0]] == 0


# ── inflate_obstacles ──────────────────────────────────────────────────
def test_inflate_zero_radius_returns_copy():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 1
    inflated = inflate_obstacles(grid, 0)
    assert np.array_equal(inflated, grid)
    # Должна быть КОПИЯ, не та же самая ссылка
    inflated[0, 0] = 1
    assert grid[0, 0] == 0


def test_inflate_radius_1():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 1
    inflated = inflate_obstacles(grid, 1)
    # Все 9 ячеек 3×3 вокруг (2,2) теперь occupied
    for j in range(1, 4):
        for i in range(1, 4):
            assert inflated[j, i] == 1
    # Краевые ячейки остались free
    assert inflated[0, 0] == 0


def test_inflate_clips_at_boundary():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[0, 0] = 1
    inflated = inflate_obstacles(grid, 2)
    # Границы grid не выходят за пределы
    assert inflated.shape == (5, 5)
    # (0,0)..(2,2) должны быть occupied
    for j in range(3):
        for i in range(3):
            assert inflated[j, i] == 1


# ── simplify_path ──────────────────────────────────────────────────────
def test_simplify_collinear_points():
    """Точки на одной прямой удаляются."""
    path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    simplified = simplify_path(path)
    assert simplified == [(0, 0), (4, 0)]


def test_simplify_keeps_corners():
    """Углы сохраняются."""
    path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    simplified = simplify_path(path)
    assert simplified == [(0, 0), (2, 0), (2, 2)]


def test_simplify_short_path():
    assert simplify_path([(0, 0)]) == [(0, 0)]
    assert simplify_path([(0, 0), (1, 1)]) == [(0, 0), (1, 1)]


def test_simplify_with_grid_los_shortcut():
    """С grid + LOS — путь упрощается до диагонали."""
    grid = np.zeros((10, 10), dtype=np.int8)
    # Зигзагообразный путь, но в пустом grid его можно сократить до прямой
    path = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2), (3, 2), (3, 3)]
    simplified = simplify_path(path, grid=grid)
    # Должен сократиться до прямого LOS от start до goal
    assert simplified[0] == (0, 0)
    assert simplified[-1] == (3, 3)
    assert len(simplified) <= 3  # возможно (0,0) → (3,3) или 2-хоп


def test_simplify_with_obstacle_no_shortcut():
    """С препятствием LOS-shortcut не происходит."""
    grid = make_grid([
        '....',
        '.##.',
        '....',
        '....',
    ])
    # Зигзаг вокруг препятствия
    path = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (2, 3), (3, 3), (3, 2), (3, 1), (3, 0)]
    simplified = simplify_path(path, grid=grid)
    assert simplified[0] == (0, 0)
    assert simplified[-1] == (3, 0)
    # Все waypoint'ы должны иметь LOS друг с другом — но не к начальной точке через стену
    # (тут просто проверяем что путь сохранён валидным)
    assert len(simplified) >= 2


# ── End-to-end: find_path + inflate + simplify ──────────────────────────
def test_e2e_inflated_obstacle_avoidance():
    """С inflation робот обходит препятствие шире."""
    grid = make_grid([
        '..........',
        '..........',
        '....#.....',
        '....#.....',
        '....#.....',
        '..........',
        '..........',
    ])
    inflated = inflate_obstacles(grid, 1)
    path = find_path(inflated, (0, 3), (9, 3))
    assert len(path) > 0
    # Каждая точка не должна касаться inflation-зоны (одна клетка вокруг столба)
    for i, j in path:
        assert inflated[j, i] == 0
