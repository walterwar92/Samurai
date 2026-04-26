"""
A* для 2D occupancy grid.

Чистая функция: на входе grid + start/goal в ячейках, на выходе путь
в ячейках. Никаких external dependencies (MQTT, ROS2, state) — это
позволяет покрыть алгоритм юнит-тестами без mock'ов.

Конвенции:
  grid          — 2D numpy array (h × w), 0=free, 1=occupied (или >0)
  start, goal   — кортежи (i, j) — column index, row index в grid
  result        — list[(i, j)] от start до goal (включая оба) или []
                  если путь не найден

Используется евклидова эвристика и 8-connected neighbours (с диагоналями).
Diagonal-cost = sqrt(2), cardinal = 1.0.
"""
from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np

# Соседи: 4 cardinal + 4 diagonal. Стоимость заранее посчитана.
_NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (1, 1, math.sqrt(2)),
)


def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    """Евклидова дистанция в ячейках."""
    di = a[0] - b[0]
    dj = a[1] - b[1]
    return math.sqrt(di * di + dj * dj)


def inflate_obstacles(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Расширить препятствия на radius_cells (для радиуса робота).

    Каждая занятая ячейка `обвешивается` квадратом 2*r+1. Возвращает
    новую grid, исходная не модифицируется.

    radius_cells <= 0 → grid возвращается без изменений (но как копия,
    чтобы вызывающая сторона могла мутировать без сюрпризов).
    """
    if radius_cells <= 0:
        return grid.copy()
    h, w = grid.shape
    inflated = grid.copy()
    occupied = np.argwhere(grid > 0)
    for j, i in occupied:
        i_min = max(0, int(i) - radius_cells)
        i_max = min(w, int(i) + radius_cells + 1)
        j_min = max(0, int(j) - radius_cells)
        j_max = min(h, int(j) + radius_cells + 1)
        inflated[j_min:j_max, i_min:i_max] = 1
    return inflated


def find_path(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    max_iters: int = 200_000,
) -> list[tuple[int, int]]:
    """Найти путь A* от start до goal в 2D grid.

    Args:
        grid: numpy 2D, 0=free, 1=occupied. Dtype любой sensible.
        start: (col, row) — НАЧАЛЬНЫЕ координаты в grid
        goal: (col, row)  — КОНЕЧНЫЕ координаты в grid
        max_iters: верхний предел итераций (защита от runaway).

    Returns:
        Список ячеек [(col, row), ...] от start до goal включая оба.
        Пустой list если путь не найден или блокированы start/goal.
    """
    h, w = grid.shape

    def in_bounds(p: tuple[int, int]) -> bool:
        return 0 <= p[0] < w and 0 <= p[1] < h

    def is_free(p: tuple[int, int]) -> bool:
        return grid[p[1], p[0]] == 0

    if not in_bounds(start) or not in_bounds(goal):
        return []
    # Если start или goal в обстаклах — путь не найдётся. Не зовём is_free
    # на goal явно, но позволим ему быть «встроенным» в препятствие?
    # Требуем чтобы и start, и goal были free — иначе сразу fail.
    if not is_free(start) or not is_free(goal):
        return []
    if start == goal:
        return [start]

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    heapq.heappush(open_set, (heuristic(start, goal), counter, start))

    iters = 0
    while open_set and iters < max_iters:
        iters += 1
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            # Восстановить путь
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cur_g = g_score[current]
        for di, dj, cost in _NEIGHBOURS:
            nb = (current[0] + di, current[1] + dj)
            if not in_bounds(nb) or not is_free(nb):
                continue
            # Diagonal через corner: запретим резку угла если оба ортогональных
            # соседа заняты (классический trick для grid A*).
            if di != 0 and dj != 0:
                if (
                    not is_free((current[0] + di, current[1]))
                    or not is_free((current[0], current[1] + dj))
                ):
                    continue
            tentative_g = cur_g + cost
            if tentative_g < g_score.get(nb, float('inf')):
                came_from[nb] = current
                g_score[nb] = tentative_g
                f = tentative_g + heuristic(nb, goal)
                counter += 1
                heapq.heappush(open_set, (f, counter, nb))

    return []


def simplify_path(
    path: list[tuple[int, int]], grid: Optional[np.ndarray] = None
) -> list[tuple[int, int]]:
    """Удалить collinear точки. Если задан grid — также пробует длинные
    шорткаты через line-of-sight (Bresenham без препятствий).

    Возвращает упрощённый список waypoints.
    """
    if len(path) < 3:
        return list(path)

    # Удаляем collinear точки. Точка curr collinear относительно prev/next,
    # если векторы (curr-prev) и (next-prev) параллельны (cross product 0).
    # Это устойчивее чем сравнение step-vector'ов и работает для уже
    # сжатых сегментов (где step может иметь длину >1).
    simplified: list[tuple[int, int]] = [path[0]]
    for i in range(1, len(path) - 1):
        prev = simplified[-1]
        curr = path[i]
        nxt = path[i + 1]
        det = (
            (curr[0] - prev[0]) * (nxt[1] - prev[1])
            - (curr[1] - prev[1]) * (nxt[0] - prev[0])
        )
        if det != 0:
            simplified.append(curr)
    simplified.append(path[-1])

    # Если есть grid — пробуем агрессивный shortcut через LOS
    if grid is not None and len(simplified) > 2:
        result: list[tuple[int, int]] = [simplified[0]]
        i = 0
        while i < len(simplified) - 1:
            j = len(simplified) - 1
            while j > i + 1:
                if _line_of_sight(grid, simplified[i], simplified[j]):
                    break
                j -= 1
            result.append(simplified[j])
            i = j
        return result

    return simplified


def _line_of_sight(
    grid: np.ndarray, a: tuple[int, int], b: tuple[int, int]
) -> bool:
    """Bresenham line: True если все ячейки между a и b свободны."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    h, w = grid.shape
    while True:
        if not (0 <= x < w and 0 <= y < h):
            return False
        if grid[y, x] != 0:
            return False
        if (x, y) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
