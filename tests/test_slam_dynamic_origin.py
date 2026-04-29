"""
Юнит-тесты для динамического origin'а в pi_nodes/nodes/slam_map_node.py (#4).

Проверяем:
  - origin не сдвигается пока робот в безопасной зоне
  - origin корректно сдвигается когда робот близко к границе
  - данные log-odds копируются с правильным offset (sliding window)
  - reset_position возвращает origin к дефолту

Запуск:
    pytest tests/test_slam_dynamic_origin.py -v

Тесты создают экземпляр SlamMapNode без MQTT — мокаем MqttNode.__init__,
subscribe, create_timer, publish, чтобы node работал в standalone-режиме.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import patch

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, REPO_ROOT)


# ── Helpers ─────────────────────────────────────────────────────────────
def _make_node():
    """Создаёт SlamMapNode со всеми внешними зависимостями MqttNode замоканными."""
    # Импортируем модули через patch — иначе MqttNode пытается коннектиться
    with patch('pi_nodes.mqtt_node.mqtt.Client'):
        from pi_nodes.nodes import slam_map_node as sm

    # Создаём через __new__ чтобы пропустить тяжёлый __init__ MqttNode
    node = sm.SlamMapNode.__new__(sm.SlamMapNode)
    # Минимально нужные атрибуты, ручная инициализация (вырезка из __init__)
    node._grid = [0.0] * (sm.MAP_CELLS * sm.MAP_CELLS)
    node._origin_x = -sm.MAP_SIZE_M / 2.0
    node._origin_y = -sm.MAP_SIZE_M / 2.0
    node._x = 0.0
    node._y = 0.0
    node._theta = 0.0
    node._pose_valid = True
    node._last_update_x = 0.0
    node._last_update_y = 0.0
    node._range_m = sm.SENSOR_MAX_RANGE
    node._range_ts = 0.0
    node._trail = []
    node._last_trail_x = 0.0
    node._last_trail_y = 0.0
    node._objects = {}
    node._obj_counter = 0
    # log_info через no-op чтобы не было setup logger'ов
    node.log_info = lambda *a, **k: None
    return node, sm


# ── Tests ───────────────────────────────────────────────────────────────
def test_origin_default():
    """Свежесозданный node имеет дефолтный origin (-5, -5)."""
    node, sm = _make_node()
    assert node._origin_x == -sm.MAP_SIZE_M / 2.0
    assert node._origin_y == -sm.MAP_SIZE_M / 2.0


def test_no_shift_when_robot_in_safe_zone():
    """Если робот в центре или рядом — никакого сдвига не должно происходить."""
    node, _sm = _make_node()
    node._x = 0.0
    node._y = 0.0
    shifted = node._maybe_shift_origin()
    assert shifted is False
    assert node._origin_x == -5.0
    assert node._origin_y == -5.0


def test_no_shift_just_inside_margin():
    """Робот в 2м от центра — далеко от края (3м запаса), сдвига нет."""
    node, _sm = _make_node()
    node._x = 2.0
    node._y = -1.0
    shifted = node._maybe_shift_origin()
    assert shifted is False


def test_shift_when_robot_near_right_edge():
    """Робот около правого края (близко к +5м) → origin сдвигается вправо."""
    node, sm = _make_node()
    # При origin=-5 и x=4.0, робот в ячейке 180 (4+5=9м, 9/0.05=180).
    # Margin 1.5м = 30 ячеек. Робот в 200-180=20 ячеек от края → шифт.
    node._x = 4.0
    node._y = 0.0
    shifted = node._maybe_shift_origin()
    assert shifted is True
    # Origin должен сдвинуться вправо так, чтобы робот оказался ближе к центру
    assert node._origin_x > -5.0
    # Проверяем: робот теперь близко к центру grid (±2 ячейки)
    rel_x_cells = (node._x - node._origin_x) / sm.CELL_SIZE_M
    assert abs(rel_x_cells - sm.MAP_CELLS / 2) < 2


def test_shift_when_robot_near_left_edge():
    """Робот около левого края (близко к -5м) → origin сдвигается влево."""
    node, sm = _make_node()
    node._x = -4.5
    node._y = 0.0
    shifted = node._maybe_shift_origin()
    assert shifted is True
    assert node._origin_x < -5.0
    rel_x_cells = (node._x - node._origin_x) / sm.CELL_SIZE_M
    assert abs(rel_x_cells - sm.MAP_CELLS / 2) < 2


def test_shift_diagonal():
    """Робот в углу (близко и к right, и к top края) → сдвиг по обеим осям."""
    node, sm = _make_node()
    node._x = 4.5
    node._y = 4.5
    shifted = node._maybe_shift_origin()
    assert shifted is True
    assert node._origin_x > -5.0
    assert node._origin_y > -5.0
    rel_x_cells = (node._x - node._origin_x) / sm.CELL_SIZE_M
    rel_y_cells = (node._y - node._origin_y) / sm.CELL_SIZE_M
    assert abs(rel_x_cells - sm.MAP_CELLS / 2) < 2
    assert abs(rel_y_cells - sm.MAP_CELLS / 2) < 2


def test_grid_data_preserved_after_shift():
    """Записанные log-odds данные должны переехать в новые ячейки целыми."""
    node, sm = _make_node()

    # Записываем уникальное log-odds в ячейку (100, 100) — это центр grid
    # для дефолтного origin (-5, -5), что соответствует мировой точке (0, 0).
    LANDMARK_VALUE = 1.234
    landmark_idx = 100 * sm.MAP_CELLS + 100
    node._grid[landmark_idx] = LANDMARK_VALUE

    # Двигаем робота к правому краю → ожидаем shift на ~30 ячеек по X
    node._x = 4.5
    node._y = 0.0
    node._maybe_shift_origin()

    # Найдём landmark — он должен быть в ячейке (100 - shift_i, 100 - shift_j),
    # т.е. оказаться в новой grid в позиции (100 - shift_i + shift_i) = 100.
    # Проще проверим: ячейка соответствующая мировой точке (0, 0) должна
    # содержать LANDMARK_VALUE.
    ci = int((0.0 - node._origin_x) / sm.CELL_SIZE_M)
    cj = int((0.0 - node._origin_y) / sm.CELL_SIZE_M)
    assert 0 <= ci < sm.MAP_CELLS
    assert 0 <= cj < sm.MAP_CELLS
    new_value = node._grid[cj * sm.MAP_CELLS + ci]
    assert new_value == pytest.approx(LANDMARK_VALUE), (
        f'Landmark в (0,0) после сдвига должен сохранить значение, '
        f'но получили {new_value}'
    )


def test_grid_data_outside_old_bounds_is_zero():
    """Ячейки в новой grid за пределами старых границ должны быть unknown (0.0)."""
    node, sm = _make_node()

    # Заполним grid не-нулями, чтобы убедиться что новые ячейки реально 0.
    for i in range(len(node._grid)):
        node._grid[i] = 0.5  # arbitrary non-zero

    # Сдвигаем робота к правому краю — origin сдвигается вправо, новые
    # ячейки появляются справа (и в новой grid это правый столбец).
    node._x = 4.5
    node._y = 0.0
    node._maybe_shift_origin()

    # Правый край новой grid должен быть unknown (0.0) — там старых данных нет.
    # Конкретно: shift_i ~ -30 (origin сдвинулся вправо), значит исходные
    # ячейки i_src = i_dst - shift_i = i_dst + 30. Для i_dst в [170..199]
    # i_src в [200..229] — за границей старого grid → 0.
    for j in range(sm.MAP_CELLS):
        for i in range(sm.MAP_CELLS - 30, sm.MAP_CELLS):
            assert node._grid[j * sm.MAP_CELLS + i] == 0.0


def test_world_to_cell_after_shift():
    """После сдвига world_to_cell должен корректно отражать робот."""
    node, sm = _make_node()
    node._x = 4.5
    node._y = 4.5
    node._maybe_shift_origin()

    # Робот в (4.5, 4.5) — после сдвига его ячейка должна быть около центра.
    ci, cj = node._world_to_cell(node._x, node._y)
    assert abs(ci - sm.MAP_CELLS // 2) < 2
    assert abs(cj - sm.MAP_CELLS // 2) < 2


def test_reset_restores_default_origin():
    """reset_position должен вернуть origin к (-5, -5)."""
    node, sm = _make_node()
    # Сначала спровоцировать сдвиг
    node._x = 4.5
    node._y = 4.5
    node._maybe_shift_origin()
    assert node._origin_x != -5.0

    # Reset
    node._reset_cb('reset_position', None)
    assert node._origin_x == -sm.MAP_SIZE_M / 2.0
    assert node._origin_y == -sm.MAP_SIZE_M / 2.0
    # Grid тоже сброшен
    assert all(v == 0.0 for v in node._grid)


def test_repeated_shifts_track_robot():
    """Серия позиций робота — каждый раз робот должен оставаться около центра."""
    node, sm = _make_node()
    positions = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 2.0), (10.0, 4.0)]
    for x, y in positions:
        node._x = x
        node._y = y
        node._maybe_shift_origin()
        # После сдвига робот должен быть в пределах grid
        ci, cj = node._world_to_cell(x, y)
        assert 0 <= ci < sm.MAP_CELLS, f'x={x} out of grid after shift'
        assert 0 <= cj < sm.MAP_CELLS, f'y={y} out of grid after shift'
        # И не очень далеко от края
        margin_cells = int(sm.SHIFT_MARGIN_M / sm.CELL_SIZE_M)
        assert margin_cells <= ci <= sm.MAP_CELLS - margin_cells, (
            f'After shift ci={ci} too close to edge for x={x}'
        )
