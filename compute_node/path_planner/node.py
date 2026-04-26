"""
PathPlannerNode — MQTT-нода планирования путей на ноутбуке (#3, 2026-04).

Подписки:
  samurai/{robot_id}/slam_map               — Pi-side ultrasonic SLAM
                                               (или ROS2 SLAM Toolbox)
  samurai/{robot_id}/odom                   — текущая поза робота
  samurai/{robot_id}/path_planner/goal      — {x, y} цель планирования
  samurai/{robot_id}/zones/update           — список запретных зон

Публикации:
  samurai/{robot_id}/path_planner/path      — {waypoints: [[x,y],...],
                                                goal: [x,y], success: bool}
  samurai/{robot_id}/path_planner/status    — {state, error?, planning_ms?}

Конфигурация:
  --robot-radius  — радиус робота в метрах (для inflate; default 0.15)
  --simplify      — упрощать ли путь (default True)

Запуск:
    python -m compute_node.path_planner

ENV:
    MQTT_BROKER, MQTT_PORT, ROBOT_ID — стандартные для compute_node
    SAMURAI_MQTT_USER / _PASS         — опциональная auth
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from typing import Optional

import numpy as np
import paho.mqtt.client as mqtt_client

from .astar import find_path, inflate_obstacles, simplify_path

log = logging.getLogger(__name__)

# Дефолтные значения grid-параметров (синхронизированы со slam_map_node).
DEFAULT_CELL_SIZE_M = 0.05
DEFAULT_MAP_CELLS = 200
DEFAULT_ROBOT_RADIUS_M = 0.15  # 15 см — типичный размер мини-робота


def _resolve_mqtt_creds() -> tuple[Optional[str], Optional[str]]:
    """Optional auth — config_loader приоритетнее, fallback ENV."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
        from config_loader import get_mqtt_credentials
        return get_mqtt_credentials()
    except ImportError:
        u = os.environ.get('SAMURAI_MQTT_USER', '').strip()
        p = os.environ.get('SAMURAI_MQTT_PASS', '')
        return (u, p) if u and p else (None, None)


class PathPlannerNode:
    """Слушает slam_map + odom + goal, планирует A*, публикует path."""

    def __init__(
        self,
        broker: str,
        port: int = 1883,
        robot_id: str = 'robot1',
        robot_radius_m: float = DEFAULT_ROBOT_RADIUS_M,
        simplify: bool = True,
        client_id: str = 'samurai_path_planner',
    ):
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._prefix = f'samurai/{robot_id}'
        self._radius_m = robot_radius_m
        self._simplify = simplify

        # Состояние мира (обновляется handlers'ами).
        # Grid сохраняем как 2D numpy, origin/resolution тоже от Pi.
        self._grid: Optional[np.ndarray] = None
        self._origin_x: float = -5.0
        self._origin_y: float = -5.0
        self._resolution: float = DEFAULT_CELL_SIZE_M
        self._map_cells: int = DEFAULT_MAP_CELLS

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_pose_valid: bool = False

        self._zones: list[dict] = []  # forbidden zones [{x1,y1,x2,y2}, ...]

        # MQTT
        self._client = mqtt_client.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)
        user, pwd = _resolve_mqtt_creds()
        if user:
            self._client.username_pw_set(user, pwd or '')
        self._auth_str = f' user={user}' if user else ' anonymous'

    # ── Lifecycle ──────────────────────────────────────────────────
    def start(self) -> None:
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        log.info(
            'PathPlanner connecting → %s:%d%s (radius=%.2fm, simplify=%s)',
            self._broker, self._port, self._auth_str,
            self._radius_m, self._simplify,
        )

    def stop(self) -> None:
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    def spin(self) -> None:
        """Блокирующий цикл — держит процесс живым."""
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    # ── MQTT callbacks ─────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed rc=%s', rc)
            return
        for topic in ('slam_map', 'odom', 'path_planner/goal', 'zones/update'):
            client.subscribe(f'{self._prefix}/{topic}', qos=0)
        log.info('PathPlanner subscribed to slam_map / odom / goal / zones')

    def _on_message(self, client, userdata, msg):
        suffix = msg.topic[len(self._prefix) + 1:]
        try:
            data = json.loads(msg.payload)
        except json.JSONDecodeError:
            log.warning('Invalid JSON on %s', suffix)
            return
        try:
            if suffix == 'slam_map':
                self._on_slam_map(data)
            elif suffix == 'odom':
                self._on_odom(data)
            elif suffix == 'path_planner/goal':
                self._on_goal(data)
            elif suffix == 'zones/update':
                self._on_zones(data)
        except Exception as exc:
            log.exception('Handler error [%s]: %s', suffix, exc)

    def _on_slam_map(self, data: dict) -> None:
        """Обновить occupancy grid из slam_map payload (obstacles + info)."""
        info = data.get('info', {})
        if not info:
            return
        w = int(info.get('width', DEFAULT_MAP_CELLS))
        h = int(info.get('height', DEFAULT_MAP_CELLS))
        res = float(info.get('resolution', DEFAULT_CELL_SIZE_M))
        ox = float(info.get('origin_x', -5.0))
        oy = float(info.get('origin_y', -5.0))

        grid = np.zeros((h, w), dtype=np.int8)
        for obs in data.get('obstacles', []):
            wx, wy = float(obs[0]), float(obs[1])
            ci = int((wx - ox) / res)
            cj = int((wy - oy) / res)
            if 0 <= ci < w and 0 <= cj < h:
                grid[cj, ci] = 1

        self._grid = grid
        self._origin_x = ox
        self._origin_y = oy
        self._resolution = res
        self._map_cells = w  # квадратный grid

    def _on_odom(self, data: dict) -> None:
        # Pi публикует одометрию в см — переводим в метры
        self._robot_x = float(data.get('x', 0.0)) / 100.0
        self._robot_y = float(data.get('y', 0.0)) / 100.0
        self._robot_pose_valid = True

    def _on_zones(self, data) -> None:
        """zones/update от dashboard. Может прийти как list или dict."""
        if isinstance(data, list):
            self._zones = data
        elif isinstance(data, dict):
            self._zones = data.get('zones', [])

    def _on_goal(self, data: dict) -> None:
        """Запрос на планирование. data: {x, y[, theta]}. Сразу планируем."""
        try:
            gx = float(data['x'])
            gy = float(data['y'])
        except (KeyError, ValueError, TypeError):
            self._publish_status('error', 'goal must contain numeric x, y')
            return
        self.plan_to(gx, gy)

    # ── Planning ───────────────────────────────────────────────────
    def plan_to(self, gx: float, gy: float) -> bool:
        """Спланировать путь от текущей позы робота до (gx, gy).

        Возвращает True если путь найден и опубликован.
        """
        if self._grid is None:
            self._publish_status('error', 'no slam_map yet')
            return False
        if not self._robot_pose_valid:
            self._publish_status('error', 'no odom yet')
            return False

        # Подготовка grid: добавим запретные зоны как occupied
        grid = self._grid.copy()
        self._mark_zones(grid)

        # Inflate под радиус робота
        inflate_cells = max(1, int(round(self._radius_m / self._resolution)))
        grid = inflate_obstacles(grid, inflate_cells)

        start_ci, start_cj = self._world_to_cell(self._robot_x, self._robot_y)
        goal_ci, goal_cj = self._world_to_cell(gx, gy)

        t0 = time.time()
        cells = find_path(grid, (start_ci, start_cj), (goal_ci, goal_cj))
        dt_ms = (time.time() - t0) * 1000

        if not cells:
            log.warning(
                'No path: robot=(%.2f,%.2f) goal=(%.2f,%.2f) [%dms]',
                self._robot_x, self._robot_y, gx, gy, dt_ms,
            )
            self._publish_status('failed', f'no path found ({dt_ms:.0f}ms)')
            self._publish_path([], (gx, gy), success=False)
            return False

        if self._simplify:
            cells = simplify_path(cells, grid=grid)

        # Конвертируем ячейки → мировые координаты (центры ячеек)
        waypoints = [self._cell_to_world(ci, cj) for ci, cj in cells]

        log.info(
            'Path planned: %d cells → %d waypoints [%dms]',
            len(cells), len(waypoints), dt_ms,
        )
        self._publish_path(waypoints, (gx, gy), success=True)
        self._publish_status('success', planning_ms=dt_ms)
        return True

    # ── Helpers ────────────────────────────────────────────────────
    def _world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        ci = int((wx - self._origin_x) / self._resolution)
        cj = int((wy - self._origin_y) / self._resolution)
        # Clamp в пределы grid (на случай если робот за границей)
        ci = max(0, min(self._map_cells - 1, ci))
        cj = max(0, min(self._map_cells - 1, cj))
        return ci, cj

    def _cell_to_world(self, ci: int, cj: int) -> list[float]:
        wx = self._origin_x + (ci + 0.5) * self._resolution
        wy = self._origin_y + (cj + 0.5) * self._resolution
        return [round(wx, 3), round(wy, 3)]

    def _mark_zones(self, grid: np.ndarray) -> None:
        """Запретные зоны от dashboard → ячейки в grid."""
        h, w = grid.shape
        for z in self._zones:
            try:
                x1, y1 = float(z['x1']), float(z['y1'])
                x2, y2 = float(z['x2']), float(z['y2'])
            except (KeyError, ValueError, TypeError):
                continue
            i1, j1 = self._world_to_cell(min(x1, x2), min(y1, y2))
            i2, j2 = self._world_to_cell(max(x1, x2), max(y1, y2))
            grid[j1:j2 + 1, i1:i2 + 1] = 1

    def _publish_path(
        self,
        waypoints: list[list[float]],
        goal: tuple[float, float],
        success: bool,
    ) -> None:
        payload = {
            'waypoints': waypoints,
            'goal': [round(goal[0], 3), round(goal[1], 3)],
            'success': success,
            'ts': time.time(),
        }
        self._client.publish(
            f'{self._prefix}/path_planner/path',
            json.dumps(payload),
            qos=1,
        )

    def _publish_status(
        self,
        state: str,
        message: Optional[str] = None,
        *,
        planning_ms: Optional[float] = None,
    ) -> None:
        payload: dict = {'state': state, 'ts': time.time()}
        if message:
            payload['message'] = message
        if planning_ms is not None:
            payload['planning_ms'] = round(planning_ms, 1)
        self._client.publish(
            f'{self._prefix}/path_planner/status',
            json.dumps(payload),
            qos=1,
        )


# ── Entry point ────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(description='Samurai path planner (A*).')
    parser.add_argument('--broker', default=os.environ.get('MQTT_BROKER', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')))
    parser.add_argument('--robot-id', default=os.environ.get('ROBOT_ID', 'robot1'))
    parser.add_argument('--robot-radius', type=float, default=DEFAULT_ROBOT_RADIUS_M)
    parser.add_argument('--no-simplify', action='store_true',
                        help='Не упрощать путь (оставить все ячейки A*)')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    node = PathPlannerNode(
        broker=args.broker,
        port=args.port,
        robot_id=args.robot_id,
        robot_radius_m=args.robot_radius,
        simplify=not args.no_simplify,
    )
    node.start()
    try:
        node.spin()
    finally:
        node.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
