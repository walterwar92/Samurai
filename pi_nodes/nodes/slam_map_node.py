#!/usr/bin/env python3
"""
slam_map_node — Lightweight occupancy grid SLAM from ultrasonic + odometry.

Builds a 2D occupancy grid map using:
    - Robot pose from odometry (x, y, theta)
    - Ultrasonic range readings (single beam, ~15° cone)
    - Detected objects (from object_detector_node via MQTT)

The map uses a log-odds representation for efficient probabilistic updates:
    - Free cells: log-odds decreases (beam passed through)
    - Occupied cells: log-odds increases (beam hit obstacle)
    - Unknown cells: log-odds = 0

Detected objects are stored in a persistent registry with world coordinates
and published as a separate layer in the slam_map JSON for the dashboard.

Subscribes:
    samurai/{robot_id}/odom       — {x, y, theta, ...}
    samurai/{robot_id}/range      — {range, ts}
    samurai/{robot_id}/detections — объекты с мировыми координатами
    samurai/{robot_id}/reset_position — reset map origin to current pose
Publishes:
    samurai/{robot_id}/slam_map      — compressed map data (JSON)
    samurai/{robot_id}/slam_map/info — map metadata
"""

import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Map Configuration ─────────────────────────────────────────
MAP_SIZE_M = 10.0          # map covers ±5m from origin (10×10m total)
CELL_SIZE_M = 0.05         # 5 cm per cell → 200×200 grid
MAP_CELLS = int(MAP_SIZE_M / CELL_SIZE_M)  # 200

# Log-odds update values
L_OCC = 0.85               # log-odds increment for occupied cell
L_FREE = -0.40             # log-odds increment for free cell (beam passed)
L_MIN = -2.0               # minimum log-odds (very confident free)
L_MAX = 3.5                # maximum log-odds (very confident occupied)
L_THRESH_OCC = 0.6         # above this → occupied (for output)
L_THRESH_FREE = -0.5       # below this → free (for output)

# Sensor model
SENSOR_FOV_RAD = 0.26      # ~15° half-cone (HC-SR04)
SENSOR_MAX_RANGE = 2.0     # m
SENSOR_MIN_RANGE = 0.02    # m
OBSTACLE_THICKNESS = 0.10  # m — how thick the obstacle marking is

# Update control
UPDATE_INTERVAL_S = 0.1    # process range readings at 10 Hz
PUBLISH_INTERVAL_S = 2.0   # publish map at 0.5 Hz (heavy payload)
MIN_MOVE_M = 0.01          # skip update if robot hasn't moved
ANGULAR_RESOLUTION = 0.05  # rad — angular step for ray tracing (~3°)

# Detected objects registry
OBJ_CLUSTER_RADIUS_M = 0.30  # м — два наблюдения ближе этого = один объект
OBJ_MAX_AGE_S        = 120.0 # с — объект удаляется если не виден N секунд
OBJ_MIN_CONF         = 0.35  # минимальная уверенность для регистрации

# Map save
MAPS_DIR = os.path.expanduser('~/maps')


class SlamMapNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('slam_map_node', **kwargs)

        # Occupancy grid (log-odds, flat array for speed)
        self._grid = [0.0] * (MAP_CELLS * MAP_CELLS)
        self._origin_x = -MAP_SIZE_M / 2.0  # world x of cell (0,0)
        self._origin_y = -MAP_SIZE_M / 2.0  # world y of cell (0,0)

        # Robot state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._pose_valid = False
        self._last_update_x = 0.0
        self._last_update_y = 0.0

        # Range state
        self._range_m = SENSOR_MAX_RANGE
        self._range_ts = 0.0

        # Robot path trail (for map overlay)
        self._trail = []  # list of (x, y)
        self._last_trail_x = 0.0
        self._last_trail_y = 0.0

        # Detected objects registry
        # key: str (уникальный ID объекта), value: dict с полями:
        #   class, colour, x, y, conf, ts, count
        self._objects: dict = {}
        self._obj_counter = 0  # для генерации уникальных ID

        # Subscribers
        self.subscribe('odom',           self._odom_cb)
        self.subscribe('range',          self._range_cb)
        self.subscribe('detections',     self._detections_cb)
        self.subscribe('reset_position', self._reset_cb, qos=1)

        # Timers
        self.create_timer(UPDATE_INTERVAL_S, self._update_map)
        self.create_timer(PUBLISH_INTERVAL_S, self._publish_map)

        self.log_info('SLAM map node ready (%dx%d grid, %.2fm/cell, %.0fm² area)',
                      MAP_CELLS, MAP_CELLS, CELL_SIZE_M, MAP_SIZE_M ** 2)

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        self._x = data.get('x', 0.0)
        self._y = data.get('y', 0.0)
        self._theta = data.get('theta', 0.0)
        self._pose_valid = True

        # Record trail
        dx = self._x - self._last_trail_x
        dy = self._y - self._last_trail_y
        if math.sqrt(dx * dx + dy * dy) > 0.03:  # every 3 cm
            self._trail.append((round(self._x, 3), round(self._y, 3)))
            self._last_trail_x = self._x
            self._last_trail_y = self._y
            # Limit trail size
            if len(self._trail) > 2000:
                self._trail = self._trail[-1500:]

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = data.get('range', SENSOR_MAX_RANGE)
        else:
            try:
                r = float(data)
            except (ValueError, TypeError):
                return
        if SENSOR_MIN_RANGE <= r <= SENSOR_MAX_RANGE:
            self._range_m = r
            self._range_ts = time.monotonic()

    def _detections_cb(self, topic, data):
        """Обновляет реестр обнаруженных объектов из detections."""
        if not isinstance(data, dict):
            return
        objects = data.get('objects', [])
        if not isinstance(objects, list):
            return

        now = time.time()
        for det in objects:
            # Пропускаем объекты без мировых координат или с низкой уверенностью
            wx = det.get('world_x')
            wy = det.get('world_y')
            conf = float(det.get('conf', 0.0))
            if wx is None or wy is None or conf < OBJ_MIN_CONF:
                continue

            cls    = det.get('class',  'unknown')
            colour = det.get('colour', 'unknown')
            dist   = det.get('distance', -1.0)

            # Ищем ближайший существующий объект того же класса и цвета
            matched_id = None
            for obj_id, obj in self._objects.items():
                if obj['class'] != cls or obj['colour'] != colour:
                    continue
                dx = obj['x'] - wx
                dy = obj['y'] - wy
                if math.sqrt(dx * dx + dy * dy) < OBJ_CLUSTER_RADIUS_M:
                    matched_id = obj_id
                    break

            if matched_id:
                # Обновляем позицию: скользящее среднее (новые наблюдения весят больше)
                obj = self._objects[matched_id]
                alpha = 0.3  # вес нового наблюдения
                obj['x']     = round(alpha * wx + (1 - alpha) * obj['x'], 3)
                obj['y']     = round(alpha * wy + (1 - alpha) * obj['y'], 3)
                obj['conf']  = round(max(obj['conf'], conf), 3)
                obj['count'] += 1
                obj['ts']    = now
                obj['dist']  = round(dist, 3)
            else:
                # Новый объект
                self._obj_counter += 1
                obj_id = f'{cls}_{colour}_{self._obj_counter}'
                self._objects[obj_id] = {
                    'id':     obj_id,
                    'class':  cls,
                    'colour': colour,
                    'x':      round(wx, 3),
                    'y':      round(wy, 3),
                    'conf':   round(conf, 3),
                    'dist':   round(dist, 3),
                    'count':  1,
                    'ts':     now,
                }
                self.log_info('Новый объект: %s %s @ (%.2f, %.2f) dist=%.2fm',
                              colour, cls, wx, wy, dist)

    def _cleanup_stale_objects(self):
        """Удаляет объекты, не обновлявшиеся дольше OBJ_MAX_AGE_S."""
        now = time.time()
        stale = [oid for oid, obj in self._objects.items()
                 if now - obj['ts'] > OBJ_MAX_AGE_S]
        for oid in stale:
            obj = self._objects.pop(oid)
            self.log_info('Объект устарел, удалён: %s %s', obj['colour'], obj['class'])

    def _reset_cb(self, topic, data):
        """Reset map when position resets."""
        self._grid = [0.0] * (MAP_CELLS * MAP_CELLS)
        self._trail = []
        self._last_trail_x = 0.0
        self._last_trail_y = 0.0
        self._objects = {}
        self.log_info('SLAM map reset (включая реестр объектов)')

    # ── Map Update (ray tracing) ───────────────────────────────
    def _update_map(self):
        if not self._pose_valid:
            return

        # Skip if robot hasn't moved enough
        dx = self._x - self._last_update_x
        dy = self._y - self._last_update_y
        if math.sqrt(dx * dx + dy * dy) < MIN_MOVE_M:
            # Still update if we have a fresh range reading
            age = time.monotonic() - self._range_ts
            if age > 0.5:
                return

        self._last_update_x = self._x
        self._last_update_y = self._y

        range_m = self._range_m
        robot_x = self._x
        robot_y = self._y
        robot_theta = self._theta

        # Ultrasonic cone: trace multiple rays within the FOV
        angle_start = robot_theta - SENSOR_FOV_RAD
        angle_end = robot_theta + SENSOR_FOV_RAD

        angle = angle_start
        while angle <= angle_end:
            self._trace_ray(robot_x, robot_y, angle, range_m)
            angle += ANGULAR_RESOLUTION

    def _trace_ray(self, ox: float, oy: float, angle: float, range_m: float):
        """Trace a single ray from (ox, oy) at angle, marking free and occupied cells."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # Step along the ray in cell-sized increments
        step = CELL_SIZE_M * 0.7  # slightly less than cell size for coverage
        dist = 0.0
        hit_range = min(range_m, SENSOR_MAX_RANGE)

        while dist < hit_range - OBSTACLE_THICKNESS:
            wx = ox + cos_a * dist
            wy = oy + sin_a * dist
            ci, cj = self._world_to_cell(wx, wy)
            if 0 <= ci < MAP_CELLS and 0 <= cj < MAP_CELLS:
                idx = cj * MAP_CELLS + ci
                self._grid[idx] = max(L_MIN, self._grid[idx] + L_FREE)
            dist += step

        # Mark occupied cells at the obstacle (if within valid range)
        if range_m < SENSOR_MAX_RANGE - 0.05:
            # Obstacle hit — mark cells near the endpoint
            for d in range(3):  # a few cells deep
                dd = hit_range + d * CELL_SIZE_M * 0.5
                wx = ox + cos_a * dd
                wy = oy + sin_a * dd
                ci, cj = self._world_to_cell(wx, wy)
                if 0 <= ci < MAP_CELLS and 0 <= cj < MAP_CELLS:
                    idx = cj * MAP_CELLS + ci
                    self._grid[idx] = min(L_MAX, self._grid[idx] + L_OCC)

    # ── Coordinate conversion ──────────────────────────────────
    def _world_to_cell(self, wx: float, wy: float):
        """Convert world coordinates to grid cell indices."""
        ci = int((wx - self._origin_x) / CELL_SIZE_M)
        cj = int((wy - self._origin_y) / CELL_SIZE_M)
        return ci, cj

    def _cell_to_world(self, ci: int, cj: int):
        """Convert grid cell indices to world coordinates (cell center)."""
        wx = self._origin_x + (ci + 0.5) * CELL_SIZE_M
        wy = self._origin_y + (cj + 0.5) * CELL_SIZE_M
        return wx, wy

    # ── Map Publishing ─────────────────────────────────────────
    def _publish_map(self):
        if not self._pose_valid:
            return

        # Удаляем устаревшие объекты перед публикацией
        self._cleanup_stale_objects()

        # Convert log-odds to occupancy values:
        # -1 = unknown, 0 = free, 100 = occupied
        occ_grid = []
        occupied_cells = []
        free_cells_count = 0

        for j in range(MAP_CELLS):
            for i in range(MAP_CELLS):
                lo = self._grid[j * MAP_CELLS + i]
                if lo > L_THRESH_OCC:
                    occ_grid.append(100)
                    wx, wy = self._cell_to_world(i, j)
                    occupied_cells.append([round(wx, 2), round(wy, 2)])
                elif lo < L_THRESH_FREE:
                    occ_grid.append(0)
                    free_cells_count += 1
                else:
                    occ_grid.append(-1)

        # Реестр обнаруженных объектов для дашборда
        detected_objects = list(self._objects.values())

        # Publish compact map data (obstacles + metadata only, not full grid)
        map_data = {
            'obstacles': occupied_cells,
            'trail': self._trail[-500:],  # last 500 trail points
            'robot': {
                'x': round(self._x, 3),
                'y': round(self._y, 3),
                'theta': round(self._theta, 3),
            },
            'detected_objects': detected_objects,
            'info': {
                'width': MAP_CELLS,
                'height': MAP_CELLS,
                'resolution': CELL_SIZE_M,
                'origin_x': self._origin_x,
                'origin_y': self._origin_y,
                'size_m': MAP_SIZE_M,
            },
            'stats': {
                'occupied':          len(occupied_cells),
                'free':              free_cells_count,
                'unknown':           MAP_CELLS * MAP_CELLS - len(occupied_cells) - free_cells_count,
                'detected_objects':  len(detected_objects),
            },
            'ts': self.timestamp(),
        }
        self.publish('slam_map', map_data)

        # Publish info separately (lighter, for quick checks)
        self.publish('slam_map/info', map_data['info'])

    # ── Save / Load map ────────────────────────────────────────
    def save_map(self, name: str):
        os.makedirs(MAPS_DIR, exist_ok=True)
        filepath = os.path.join(MAPS_DIR, f'{name}.json')
        data = {
            'name': name,
            'grid': self._grid,
            'width': MAP_CELLS,
            'height': MAP_CELLS,
            'resolution': CELL_SIZE_M,
            'origin_x': self._origin_x,
            'origin_y': self._origin_y,
            'saved_at': time.time(),
        }
        with open(filepath, 'w') as f:
            json.dump(data, f)
        self.log_info('Map saved: %s', filepath)

    def load_map(self, name: str):
        filepath = os.path.join(MAPS_DIR, f'{name}.json')
        if not os.path.exists(filepath):
            self.log_warn('Map file not found: %s', filepath)
            return
        with open(filepath, 'r') as f:
            data = json.load(f)
        self._grid = data.get('grid', [0.0] * (MAP_CELLS * MAP_CELLS))
        self.log_info('Map loaded: %s', name)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = SlamMapNode(broker=args.broker, port=args.port,
                       robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
