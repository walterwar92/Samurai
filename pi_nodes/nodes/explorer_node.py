#!/usr/bin/env python3
"""
explorer_node — Автономное исследование местности (frontier / spiral / zigzag).

Использует карту от slam_map_node для поиска неисследованных областей
и автоматически перемещает робота для полного покрытия территории.

Стратегии исследования:
  - frontier: поиск границ между известными и неизвестными клетками,
              движение к ближайшему фронтиру
  - spiral:   расширяющаяся спираль от текущей позиции
  - zigzag:   систематическое покрытие зигзагом (вперёд-назад)

Безопасность:
  - Остановка при range < 0.18 м (препятствие)
  - Замедление при range < 0.35 м
  - Максимальный радиус исследования от Home (по умолчанию 4 м)
  - Автовозврат при заряде батареи < 25%

Subscribes:
    samurai/{robot_id}/slam_map            — карта (obstacles + info)
    samurai/{robot_id}/odom                — позиция робота
    samurai/{robot_id}/range               — ультразвук
    samurai/{robot_id}/battery             — заряд батареи
    samurai/{robot_id}/explorer/command    — "start"/"stop"/"spiral"/"zigzag"/"frontier"
Publishes:
    samurai/{robot_id}/cmd_vel             — команды движения
    samurai/{robot_id}/explorer/status     — прогресс исследования
"""

import json
import math
import os
import sys
import time
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Safety thresholds ────────────────────────────────────────
RANGE_STOP_M = 0.18          # emergency stop distance
RANGE_SLOW_M = 0.35          # slow down distance
RANGE_MAX = 2.0              # sensor max (no obstacle)

# ── Movement speeds ──────────────────────────────────────────
LINEAR_SPEED = 0.12           # m/s normal forward
LINEAR_SLOW = 0.05            # m/s when obstacle near
ANGULAR_SPEED = 0.6           # rad/s turning speed
ANGULAR_SLOW = 0.3            # rad/s slow turn

# ── Exploration limits ───────────────────────────────────────
MAX_RADIUS_M = 4.0            # max exploration radius from home
BATTERY_LOW_PCT = 25          # auto-return threshold (%)

# ── Map constants (match slam_map_node defaults) ─────────────
DEFAULT_CELL_SIZE = 0.05      # 5 cm
DEFAULT_MAP_CELLS = 200       # 200x200
DEFAULT_MAP_SIZE = 10.0       # 10 m

# ── Timers ───────────────────────────────────────────────────
CONTROL_HZ = 5                # main control loop rate
STATUS_INTERVAL_S = 2.0       # status publish interval
MAP_PROCESS_INTERVAL_S = 1.0  # frontier recalculation interval

# ── Frontier detection ───────────────────────────────────────
FRONTIER_MIN_SIZE = 3         # min cells to consider a frontier
GOAL_REACHED_M = 0.15         # distance to consider goal reached
GOAL_TIMEOUT_S = 30.0         # give up on goal after this time
HEADING_TOLERANCE_RAD = 0.20  # acceptable heading error

# ── Spiral / Zigzag parameters ───────────────────────────────
SPIRAL_INITIAL_RADIUS = 0.3  # m — starting radius
SPIRAL_RADIUS_STEP = 0.25    # m — how much radius grows per loop
ZIGZAG_WIDTH = 0.5            # m — lane spacing
ZIGZAG_LENGTH = 3.0           # m — forward travel per lane

# ── States ───────────────────────────────────────────────────
STATE_IDLE = 'IDLE'
STATE_FRONTIER = 'FRONTIER'
STATE_SPIRAL = 'SPIRAL'
STATE_ZIGZAG = 'ZIGZAG'
STATE_ROTATING = 'ROTATING'
STATE_DRIVING = 'DRIVING'
STATE_OBSTACLE_AVOID = 'OBSTACLE_AVOID'
STATE_RETURNING_HOME = 'RETURNING_HOME'
STATE_DONE = 'DONE'


class ExplorerNode(MqttNode):
    def __init__(self, max_radius=MAX_RADIUS_M, **kwargs):
        super().__init__('explorer_node', **kwargs)

        # ── Robot state ──────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._pose_valid = False
        self._range_m = RANGE_MAX
        self._battery_pct = 100.0

        # ── Home position (captured at first odom) ───────────
        self._home_x = 0.0
        self._home_y = 0.0
        self._home_set = False

        # ── Map data ─────────────────────────────────────────
        self._obstacles = []       # list of [x, y]
        self._map_info = None      # dict: width, height, resolution, origin_x/y
        self._map_stats = None     # dict: occupied, free, unknown
        self._map_ts = 0.0

        # ── Exploration state ────────────────────────────────
        self._state = STATE_IDLE
        self._strategy = 'frontier'   # 'frontier' / 'spiral' / 'zigzag'
        self._active = False
        self._max_radius = max_radius

        # ── Goal tracking ────────────────────────────────────
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_set = False
        self._goal_start_time = 0.0

        # ── Frontier state ───────────────────────────────────
        self._frontiers = []         # list of (x, y) frontier centroids
        self._last_frontier_calc = 0.0

        # ── Spiral state ─────────────────────────────────────
        self._spiral_angle = 0.0
        self._spiral_radius = SPIRAL_INITIAL_RADIUS
        self._spiral_center_x = 0.0
        self._spiral_center_y = 0.0

        # ── Zigzag state ─────────────────────────────────────
        self._zz_lane = 0            # current lane index
        self._zz_direction = 1       # +1 forward, -1 backward
        self._zz_start_x = 0.0
        self._zz_start_y = 0.0
        self._zz_start_theta = 0.0
        self._zz_phase = 'drive'     # 'drive' or 'turn'
        self._zz_driven = 0.0        # distance driven in current lane

        # ── Obstacle avoidance ───────────────────────────────
        self._avoid_dir = 1.0        # +1 or -1 rotation
        self._avoid_start = 0.0

        # ── Stats ────────────────────────────────────────────
        self._explored_pct = 0.0
        self._start_time = 0.0
        self._goals_reached = 0

        # ── Subscribers ──────────────────────────────────────
        self.subscribe('slam_map',          self._slam_map_cb)
        self.subscribe('odom',              self._odom_cb)
        self.subscribe('range',             self._range_cb)
        self.subscribe('battery',           self._battery_cb)
        self.subscribe('explorer/command',  self._command_cb)

        # ── Timers ───────────────────────────────────────────
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.create_timer(STATUS_INTERVAL_S, self._publish_status)

        self.log_info('Explorer node ready (max_radius=%.1fm, battery_low=%d%%)',
                      self._max_radius, BATTERY_LOW_PCT)

    # ══════════════════════════════════════════════════════════
    # MQTT callbacks
    # ══════════════════════════════════════════════════════════

    def _odom_cb(self, topic, data):
        if not isinstance(data, dict):
            return
        self._x = data.get('x', 0.0) / 100.0      # cm → m
        self._y = data.get('y', 0.0) / 100.0      # cm → m
        self._theta = data.get('theta', 0.0)
        self._pose_valid = True
        if not self._home_set:
            self._home_x = self._x
            self._home_y = self._y
            self._home_set = True

    def _range_cb(self, topic, data):
        if isinstance(data, dict):
            r = data.get('range', RANGE_MAX)
        else:
            try:
                r = float(data)
            except (ValueError, TypeError):
                return
        if math.isfinite(r) and r > 0:
            self._range_m = r

    def _battery_cb(self, topic, data):
        if isinstance(data, dict):
            self._battery_pct = data.get('percent', data.get('level', 100.0))
        else:
            try:
                self._battery_pct = float(data)
            except (ValueError, TypeError):
                pass

    def _slam_map_cb(self, topic, data):
        """Parse slam_map JSON from slam_map_node."""
        if not isinstance(data, dict):
            return
        self._obstacles = data.get('obstacles', [])
        self._map_info = data.get('info', None)
        self._map_stats = data.get('stats', None)
        self._map_ts = time.monotonic()

        # Calculate exploration progress
        if self._map_stats:
            total = (self._map_stats.get('occupied', 0) +
                     self._map_stats.get('free', 0) +
                     self._map_stats.get('unknown', 0))
            if total > 0:
                known = self._map_stats.get('occupied', 0) + self._map_stats.get('free', 0)
                self._explored_pct = round(100.0 * known / total, 1)

    def _command_cb(self, topic, data):
        """Handle explorer commands: start/stop/frontier/spiral/zigzag."""
        if isinstance(data, dict):
            cmd = data.get('command', data.get('cmd', '')).lower().strip()
        else:
            cmd = str(data).lower().strip()

        if cmd == 'stop':
            self.log_info('Команда: STOP — остановка исследования')
            self._stop_exploration()
        elif cmd == 'start':
            self.log_info('Команда: START (стратегия=%s)', self._strategy)
            self._start_exploration(self._strategy)
        elif cmd in ('frontier', 'spiral', 'zigzag'):
            self.log_info('Команда: %s — запуск стратегии', cmd.upper())
            self._start_exploration(cmd)
        else:
            self.log_warn('Неизвестная команда explorer: %s', cmd)

    # ══════════════════════════════════════════════════════════
    # Exploration lifecycle
    # ══════════════════════════════════════════════════════════

    def _start_exploration(self, strategy):
        self._strategy = strategy
        self._active = True
        self._start_time = time.monotonic()
        self._goals_reached = 0
        self._goal_set = False

        if strategy == 'spiral':
            self._state = STATE_SPIRAL
            self._spiral_angle = self._theta
            self._spiral_radius = SPIRAL_INITIAL_RADIUS
            self._spiral_center_x = self._x
            self._spiral_center_y = self._y
        elif strategy == 'zigzag':
            self._state = STATE_ZIGZAG
            self._zz_lane = 0
            self._zz_direction = 1
            self._zz_start_x = self._x
            self._zz_start_y = self._y
            self._zz_start_theta = self._theta
            self._zz_phase = 'drive'
            self._zz_driven = 0.0
        else:
            self._state = STATE_FRONTIER

        self.log_info('Исследование запущено: стратегия=%s', strategy)

    def _stop_exploration(self):
        self._active = False
        self._state = STATE_IDLE
        self._goal_set = False
        self._send_cmd_vel(0.0, 0.0)

    # ══════════════════════════════════════════════════════════
    # Main control loop
    # ══════════════════════════════════════════════════════════

    def _control_loop(self):
        if not self._active or not self._pose_valid:
            return

        # ── Safety checks ────────────────────────────────────
        # Battery low → return home
        if self._battery_pct < BATTERY_LOW_PCT and self._state != STATE_RETURNING_HOME:
            self.log_warn('Батарея низкая (%.0f%%) — возврат домой', self._battery_pct)
            self._state = STATE_RETURNING_HOME
            self._set_goal(self._home_x, self._home_y)

        # Max radius check
        dist_from_home = self._dist(self._x, self._y, self._home_x, self._home_y)
        if (dist_from_home > self._max_radius
                and self._state not in (STATE_RETURNING_HOME, STATE_IDLE, STATE_DONE)):
            self.log_warn('Превышен радиус исследования (%.2fm > %.1fm) — возврат',
                          dist_from_home, self._max_radius)
            self._state = STATE_RETURNING_HOME
            self._set_goal(self._home_x, self._home_y)

        # Obstacle: emergency stop or avoidance
        if self._range_m < RANGE_STOP_M:
            if self._state != STATE_OBSTACLE_AVOID:
                self.log_warn('Препятствие (%.2fm) — избегание', self._range_m)
                self._avoid_dir = 1.0 if (self._goals_reached % 2 == 0) else -1.0
                self._avoid_start = time.monotonic()
                self._state = STATE_OBSTACLE_AVOID
            self._do_obstacle_avoid()
            return

        if self._state == STATE_OBSTACLE_AVOID:
            # Obstacle cleared
            if self._range_m > RANGE_SLOW_M:
                self.log_info('Путь свободен — продолжение исследования')
                self._restore_strategy_state()
            else:
                self._do_obstacle_avoid()
                return

        # ── Strategy dispatch ────────────────────────────────
        if self._state == STATE_RETURNING_HOME:
            self._do_go_to_goal()
            if self._goal_reached():
                self.log_info('Вернулся домой')
                self._stop_exploration()
                self._state = STATE_DONE
        elif self._state == STATE_FRONTIER:
            self._do_frontier()
        elif self._state == STATE_SPIRAL:
            self._do_spiral()
        elif self._state == STATE_ZIGZAG:
            self._do_zigzag()

    # ══════════════════════════════════════════════════════════
    # Strategy: Frontier-based
    # ══════════════════════════════════════════════════════════

    def _do_frontier(self):
        now = time.monotonic()

        # Recalculate frontiers periodically
        if now - self._last_frontier_calc > MAP_PROCESS_INTERVAL_S:
            self._last_frontier_calc = now
            self._find_frontiers()

        # If we have a goal, drive to it
        if self._goal_set:
            if self._goal_reached():
                self.log_info('Фронтир достигнут (%.2f, %.2f)', self._goal_x, self._goal_y)
                self._goals_reached += 1
                self._goal_set = False
            elif now - self._goal_start_time > GOAL_TIMEOUT_S:
                self.log_warn('Таймаут цели — выбор нового фронтира')
                self._goal_set = False
            else:
                self._do_go_to_goal()
                return

        # Pick next frontier
        if not self._goal_set:
            if self._frontiers:
                # Sort by distance, pick nearest
                self._frontiers.sort(
                    key=lambda f: self._dist(self._x, self._y, f[0], f[1]))
                fx, fy = self._frontiers.pop(0)
                self._set_goal(fx, fy)
                self.log_info('Новый фронтир: (%.2f, %.2f), dist=%.2fm',
                              fx, fy, self._dist(self._x, self._y, fx, fy))
            else:
                # No frontiers found — exploration may be complete
                if self._map_stats and self._map_stats.get('unknown', 0) < 50:
                    self.log_info('Нет фронтиров — исследование завершено (%.1f%%)',
                                  self._explored_pct)
                    self._state = STATE_RETURNING_HOME
                    self._set_goal(self._home_x, self._home_y)
                else:
                    # Rotate in place to discover new frontiers
                    self._send_cmd_vel(0.0, ANGULAR_SPEED)

    def _find_frontiers(self):
        """Find frontier cells from the slam_map obstacle/info data.

        A frontier cell is a free cell adjacent to an unknown cell.
        We reconstruct a simple grid status from obstacles + info,
        then cluster frontier cells into centroids.
        """
        if self._map_info is None:
            self._frontiers = []
            return

        info = self._map_info
        res = info.get('resolution', DEFAULT_CELL_SIZE)
        width = info.get('width', DEFAULT_MAP_CELLS)
        height = info.get('height', DEFAULT_MAP_CELLS)
        origin_x = info.get('origin_x', -DEFAULT_MAP_SIZE / 2.0)
        origin_y = info.get('origin_y', -DEFAULT_MAP_SIZE / 2.0)

        # Build a set of occupied cell indices for fast lookup
        occ_set = set()
        for obs in self._obstacles:
            if isinstance(obs, (list, tuple)) and len(obs) >= 2:
                ci = int((obs[0] - origin_x) / res)
                cj = int((obs[1] - origin_y) / res)
                if 0 <= ci < width and 0 <= cj < height:
                    occ_set.add((ci, cj))

        # We don't have the full grid, but we can infer:
        # - Cells near the robot trail are likely free
        # - Cells with obstacles are occupied
        # - Everything else is unknown
        # For a proper frontier search, we use a BFS from robot position
        # marking cells within sensor range as known-free if not occupied.
        #
        # Simplified approach: use the map stats and obstacles to estimate
        # frontier regions. Scan a grid around the robot for cells that
        # are near the boundary of explored area.

        # Build a simple known/unknown grid around the robot
        # (limited scan area for performance)
        scan_radius_cells = int(self._max_radius / res)
        robot_ci = int((self._x - origin_x) / res)
        robot_cj = int((self._y - origin_y) / res)

        # Limit scan area
        ci_min = max(0, robot_ci - scan_radius_cells)
        ci_max = min(width, robot_ci + scan_radius_cells)
        cj_min = max(0, robot_cj - scan_radius_cells)
        cj_max = min(height, robot_cj + scan_radius_cells)

        # Heuristic: cells are "known free" if they are within a certain
        # distance from the robot's path and not occupied.
        # Since we only have obstacles list, we mark:
        #   - occupied cells from obstacles
        #   - free cells in a radius around robot (simplified)
        # A frontier is a free cell next to an unknown cell.

        # For performance, sample the grid at coarser resolution
        step = max(1, int(0.15 / res))  # ~15 cm steps

        # Estimate "known" radius from the explored percentage
        # Use obstacles as occupied markers, treat area near robot path as free
        known_radius_cells = int(1.5 / res)  # cells we consider "known" around robot

        frontier_cells = []
        for cj in range(cj_min, cj_max, step):
            for ci in range(ci_min, ci_max, step):
                if (ci, cj) in occ_set:
                    continue

                # Check if this cell is near the robot (known free)
                dx = ci - robot_ci
                dy = cj - robot_cj
                dist_cells = math.sqrt(dx * dx + dy * dy)

                if dist_cells > known_radius_cells:
                    continue  # too far, likely unknown

                # Check if any neighbor is "unknown" (far from robot, not occupied)
                is_frontier = False
                for di, dj in [(-step, 0), (step, 0), (0, -step), (0, step)]:
                    ni, nj = ci + di, cj + dj
                    if not (0 <= ni < width and 0 <= nj < height):
                        continue
                    ndx = ni - robot_ci
                    ndy = nj - robot_cj
                    ndist = math.sqrt(ndx * ndx + ndy * ndy)
                    if ndist > known_radius_cells and (ni, nj) not in occ_set:
                        is_frontier = True
                        break

                if is_frontier:
                    wx = origin_x + (ci + 0.5) * res
                    wy = origin_y + (cj + 0.5) * res
                    # Skip if outside max radius
                    if self._dist(wx, wy, self._home_x, self._home_y) <= self._max_radius:
                        frontier_cells.append((wx, wy))

        # Cluster frontiers: simple grid-based clustering
        self._frontiers = self._cluster_points(frontier_cells, cluster_radius=0.3)

    def _cluster_points(self, points, cluster_radius=0.3):
        """Cluster nearby points into centroids."""
        if not points:
            return []
        clusters = []
        used = [False] * len(points)

        for i, (px, py) in enumerate(points):
            if used[i]:
                continue
            cx, cy = px, py
            count = 1
            used[i] = True
            for j in range(i + 1, len(points)):
                if used[j]:
                    continue
                if self._dist(px, py, points[j][0], points[j][1]) < cluster_radius:
                    cx += points[j][0]
                    cy += points[j][1]
                    count += 1
                    used[j] = True
            if count >= FRONTIER_MIN_SIZE:
                clusters.append((cx / count, cy / count))

        return clusters

    # ══════════════════════════════════════════════════════════
    # Strategy: Spiral
    # ══════════════════════════════════════════════════════════

    def _do_spiral(self):
        """Drive in an expanding spiral from the start position."""
        # Compute next point on spiral
        self._spiral_angle += 0.15  # angular step per tick
        # Radius grows linearly with angle
        self._spiral_radius = (SPIRAL_INITIAL_RADIUS +
                               SPIRAL_RADIUS_STEP * self._spiral_angle / (2 * math.pi))

        # Check radius limit
        target_x = self._spiral_center_x + self._spiral_radius * math.cos(self._spiral_angle)
        target_y = self._spiral_center_y + self._spiral_radius * math.sin(self._spiral_angle)

        if self._dist(target_x, target_y, self._home_x, self._home_y) > self._max_radius:
            self.log_info('Спираль достигла макс. радиуса — завершение')
            self._state = STATE_RETURNING_HOME
            self._set_goal(self._home_x, self._home_y)
            return

        self._set_goal(target_x, target_y)
        self._do_go_to_goal()

    # ══════════════════════════════════════════════════════════
    # Strategy: Zigzag
    # ══════════════════════════════════════════════════════════

    def _do_zigzag(self):
        """Drive back and forth in parallel lanes."""
        if self._zz_phase == 'drive':
            # Drive forward in current direction
            heading = self._zz_start_theta + (0 if self._zz_direction > 0 else math.pi)
            heading = self._normalize_angle(heading)

            # Calculate how far we've driven in this lane
            target_x = (self._zz_start_x +
                        ZIGZAG_WIDTH * self._zz_lane *
                        math.cos(self._zz_start_theta + math.pi / 2))
            target_y = (self._zz_start_y +
                        ZIGZAG_WIDTH * self._zz_lane *
                        math.sin(self._zz_start_theta + math.pi / 2))

            # End of lane target
            end_x = target_x + ZIGZAG_LENGTH * self._zz_direction * math.cos(self._zz_start_theta)
            end_y = target_y + ZIGZAG_LENGTH * self._zz_direction * math.sin(self._zz_start_theta)

            # Check radius
            if self._dist(end_x, end_y, self._home_x, self._home_y) > self._max_radius:
                self.log_info('Зигзаг достиг макс. радиуса — завершение')
                self._state = STATE_RETURNING_HOME
                self._set_goal(self._home_x, self._home_y)
                return

            self._set_goal(end_x, end_y)

            if self._goal_reached():
                # Switch to turning phase
                self._zz_phase = 'turn'
                self._zz_lane += 1
                self._zz_direction *= -1
                self._goals_reached += 1
            else:
                self._do_go_to_goal()

        elif self._zz_phase == 'turn':
            # Turn to face next lane
            next_lane_x = (self._zz_start_x +
                           ZIGZAG_WIDTH * self._zz_lane *
                           math.cos(self._zz_start_theta + math.pi / 2))
            next_lane_y = (self._zz_start_y +
                           ZIGZAG_WIDTH * self._zz_lane *
                           math.sin(self._zz_start_theta + math.pi / 2))
            self._set_goal(next_lane_x, next_lane_y)

            if self._goal_reached():
                self._zz_phase = 'drive'
            else:
                self._do_go_to_goal()

    # ══════════════════════════════════════════════════════════
    # Go-to-goal controller
    # ══════════════════════════════════════════════════════════

    def _do_go_to_goal(self):
        """Simple proportional controller to drive toward current goal."""
        if not self._goal_set:
            self._send_cmd_vel(0.0, 0.0)
            return

        dx = self._goal_x - self._x
        dy = self._goal_y - self._y
        dist = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)
        heading_err = self._normalize_angle(target_heading - self._theta)

        # First: rotate to face goal
        if abs(heading_err) > HEADING_TOLERANCE_RAD:
            angular_z = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, heading_err * 2.0))
            self._send_cmd_vel(0.0, angular_z)
            return

        # Drive forward with proportional angular correction
        if self._range_m < RANGE_SLOW_M:
            speed = LINEAR_SLOW
        else:
            speed = min(LINEAR_SPEED, dist * 1.0)  # slow down near goal
            speed = max(LINEAR_SLOW, speed)

        angular_z = heading_err * 1.5  # proportional heading correction
        angular_z = max(-ANGULAR_SLOW, min(ANGULAR_SLOW, angular_z))

        self._send_cmd_vel(speed, angular_z)

    def _goal_reached(self):
        if not self._goal_set:
            return True
        return self._dist(self._x, self._y, self._goal_x, self._goal_y) < GOAL_REACHED_M

    def _set_goal(self, x, y):
        self._goal_x = x
        self._goal_y = y
        self._goal_set = True
        self._goal_start_time = time.monotonic()

    # ══════════════════════════════════════════════════════════
    # Obstacle avoidance
    # ══════════════════════════════════════════════════════════

    def _do_obstacle_avoid(self):
        """Rotate in place until obstacle is cleared."""
        elapsed = time.monotonic() - self._avoid_start
        if elapsed > 8.0:
            # Stuck for too long, try other direction
            self._avoid_dir *= -1
            self._avoid_start = time.monotonic()
            self.log_warn('Смена направления избегания (застрял)')

        if self._range_m < RANGE_STOP_M:
            # Emergency: back up slightly
            self._send_cmd_vel(-LINEAR_SLOW, ANGULAR_SPEED * self._avoid_dir)
        else:
            self._send_cmd_vel(0.0, ANGULAR_SPEED * self._avoid_dir)

    def _restore_strategy_state(self):
        """Return to the active exploration strategy after obstacle avoidance."""
        if self._strategy == 'frontier':
            self._state = STATE_FRONTIER
        elif self._strategy == 'spiral':
            self._state = STATE_SPIRAL
        elif self._strategy == 'zigzag':
            self._state = STATE_ZIGZAG
        else:
            self._state = STATE_FRONTIER

    # ══════════════════════════════════════════════════════════
    # Command publishing
    # ══════════════════════════════════════════════════════════

    def _send_cmd_vel(self, linear_x, angular_z):
        self.publish('cmd_vel', {
            'linear_x': round(linear_x, 4),
            'angular_z': round(angular_z, 4),
        })

    # ══════════════════════════════════════════════════════════
    # Status publishing
    # ══════════════════════════════════════════════════════════

    def _publish_status(self):
        elapsed = time.monotonic() - self._start_time if self._active else 0.0
        dist_home = self._dist(self._x, self._y, self._home_x, self._home_y)

        status = {
            'active': self._active,
            'state': self._state,
            'strategy': self._strategy,
            'explored_pct': self._explored_pct,
            'goals_reached': self._goals_reached,
            'distance_from_home': round(dist_home, 2),
            'max_radius': self._max_radius,
            'battery_pct': round(self._battery_pct, 1),
            'elapsed_s': round(elapsed, 1),
            'frontiers_count': len(self._frontiers),
            'robot': {
                'x': round(self._x, 3),
                'y': round(self._y, 3),
                'theta': round(self._theta, 3),
            },
            'ts': self.timestamp(),
        }
        if self._goal_set:
            status['goal'] = {
                'x': round(self._goal_x, 3),
                'y': round(self._goal_y, 3),
            }

        self.publish('explorer/status', status, retain=True)

    # ══════════════════════════════════════════════════════════
    # Utilities
    # ══════════════════════════════════════════════════════════

    @staticmethod
    def _dist(x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def on_shutdown(self):
        self._send_cmd_vel(0.0, 0.0)
        self._active = False
        self.log_info('Explorer остановлен')


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Explorer node — автономное исследование местности')
    parser.add_argument('--broker', default='127.0.0.1',
                        help='MQTT broker address')
    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT broker port')
    parser.add_argument('--robot-id', default='robot1',
                        help='Robot ID for MQTT topics')
    parser.add_argument('--max-radius', type=float, default=MAX_RADIUS_M,
                        help='Max exploration radius from home (m)')
    args = parser.parse_args()

    node = ExplorerNode(
        max_radius=args.max_radius,
        broker=args.broker,
        port=args.port,
        robot_id=args.robot_id,
    )
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
