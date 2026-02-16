#!/usr/bin/env python3
"""
Samurai Simulator — standalone robot simulation (no ROS2 / no hardware).

Simulates the full ball-hunting robot: arena, physics, sensors, YOLO detection,
FSM state machine, and serves a live web dashboard.

Usage:
    python simulator.py
    → Open http://localhost:5000
"""

import json
import math
import os
import random
import re
import threading
import time
from collections import deque

import cv2
import numpy as np

import base64

from flask import Flask, Response, render_template, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS


# ═════════════════════════════════════════════════════════════════
# API Helpers
# ═════════════════════════════════════════════════════════════════

def json_ok(data):
    return jsonify({"ok": True, "data": data})


def json_err(message, code=400):
    return jsonify({"ok": False, "error": message}), code


ALL_STATES = [
    'IDLE', 'SEARCHING', 'TARGETING', 'APPROACHING',
    'GRABBING', 'BURNING', 'CALLING', 'RETURNING',
]


# ═════════════════════════════════════════════════════════════════
# Constants (matching real robot parameters)
# ═════════════════════════════════════════════════════════════════

ARENA_W = 3.0        # metres
ARENA_H = 3.0
ROBOT_RADIUS = 0.12  # metres
WHEEL_BASE = 0.17
MAX_LINEAR = 0.3     # m/s
MAX_ANGULAR = 2.0    # rad/s
BALL_RADIUS = 0.02   # metres
BALL_DIAMETER_M = 0.04
FOCAL_LENGTH_PX = 500.0
CAM_W, CAM_H = 640, 480
CAM_FOV = math.radians(60)   # horizontal field of view
ULTRASONIC_MAX = 2.0
ULTRASONIC_MIN = 0.02
ULTRASONIC_FOV = 0.26  # ~15° half-cone

COLOUR_BGR = {
    'red':    (0, 0, 220),
    'blue':   (220, 100, 30),
    'green':  (50, 200, 50),
    'yellow': (0, 230, 230),
    'orange': (0, 140, 255),
}

SIM_DT = 0.05  # 20 Hz
PATH_GRID_RES = 0.05  # 5 cm per cell for A* grid
PATH_SAFETY_MARGIN = 0.10  # 10 cm extra clearance around obstacles


# ═════════════════════════════════════════════════════════════════
# A* Pathfinder — grid-based pathfinding around forbidden zones
# ═════════════════════════════════════════════════════════════════

import heapq

def _build_grid(arena, zones, robot_radius):
    """Build occupancy grid: True = blocked, False = free."""
    cols = int(arena.width / PATH_GRID_RES)
    rows = int(arena.height / PATH_GRID_RES)
    grid = [[False] * cols for _ in range(rows)]
    margin = robot_radius + PATH_SAFETY_MARGIN

    for r in range(rows):
        for c in range(cols):
            wx = (c + 0.5) * PATH_GRID_RES
            wy = (r + 0.5) * PATH_GRID_RES

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


def _world_to_grid(wx, wy):
    """Convert world metres → grid cell (col, row)."""
    return int(wx / PATH_GRID_RES), int(wy / PATH_GRID_RES)


def _grid_to_world(c, r):
    """Convert grid cell → world centre metres."""
    return (c + 0.5) * PATH_GRID_RES, (r + 0.5) * PATH_GRID_RES


def find_path(arena, zones, start_xy, goal_xy, robot_radius=ROBOT_RADIUS):
    """A* pathfinding from start to goal, avoiding walls and forbidden zones.
    Returns list of (x, y) world-coordinate waypoints, or [] if no path."""
    grid, rows, cols = _build_grid(arena, zones, robot_radius)

    sc, sr = _world_to_grid(*start_xy)
    gc, gr = _world_to_grid(*goal_xy)

    # Clamp to grid bounds
    sc = max(0, min(cols - 1, sc))
    sr = max(0, min(rows - 1, sr))
    gc = max(0, min(cols - 1, gc))
    gr = max(0, min(rows - 1, gr))

    # If start or goal is blocked, find nearest free cell
    if grid[sr][sc]:
        sr, sc = _nearest_free(grid, sr, sc, rows, cols)
    if grid[gr][gc]:
        gr, gc = _nearest_free(grid, gr, gc, rows, cols)

    if sr is None or gr is None:
        return []

    # A* with 8-directional movement
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
    COSTS = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

    def heuristic(r1, c1, r2, c2):
        dr = abs(r1 - r2)
        dc = abs(c1 - c2)
        return max(dr, dc) + 0.414 * min(dr, dc)  # octile distance

    open_set = [(heuristic(sr, sc, gr, gc), 0.0, sr, sc)]
    g_cost = {(sr, sc): 0.0}
    came_from = {}

    while open_set:
        _f, g, r, c = heapq.heappop(open_set)

        if r == gr and c == gc:
            # Reconstruct path
            path = []
            while (r, c) in came_from:
                path.append(_grid_to_world(c, r))
                r, c = came_from[(r, c)]
            path.append(_grid_to_world(sc, sr))
            path.reverse()
            return _smooth_path(path, grid, rows, cols)

        if g > g_cost.get((r, c), float('inf')):
            continue

        for (dr, dc), cost in zip(DIRS, COSTS):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                ng = g + cost
                if ng < g_cost.get((nr, nc), float('inf')):
                    g_cost[(nr, nc)] = ng
                    f = ng + heuristic(nr, nc, gr, gc)
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_set, (f, ng, nr, nc))

    return []  # No path found


def _nearest_free(grid, r, c, rows, cols):
    """BFS to find nearest free cell."""
    from collections import deque as dq
    visited = set()
    queue = dq([(r, c)])
    visited.add((r, c))
    while queue:
        cr, cc = queue.popleft()
        if not grid[cr][cc]:
            return cr, cc
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))
    return None, None


def _line_of_sight(x0, y0, x1, y1, grid, rows, cols):
    """Bresenham check: True if straight line between two world points is free."""
    c0, r0 = _world_to_grid(x0, y0)
    c1, r1 = _world_to_grid(x1, y1)
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
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c0 += sc
        if e2 < dc:
            err += dc
            r0 += sr
    return True


def _smooth_path(path, grid=None, rows=0, cols=0):
    """Reduce path points using line-of-sight pruning against the grid."""
    if len(path) <= 2:
        return path
    if grid is None:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        best = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if _line_of_sight(path[i][0], path[i][1],
                              path[j][0], path[j][1],
                              grid, rows, cols):
                best = j
                break
        smoothed.append(path[best])
        i = best
    return smoothed


# ═════════════════════════════════════════════════════════════════
# SimArena — 2D world with walls, balls, and forbidden zones
# ═════════════════════════════════════════════════════════════════

class SimArena:
    def __init__(self):
        self.width = ARENA_W
        self.height = ARENA_H
        self.balls = []
        self.forbidden_zones = []  # list of {id, x1, y1, x2, y2}
        self._zone_counter = 0
        self._spawn_balls()

    def _spawn_balls(self):
        colours = list(COLOUR_BGR.keys())
        positions = [
            (0.8, 0.6), (2.2, 0.8), (1.5, 2.0), (0.5, 2.3), (2.5, 1.8),
        ]
        for i, colour in enumerate(colours):
            x, y = positions[i]
            self.balls.append({
                'x': x, 'y': y,
                'colour': colour,
                'radius': BALL_RADIUS,
                'grabbed': False,
            })

    def add_zone(self, x1, y1, x2, y2):
        """Add a forbidden zone (rectangle). Returns zone id."""
        self._zone_counter += 1
        zone = {
            'id': self._zone_counter,
            'x1': min(x1, x2), 'y1': min(y1, y2),
            'x2': max(x1, x2), 'y2': max(y1, y2),
        }
        self.forbidden_zones.append(zone)
        return zone

    def remove_zone(self, zone_id):
        """Remove a forbidden zone by id. Returns True if found."""
        for i, z in enumerate(self.forbidden_zones):
            if z['id'] == zone_id:
                self.forbidden_zones.pop(i)
                return True
        return False

    def clear_zones(self):
        """Remove all forbidden zones."""
        self.forbidden_zones.clear()

    def point_in_zone(self, x, y):
        """Check if a world point is inside any forbidden zone."""
        for z in self.forbidden_zones:
            if z['x1'] <= x <= z['x2'] and z['y1'] <= y <= z['y2']:
                return True
        return False

    def reset(self):
        for b in self.balls:
            b['grabbed'] = False
        self._spawn_balls()
        # Note: forbidden zones are preserved on reset


# ═════════════════════════════════════════════════════════════════
# SimRobot — 2D robot physics
# ═════════════════════════════════════════════════════════════════

class SimRobot:
    def __init__(self, arena: SimArena):
        self.arena = arena
        self.x = ARENA_W / 2.0
        self.y = ARENA_H / 2.0
        self.theta = 0.0  # heading (radians)
        self.v_linear = 0.0
        self.v_angular = 0.0
        self.claw_open = False
        self.laser_on = False
        self._prev_v_linear = 0.0

    def set_velocity(self, linear: float, angular: float):
        self.v_linear = max(-MAX_LINEAR, min(MAX_LINEAR, linear))
        self.v_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    def stop(self):
        self.v_linear = 0.0
        self.v_angular = 0.0

    def tick(self, dt: float):
        self._prev_v_linear = self.v_linear
        # Save previous position for zone collision
        prev_x, prev_y = self.x, self.y
        # Integrate velocities
        self.x += self.v_linear * math.cos(self.theta) * dt
        self.y += self.v_linear * math.sin(self.theta) * dt
        self.theta += self.v_angular * dt
        # Normalise theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        # Wall collision — clamp inside arena
        margin = ROBOT_RADIUS
        self.x = max(margin, min(self.arena.width - margin, self.x))
        self.y = max(margin, min(self.arena.height - margin, self.y))
        # Forbidden zone collision — push robot back
        for z in self.arena.forbidden_zones:
            zx1 = z['x1'] - margin
            zy1 = z['y1'] - margin
            zx2 = z['x2'] + margin
            zy2 = z['y2'] + margin
            if zx1 <= self.x <= zx2 and zy1 <= self.y <= zy2:
                self.x = prev_x
                self.y = prev_y
                self.v_linear = 0.0
                break


# ═════════════════════════════════════════════════════════════════
# SimSensors — ultrasonic ray-cast + IMU
# ═════════════════════════════════════════════════════════════════

class SimSensors:
    def __init__(self):
        self.range_m = ULTRASONIC_MAX
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_gyro_z = 0.0
        self.accel_x = 0.0

    def update(self, robot: SimRobot, arena: SimArena):
        self._update_ultrasonic(robot, arena)
        self._update_imu(robot)

    def _update_ultrasonic(self, robot: SimRobot, arena: SimArena):
        """Ray-cast forward from robot to find nearest obstacle."""
        rx, ry = robot.x, robot.y
        dx = math.cos(robot.theta)
        dy = math.sin(robot.theta)
        best = ULTRASONIC_MAX

        # Check walls
        # Right wall (x = arena.width)
        if dx > 0:
            t = (arena.width - rx) / dx
            if ULTRASONIC_MIN < t < best:
                best = t
        # Left wall (x = 0)
        if dx < 0:
            t = -rx / dx
            if ULTRASONIC_MIN < t < best:
                best = t
        # Top wall (y = arena.height)
        if dy > 0:
            t = (arena.height - ry) / dy
            if ULTRASONIC_MIN < t < best:
                best = t
        # Bottom wall (y = 0)
        if dy < 0:
            t = -ry / dy
            if ULTRASONIC_MIN < t < best:
                best = t

        # Check balls
        for ball in arena.balls:
            if ball['grabbed']:
                continue
            bx, by = ball['x'], ball['y']
            # Distance from ray to ball centre
            to_ball_x = bx - rx
            to_ball_y = by - ry
            proj = to_ball_x * dx + to_ball_y * dy  # projection on ray
            if proj < ULTRASONIC_MIN or proj > best:
                continue
            perp = abs(to_ball_x * dy - to_ball_y * dx)  # perpendicular dist
            if perp < ball['radius'] + 0.05:  # ultrasonic cone
                if proj < best:
                    best = proj

        # Add noise
        noise = random.gauss(0, 0.005)
        self.range_m = max(ULTRASONIC_MIN, min(ULTRASONIC_MAX, best + noise))

    def _update_imu(self, robot: SimRobot):
        noise = random.gauss(0, 0.3)
        self.imu_yaw = math.degrees(robot.theta) + noise
        self.imu_pitch = random.gauss(0, 0.2)
        self.imu_roll = random.gauss(0, 0.2)
        self.imu_gyro_z = robot.v_angular
        self.accel_x = (robot.v_linear - robot._prev_v_linear) / SIM_DT


# ═════════════════════════════════════════════════════════════════
# SimDetector — geometric "YOLO" detection
# ═════════════════════════════════════════════════════════════════

class SimDetector:
    def __init__(self):
        self.detections = []
        self.annotated_frame = None

    def update(self, robot: SimRobot, arena: SimArena):
        """Detect visible balls and render camera view."""
        self.detections = []
        frame = self._render_camera(robot, arena)
        self.annotated_frame = frame

    def _render_camera(self, robot: SimRobot, arena: SimArena) -> np.ndarray:
        """Render first-person camera view."""
        # Dark grey floor
        frame = np.full((CAM_H, CAM_W, 3), (60, 60, 55), dtype=np.uint8)

        # Horizon line at 40% from top
        horizon_y = int(CAM_H * 0.4)
        # Sky (lighter grey)
        frame[:horizon_y, :] = (90, 85, 80)
        # Floor gradient
        for row in range(horizon_y, CAM_H):
            t = (row - horizon_y) / (CAM_H - horizon_y)
            grey = int(55 + t * 20)
            frame[row, :] = (grey, grey, grey - 5)

        # Render walls as perspective lines
        self._draw_walls(frame, robot, arena, horizon_y)

        # Render balls in view
        visible = []
        for ball in arena.balls:
            if ball['grabbed']:
                continue
            bx, by = ball['x'], ball['y']
            # Vector from robot to ball
            dx = bx - robot.x
            dy = by - robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.01 or dist > 3.0:
                continue

            # Angle to ball relative to robot heading
            angle = math.atan2(dy, dx) - robot.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > CAM_FOV / 2:
                continue

            visible.append((dist, angle, ball))

        # Sort by distance (far first, so near balls draw on top)
        visible.sort(key=lambda v: -v[0])

        for dist, angle, ball in visible:
            # Project to screen
            screen_x = int(CAM_W / 2 + (angle / (CAM_FOV / 2)) * (CAM_W / 2))

            # Apparent size
            apparent_px = int(FOCAL_LENGTH_PX * BALL_DIAMETER_M / dist)
            apparent_px = max(4, min(200, apparent_px))

            # Vertical position: balls are on the floor, lower = closer
            screen_y = horizon_y + int((1.0 - 0.03 / max(0.1, dist)) *
                                        (CAM_H - horizon_y) * 0.7)

            colour_bgr = COLOUR_BGR.get(ball['colour'], (200, 200, 200))

            # Draw ball (circle with highlight)
            cv2.circle(frame, (screen_x, screen_y), apparent_px, colour_bgr, -1)
            # Highlight
            hl_x = screen_x - apparent_px // 4
            hl_y = screen_y - apparent_px // 4
            hl_r = max(1, apparent_px // 4)
            hl_colour = tuple(min(255, c + 60) for c in colour_bgr)
            cv2.circle(frame, (hl_x, hl_y), hl_r, hl_colour, -1)

            # Shadow
            shadow_y = screen_y + apparent_px
            cv2.ellipse(frame, (screen_x, shadow_y),
                        (apparent_px, apparent_px // 4), 0, 0, 360,
                        (30, 30, 25), -1)

            # Detection bounding box
            x1 = screen_x - apparent_px
            y1 = screen_y - apparent_px
            w = apparent_px * 2
            h = apparent_px * 2
            conf = max(0.5, min(0.99, 1.0 - dist / 3.0))

            det = {
                'colour': ball['colour'],
                'class': 'sports ball',
                'x': max(0, x1), 'y': max(0, y1),
                'w': w, 'h': h,
                'conf': round(conf, 3),
                'distance': round(dist, 3),
            }
            self.detections.append(det)

            # Annotate
            label = f"{ball['colour']} {conf:.2f} {dist:.2f}m"
            cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # HUD overlay
        self._draw_hud(frame, robot)

        return frame

    def _draw_walls(self, frame, robot, arena, horizon_y):
        """Draw arena walls as simple perspective lines."""
        corners_world = [
            (0, 0), (arena.width, 0),
            (arena.width, arena.height), (0, arena.height),
        ]
        for i in range(4):
            cx, cy = corners_world[i]
            dx = cx - robot.x
            dy = cy - robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.1:
                continue
            angle = math.atan2(dy, dx) - robot.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > CAM_FOV / 2 + 0.3:
                continue
            sx = int(CAM_W / 2 + (angle / (CAM_FOV / 2)) * (CAM_W / 2))
            wall_h = int(min(200, 80 / max(0.3, dist)))
            cv2.line(frame, (sx, horizon_y - wall_h),
                     (sx, horizon_y + wall_h // 2), (100, 100, 110), 2)

    def _draw_hud(self, frame, robot):
        """Draw HUD: crosshair + compass."""
        # Crosshair
        cx, cy = CAM_W // 2, CAM_H // 2
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

        # Compass
        yaw_deg = math.degrees(robot.theta)
        cv2.putText(frame, f"YAW: {yaw_deg:.0f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 200), 1)

    def get_closest_detection(self, target_colour: str = ''):
        """Get closest detection matching target colour."""
        matches = self.detections
        if target_colour:
            matches = [d for d in matches if d['colour'] == target_colour]
        if not matches:
            return None
        return min(matches, key=lambda d: d['distance'])


# ═════════════════════════════════════════════════════════════════
# FSM — ported from fsm_node.py (no ROS2)
# ═════════════════════════════════════════════════════════════════

class State:
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    TARGETING = 'TARGETING'
    APPROACHING = 'APPROACHING'
    GRABBING = 'GRABBING'
    BURNING = 'BURNING'
    CALLING = 'CALLING'
    RETURNING = 'RETURNING'


class SimFSM:
    def __init__(self, robot: SimRobot, arena: SimArena):
        self.robot = robot
        self.arena = arena
        self.state = State.IDLE
        self.target_colour = ''
        self.target_action = ''  # 'grab' or 'burn'
        self._timeout = 0.0
        self._last_detection = None
        self._last_steer = 0.0
        self._lost_frames = 0
        self._search_start_theta = 0.0   # heading at start of search
        self._search_accumulated = 0.0    # total rotation accumulated
        self._prev_theta = 0.0            # previous theta for tracking rotation
        self.log = []  # for logging transitions

        # Manual control commands
        self._manual_mode = False
        self._manual_target_theta = None  # target angle for rotation
        self._manual_distance = 0.0       # remaining distance to travel
        self._manual_direction = None     # 'forward', 'backward', 'left', 'right'

        # Pathfinding
        self.planned_path = []            # list of (x, y) waypoints
        self._path_waypoint_idx = 0       # current waypoint index
        self._path_replan_timer = 0.0     # replan every N seconds

        # Remembered ball position (keeps last known world coords)
        self._remembered_ball = None      # (x, y) or None

        # Persistent target memory — survives SEARCHING, only cleared on IDLE/new target
        self._last_known_target = None    # (x, y) or None

    def voice_command(self, text: str):
        text = text.lower().strip()
        self._log(f'Команда: "{text}"')

        # Manual rotation commands
        if 'развернись' in text or 'разворот' in text:
            self._manual_rotate(180)
            return

        # Turn N degrees
        if 'повернись' in text or 'поверни' in text:
            degrees = self._extract_number(text)
            if degrees is not None:
                self._manual_rotate(degrees)
                return

        # Move commands: иди вперед/назад/влево/вправо на N см
        if 'иди' in text or 'двигайся' in text:
            distance_cm = self._extract_number(text)
            if distance_cm is not None:
                distance_m = distance_cm / 100.0
                if 'вперед' in text or 'вперёд' in text:
                    self._manual_move('forward', distance_m)
                    return
                elif 'назад' in text:
                    self._manual_move('backward', distance_m)
                    return
                elif 'влево' in text:
                    self._manual_move('left', distance_m)
                    return
                elif 'вправо' in text:
                    self._manual_move('right', distance_m)
                    return

        # Autonomous commands
        if 'вызови' in text and 'машин' in text:
            self._transition(State.CALLING)
            return
        if 'получи' in text or 'возьми' in text or 'найди' in text:
            self.target_action = 'grab'
            self.target_colour = self._extract_colour(text)
            c = self.target_colour or 'любой'
            self._log(f'Цель: захватить {c} мяч')
            self._transition(State.SEARCHING)
            return
        if 'сожги' in text or 'прожги' in text or 'лазер' in text:
            self.target_action = 'burn'
            self.target_colour = self._extract_colour(text)
            self._transition(State.SEARCHING)
            return
        if 'стоп' in text or 'остановись' in text:
            self._manual_mode = False
            self._transition(State.IDLE)
            return
        if 'домой' in text or 'вернись' in text:
            self._transition(State.RETURNING)
            return

    def _extract_colour(self, text: str) -> str:
        colours = {
            'красн': 'red', 'синий': 'blue', 'синего': 'blue',
            'зелен': 'green', 'желт': 'yellow', 'бел': 'white',
            'черн': 'black', 'оранж': 'orange',
        }
        for rus, eng in colours.items():
            if rus in text:
                return eng
        return ''

    def _extract_number(self, text: str) -> float:
        """Extract first number from text."""
        numbers = re.findall(r'\d+\.?\d*', text)
        if numbers:
            return float(numbers[0])
        return None

    def _manual_rotate(self, degrees: float):
        """Rotate robot by N degrees (positive = counter-clockwise)."""
        self._manual_mode = True
        target_rad = math.radians(degrees)
        self._manual_target_theta = self.robot.theta + target_rad
        # Normalize to [-pi, pi]
        self._manual_target_theta = math.atan2(
            math.sin(self._manual_target_theta),
            math.cos(self._manual_target_theta)
        )
        self._log(f'Поворот на {degrees}°')

    def _manual_move(self, direction: str, distance: float):
        """Move robot in direction for distance meters."""
        self._manual_mode = True
        self._manual_direction = direction
        self._manual_distance = distance
        self._log(f'Движение {direction} на {distance*100:.0f} см')

    def _transition(self, new_state: str):
        old = self.state
        self.state = new_state
        self._timeout = 0.0
        self._lost_frames = 0
        self._log(f'FSM: {old} -> {new_state}')

        if new_state == State.IDLE:
            self.robot.stop()
            self.robot.laser_on = False
            self.planned_path = []
            self._remembered_ball = None
            self._last_known_target = None
        elif new_state == State.SEARCHING:
            self._search_start_theta = self.robot.theta
            self._search_accumulated = 0.0
            self._prev_theta = self.robot.theta
            self.planned_path = []
            self._remembered_ball = None
        elif new_state == State.RETURNING:
            self._plan_path_to(self.arena.width / 2.0, self.arena.height / 2.0)
        elif new_state == State.CALLING:
            self._log('Вызываю вторую машину!')
            self._transition(State.IDLE)

    def _plan_path_to(self, goal_x, goal_y):
        """Plan path from robot's current position to goal, avoiding zones."""
        start = (self.robot.x, self.robot.y)
        goal = (goal_x, goal_y)
        self.planned_path = find_path(
            self.arena, self.arena.forbidden_zones, start, goal
        )
        self._path_waypoint_idx = 0
        self._path_replan_timer = 0.0
        if self.planned_path:
            self._log(f'Маршрут: {len(self.planned_path)} точек')
        else:
            self._log('Маршрут не найден!')

    def _follow_path(self, dt: float) -> bool:
        """Follow planned path waypoint by waypoint.
        Returns True if still following, False if path complete or empty."""
        if not self.planned_path or self._path_waypoint_idx >= len(self.planned_path):
            self.robot.stop()
            self.planned_path = []
            return False

        # Current target waypoint
        wx, wy = self.planned_path[self._path_waypoint_idx]
        dx = wx - self.robot.x
        dy = wy - self.robot.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Reached waypoint? Move to next
        if dist < 0.10:
            self._path_waypoint_idx += 1
            if self._path_waypoint_idx >= len(self.planned_path):
                self.robot.stop()
                self.planned_path = []
                return False
            wx, wy = self.planned_path[self._path_waypoint_idx]
            dx = wx - self.robot.x
            dy = wy - self.robot.y
            dist = math.sqrt(dx * dx + dy * dy)

        # Steer towards waypoint
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.4:
            # Need to turn first
            self.robot.set_velocity(0.0, angle_error * 1.5)
        else:
            # Move + steer
            speed = min(0.15, dist * 0.5)
            self.robot.set_velocity(speed, angle_error * 1.0)

        return True

    def tick(self, sensors: SimSensors, detector: SimDetector, dt: float):
        self._last_detection = detector.get_closest_detection(self.target_colour)

        # Manual control mode has priority
        if self._manual_mode:
            self._do_manual_control(dt)
            return

        if self.state == State.IDLE:
            pass
        elif self.state == State.SEARCHING:
            self._do_search()
        elif self.state == State.TARGETING:
            self._do_targeting(dt)
        elif self.state == State.APPROACHING:
            self._do_approach(sensors, dt)
        elif self.state == State.GRABBING:
            self._do_grab(dt)
        elif self.state == State.BURNING:
            self._do_burn(dt)
        elif self.state == State.RETURNING:
            self._do_return(dt)

    # ── MANUAL CONTROL: direct movement commands ──────────────
    def _do_manual_control(self, dt: float):
        """Execute manual rotation or movement commands."""
        # Handle rotation command
        if self._manual_target_theta is not None:
            current = self.robot.theta
            target = self._manual_target_theta

            # Calculate shortest angle difference
            error = target - current
            error = math.atan2(math.sin(error), math.cos(error))

            # If close enough, stop
            if abs(error) < math.radians(2):  # 2 degree tolerance
                self.robot.stop()
                self._manual_mode = False
                self._manual_target_theta = None
                self._log('Поворот завершён')
                return

            # Rotate towards target (smooth PID-like control)
            angular = max(-1.0, min(1.0, error * 2.0))
            self.robot.set_velocity(0.0, angular)
            return

        # Handle movement command
        if self._manual_direction and self._manual_distance > 0:
            speed = 0.15  # m/s
            distance_step = speed * dt

            if distance_step >= self._manual_distance:
                # Reached target
                self.robot.stop()
                self._manual_mode = False
                self._manual_distance = 0.0
                self._manual_direction = None
                self._log('Движение завершено')
                return

            # Calculate velocities based on direction
            if self._manual_direction == 'forward':
                self.robot.set_velocity(speed, 0.0)
            elif self._manual_direction == 'backward':
                self.robot.set_velocity(-speed, 0.0)
            elif self._manual_direction == 'left':
                # Strafe left (rotate + move)
                self.robot.set_velocity(speed * 0.5, 0.8)
            elif self._manual_direction == 'right':
                # Strafe right (rotate + move)
                self.robot.set_velocity(speed * 0.5, -0.8)

            self._manual_distance -= distance_step
            return

        # No active manual command
        self._manual_mode = False
        self.robot.stop()

    # ── SEARCHING: smart rotation toward last known target ─────
    def _do_search(self):
        # Track accumulated rotation
        delta = self._prev_theta - self.robot.theta
        # Handle wrap-around
        delta = math.atan2(math.sin(delta), math.cos(delta))
        self._search_accumulated += abs(delta)
        self._prev_theta = self.robot.theta

        # Full 360° without finding → not found
        if self._search_accumulated > 2 * math.pi + 0.3:
            c = self.target_colour or 'любой'
            self._log(f'Объект не найден ({c}) — полный оборот')
            self._transition(State.IDLE)
            return

        det = self._last_detection
        if det:
            # Ball entered camera view → stop smoothly and go to TARGETING
            self._log(f'Обнаружен {det["colour"]} мяч — наведение')
            self.robot.stop()
            # Initialize steering for smooth transition
            self._last_steer = 0.0
            self._transition(State.TARGETING)
            return

        # Choose rotation direction based on last known target position
        if self._last_known_target:
            dx = self._last_known_target[0] - self.robot.x
            dy = self._last_known_target[1] - self.robot.y
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.robot.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            # Rotate toward the remembered target (positive = CCW, negative = CW)
            direction = 0.4 if angle_diff > 0 else -0.4
        else:
            # No memory — default clockwise
            direction = -0.3

        self.robot.set_velocity(0.0, direction)

    # ── TARGETING: rotate to centre ball, then approach ────────
    def _do_targeting(self, dt: float):
        self._timeout += dt
        det = self._last_detection

        if det is None:
            # Lost ball during targeting — keep turning gently
            self._lost_frames += 1
            # Reduce turning speed significantly when lost
            self.robot.set_velocity(0.0, self._last_steer * 0.2)
            if self._lost_frames > 50:  # ~2.5 sec (more patience)
                self._log('Потерял мяч при наведении — поиск')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0

        # Update persistent target memory
        ball_dist = det['distance']
        ball_angle = self.robot.theta + ((det['x'] + det['w'] / 2.0) - CAM_W / 2.0) / (CAM_W / 2.0) * (CAM_FOV / 2)
        self._last_known_target = (
            self.robot.x + ball_dist * math.cos(ball_angle),
            self.robot.y + ball_dist * math.sin(ball_angle),
        )

        # Calculate how far ball is from vertical center line
        ball_center_x = det['x'] + det['w'] / 2.0  # Ball center in pixels
        screen_center_x = CAM_W / 2.0  # Screen center (320 pixels)

        # Distance from center in pixels
        pixel_offset = ball_center_x - screen_center_x

        # Normalize to [-1, 1]: negative = left of center, positive = right of center
        normalized_offset = pixel_offset / screen_center_x
        abs_offset = abs(normalized_offset)

        # Log for debugging
        direction = "справа" if pixel_offset > 0 else "слева"
        self._log(f'Объект {direction} (offset={pixel_offset:.0f}px, norm={normalized_offset:.2f})')

        # Check if ball is centered enough
        if abs_offset < 0.25:  # Within 25% of screen width from center
            # Ball is centered → proceed to approach
            self.robot.stop()
            self._log(f'✓ Объект в центре — приближаюсь к {det["colour"]}')
            self._transition(State.APPROACHING)
            return

        # Calculate turning direction:
        # If ball is RIGHT of center (pixel_offset > 0) → turn RIGHT (positive angular)
        # If ball is LEFT of center (pixel_offset < 0) → turn LEFT (negative angular)
        # NOTE: Positive angular velocity → counter-clockwise (left turn)
        #       Negative angular velocity → clockwise (right turn)
        # So we need to INVERT the sign: turn right when ball is right
        target_angular = normalized_offset * 0.6  # Turn TOWARDS the ball (no minus!)

        # Smooth the angular velocity using exponential moving average
        if abs(self._last_steer) < 0.01:
            # First time targeting, start fresh
            angular = target_angular
        else:
            # Smooth with previous value
            angular = self._last_steer * 0.5 + target_angular * 0.5

        self._last_steer = angular

        # Apply rotation (only angular, no linear movement)
        self.robot.set_velocity(0.0, angular)

    # ── APPROACHING: drive to ball, avoiding forbidden zones ───
    def _do_approach(self, sensors: SimSensors, dt: float):
        self._timeout += dt
        det = self._last_detection

        if self._timeout > 30.0:
            self._log('Таймаут приближения')
            self._remembered_ball = None
            self._transition(State.SEARCHING)
            return

        # Check if we're close enough to grab/burn (works with or without detection)
        if sensors.range_m < 0.10:
            self.robot.stop()
            self.planned_path = []
            self._remembered_ball = None
            colour = det['colour'] if det else self.target_colour or '?'
            if self.target_action == 'burn':
                self._log(f'Достиг цели — жгу {colour} мяч')
                self._transition(State.BURNING)
            else:
                self._log(f'Достиг цели — захватываю {colour} мяч')
                self._transition(State.GRABBING)
            return

        if det is not None:
            # Ball is visible — update remembered position
            self._lost_frames = 0
            ball_dist = det['distance']
            ball_angle = self.robot.theta + ((det['x'] + det['w'] / 2.0) - CAM_W / 2.0) / (CAM_W / 2.0) * (CAM_FOV / 2)
            ball_wx = self.robot.x + ball_dist * math.cos(ball_angle)
            ball_wy = self.robot.y + ball_dist * math.sin(ball_angle)
            self._remembered_ball = (ball_wx, ball_wy)
            self._last_known_target = (ball_wx, ball_wy)

            # Replan path periodically
            self._path_replan_timer += dt
            if not self.planned_path or self._path_replan_timer > 2.0:
                self._plan_path_to(ball_wx, ball_wy)

            # Follow path if far enough
            if self.planned_path and ball_dist > 0.3:
                self._follow_path(dt)
                return

            # Close enough — direct visual approach
            self.planned_path = []
            ball_center_x = det['x'] + det['w'] / 2.0
            pixel_offset = ball_center_x - CAM_W / 2.0
            normalized_offset = pixel_offset / (CAM_W / 2.0)
            abs_offset = abs(normalized_offset)

            # If ball drifted too far off center → back to TARGETING
            if abs_offset > 0.65:
                self.robot.stop()
                self._log('Объект ушёл из центра — повторное наведение')
                self._transition(State.TARGETING)
                return

            target_angular = normalized_offset * 0.4
            angular = self._last_steer * 0.5 + target_angular * 0.5
            self._last_steer = angular

            linear = 0.10 if abs_offset < 0.20 else 0.05
            self.robot.set_velocity(linear, angular)
        else:
            # Ball NOT visible — follow path to remembered position
            self._lost_frames += 1

            if self._remembered_ball and self.planned_path:
                # Keep following the planned path to the remembered position
                still_going = self._follow_path(dt)
                if not still_going:
                    # Arrived at remembered position but ball not visible
                    self._log('Прибыл к запомненной позиции — ищу объект')
                    self._remembered_ball = None
                    self._transition(State.SEARCHING)
                return

            if self._remembered_ball and not self.planned_path:
                # Have remembered position but no path — plan one
                self._plan_path_to(*self._remembered_ball)
                if self.planned_path:
                    self._follow_path(dt)
                    return

            # No remembered position and no path — search
            if self._lost_frames > 60:
                self._log('Мяч потерян — возврат к поиску')
                self._remembered_ball = None
                self._transition(State.SEARCHING)

    def _do_grab(self, dt: float):
        self._timeout += dt
        if self._timeout < 1.0:
            self.robot.claw_open = True
            self.robot.set_velocity(0.05, 0.0)
        elif self._timeout < 2.0:
            self.robot.stop()
        elif self._timeout < 3.0:
            self.robot.claw_open = False
            # Actually grab the nearest ball
            self._grab_nearest_ball()
        else:
            self._log('Мяч захвачен!')
            self._transition(State.RETURNING)

    def _do_burn(self, dt: float):
        self._timeout += dt
        if self._timeout < 0.5:
            self.robot.laser_on = True
        elif self._timeout < 5.0:
            pass  # hold
        else:
            self.robot.laser_on = False
            self._log('Прожиг завершён')
            self._transition(State.SEARCHING)

    def _do_return(self, dt: float):
        """Navigate toward home using pathfinding around forbidden zones."""
        home_x = self.arena.width / 2.0
        home_y = self.arena.height / 2.0
        dx = home_x - self.robot.x
        dy = home_y - self.robot.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.15:
            self._log('Вернулся домой')
            self._transition(State.IDLE)
            return

        # Replan if needed
        self._path_replan_timer += dt
        if not self.planned_path or self._path_replan_timer > 3.0:
            self._plan_path_to(home_x, home_y)

        # Follow path if we have one
        if self.planned_path:
            still_going = self._follow_path(dt)
            if not still_going:
                # Path complete — check if actually home
                if dist < 0.15:
                    self._log('Вернулся домой')
                    self._transition(State.IDLE)
            return

        # Fallback: direct navigation (no zones)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.3:
            self.robot.set_velocity(0.0, angle_error * 1.5)
        else:
            self.robot.set_velocity(0.15, angle_error * 0.8)

    def _grab_nearest_ball(self):
        for ball in self.arena.balls:
            if ball['grabbed']:
                continue
            dx = ball['x'] - self.robot.x
            dy = ball['y'] - self.robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.20:
                ball['grabbed'] = True
                self._log(f'Схватил {ball["colour"]} мяч!')
                return

    def _log(self, text: str):
        self.log.append({
            'text': text,
            'time': time.strftime('%H:%M:%S'),
        })
        # Keep last 50
        if len(self.log) > 50:
            self.log = self.log[-50:]

    def get_status(self) -> dict:
        return {
            'state': self.state,
            'target_colour': self.target_colour,
            'target_action': self.target_action,
            'range_m': 0.0,
        }


# ═════════════════════════════════════════════════════════════════
# Map Renderer — top-down view of arena
# ═════════════════════════════════════════════════════════════════

class MapRenderer:
    def __init__(self, arena: SimArena, scale: int = 100):
        self.arena = arena
        self.scale = scale  # pixels per metre
        self.w = int(arena.width * scale)
        self.h = int(arena.height * scale)

    def render(self, robot: SimRobot, scan_points=None,
               planned_path=None) -> bytes:
        img = np.full((self.h, self.w, 3), 240, dtype=np.uint8)

        # Walls
        cv2.rectangle(img, (0, 0), (self.w - 1, self.h - 1), (30, 30, 30), 3)

        # Grid
        for i in range(1, int(self.arena.width)):
            x = int(i * self.scale)
            cv2.line(img, (x, 0), (x, self.h), (210, 210, 210), 1)
        for i in range(1, int(self.arena.height)):
            y = int(i * self.scale)
            cv2.line(img, (0, y), (self.w, y), (210, 210, 210), 1)

        # Forbidden zones (semi-transparent red)
        overlay = img.copy()
        for zone in self.arena.forbidden_zones:
            px1 = int(zone['x1'] * self.scale)
            py1 = self.h - int(zone['y2'] * self.scale)  # flip Y
            px2 = int(zone['x2'] * self.scale)
            py2 = self.h - int(zone['y1'] * self.scale)
            cv2.rectangle(overlay, (px1, py1), (px2, py2), (0, 0, 200), -1)
            cv2.rectangle(img, (px1, py1), (px2, py2), (0, 0, 180), 2)
            # Zone label
            cx = (px1 + px2) // 2
            cy = (py1 + py2) // 2
            cv2.putText(img, 'X', (cx - 5, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        # Blend overlay for semi-transparency (40% opacity)
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)

        # Balls
        for ball in self.arena.balls:
            if ball['grabbed']:
                continue
            bx = int(ball['x'] * self.scale)
            by = self.h - int(ball['y'] * self.scale)  # flip Y
            colour_bgr = COLOUR_BGR.get(ball['colour'], (200, 200, 200))
            cv2.circle(img, (bx, by), max(3, int(ball['radius'] * self.scale * 2)),
                       colour_bgr, -1)
            cv2.circle(img, (bx, by), max(3, int(ball['radius'] * self.scale * 2)),
                       (0, 0, 0), 1)

        # Scan points
        if scan_points:
            for pt in scan_points:
                sx = int(pt[0] * self.scale)
                sy = self.h - int(pt[1] * self.scale)
                cv2.circle(img, (sx, sy), 2, (200, 160, 60), -1)

        # Planned path (yellow-green polyline)
        if planned_path and len(planned_path) >= 2:
            pts = []
            for wx, wy in planned_path:
                px = int(wx * self.scale)
                py = self.h - int(wy * self.scale)
                pts.append([px, py])
            pts_arr = np.array(pts, dtype=np.int32)
            cv2.polylines(img, [pts_arr], False, (0, 200, 100), 2,
                          cv2.LINE_AA)
            # Draw waypoint dots
            for p in pts:
                cv2.circle(img, (p[0], p[1]), 3, (0, 180, 80), -1)

        # Robot
        rx = int(robot.x * self.scale)
        ry = self.h - int(robot.y * self.scale)
        r_px = max(4, int(ROBOT_RADIUS * self.scale))

        # Robot body
        cv2.circle(img, (rx, ry), r_px, (79, 195, 247), -1)
        cv2.circle(img, (rx, ry), r_px, (40, 100, 130), 2)

        # Direction arrow
        arrow_len = r_px + 8
        ax = int(rx + arrow_len * math.cos(robot.theta))
        ay = int(ry - arrow_len * math.sin(robot.theta))  # flip Y
        cv2.arrowedLine(img, (rx, ry), (ax, ay), (40, 100, 130), 2,
                        tipLength=0.3)

        # Laser beam
        if robot.laser_on:
            lx = int(rx + 50 * math.cos(robot.theta))
            ly = int(ry - 50 * math.sin(robot.theta))
            cv2.line(img, (rx, ry), (lx, ly), (0, 0, 255), 2)

        # FOV cone
        fov_len = int(ULTRASONIC_MAX * self.scale * 0.4)
        for sign in (-1, 1):
            a = robot.theta + sign * CAM_FOV / 2
            fx = int(rx + fov_len * math.cos(a))
            fy = int(ry - fov_len * math.sin(a))
            cv2.line(img, (rx, ry), (fx, fy), (150, 200, 150), 1)

        _, png = cv2.imencode('.png', img)
        return png.tobytes()

    def get_map_info(self) -> dict:
        return {
            'width': self.w,
            'height': self.h,
            'resolution': 1.0 / self.scale,
            'origin_x': 0.0,
            'origin_y': 0.0,
        }


# ═════════════════════════════════════════════════════════════════
# Flask Application + Simulation Loop
# ═════════════════════════════════════════════════════════════════

def main():
    # Create simulation objects
    arena = SimArena()
    robot = SimRobot(arena)
    sensors = SimSensors()
    detector = SimDetector()
    fsm = SimFSM(robot, arena)
    map_renderer = MapRenderer(arena, scale=100)

    lock = threading.Lock()
    latest_frame = [None]       # mutable container for JPEG bytes
    latest_map_png = [None]     # mutable container for map PNG
    sim_time = [0.0]            # simulation time counter

    # ── Flask app ─────────────────────────────────────────────
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    app = Flask(__name__, template_folder=template_dir)
    CORS(app)
    socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

    @app.route('/')
    def index():
        return render_template('dashboard.html')

    @app.route('/admin')
    def admin_panel():
        return render_template('admin.html')

    @app.route('/video_feed')
    def video_feed():
        def generate():
            while True:
                with lock:
                    frame = latest_frame[0]
                if frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n'
                           + frame + b'\r\n')
                time.sleep(0.05)
        return Response(generate(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/map.png')
    def map_image():
        with lock:
            png = latest_map_png[0]
        if png is None:
            return Response(b'', mimetype='image/png')
        return Response(png, mimetype='image/png')

    @socketio.on('send_command')
    def handle_command(data):
        text = data.get('text', '')
        if text:
            with lock:
                fsm.voice_command(text)

    @socketio.on('reset_sim')
    def handle_reset(_data=None):
        with lock:
            arena.reset()
            robot.x = arena.width / 2.0
            robot.y = arena.height / 2.0
            robot.theta = 0.0
            robot.stop()
            robot.laser_on = False
            robot.claw_open = False
            fsm.state = State.IDLE
            fsm.target_colour = ''
            fsm.target_action = ''
            fsm._timeout = 0.0
            fsm._log('Симуляция сброшена')

    # ── REST API ────────────────────────────────────────────────

    # ── 1. System Status ──

    @app.route('/api/status')
    def api_status():
        with lock:
            remaining = sum(1 for b in arena.balls if not b['grabbed'])
            data = {
                'sim_time': round(sim_time[0], 2),
                'sim_dt': SIM_DT,
                'sim_hz': int(1.0 / SIM_DT),
                'robot': {
                    'x': round(robot.x, 3), 'y': round(robot.y, 3),
                    'theta': round(robot.theta, 3),
                    'v_linear': round(robot.v_linear, 4),
                    'v_angular': round(robot.v_angular, 4),
                    'claw_open': robot.claw_open,
                    'laser_on': robot.laser_on,
                },
                'fsm': {
                    'state': fsm.state,
                    'target_colour': fsm.target_colour,
                    'target_action': fsm.target_action,
                },
                'sensors': {
                    'ultrasonic_m': round(sensors.range_m, 3),
                    'imu': {
                        'yaw': round(sensors.imu_yaw, 1),
                        'pitch': round(sensors.imu_pitch, 1),
                        'roll': round(sensors.imu_roll, 1),
                        'gyro_z': round(sensors.imu_gyro_z, 3),
                        'accel_x': round(sensors.accel_x, 3),
                    },
                },
                'arena': {
                    'width': arena.width, 'height': arena.height,
                    'balls_total': len(arena.balls),
                    'balls_remaining': remaining,
                },
                'detections_count': len(detector.detections),
            }
        return json_ok(data)

    # ── 2. Robot State and Control ──

    @app.route('/api/robot/pose')
    def api_robot_pose():
        with lock:
            data = {
                'x': round(robot.x, 3),
                'y': round(robot.y, 3),
                'theta': round(robot.theta, 3),
                'theta_deg': round(math.degrees(robot.theta), 1),
            }
        return json_ok(data)

    @app.route('/api/robot/velocity', methods=['GET', 'POST'])
    def api_robot_velocity():
        if request.method == 'GET':
            with lock:
                data = {
                    'linear': round(robot.v_linear, 4),
                    'angular': round(robot.v_angular, 4),
                    'max_linear': MAX_LINEAR,
                    'max_angular': MAX_ANGULAR,
                }
            return json_ok(data)

        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        linear = body.get('linear')
        angular = body.get('angular')
        if linear is not None and not isinstance(linear, (int, float)):
            return json_err("'linear' must be a number")
        if angular is not None and not isinstance(angular, (int, float)):
            return json_err("'angular' must be a number")
        with lock:
            new_lin = float(linear) if linear is not None else robot.v_linear
            new_ang = float(angular) if angular is not None else robot.v_angular
            robot.set_velocity(new_lin, new_ang)
            result = {
                'linear': round(robot.v_linear, 4),
                'angular': round(robot.v_angular, 4),
            }
        return json_ok(result)

    @app.route('/api/robot/stop', methods=['POST'])
    def api_robot_stop():
        with lock:
            robot.stop()
        return json_ok({'linear': 0.0, 'angular': 0.0})

    @app.route('/api/robot/reset', methods=['POST'])
    def api_robot_reset():
        with lock:
            arena.reset()
            robot.x = arena.width / 2.0
            robot.y = arena.height / 2.0
            robot.theta = 0.0
            robot.stop()
            robot.laser_on = False
            robot.claw_open = False
            fsm.state = State.IDLE
            fsm.target_colour = ''
            fsm.target_action = ''
            fsm._timeout = 0.0
            sim_time[0] = 0.0
            fsm._log('Симуляция сброшена (API)')
        return json_ok({'message': 'Simulation reset'})

    # ── 3. Sensors ──

    @app.route('/api/sensors')
    def api_sensors():
        with lock:
            data = {
                'ultrasonic': {
                    'range_m': round(sensors.range_m, 3),
                    'min_range': ULTRASONIC_MIN,
                    'max_range': ULTRASONIC_MAX,
                    'field_of_view_rad': ULTRASONIC_FOV,
                },
                'imu': {
                    'yaw_deg': round(sensors.imu_yaw, 1),
                    'pitch_deg': round(sensors.imu_pitch, 1),
                    'roll_deg': round(sensors.imu_roll, 1),
                    'gyro_z_rad_s': round(sensors.imu_gyro_z, 3),
                    'accel_x_m_s2': round(sensors.accel_x, 3),
                },
            }
        return json_ok(data)

    @app.route('/api/sensors/ultrasonic')
    def api_sensors_ultrasonic():
        with lock:
            data = {
                'range_m': round(sensors.range_m, 3),
                'min_range': ULTRASONIC_MIN,
                'max_range': ULTRASONIC_MAX,
                'field_of_view_rad': ULTRASONIC_FOV,
            }
        return json_ok(data)

    @app.route('/api/sensors/imu')
    def api_sensors_imu():
        with lock:
            data = {
                'yaw_deg': round(sensors.imu_yaw, 1),
                'pitch_deg': round(sensors.imu_pitch, 1),
                'roll_deg': round(sensors.imu_roll, 1),
                'gyro_z_rad_s': round(sensors.imu_gyro_z, 3),
                'accel_x_m_s2': round(sensors.accel_x, 3),
            }
        return json_ok(data)

    # ── 4. Detection ──

    @app.route('/api/detection')
    def api_detection():
        with lock:
            dets = list(detector.detections)
        return json_ok({'count': len(dets), 'detections': dets})

    @app.route('/api/detection/closest')
    def api_detection_closest():
        colour = request.args.get('colour', '')
        with lock:
            det = detector.get_closest_detection(colour)
        if det:
            return json_ok({'found': True, 'detection': det})
        return json_ok({'found': False, 'detection': None})

    # ── 5. Actuators ──

    @app.route('/api/actuators')
    def api_actuators():
        with lock:
            data = {
                'claw_open': robot.claw_open,
                'laser_on': robot.laser_on,
            }
        return json_ok(data)

    @app.route('/api/actuators/claw', methods=['POST'])
    def api_actuators_claw():
        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        open_val = body.get('open')
        if not isinstance(open_val, bool):
            return json_err("'open' must be a boolean")
        with lock:
            robot.claw_open = open_val
        return json_ok({'claw_open': open_val})

    @app.route('/api/actuators/laser', methods=['POST'])
    def api_actuators_laser():
        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        on_val = body.get('on')
        if not isinstance(on_val, bool):
            return json_err("'on' must be a boolean")
        with lock:
            robot.laser_on = on_val
        return json_ok({'laser_on': on_val})

    # ── 6. FSM ──

    @app.route('/api/fsm')
    def api_fsm():
        with lock:
            data = {
                'state': fsm.state,
                'target_colour': fsm.target_colour,
                'target_action': fsm.target_action,
                'all_states': ALL_STATES,
            }
        return json_ok(data)

    @app.route('/api/fsm/states')
    def api_fsm_states():
        with lock:
            current = fsm.state
        return json_ok({'states': ALL_STATES, 'current': current})

    @app.route('/api/fsm/command', methods=['POST'])
    def api_fsm_command():
        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        text = body.get('text', '')
        if not text or not isinstance(text, str):
            return json_err("'text' field is required and must be a string")
        with lock:
            fsm.voice_command(text)
            data = {
                'command': text,
                'new_state': fsm.state,
                'target_colour': fsm.target_colour,
                'target_action': fsm.target_action,
            }
        return json_ok(data)

    @app.route('/api/fsm/transition', methods=['POST'])
    def api_fsm_transition():
        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        new_state = body.get('state', '')
        if new_state not in ALL_STATES:
            return json_err(
                f"Invalid state. Valid states: {', '.join(ALL_STATES)}")
        with lock:
            old = fsm.state
            fsm._transition(new_state)
            data = {'previous_state': old, 'new_state': fsm.state}
        return json_ok(data)

    # ── 7. Arena ──

    @app.route('/api/arena')
    def api_arena():
        with lock:
            remaining = sum(1 for b in arena.balls if not b['grabbed'])
            grabbed = sum(1 for b in arena.balls if b['grabbed'])
            data = {
                'width': arena.width,
                'height': arena.height,
                'robot_radius': ROBOT_RADIUS,
                'ball_radius': BALL_RADIUS,
                'balls_total': len(arena.balls),
                'balls_remaining': remaining,
                'balls_grabbed': grabbed,
            }
        return json_ok(data)

    @app.route('/api/arena/balls')
    def api_arena_balls():
        with lock:
            balls = []
            for i, b in enumerate(arena.balls):
                balls.append({
                    'id': i,
                    'colour': b['colour'],
                    'x': round(b['x'], 3),
                    'y': round(b['y'], 3),
                    'radius': b['radius'],
                    'grabbed': b['grabbed'],
                })
        return json_ok({'balls': balls})

    # ── 7b. Forbidden Zones ──

    @app.route('/api/zones')
    def api_zones():
        with lock:
            zones = list(arena.forbidden_zones)
        return json_ok({'zones': zones, 'count': len(zones)})

    @app.route('/api/zones', methods=['POST'])
    def api_zones_add():
        body = request.get_json(silent=True)
        if body is None:
            return json_err("Request body must be JSON")
        x1 = body.get('x1')
        y1 = body.get('y1')
        x2 = body.get('x2')
        y2 = body.get('y2')
        if any(v is None for v in [x1, y1, x2, y2]):
            return json_err("x1, y1, x2, y2 are required")
        if not all(isinstance(v, (int, float)) for v in [x1, y1, x2, y2]):
            return json_err("x1, y1, x2, y2 must be numbers")
        with lock:
            zone = arena.add_zone(float(x1), float(y1), float(x2), float(y2))
            fsm._log(f'Добавлена запретная зона #{zone["id"]}')
        return json_ok({'zone': zone})

    @app.route('/api/zones/<int:zone_id>', methods=['DELETE'])
    def api_zones_delete(zone_id):
        with lock:
            removed = arena.remove_zone(zone_id)
            if removed:
                fsm._log(f'Удалена запретная зона #{zone_id}')
        if removed:
            return json_ok({'removed': zone_id})
        return json_err(f"Zone {zone_id} not found", 404)

    @app.route('/api/zones/clear', methods=['POST'])
    def api_zones_clear():
        with lock:
            arena.clear_zones()
            fsm._log('Все запретные зоны удалены')
        return json_ok({'message': 'All zones cleared'})

    @app.route('/api/path')
    def api_path():
        with lock:
            path = [(round(x, 3), round(y, 3)) for x, y in fsm.planned_path]
        return json_ok({'path': path, 'count': len(path)})

    # ── 8. Map and Camera ──

    @app.route('/api/map/image')
    def api_map_image():
        with lock:
            png = latest_map_png[0]
        if png is None:
            return json_err("Map not ready", 503)
        return Response(png, mimetype='image/png',
                        headers={'Cache-Control': 'no-cache'})

    @app.route('/api/map/info')
    def api_map_info():
        with lock:
            info = map_renderer.get_map_info()
            data = {
                'width_px': info['width'],
                'height_px': info['height'],
                'resolution_m_per_px': info['resolution'],
                'origin_x': info['origin_x'],
                'origin_y': info['origin_y'],
                'arena_width_m': arena.width,
                'arena_height_m': arena.height,
            }
        return json_ok(data)

    @app.route('/api/camera/frame')
    def api_camera_frame():
        with lock:
            frame = latest_frame[0]
        if frame is None:
            return json_err("No camera frame available", 503)
        return Response(frame, mimetype='image/jpeg',
                        headers={'Cache-Control': 'no-cache'})

    @app.route('/api/camera/frame.json')
    def api_camera_frame_json():
        with lock:
            frame = latest_frame[0]
        if frame is None:
            return json_err("No camera frame available", 503)
        b64 = base64.b64encode(frame).decode('ascii')
        return json_ok({
            'format': 'jpeg',
            'width': CAM_W,
            'height': CAM_H,
            'base64': b64,
        })

    # ── 9. Event Log ──

    @app.route('/api/log')
    def api_log():
        limit = request.args.get('limit', 20, type=int)
        limit = max(1, min(50, limit))
        with lock:
            entries = fsm.log[-limit:]
        return json_ok({'count': len(entries), 'entries': entries})

    # ── API Error Handlers ──

    @app.errorhandler(404)
    def handle_404(e):
        if request.path.startswith('/api/'):
            return json_err("Endpoint not found", 404)
        return e

    @app.errorhandler(405)
    def handle_405(e):
        if request.path.startswith('/api/'):
            return json_err("Method not allowed", 405)
        return e

    # ── State emit loop ───────────────────────────────────────
    def emit_state():
        while True:
            with lock:
                status = fsm.get_status()
                status['range_m'] = round(sensors.range_m, 3)

                det = detector.get_closest_detection(fsm.target_colour)
                det_all = detector.detections[0] if detector.detections else {}

                # All detections list
                all_dets = list(detector.detections)

                # Remembered ball position
                rem_ball = None
                if fsm._remembered_ball:
                    rem_ball = {
                        'x': round(fsm._remembered_ball[0], 3),
                        'y': round(fsm._remembered_ball[1], 3),
                    }

                # Balls info
                balls_info = []
                for i, b in enumerate(arena.balls):
                    balls_info.append({
                        'id': i,
                        'colour': b['colour'],
                        'x': round(b['x'], 3),
                        'y': round(b['y'], 3),
                        'grabbed': b['grabbed'],
                    })

                state = {
                    'status': status,
                    'detection': det_all,
                    'all_detections': all_dets,
                    'range_m': round(sensors.range_m, 3),
                    'imu_ypr': [
                        round(sensors.imu_yaw, 1),
                        round(sensors.imu_pitch, 1),
                        round(sensors.imu_roll, 1),
                    ],
                    'imu_gyro_z': round(sensors.imu_gyro_z, 3),
                    'imu_accel_x': round(sensors.accel_x, 3),
                    'pose': {
                        'x': round(robot.x, 3),
                        'y': round(robot.y, 3),
                        'yaw': round(robot.theta, 3),
                        'yaw_deg': round(math.degrees(robot.theta), 1),
                    },
                    'velocity': {
                        'linear': round(robot.v_linear, 4),
                        'angular': round(robot.v_angular, 4),
                    },
                    'actuators': {
                        'claw_open': robot.claw_open,
                        'laser_on': robot.laser_on,
                    },
                    'map_info': map_renderer.get_map_info(),
                    'scan_points': [],
                    'voice_log': fsm.log[-20:],
                    'zones': list(arena.forbidden_zones),
                    'planned_path': [(round(x, 3), round(y, 3))
                                     for x, y in fsm.planned_path],
                    'remembered_ball': rem_ball,
                    'last_known_target': {
                        'x': round(fsm._last_known_target[0], 3),
                        'y': round(fsm._last_known_target[1], 3),
                    } if fsm._last_known_target else None,
                    'balls': balls_info,
                    'sim_time': round(sim_time[0], 2),
                    'arena_size': {
                        'width': arena.width,
                        'height': arena.height,
                    },
                    'lost_frames': fsm._lost_frames,
                }
            socketio.emit('state_update', state)
            socketio.sleep(0.25)

    socketio.start_background_task(emit_state)

    # ── Simulation loop (20 Hz) ───────────────────────────────
    def sim_loop():
        print('[SIM] Simulation started — 20 Hz')
        while True:
            with lock:
                sim_time[0] += SIM_DT
                robot.tick(SIM_DT)
                sensors.update(robot, arena)
                detector.update(robot, arena)
                fsm.tick(sensors, detector, SIM_DT)

                # Encode camera frame
                if detector.annotated_frame is not None:
                    _, jpeg = cv2.imencode('.jpg', detector.annotated_frame,
                                          [cv2.IMWRITE_JPEG_QUALITY, 75])
                    latest_frame[0] = jpeg.tobytes()

                # Render map
                latest_map_png[0] = map_renderer.render(
                    robot, planned_path=fsm.planned_path)

            time.sleep(SIM_DT)

    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()

    # ── Start server ──────────────────────────────────────────
    print('='*55)
    print('  SAMURAI SIMULATOR')
    print('  Dashboard: http://localhost:5000')
    print('  REST API:  http://localhost:5000/api/status')
    print('='*55)
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
