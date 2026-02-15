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


# ═════════════════════════════════════════════════════════════════
# SimArena — 2D world with walls and balls
# ═════════════════════════════════════════════════════════════════

class SimArena:
    def __init__(self):
        self.width = ARENA_W
        self.height = ARENA_H
        self.balls = []
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

    def reset(self):
        for b in self.balls:
            b['grabbed'] = False
        self._spawn_balls()


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

    def voice_command(self, text: str):
        text = text.lower().strip()
        self._log(f'Команда: "{text}"')

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

    def _transition(self, new_state: str):
        old = self.state
        self.state = new_state
        self._timeout = 0.0
        self._lost_frames = 0
        self._log(f'FSM: {old} -> {new_state}')

        if new_state == State.IDLE:
            self.robot.stop()
            self.robot.laser_on = False
        elif new_state == State.SEARCHING:
            self._search_start_theta = self.robot.theta
            self._search_accumulated = 0.0
            self._prev_theta = self.robot.theta
        elif new_state == State.CALLING:
            self._log('Вызываю вторую машину!')
            self._transition(State.IDLE)

    def tick(self, sensors: SimSensors, detector: SimDetector, dt: float):
        self._last_detection = detector.get_closest_detection(self.target_colour)

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

    # ── SEARCHING: clockwise rotation, 360° check ─────────────
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
            # Ball entered camera view → stop and go to TARGETING
            self._log(f'Обнаружен {det["colour"]} мяч — наведение')
            self.robot.stop()
            self._transition(State.TARGETING)
            return

        # Rotate clockwise (negative angular.z in ROS convention)
        self.robot.set_velocity(0.0, -0.4)

    # ── TARGETING: rotate to centre ball, then approach ────────
    def _do_targeting(self, dt: float):
        self._timeout += dt
        det = self._last_detection

        if det is None:
            # Lost ball during targeting — return to search
            self._lost_frames += 1
            # Keep turning gently in last direction
            self.robot.set_velocity(0.0, self._last_steer * 0.3)
            if self._lost_frames > 30:  # ~1.5 sec
                self._log('Потерял мяч при наведении — поиск')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0

        # How far is ball from centre
        ball_cx = det['x'] + det['w'] / 2.0
        error_x = (ball_cx - CAM_W / 2) / (CAM_W / 2)  # [-1, 1]
        abs_error = abs(error_x)

        if abs_error < 0.15:
            # Ball centred → go approach
            self.robot.stop()
            self._log(f'Цель по центру — приближение к {det["colour"]}')
            self._transition(State.APPROACHING)
            return

        # Turn in place to centre the ball (smooth)
        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.4 + target_angular * 0.6
        self._last_steer = angular
        self.robot.set_velocity(0.0, angular)

    # ── APPROACHING: drive straight to centred ball ────────────
    def _do_approach(self, sensors: SimSensors, dt: float):
        self._timeout += dt
        det = self._last_detection

        if self._timeout > 30.0:
            self._log('Таймаут приближения')
            self._transition(State.SEARCHING)
            return

        if det is None:
            self._lost_frames += 1
            # Slow creep + gentle last-direction turn
            self.robot.set_velocity(0.02, self._last_steer * 0.3)
            if self._lost_frames > 40:
                self._log('Мяч потерян — возврат к поиску')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0

        ball_cx = det['x'] + det['w'] / 2.0
        error_x = (ball_cx - CAM_W / 2) / (CAM_W / 2)
        abs_error = abs(error_x)

        # If ball drifted too far off centre → back to TARGETING
        if abs_error > 0.5:
            self.robot.stop()
            self._transition(State.TARGETING)
            return

        # Small correction + forward drive
        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.3 + target_angular * 0.7
        self._last_steer = angular

        # Drive forward (slower if slightly off-centre)
        linear = 0.12 if abs_error < 0.15 else 0.06

        # Close enough?
        if sensors.range_m < 0.10:
            self.robot.stop()
            if self.target_action == 'burn':
                self._transition(State.BURNING)
            else:
                self._transition(State.GRABBING)
            return

        self.robot.set_velocity(linear, angular)

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
        """Navigate toward home (0, 0 is centre of arena)."""
        home_x = self.arena.width / 2.0
        home_y = self.arena.height / 2.0
        dx = home_x - self.robot.x
        dy = home_y - self.robot.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.15:
            self._log('Вернулся домой')
            self._transition(State.IDLE)
            return

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

    def render(self, robot: SimRobot, scan_points=None) -> bytes:
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

                state = {
                    'status': status,
                    'detection': det_all,
                    'range_m': round(sensors.range_m, 3),
                    'imu_ypr': [
                        round(sensors.imu_yaw, 1),
                        round(sensors.imu_pitch, 1),
                        round(sensors.imu_roll, 1),
                    ],
                    'pose': {
                        'x': round(robot.x, 3),
                        'y': round(robot.y, 3),
                        'yaw': round(robot.theta, 3),
                    },
                    'map_info': map_renderer.get_map_info(),
                    'scan_points': [],
                    'voice_log': fsm.log[-20:],
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
                latest_map_png[0] = map_renderer.render(robot)

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
