"""
MapRenderer — top-down PNG renderer for the Flask simulator (#44 phase 5).

Extracted from compute_node/simulator.py. Pure rendering: takes the
arena/robot state and produces a PNG byte string suitable for serving
over HTTP. No Flask coupling.

Requires `numpy` and `opencv-python` at runtime — both are already in
requirements.txt for the YOLO / camera path.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

import cv2
import numpy as np

# Defaults match simulator.py's constants. Caller can override per
# instance for alternate rigs (different ROBOT_RADIUS, FOV, etc.).
DEFAULT_ROBOT_RADIUS = 0.12             # m
DEFAULT_ULTRASONIC_MAX = 2.0            # m — used to scale FOV cone draw
DEFAULT_CAM_FOV = math.radians(60)      # rad
DEFAULT_COLOUR_BGR = {
    'red':    (0, 0, 220),
    'blue':   (220, 100, 30),
    'green':  (50, 200, 50),
    'yellow': (0, 230, 230),
    'orange': (0, 140, 255),
}


class MapRenderer:
    """Renders the simulator's top-down map as a PNG.

    Y-axis is flipped (image coordinates have origin in top-left, world
    coordinates in bottom-left). All `arena.balls` and `arena.forbidden_zones`
    are drawn; optional overlays for laser scan points and a planned path
    are drawn when supplied.
    """

    def __init__(self, arena, scale: int = 100,
                 robot_radius: float = DEFAULT_ROBOT_RADIUS,
                 ultrasonic_max: float = DEFAULT_ULTRASONIC_MAX,
                 cam_fov: float = DEFAULT_CAM_FOV,
                 colour_bgr: Optional[dict] = None):
        self.arena = arena
        self.scale = scale     # pixels per metre
        self.w = int(arena.width * scale)
        self.h = int(arena.height * scale)
        self.robot_radius = robot_radius
        self.ultrasonic_max = ultrasonic_max
        self.cam_fov = cam_fov
        self.colour_bgr = colour_bgr if colour_bgr is not None else DEFAULT_COLOUR_BGR

    # ── Public API ─────────────────────────────────────────────────────
    def render(self, robot,
               scan_points: Optional[Iterable[tuple[float, float]]] = None,
               planned_path: Optional[Iterable[tuple[float, float]]] = None
               ) -> bytes:
        """Return a PNG byte string of the current world view."""
        img = np.full((self.h, self.w, 3), 240, dtype=np.uint8)
        self._draw_walls_and_grid(img)
        self._draw_zones(img)
        self._draw_balls(img)
        self._draw_scan(img, scan_points)
        self._draw_planned_path(img, planned_path)
        self._draw_robot(img, robot)
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

    # ── Drawing primitives ─────────────────────────────────────────────
    def _draw_walls_and_grid(self, img) -> None:
        cv2.rectangle(img, (0, 0), (self.w - 1, self.h - 1), (30, 30, 30), 3)
        for i in range(1, int(self.arena.width)):
            x = int(i * self.scale)
            cv2.line(img, (x, 0), (x, self.h), (210, 210, 210), 1)
        for i in range(1, int(self.arena.height)):
            y = int(i * self.scale)
            cv2.line(img, (0, y), (self.w, y), (210, 210, 210), 1)

    def _draw_zones(self, img) -> None:
        if not self.arena.forbidden_zones:
            return
        overlay = img.copy()
        for zone in self.arena.forbidden_zones:
            px1 = int(zone['x1'] * self.scale)
            py1 = self.h - int(zone['y2'] * self.scale)   # flip Y
            px2 = int(zone['x2'] * self.scale)
            py2 = self.h - int(zone['y1'] * self.scale)
            cv2.rectangle(overlay, (px1, py1), (px2, py2), (0, 0, 200), -1)
            cv2.rectangle(img, (px1, py1), (px2, py2), (0, 0, 180), 2)
            cx = (px1 + px2) // 2
            cy = (py1 + py2) // 2
            cv2.putText(img, 'X', (cx - 5, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)

    def _draw_balls(self, img) -> None:
        for ball in self.arena.balls:
            if ball.get('grabbed', False):
                continue
            bx = int(ball['x'] * self.scale)
            by = self.h - int(ball['y'] * self.scale)
            colour = self.colour_bgr.get(ball['colour'], (200, 200, 200))
            r = max(3, int(ball.get('radius', 0.02) * self.scale * 2))
            cv2.circle(img, (bx, by), r, colour, -1)
            cv2.circle(img, (bx, by), r, (0, 0, 0), 1)

    def _draw_scan(self, img, scan_points) -> None:
        if not scan_points:
            return
        for pt in scan_points:
            sx = int(pt[0] * self.scale)
            sy = self.h - int(pt[1] * self.scale)
            cv2.circle(img, (sx, sy), 2, (200, 160, 60), -1)

    def _draw_planned_path(self, img, planned_path) -> None:
        if not planned_path:
            return
        pts = []
        for wx, wy in planned_path:
            px = int(wx * self.scale)
            py = self.h - int(wy * self.scale)
            pts.append([px, py])
        if len(pts) < 2:
            return
        pts_arr = np.array(pts, dtype=np.int32)
        cv2.polylines(img, [pts_arr], False, (0, 200, 100), 2, cv2.LINE_AA)
        for p in pts:
            cv2.circle(img, (p[0], p[1]), 3, (0, 180, 80), -1)

    def _draw_robot(self, img, robot) -> None:
        rx = int(robot.x * self.scale)
        ry = self.h - int(robot.y * self.scale)
        r_px = max(4, int(self.robot_radius * self.scale))
        cv2.circle(img, (rx, ry), r_px, (79, 195, 247), -1)
        cv2.circle(img, (rx, ry), r_px, (40, 100, 130), 2)
        # Direction arrow
        arrow_len = r_px + 8
        ax = int(rx + arrow_len * math.cos(robot.theta))
        ay = int(ry - arrow_len * math.sin(robot.theta))
        cv2.arrowedLine(img, (rx, ry), (ax, ay), (40, 100, 130), 2,
                        tipLength=0.3)
        # FOV cone
        fov_len = int(self.ultrasonic_max * self.scale * 0.4)
        for sign in (-1, 1):
            a = robot.theta + sign * self.cam_fov / 2
            fx = int(rx + fov_len * math.cos(a))
            fy = int(ry - fov_len * math.sin(a))
            cv2.line(img, (rx, ry), (fx, fy), (150, 200, 150), 1)
