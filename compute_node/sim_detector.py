"""
SimDetector — geometric ball detection + first-person camera renderer
for the Flask simulator (#44 phase 6).

Extracted from compute_node/simulator.py. Builds a synthetic camera
frame from the world geometry (no real CV) and emits YOLO-style
detections so the dashboard can be exercised end-to-end without a
real camera.

Requires `numpy` and `opencv-python`.
"""
from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np

# Defaults — match simulator.py's hardcoded values.
DEFAULT_CAM_W = 640
DEFAULT_CAM_H = 480
DEFAULT_FOCAL_LENGTH_PX = 500.0
DEFAULT_BALL_DIAMETER_M = 0.04
DEFAULT_CAM_FOV = math.radians(60)
DEFAULT_MAX_DETECT_RANGE = 3.0    # m — beyond this, balls are too small to detect

DEFAULT_COLOUR_BGR = {
    'red':    (0, 0, 220),
    'blue':   (220, 100, 30),
    'green':  (50, 200, 50),
    'yellow': (0, 230, 230),
    'orange': (0, 140, 255),
}


class SimDetector:
    """Geometric "YOLO" — projects visible balls into a synthetic camera
    image, computes pinhole-projected bounding boxes, fakes a confidence
    score from distance.

    `arena` and `robot` are duck-typed: arena needs `.width`, `.height`,
    `.balls`; robot needs `.x`, `.y`, `.theta`.
    """

    def __init__(self,
                 cam_w: int = DEFAULT_CAM_W,
                 cam_h: int = DEFAULT_CAM_H,
                 focal_length_px: float = DEFAULT_FOCAL_LENGTH_PX,
                 ball_diameter_m: float = DEFAULT_BALL_DIAMETER_M,
                 cam_fov: float = DEFAULT_CAM_FOV,
                 max_range: float = DEFAULT_MAX_DETECT_RANGE,
                 colour_bgr: Optional[dict] = None):
        self.detections: list[dict] = []
        self.annotated_frame: Optional[np.ndarray] = None

        self.cam_w = cam_w
        self.cam_h = cam_h
        self.focal_length_px = focal_length_px
        self.ball_diameter_m = ball_diameter_m
        self.cam_fov = cam_fov
        self.max_range = max_range
        self.colour_bgr = colour_bgr if colour_bgr is not None else DEFAULT_COLOUR_BGR

    # ── Public API ─────────────────────────────────────────────────────
    def update(self, robot, arena) -> None:
        """Detect visible balls and render the camera view."""
        self.detections = []
        self.annotated_frame = self._render_camera(robot, arena)

    def get_closest_detection(self, target_colour: str = '') -> Optional[dict]:
        matches = self.detections
        if target_colour:
            matches = [d for d in matches if d['colour'] == target_colour]
        if not matches:
            return None
        return min(matches, key=lambda d: d['distance'])

    # ── Rendering ──────────────────────────────────────────────────────
    def _render_camera(self, robot, arena) -> np.ndarray:
        # Floor / horizon background
        frame = np.full((self.cam_h, self.cam_w, 3), (60, 60, 55), dtype=np.uint8)
        horizon_y = int(self.cam_h * 0.4)
        # Sky band
        frame[:horizon_y, :] = (90, 85, 80)
        # Floor gradient (lighter as we approach the bottom)
        for row in range(horizon_y, self.cam_h):
            t = (row - horizon_y) / (self.cam_h - horizon_y)
            grey = int(55 + t * 20)
            frame[row, :] = (grey, grey, grey - 5)

        self._draw_walls(frame, robot, arena, horizon_y)

        # Visible balls — sorted far-to-near so close ones overdraw.
        visible = []
        for ball in arena.balls:
            if ball.get('grabbed', False):
                continue
            dx = ball['x'] - robot.x
            dy = ball['y'] - robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.01 or dist > self.max_range:
                continue
            angle = math.atan2(dy, dx) - robot.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))   # normalise
            if abs(angle) > self.cam_fov / 2:
                continue
            visible.append((dist, angle, ball))
        visible.sort(key=lambda v: -v[0])

        for dist, angle, ball in visible:
            screen_x = int(self.cam_w / 2 + (angle / (self.cam_fov / 2)) * (self.cam_w / 2))

            # Pinhole apparent size: pixels = f × diameter / distance
            apparent_px = int(self.focal_length_px * self.ball_diameter_m / dist)
            apparent_px = max(4, min(200, apparent_px))

            # Vertical: balls sit on the floor; closer = lower on screen
            screen_y = horizon_y + int(
                (1.0 - 0.03 / max(0.1, dist)) * (self.cam_h - horizon_y) * 0.7
            )

            colour = self.colour_bgr.get(ball['colour'], (200, 200, 200))

            # Ball + highlight + shadow
            cv2.circle(frame, (screen_x, screen_y), apparent_px, colour, -1)
            hl_x = screen_x - apparent_px // 4
            hl_y = screen_y - apparent_px // 4
            hl_r = max(1, apparent_px // 4)
            hl_colour = tuple(min(255, c + 60) for c in colour)
            cv2.circle(frame, (hl_x, hl_y), hl_r, hl_colour, -1)
            shadow_y = screen_y + apparent_px
            cv2.ellipse(frame, (screen_x, shadow_y),
                        (apparent_px, apparent_px // 4), 0, 0, 360,
                        (30, 30, 25), -1)

            # Detection bounding box
            x1 = screen_x - apparent_px
            y1 = screen_y - apparent_px
            w = apparent_px * 2
            h = apparent_px * 2
            conf = max(0.5, min(0.99, 1.0 - dist / self.max_range))
            self.detections.append({
                'colour': ball['colour'],
                'class': 'sports ball',
                'x': max(0, x1), 'y': max(0, y1),
                'w': w, 'h': h,
                'conf': round(conf, 3),
                'distance': round(dist, 3),
            })
            label = f"{ball['colour']} {conf:.2f} {dist:.2f}m"
            cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        self._draw_hud(frame, robot)
        return frame

    def _draw_walls(self, frame, robot, arena, horizon_y) -> None:
        """Stylised arena corners as perspective tick marks."""
        corners = [
            (0, 0), (arena.width, 0),
            (arena.width, arena.height), (0, arena.height),
        ]
        for cx, cy in corners:
            dx = cx - robot.x
            dy = cy - robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.1:
                continue
            angle = math.atan2(dy, dx) - robot.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > self.cam_fov / 2 + 0.3:
                continue
            sx = int(self.cam_w / 2 + (angle / (self.cam_fov / 2)) * (self.cam_w / 2))
            wall_h = int(min(200, 80 / max(0.3, dist)))
            cv2.line(frame, (sx, horizon_y - wall_h),
                     (sx, horizon_y + wall_h // 2),
                     (100, 100, 110), 2)

    def _draw_hud(self, frame, robot) -> None:
        cx, cy = self.cam_w // 2, self.cam_h // 2
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)
        cv2.putText(frame, f"YAW: {math.degrees(robot.theta):.0f}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 200, 200), 1)
