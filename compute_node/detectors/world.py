"""
WorldProjector — проекция пиксельных координат в мировые (карта).

Использует:
  - позу робота (x, y, theta)
  - пиксельный X центра объекта (для horizontal angle offset)
  - оценённую дистанцию

Не учитывает наклон камеры по pitch (для floor-plan navigation этого хватает).
"""
from __future__ import annotations

import math


class WorldProjector:
    """
    fov_rad: горизонтальный FOV камеры (радианы). Default 60° = 1.047.
    """

    def __init__(self,
                 fov_rad: float = math.radians(60.0)):
        self._fov = fov_rad

    def project(self,
                rx: float, ry: float, theta: float,
                cx_px: float, dist_m: float,
                img_width: int) -> tuple[float, float]:
        """
        Args:
            rx, ry: поза робота в мировых координатах (м)
            theta: поворот робота (рад, +X = вперёд)
            cx_px: пиксельный центр объекта по X
            dist_m: оценённая дистанция до объекта (м)
            img_width: ширина изображения в пикселях

        Returns:
            (world_x, world_y) в мировых координатах (м)
        """
        # Угол от центра кадра до объекта (норм. [-1; 1] × FOV/2)
        cx_norm = (cx_px - img_width / 2.0) / max(1.0, img_width / 2.0)
        angle_offset = cx_norm * (self._fov / 2.0)
        obj_angle = theta + angle_offset
        wx = rx + dist_m * math.cos(obj_angle)
        wy = ry + dist_m * math.sin(obj_angle)
        return wx, wy
