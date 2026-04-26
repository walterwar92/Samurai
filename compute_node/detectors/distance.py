"""
DistanceEstimator — оценка дистанции до объекта по bbox + ультразвук.

Методы:
  'mono'  — только монокулярная: dist = (real_height_m × focal_px) / bbox_height_px
  'ultra' — только ультразвук (объект очень близко к центру кадра)
  'blend' — взвешенное среднее mono + ultra (объект около центра)
  'none'  — bbox слишком мал
"""
from __future__ import annotations

# Реальные размеры объектов (высота, м) для монокулярной глубины
DEFAULT_OBJECT_SIZES: dict[str, float] = {
    'sports ball':  0.04,
    'ball':         0.04,
    'tennis ball':  0.067,
    'orange':       0.08,
    'apple':        0.08,
    'bottle':       0.25,
    'cup':          0.12,
    'person':       1.70,
    'chair':        0.90,
    'cat':          0.30,
    'dog':          0.40,
    'car':          1.50,
    'bicycle':      1.10,
    'object':       0.04,   # YOLO generic class → считаем мячом
}


class DistanceEstimator:
    """
    Параметры:
      focal_px              — фокусное расстояние камеры в пикселях (~500 для 640px CSI)
      default_object_size_m — для неизвестных классов
      ultrasonic_max_age_s  — максимальный возраст показания УЗ для использования
      center_threshold      — |cx_norm| < этого → объект около центра (УЗ применим)
      tight_center          — |cx_norm| < этого → объект ровно по центру (УЗ доминирует)
      blend_uz_weight       — вес УЗ при blend
    """

    def __init__(self,
                 focal_px: float = 500.0,
                 object_sizes: dict[str, float] | None = None,
                 default_object_size_m: float = 0.10,
                 ultrasonic_max_age_s: float = 0.5,
                 center_threshold: float = 0.25,
                 tight_center: float = 0.10,
                 blend_uz_weight: float = 0.65):
        self._focal = focal_px
        self._sizes = dict(DEFAULT_OBJECT_SIZES)
        if object_sizes:
            self._sizes.update(object_sizes)
        self._default_size = default_object_size_m
        self._uz_max_age = ultrasonic_max_age_s
        self._center_thr = center_threshold
        self._tight_center = tight_center
        self._uz_weight = blend_uz_weight

    def estimate(self,
                 cls_name: str,
                 bbox_h_px: int,
                 cx_norm: float,
                 ultrasonic_m: float = 2.0,
                 ultrasonic_age_s: float = 999.0) -> tuple[float, str]:
        """
        Args:
            cls_name: имя класса YOLO (для лукапа реального размера)
            bbox_h_px: высота bbox в пикселях
            cx_norm: нормализованная X-координата центра bbox в [-1; 1]
            ultrasonic_m: последнее показание УЗ в метрах
            ultrasonic_age_s: возраст показания

        Returns:
            (distance_m, method) — distance=-1.0 если bbox слишком мал
        """
        if bbox_h_px <= 5:
            return -1.0, 'none'

        # Монокулярная: distance = (object_height_m * focal_px) / bbox_height_px
        obj_h = self._sizes.get(cls_name, self._default_size)
        mono = (obj_h * self._focal) / bbox_h_px

        uz_valid = (ultrasonic_age_s < self._uz_max_age and ultrasonic_m < 1.9)

        # Объект ровно по центру + УЗ свежий → доверяем УЗ больше
        if uz_valid and abs(cx_norm) < self._tight_center:
            return round(0.80 * ultrasonic_m + 0.20 * mono, 3), 'ultra'

        # Объект около центра → blend
        if uz_valid and abs(cx_norm) < self._center_thr:
            blend = self._uz_weight * ultrasonic_m + (1 - self._uz_weight) * mono
            return round(blend, 3), 'blend'

        # По умолчанию — только монокулярная
        return round(mono, 3), 'mono'
