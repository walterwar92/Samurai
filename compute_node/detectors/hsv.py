"""
HSV-классификатор цветов — единственный источник истины для всех детекторов.

Раньше HSV-диапазоны были захардкожены в каждом из 3 детекторов с разными
значениями (рассинхрон → одни видят красный, другие нет).
Сейчас читается из config.yaml секции `hsv_colours`. Hardcoded fallback —
если config недоступен.
"""
from __future__ import annotations

import os
import sys
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore


# Hardcoded fallback (используется если config.yaml недоступен)
_DEFAULT_HSV_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    'red':    [((0, 100, 100), (10, 255, 255)),
               ((160, 100, 100), (180, 255, 255))],
    'orange': [((10, 100, 100), (25, 255, 255))],
    'yellow': [((25, 100, 100), (35, 255, 255))],
    'green':  [((35, 100, 100), (85, 255, 255))],
    'blue':   [((85, 100, 100), (130, 255, 255))],
    'white':  [((0, 0, 200), (180, 30, 255))],
    'black':  [((0, 0, 0), (180, 255, 50))],
}


def load_hsv_ranges_from_config(
    config_path: Optional[str] = None,
) -> dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]]:
    """
    Читает hsv_colours из config.yaml. Возвращает dict в формате
    {colour: [((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi)), ...]}.

    Поддерживаемые форматы:
      hsv_colours:
        red:
          - [0, 100, 100, 10, 255, 255]    # flat 6-tuple
          - [160, 100, 100, 180, 255, 255]
        green:
          - [[35, 100, 100], [85, 255, 255]]  # nested pairs

    При ошибке/отсутствии — возвращает _DEFAULT_HSV_RANGES.
    """
    cfg_data: dict = {}
    try:
        import yaml
    except ImportError:
        return _DEFAULT_HSV_RANGES

    if config_path is None:
        # config.yaml в корне проекта (на 2 уровня выше этого файла)
        config_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'config.yaml')

    if not os.path.exists(config_path):
        return _DEFAULT_HSV_RANGES

    try:
        with open(config_path, encoding='utf-8') as f:
            cfg_data = yaml.safe_load(f) or {}
    except Exception:
        return _DEFAULT_HSV_RANGES

    raw = cfg_data.get('hsv_colours')
    if not isinstance(raw, dict):
        return _DEFAULT_HSV_RANGES

    parsed: dict[str, list[tuple]] = {}
    for colour, range_list in raw.items():
        if not isinstance(range_list, list):
            continue
        ranges = []
        for r in range_list:
            if not isinstance(r, list):
                continue
            if len(r) == 6:
                ranges.append(((int(r[0]), int(r[1]), int(r[2])),
                               (int(r[3]), int(r[4]), int(r[5]))))
            elif len(r) == 2 and isinstance(r[0], list) and isinstance(r[1], list):
                ranges.append((tuple(int(x) for x in r[0]),
                               tuple(int(x) for x in r[1])))
        if ranges:
            parsed[colour] = ranges

    return parsed if parsed else _DEFAULT_HSV_RANGES


class HSVClassifier:
    """
    Классифицирует доминирующий цвет ROI по HSV-диапазонам.

    threshold (0..1): минимальная доля пикселей цвета в ROI.
    Если ни один цвет не превышает threshold — возвращает 'unknown'.
    """

    def __init__(self,
                 ranges: Optional[dict] = None,
                 threshold: float = 0.15):
        if cv2 is None:
            raise RuntimeError('cv2 (opencv-python) недоступен — HSVClassifier невозможен')
        self._ranges = ranges if ranges is not None else _DEFAULT_HSV_RANGES
        self._threshold = threshold

    @classmethod
    def from_config(cls, threshold: float = 0.15,
                    config_path: Optional[str] = None) -> 'HSVClassifier':
        return cls(ranges=load_hsv_ranges_from_config(config_path),
                   threshold=threshold)

    @property
    def colours(self) -> list[str]:
        return list(self._ranges.keys())

    def classify(self, roi_hsv: np.ndarray) -> str:
        """Возвращает имя цвета или 'unknown' если ни один не доминирует."""
        if roi_hsv is None or roi_hsv.size == 0:
            return 'unknown'
        h, w = roi_hsv.shape[:2]
        total = h * w
        if total == 0:
            return 'unknown'

        best_colour = 'unknown'
        best_ratio = 0.0
        for colour, ranges in self._ranges.items():
            mask = np.zeros((h, w), dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(roi_hsv, np.array(lo), np.array(hi))
            ratio = float(np.count_nonzero(mask)) / total
            if ratio > best_ratio and ratio > self._threshold:
                best_ratio = ratio
                best_colour = colour
        return best_colour

    def update_range(self,
                     colour: str,
                     lo: tuple[int, int, int],
                     hi: tuple[int, int, int]):
        """Заменить диапазон цвета (для калибратора)."""
        self._ranges[colour] = [(lo, hi)]

    def get_ranges(self) -> dict:
        """Текущие диапазоны (для экспорта в config.yaml)."""
        return {c: list(r) for c, r in self._ranges.items()}
