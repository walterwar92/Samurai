"""
RobotBlackboard — общее состояние, читаемое всеми behaviours дерева.

py_trees имеет встроенный Blackboard, но он немного многословный для
маленького проекта. Используем простой dataclass + RLock — публикации
из MQTT-handlers пишут сюда, behaviours читают.

В будущем можно мигрировать на py_trees.blackboard.Client (даст
возможность использовать встроенные `CheckBlackboardVariable`
behaviours для условий без своего кода).
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class _BallDetection:
    """YOLO-детекция мяча (последний кадр)."""
    colour: str = ''
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0
    conf: float = 0.0
    distance: float = -1.0
    fresh: bool = False  # True если последний _tick видел свежий кадр


@dataclass
class _Pose:
    """Поза робота из одометрии."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class RobotBlackboard:
    """Thread-safe state для BT.

    Поля:
      pose             — текущая поза робота (м, рад)
      range_m          — последнее показание ультразвука (м); float('inf') если нет
      detection        — последняя YOLO-детекция (см. _BallDetection)
      target_colour    — текущая цель ('red'/'blue'/.../'' для any)
      target_action    — 'grab' | '' (что делать с найденным)
      grabbed          — True если клешня сейчас закрыта на мяче
      manual_override  — True пока приходят cmd_vel/manual (BT тогда idle)
      home_xy          — куда возвращаться (по умолчанию (0,0))

    Action callbacks (заполняются runner.py):
      send_cmd_vel(linear, angular)
      send_claw(open: bool)
      log(msg, level='info')
    """

    def __init__(self):
        self._lock = threading.RLock()
        self.pose = _Pose()
        self.range_m: float = float('inf')
        self.detection = _BallDetection()
        self.target_colour: str = ''
        self.target_action: str = ''
        self.grabbed: bool = False
        self.manual_override: bool = False
        self.home_xy: tuple[float, float] = (0.0, 0.0)

        # Action hooks — устанавливает runner перед стартом дерева.
        # Type: (linear: float, angular: float) -> None
        self.send_cmd_vel = lambda lin, ang: None
        # Type: (open: bool) -> None
        self.send_claw = lambda is_open: None
        # Type: (msg: str, level: str) -> None
        self.log = lambda msg, level='info': None

    # ── Atomic update helpers (вызываются из MQTT callbacks) ──────────
    def update_pose(self, x: float, y: float, theta: float) -> None:
        with self._lock:
            self.pose.x = x
            self.pose.y = y
            self.pose.theta = theta

    def update_range(self, r: float) -> None:
        with self._lock:
            self.range_m = r

    def update_detection(self, det: Optional[dict]) -> None:
        with self._lock:
            if det is None:
                self.detection.fresh = False
                return
            self.detection.colour = str(det.get('colour', ''))
            self.detection.x = int(det.get('x', 0))
            self.detection.y = int(det.get('y', 0))
            self.detection.w = int(det.get('w', 0))
            self.detection.h = int(det.get('h', 0))
            self.detection.conf = float(det.get('conf', 0.0))
            self.detection.distance = float(det.get('distance', -1.0))
            self.detection.fresh = True

    def set_target(self, colour: str, action: str = 'grab') -> None:
        with self._lock:
            self.target_colour = colour
            self.target_action = action

    def clear_target(self) -> None:
        with self._lock:
            self.target_colour = ''
            self.target_action = ''
            self.grabbed = False

    def set_manual_override(self, on: bool) -> None:
        with self._lock:
            self.manual_override = on

    # ── Read snapshot (для status payload) ────────────────────────────
    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                'pose': {'x': self.pose.x, 'y': self.pose.y, 'theta': self.pose.theta},
                'range_m': self.range_m if self.range_m != float('inf') else -1.0,
                'detection': {
                    'colour': self.detection.colour,
                    'distance': self.detection.distance,
                    'fresh': self.detection.fresh,
                },
                'target_colour': self.target_colour,
                'target_action': self.target_action,
                'grabbed': self.grabbed,
                'manual_override': self.manual_override,
            }
