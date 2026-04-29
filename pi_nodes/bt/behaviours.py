"""
Behaviour-листья для главного BT робота (#1, 2026-04).

Каждый класс наследуется от py_trees.behaviour.Behaviour и реализует
update() — возвращает Status.SUCCESS / FAILURE / RUNNING.

Используют RobotBlackboard как разделяемое состояние (снапшот сенсоров +
action callbacks для cmd_vel / claw / log).

Конвенции:
  - HasTarget       — Condition: True если целевой цвет задан
  - BallVisible     — Condition: True если YOLO видит мяч (опц. фильтр цвета)
  - BallClose       — Condition: True если ультразвук < threshold
  - SearchSpin      — Action: крутится на месте, ищет мяч (RUNNING пока время)
  - CenterBall      — Action: поворачивается чтобы мяч был в центре кадра
  - ApproachBall    — Action: едет к мячу с коррекцией yaw
  - GrabBall        — Action: открывает клешню → подъезжает → закрывает
  - StopMotion      — Action: cmd_vel(0, 0); сразу SUCCESS
  - ReturnHome      — Action: edет к (home_x, home_y) (через odom)
  - HoldIdle        — Action: ничего не делает, RUNNING (для root selector)
"""
from __future__ import annotations

import math
import time

import py_trees

from .blackboard import RobotBlackboard

Status = py_trees.common.Status


# ── Conditions (быстрые проверки, всегда SUCCESS/FAILURE) ──────────────
class ManualOverrideActive(py_trees.behaviour.Behaviour):
    """SUCCESS если приходит cmd_vel/manual — BT не должен мешать."""

    def __init__(self, bb: RobotBlackboard, name: str = 'ManualOverride?'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        return Status.SUCCESS if self._bb.manual_override else Status.FAILURE


class HasTarget(py_trees.behaviour.Behaviour):
    """SUCCESS если задана цель (target_colour непустой ИЛИ action='grab')."""

    def __init__(self, bb: RobotBlackboard, name: str = 'HasTarget?'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        return (
            Status.SUCCESS
            if (self._bb.target_colour or self._bb.target_action == 'grab')
            else Status.FAILURE
        )


class BallVisible(py_trees.behaviour.Behaviour):
    """SUCCESS если YOLO видит мяч (опционально нужного цвета)."""

    def __init__(self, bb: RobotBlackboard, name: str = 'BallVisible?'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        d = self._bb.detection
        if not d.fresh:
            return Status.FAILURE
        target = self._bb.target_colour
        if target and d.colour != target:
            return Status.FAILURE
        return Status.SUCCESS


class BallClose(py_trees.behaviour.Behaviour):
    """SUCCESS если ультразвук < threshold (мяч в досягаемости клешни)."""

    def __init__(
        self, bb: RobotBlackboard, threshold_m: float = 0.10,
        name: str = 'BallClose?',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._threshold = threshold_m

    def update(self) -> Status:
        return (
            Status.SUCCESS if self._bb.range_m < self._threshold
            else Status.FAILURE
        )


class HasGrabbed(py_trees.behaviour.Behaviour):
    """SUCCESS если grabbed=True (мяч в клешне)."""

    def __init__(self, bb: RobotBlackboard, name: str = 'Grabbed?'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        return Status.SUCCESS if self._bb.grabbed else Status.FAILURE


# ── Actions ────────────────────────────────────────────────────────────
class SearchSpin(py_trees.behaviour.Behaviour):
    """Крутится на месте до полного оборота. RUNNING всё это время.

    SUCCESS если за время поиска появилась видимость мяча.
    FAILURE если прошёл полный оборот без находки (timeout).
    """

    def __init__(
        self, bb: RobotBlackboard,
        angular_speed: float = 0.4,
        max_angle_rad: float = 2 * math.pi + 0.3,
        tick_dt: float = 0.1,
        name: str = 'SearchSpin',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._angular = angular_speed
        self._max_angle = max_angle_rad
        self._tick_dt = tick_dt
        self._accumulated = 0.0

    def initialise(self):
        self._accumulated = 0.0

    def update(self) -> Status:
        d = self._bb.detection
        if d.fresh and (not self._bb.target_colour or d.colour == self._bb.target_colour):
            self._bb.send_cmd_vel(0.0, 0.0)
            return Status.SUCCESS
        if self._accumulated > self._max_angle:
            self._bb.send_cmd_vel(0.0, 0.0)
            self._bb.log(
                f'Ball not found ({self._bb.target_colour or "any"}) — full rotation',
                'warn',
            )
            return Status.FAILURE
        self._accumulated += abs(self._angular) * self._tick_dt
        self._bb.send_cmd_vel(0.0, -self._angular)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:  # type: ignore[override]
        if new_status != Status.RUNNING:
            self._bb.send_cmd_vel(0.0, 0.0)


class CenterBall(py_trees.behaviour.Behaviour):
    """Крутит шасси так чтобы мяч был в центре кадра (image_cx ~ 320).

    SUCCESS когда |error_x| < threshold. FAILURE если мяч пропал.
    """

    IMG_CX = 320

    def __init__(
        self, bb: RobotBlackboard,
        gain: float = 0.8,
        smoothing: float = 0.4,
        threshold: float = 0.15,
        max_lost_frames: int = 15,
        name: str = 'CenterBall',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._gain = gain
        self._smoothing = smoothing
        self._threshold = threshold
        self._max_lost = max_lost_frames
        self._last_steer = 0.0
        self._lost_frames = 0

    def initialise(self):
        self._last_steer = 0.0
        self._lost_frames = 0

    def update(self) -> Status:
        d = self._bb.detection
        if not d.fresh or (self._bb.target_colour and d.colour != self._bb.target_colour):
            self._lost_frames += 1
            if self._lost_frames > self._max_lost:
                self._bb.send_cmd_vel(0.0, 0.0)
                return Status.FAILURE
            self._bb.send_cmd_vel(0.0, self._last_steer * 0.3)
            return Status.RUNNING

        self._lost_frames = 0
        ball_cx = d.x + d.w / 2.0
        error_x = (ball_cx - self.IMG_CX) / self.IMG_CX
        if abs(error_x) < self._threshold:
            self._bb.send_cmd_vel(0.0, 0.0)
            return Status.SUCCESS

        target_angular = -error_x * self._gain
        angular = (
            self._last_steer * self._smoothing
            + target_angular * (1.0 - self._smoothing)
        )
        self._last_steer = angular
        self._bb.send_cmd_vel(0.0, angular)
        return Status.RUNNING


class ApproachBall(py_trees.behaviour.Behaviour):
    """Едет к мячу с коррекцией yaw (комбо center+forward).

    SUCCESS когда range_m < grab_threshold.
    FAILURE если мяч пропадает на >max_lost кадров либо timeout.
    """

    IMG_CX = 320

    def __init__(
        self, bb: RobotBlackboard,
        grab_threshold_m: float = 0.10,
        timeout_s: float = 30.0,
        max_lost_frames: int = 20,
        tick_dt: float = 0.1,
        name: str = 'ApproachBall',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._grab_threshold = grab_threshold_m
        self._timeout = timeout_s
        self._max_lost = max_lost_frames
        self._tick_dt = tick_dt
        self._elapsed = 0.0
        self._last_steer = 0.0
        self._lost_frames = 0

    def initialise(self):
        self._elapsed = 0.0
        self._last_steer = 0.0
        self._lost_frames = 0

    def update(self) -> Status:
        self._elapsed += self._tick_dt
        if self._elapsed > self._timeout:
            self._bb.send_cmd_vel(0.0, 0.0)
            self._bb.log('Approach timeout', 'warn')
            return Status.FAILURE

        if self._bb.range_m < self._grab_threshold:
            self._bb.send_cmd_vel(0.0, 0.0)
            return Status.SUCCESS

        d = self._bb.detection
        if not d.fresh or (self._bb.target_colour and d.colour != self._bb.target_colour):
            self._lost_frames += 1
            if self._lost_frames > self._max_lost:
                self._bb.send_cmd_vel(0.0, 0.0)
                return Status.FAILURE
            # медленно ползём вперёд по последнему вектору
            self._bb.send_cmd_vel(0.02, self._last_steer * 0.3)
            return Status.RUNNING

        self._lost_frames = 0
        ball_cx = d.x + d.w / 2.0
        error_x = (ball_cx - self.IMG_CX) / self.IMG_CX
        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.3 + target_angular * 0.7
        self._last_steer = angular
        linear = 0.12 if abs(error_x) < 0.15 else 0.06
        self._bb.send_cmd_vel(linear, angular)
        return Status.RUNNING


class GrabBall(py_trees.behaviour.Behaviour):
    """Тайминг открытия/закрытия клешни.

    Скрипт:
      0..1с    cmd_vel(0.05, 0)  + claw(open)   — последний доезд
      1..2с    cmd_vel(0, 0)                    — стоп
      2..3с    claw(close)                       — захват
      >3с      grabbed=True, SUCCESS

    Никогда не FAILURE — скрипт детерминированный по времени.
    """

    def __init__(
        self, bb: RobotBlackboard,
        tick_dt: float = 0.1,
        name: str = 'GrabBall',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._tick_dt = tick_dt
        self._elapsed = 0.0
        self._closed = False

    def initialise(self):
        self._elapsed = 0.0
        self._closed = False
        self._bb.send_claw(True)  # open
        self._bb.log('Grabbing: claw open', 'info')

    def update(self) -> Status:
        self._elapsed += self._tick_dt
        if self._elapsed < 1.0:
            self._bb.send_cmd_vel(0.05, 0.0)
        elif self._elapsed < 2.0:
            self._bb.send_cmd_vel(0.0, 0.0)
        elif self._elapsed < 3.0:
            if not self._closed:
                self._bb.send_claw(False)  # close
                self._closed = True
                self._bb.log('Grabbing: claw close', 'info')
        else:
            self._bb.grabbed = True
            self._bb.log('Ball grabbed!', 'info')
            return Status.SUCCESS
        return Status.RUNNING


class ReturnHome(py_trees.behaviour.Behaviour):
    """Едет к home_xy (используя одометрию).

    Простая proportional controller: yaw к точке + forward пока далеко.
    SUCCESS когда расстояние < 0.15м.
    """

    def __init__(
        self, bb: RobotBlackboard,
        arrival_threshold_m: float = 0.15,
        max_linear: float = 0.10,
        max_angular: float = 0.5,
        timeout_s: float = 60.0,
        tick_dt: float = 0.1,
        name: str = 'ReturnHome',
    ):
        super().__init__(name=name)
        self._bb = bb
        self._arrival = arrival_threshold_m
        self._max_lin = max_linear
        self._max_ang = max_angular
        self._timeout = timeout_s
        self._tick_dt = tick_dt
        self._elapsed = 0.0

    def initialise(self):
        self._elapsed = 0.0

    def update(self) -> Status:
        self._elapsed += self._tick_dt
        if self._elapsed > self._timeout:
            self._bb.send_cmd_vel(0.0, 0.0)
            return Status.FAILURE

        hx, hy = self._bb.home_xy
        dx = hx - self._bb.pose.x
        dy = hy - self._bb.pose.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < self._arrival:
            self._bb.send_cmd_vel(0.0, 0.0)
            self._bb.log('Returned home', 'info')
            return Status.SUCCESS

        # Угол к цели в мировых координатах
        target_yaw = math.atan2(dy, dx)
        yaw_err = _normalise_angle(target_yaw - self._bb.pose.theta)
        angular = max(-self._max_ang, min(self._max_ang, 1.5 * yaw_err))
        # Едем вперёд только если смотрим в нужную сторону
        linear = self._max_lin if abs(yaw_err) < 0.4 else 0.0
        self._bb.send_cmd_vel(linear, angular)
        return Status.RUNNING


class StopMotion(py_trees.behaviour.Behaviour):
    """Один тик: cmd_vel(0,0). Используется в idle-ветке."""

    def __init__(self, bb: RobotBlackboard, name: str = 'Stop'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        self._bb.send_cmd_vel(0.0, 0.0)
        return Status.SUCCESS


class HoldIdle(py_trees.behaviour.Behaviour):
    """Возвращает RUNNING, ничего не делает (идле в root selector)."""

    def __init__(self, bb: RobotBlackboard, name: str = 'Idle'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        return Status.RUNNING


class ClearTarget(py_trees.behaviour.Behaviour):
    """Сбросить target_colour/action — после успешного grab+return."""

    def __init__(self, bb: RobotBlackboard, name: str = 'ClearTarget'):
        super().__init__(name=name)
        self._bb = bb

    def update(self) -> Status:
        self._bb.clear_target()
        return Status.SUCCESS


# ── Helpers ────────────────────────────────────────────────────────────
def _normalise_angle(rad: float) -> float:
    """Привести угол к диапазону [-pi, pi]."""
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad
