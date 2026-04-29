"""
Юнит-тесты для pi_nodes/bt/* (#1, 2026-04).

Проверяем:
  - Conditions возвращают правильный Status (HasTarget, BallVisible, ...)
  - Actions публикуют ожидаемые cmd_vel/claw через bb-callback'и
  - Главное дерево корректно переключается между ветками
  - Полный сценарий grab: search → center → approach → grab → return → clear

Запуск:
    pytest tests/test_bt_behaviours.py -v
"""
from __future__ import annotations

import math
import os
import sys

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, REPO_ROOT)

import py_trees  # noqa: E402

from pi_nodes.bt import RobotBlackboard, BehaviourTreeRunner  # noqa: E402
from pi_nodes.bt.behaviours import (  # noqa: E402
    ApproachBall,
    BallClose,
    BallVisible,
    CenterBall,
    ClearTarget,
    GrabBall,
    HasGrabbed,
    HasTarget,
    HoldIdle,
    ManualOverrideActive,
    ReturnHome,
    SearchSpin,
    StopMotion,
)

Status = py_trees.common.Status


# ── Helpers ─────────────────────────────────────────────────────────────
def make_bb() -> tuple[RobotBlackboard, list]:
    """Blackboard + список published действий (для assertion)."""
    bb = RobotBlackboard()
    actions: list = []
    bb.send_cmd_vel = lambda lin, ang: actions.append(('vel', lin, ang))
    bb.send_claw = lambda is_open: actions.append(('claw', is_open))
    bb.log = lambda msg, level='info': None
    return bb, actions


# ── Conditions ──────────────────────────────────────────────────────────
def test_has_target_initially_false():
    bb, _ = make_bb()
    assert HasTarget(bb).update() == Status.FAILURE


def test_has_target_after_set():
    bb, _ = make_bb()
    bb.set_target('red')
    assert HasTarget(bb).update() == Status.SUCCESS


def test_ball_visible_only_when_fresh():
    bb, _ = make_bb()
    assert BallVisible(bb).update() == Status.FAILURE
    bb.update_detection({'colour': 'red', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    assert BallVisible(bb).update() == Status.SUCCESS


def test_ball_visible_filters_by_colour():
    bb, _ = make_bb()
    bb.set_target('blue')
    bb.update_detection({'colour': 'red', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    assert BallVisible(bb).update() == Status.FAILURE
    bb.update_detection({'colour': 'blue', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    assert BallVisible(bb).update() == Status.SUCCESS


def test_ball_close_threshold():
    bb, _ = make_bb()
    bb.update_range(0.5)
    assert BallClose(bb, threshold_m=0.10).update() == Status.FAILURE
    bb.update_range(0.05)
    assert BallClose(bb, threshold_m=0.10).update() == Status.SUCCESS


def test_manual_override_condition():
    bb, _ = make_bb()
    assert ManualOverrideActive(bb).update() == Status.FAILURE
    bb.set_manual_override(True)
    assert ManualOverrideActive(bb).update() == Status.SUCCESS


def test_has_grabbed():
    bb, _ = make_bb()
    assert HasGrabbed(bb).update() == Status.FAILURE
    bb.grabbed = True
    assert HasGrabbed(bb).update() == Status.SUCCESS


# ── Actions ─────────────────────────────────────────────────────────────
def test_stop_motion_publishes_zero():
    bb, actions = make_bb()
    assert StopMotion(bb).update() == Status.SUCCESS
    assert actions == [('vel', 0.0, 0.0)]


def test_search_spin_running_then_success_when_visible():
    bb, actions = make_bb()
    spin = SearchSpin(bb, angular_speed=0.4, max_angle_rad=10.0)
    spin.initialise()

    # Без цели — крутится (RUNNING)
    assert spin.update() == Status.RUNNING
    assert actions[-1] == ('vel', 0.0, -0.4)

    # Появился мяч — SUCCESS
    bb.update_detection({'colour': 'red', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    assert spin.update() == Status.SUCCESS


def test_search_spin_failure_after_full_rotation():
    """SearchSpin даёт FAILURE когда accumulated превышает max_angle.

    max_angle=1.0, +=0.04/тик → ~25 тиков до переполнения, FAILURE
    наступает в пределах max_ticks. Не проверяем точное число тиков
    (float-арифметика не предсказуема), а лишь конечный переход.
    """
    bb, _ = make_bb()
    spin = SearchSpin(bb, angular_speed=0.4, max_angle_rad=1.0)
    spin.initialise()
    saw_failure = False
    for _ in range(50):
        s = spin.update()
        if s == Status.FAILURE:
            saw_failure = True
            break
        assert s == Status.RUNNING
    assert saw_failure, 'SearchSpin не дал FAILURE за 50 тиков (max_angle=1.0)'


def test_center_ball_success_when_centred():
    bb, actions = make_bb()
    bb.update_detection({'colour': 'red', 'x': 295, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    cb = CenterBall(bb, threshold=0.15)
    cb.initialise()
    assert cb.update() == Status.SUCCESS
    assert actions[-1] == ('vel', 0.0, 0.0)


def test_center_ball_running_when_off_centre():
    bb, _ = make_bb()
    bb.update_detection({'colour': 'red', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    cb = CenterBall(bb, threshold=0.15)
    cb.initialise()
    assert cb.update() == Status.RUNNING


def test_center_ball_failure_after_lost():
    bb, _ = make_bb()
    cb = CenterBall(bb, max_lost_frames=2)
    cb.initialise()
    # Нет fresh detection — теряем кадры
    assert cb.update() == Status.RUNNING
    assert cb.update() == Status.RUNNING
    assert cb.update() == Status.FAILURE


def test_approach_success_when_close():
    bb, actions = make_bb()
    bb.update_detection({'colour': 'red', 'x': 295, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    bb.update_range(0.05)
    ap = ApproachBall(bb)
    ap.initialise()
    assert ap.update() == Status.SUCCESS
    assert actions[-1] == ('vel', 0.0, 0.0)


def test_approach_running_until_close():
    bb, actions = make_bb()
    bb.update_detection({'colour': 'red', 'x': 295, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    bb.update_range(1.0)
    ap = ApproachBall(bb)
    ap.initialise()
    assert ap.update() == Status.RUNNING
    # cmd_vel должен быть с положительным линейным
    last = actions[-1]
    assert last[0] == 'vel'
    assert last[1] > 0


def test_grab_ball_sequence():
    bb, actions = make_bb()
    bb.log = lambda msg, level='info': None
    grab = GrabBall(bb, tick_dt=1.1)  # большие шаги — быстро пройти все этапы
    grab.initialise()
    # Initialise сразу опубликовала claw(open)
    assert ('claw', True) in actions
    actions.clear()

    # tick 1 (elapsed=1.1): фаза 1..2с → cmd_vel(0,0)
    assert grab.update() == Status.RUNNING
    assert ('vel', 0.0, 0.0) in actions

    # tick 2 (elapsed=2.2): фаза 2..3с → claw(close)
    actions.clear()
    assert grab.update() == Status.RUNNING
    assert ('claw', False) in actions

    # tick 3 (elapsed=3.3): >3с → SUCCESS, grabbed=True
    actions.clear()
    assert grab.update() == Status.SUCCESS
    assert bb.grabbed is True


def test_return_home_success_when_arrived():
    bb, actions = make_bb()
    bb.home_xy = (0.0, 0.0)
    bb.update_pose(0.05, 0.05, 0.0)  # уже близко
    rh = ReturnHome(bb, arrival_threshold_m=0.15)
    rh.initialise()
    assert rh.update() == Status.SUCCESS
    assert actions[-1] == ('vel', 0.0, 0.0)


def test_return_home_drives_when_far():
    bb, actions = make_bb()
    bb.home_xy = (0.0, 0.0)
    bb.update_pose(2.0, 0.0, math.pi)  # 2м справа, смотрит в правильную сторону
    rh = ReturnHome(bb)
    rh.initialise()
    assert rh.update() == Status.RUNNING
    last = actions[-1]
    assert last[0] == 'vel'
    assert last[1] > 0  # forward


def test_clear_target():
    bb, _ = make_bb()
    bb.set_target('red', 'grab')
    bb.grabbed = True
    assert ClearTarget(bb).update() == Status.SUCCESS
    assert bb.target_colour == ''
    assert bb.grabbed is False


# ── Tree composition ───────────────────────────────────────────────────
def test_tree_idle_when_no_target():
    bb, _ = make_bb()
    runner = BehaviourTreeRunner(bb)
    runner.tick()
    assert 'Idle' in runner.active_path()


def test_tree_searches_when_target_set_no_ball():
    bb, _ = make_bb()
    bb.set_target('red')
    runner = BehaviourTreeRunner(bb)
    runner.tick()
    assert 'SearchSpin' in runner.active_path()


def test_tree_centers_when_ball_visible():
    bb, _ = make_bb()
    bb.set_target('red')
    bb.update_detection({'colour': 'red', 'x': 100, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    runner = BehaviourTreeRunner(bb)
    runner.tick()
    assert 'CenterBall' in runner.active_path()


def test_tree_approaches_when_centered():
    bb, _ = make_bb()
    bb.set_target('red')
    bb.update_detection({'colour': 'red', 'x': 295, 'y': 100, 'w': 50, 'h': 50, 'conf': 0.9})
    bb.update_range(1.0)  # далеко от ультразвука
    runner = BehaviourTreeRunner(bb)
    # Может занять несколько тиков чтобы CenterBall дал SUCCESS и
    # перешло на ApproachBall.
    for _ in range(3):
        runner.tick()
        if 'ApproachBall' in runner.active_path():
            break
    assert 'ApproachBall' in runner.active_path()


def test_tree_manual_override_wins():
    bb, actions = make_bb()
    bb.set_target('red')
    bb.set_manual_override(True)
    runner = BehaviourTreeRunner(bb)
    runner.tick()
    assert 'ManualOverride' in runner.active_path()
    assert ('vel', 0.0, 0.0) in actions


def test_tree_returns_home_after_grab():
    bb, _ = make_bb()
    bb.set_target('red')
    bb.grabbed = True
    bb.update_pose(2.0, 0.0, math.pi)
    runner = BehaviourTreeRunner(bb)
    runner.tick()
    assert 'ReturnHome' in runner.active_path()


def test_tree_clears_target_after_arrival():
    bb, _ = make_bb()
    bb.set_target('red')
    bb.grabbed = True
    bb.update_pose(0.0, 0.0, 0.0)  # уже дома
    runner = BehaviourTreeRunner(bb)
    # Ход 1: ReturnHome выдаст SUCCESS, sequence перейдёт на ClearTarget
    runner.tick()
    runner.tick()
    assert bb.target_colour == ''
    assert bb.grabbed is False
