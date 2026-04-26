"""
Корневое BT для робота-охотника за мячами (#1, 2026-04).

Структура:

    root: Selector "Hunt"
      ├─ ManualOverride? (Conditional Sequence)
      │    └─ Stop  (когда manual — BT не управляет, но публикует stop
      │             чтобы автономные cmd_vel не конкурировали)
      │
      ├─ HasGrabbed?  → Sequence "DeliverHome"
      │     ├─ ReturnHome
      │     └─ ClearTarget
      │
      ├─ HasTarget? → Sequence "GrabRoutine"
      │     ├─ Selector "FindAndCenter"
      │     │     ├─ Sequence "AlreadyVisible"
      │     │     │     ├─ BallVisible?
      │     │     │     └─ CenterBall
      │     │     └─ SearchSpin       ← spin до полного оборота
      │     ├─ ApproachBall            ← к мячу пока range >= 10cm
      │     ├─ BallClose?              ← страховка: ультразвук < 10cm
      │     └─ GrabBall                ← скрипт открыть/закрыть
      │
      └─ Idle (HoldIdle, RUNNING)      ← дефолтный fallback

Selector выполняет ветки слева-направо: первая SUCCESS/RUNNING выигрывает.
Sequence выполняет слева-направо: останавливается на первой FAILURE.

Если grab завершился успешно (GrabBall→SUCCESS, grabbed=True), на
следующем тике HasGrabbed? выберет ветку DeliverHome.
"""
from __future__ import annotations

import py_trees

from .behaviours import (
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
from .blackboard import RobotBlackboard


def build_main_tree(bb: RobotBlackboard) -> py_trees.behaviour.Behaviour:
    """Собрать главное BT робота. Возвращает root узел.

    Используется py_trees.trees.BehaviourTree(root) в runner.py для tick'а.
    """
    # ── Manual override branch ─────────────────────────────────────────
    manual_seq = py_trees.composites.Sequence(
        name='ManualOverride', memory=False,
        children=[ManualOverrideActive(bb), StopMotion(bb)],
    )

    # ── Deliver-home branch (после grab) ───────────────────────────────
    deliver_seq = py_trees.composites.Sequence(
        name='DeliverHome', memory=True,
        children=[HasGrabbed(bb), ReturnHome(bb), ClearTarget(bb)],
    )

    # ── Find/Center sub-tree ───────────────────────────────────────────
    already_visible = py_trees.composites.Sequence(
        name='AlreadyVisible', memory=False,
        children=[BallVisible(bb), CenterBall(bb)],
    )
    find_and_center = py_trees.composites.Selector(
        name='FindAndCenter', memory=False,
        children=[already_visible, SearchSpin(bb)],
    )

    # ── Grab routine ───────────────────────────────────────────────────
    grab_seq = py_trees.composites.Sequence(
        name='GrabRoutine', memory=True,
        children=[
            HasTarget(bb),
            find_and_center,
            ApproachBall(bb),
            BallClose(bb),
            GrabBall(bb),
        ],
    )

    # ── Idle fallback ──────────────────────────────────────────────────
    idle = HoldIdle(bb, name='Idle')

    # ── Root selector ──────────────────────────────────────────────────
    root = py_trees.composites.Selector(
        name='Hunt', memory=False,
        children=[manual_seq, deliver_seq, grab_seq, idle],
    )
    return root
