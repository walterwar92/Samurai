"""
pi_nodes.bt — Behaviour Tree (py_trees) для робота Samurai (#1, 2026-04).

Альтернативная реализация автономного поведения по сравнению с
fsm_node.py (классический FSM на if/elif). Преимущества BT:
  - Декларативная композиция: цели/проверки/действия — узлы;
    selector/sequence — операторы. Логика читается как дерево.
  - Прерывание: высоко-приоритетные ветки могут останавливать
    низкоприоритетные действия (например emergency stop).
  - Переиспользование: одно поведение (Approach) используется
    в нескольких ситуациях (поиск мяча, follow-me).

Архитектура:
  bt/
    blackboard.py — типы для общего state (поза, детекция, цель)
    behaviours.py — листья дерева (Search, Approach, Grab, Return, ...)
    tree.py       — корневое дерево + factory build_main_tree(...)
    runner.py     — обёртка для tick'а (через py_trees.trees.BehaviourTree)

Использование:
  from pi_nodes.bt import build_main_tree, run_tree
  tree = build_main_tree(robot_io)
  while True:
      tree.tick()
      time.sleep(0.1)

Решение #1 (A): py_trees как стандартная ROS2-community библиотека.
"""
from .tree import build_main_tree
from .runner import BehaviourTreeRunner
from .blackboard import RobotBlackboard

__all__ = ['build_main_tree', 'BehaviourTreeRunner', 'RobotBlackboard']
