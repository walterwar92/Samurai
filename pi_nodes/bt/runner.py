"""
BehaviourTreeRunner — обёртка для py_trees.trees.BehaviourTree с тиком.

Минимальная вокруг py_trees: сахар для setup() + tick() + repr дерева
для логирования. Не содержит MQTT-логики — её добавляет fsm_bt_node.py.
"""
from __future__ import annotations

import py_trees

from .blackboard import RobotBlackboard
from .tree import build_main_tree


class BehaviourTreeRunner:
    """Композит RobotBlackboard + BehaviourTree.

    Использование:
        runner = BehaviourTreeRunner(bb)
        runner.tick()                   # один шаг (call периодически)
        runner.snapshot_state()         # для status payload
    """

    def __init__(self, bb: RobotBlackboard):
        self._bb = bb
        self._root = build_main_tree(bb)
        self._tree = py_trees.trees.BehaviourTree(self._root)
        self._tree.setup(timeout=15)

    @property
    def blackboard(self) -> RobotBlackboard:
        return self._bb

    def tick(self) -> None:
        """Один тик дерева."""
        self._tree.tick()

    def root_status(self) -> py_trees.common.Status:
        return self._root.status

    def active_path(self) -> str:
        """Имена tick'нутых узлов от root до tip через ' > '.

        Tip — самый глубокий узел, который py_trees реально tick'нул в
        последнем проходе. Для RUNNING-веток это активный лист, для
        SUCCESS/FAILURE — последний выполненный лист ветки. Если ничего
        ещё не выполнялось (статус INVALID) — возвращаем имя root.
        """
        tip = self._root.tip()
        if tip is None:
            return self._root.name
        # Собираем путь от root до tip через _parent ссылки py_trees.
        names: list[str] = []
        node = tip
        while node is not None:
            names.append(node.name)
            node = node.parent
        return ' > '.join(reversed(names))

    def snapshot_state(self) -> dict:
        """Снапшот для публикации в MQTT status."""
        bb = self._bb.snapshot()
        bb['bt_status'] = str(self.root_status())
        bb['bt_active'] = self.active_path()
        return bb
