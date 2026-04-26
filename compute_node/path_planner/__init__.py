"""
compute_node.path_planner — A* path planner для робота на ноутбуке (#3).

Запуск:
    python -m compute_node.path_planner

Архитектура:
  - astar.py  — чистый алгоритм A* на 2D occupancy grid (без MQTT/state)
  - node.py   — MQTT-нода: подписки на slam_map / odom / goal,
                публикация planned path / status

Подход (решение #3 = B): планировщик живёт на ноутбуке, не на Pi.
Pi занят 22 нодами, у ноутбука есть GPU и больше CPU. Планировщик
читает слой obstacles из slam_map (от Pi или dynamic-origin SLAM из #4),
расширяет препятствия на радиус робота и строит путь A*.

Препятствия:
  - obstacles из state.map.slam (Pi-side ultrasonic SLAM)
  - запретные зоны (forbidden zones) от dashboard

Output: список waypoints [[x, y], ...] в мировых координатах,
публикуется в samurai/{robot_id}/path_planner/path.
"""
__version__ = '1.0.0'
