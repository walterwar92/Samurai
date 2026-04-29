"""
compute_node.dashboard — FastAPI/SocketIO web-сервер на ноутбуке.

Замена монолитного compute_node/dashboard_node.py (1840 строк) на пакет
с разбивкой по доменам:

    dashboard/
    ├── __main__.py        — точка входа: python -m compute_node.dashboard
    ├── app.py             — FastAPI factory + глобальная сборка
    ├── state.py           — DashboardState (агрегированное состояние)
    ├── mqtt_handlers.py   — paho-mqtt subscriptions от Pi
    ├── ros2_subscribers.py — rclpy subscriptions (SLAM, EKF, YOLO)
    ├── schemas/           — Pydantic модели (request/response)
    │   ├── common.py
    │   ├── robot.py, sensors.py, detection.py, actuators.py
    │   ├── maps.py, control.py
    └── routers/           — APIRouter по доменам
        ├── robot.py, sensors.py, actuators.py, detection.py
        ├── fsm.py, maps.py, control.py, system.py
        ├── camera.py, samcan.py

Workflow миграции (incremental):
  C1 (этот коммит):     skeleton + __init__.py + __main__.py stub
  C2:  Pydantic schemas
  C3:  DashboardState class
  C4:  MQTT/ROS2 handlers
  C5-C10:  routers по доменам
  C11: /api/v1/ префикс + deprecated алиасы
  C12: TS-клиент через openapi-typescript-codegen
  C13: удалить старый dashboard_node.py + обновить launch.py

Старый compute_node/dashboard_node.py продолжает работать на каждом
этапе, миграция incremental.
"""

__version__ = '2.0.0-dev'  # 2.x = после refactor (#7)
