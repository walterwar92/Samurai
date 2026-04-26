"""
FastAPI Depends helpers — извлекают синглтоны из app.state.

В app.py (C11) factory кладёт ссылки в app.state:

    app.state.dashboard_state = state
    app.state.mqtt_handlers   = mqtt
    app.state.ros2_subscribers = ros2_subs

Routers получают их через type-аннотации:

    from ._deps import StateDep, MQTTDep

    @router.get('/pose')
    async def get_pose(state: StateDep) -> PoseResponse: ...

    @router.post('/velocity')
    async def set_velocity(cmd: VelocityCommand, mqtt: MQTTDep) -> CommandAck: ...

ROS2Dep типизирован как Optional — на CI/тестах rclpy может быть
недоступен (нужны ROS2 deb-пакеты). Routers, которые требуют ROS2
(только save_map/load_map/yolo_enable), валидируют наличие сами.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Annotated, Optional

from fastapi import Depends, HTTPException, Request

if TYPE_CHECKING:
    from ..mqtt_handlers import MQTTHandlers
    from ..ros2_subscribers import ROS2Subscribers
    from ..state import DashboardState


def get_state(request: Request) -> 'DashboardState':
    state = getattr(request.app.state, 'dashboard_state', None)
    if state is None:
        raise HTTPException(500, 'dashboard_state not configured on app')
    return state


def get_mqtt(request: Request) -> 'MQTTHandlers':
    mqtt = getattr(request.app.state, 'mqtt_handlers', None)
    if mqtt is None:
        raise HTTPException(503, 'MQTT broker not configured (set MQTT_BROKER env)')
    return mqtt


def get_ros2(request: Request) -> Optional['ROS2Subscribers']:
    """Возвращает ROS2Subscribers или None если ROS2 недоступен.

    Endpoints, требующие ROS2 (save_map, load_map, yolo_enable), должны
    проверить is None и вернуть 503.
    """
    return getattr(request.app.state, 'ros2_subscribers', None)


# Type aliases для аннотаций в endpoints (PEP 593 Annotated).
StateDep = Annotated['DashboardState', Depends(get_state)]
MQTTDep = Annotated['MQTTHandlers', Depends(get_mqtt)]
ROS2Dep = Annotated[Optional['ROS2Subscribers'], Depends(get_ros2)]
