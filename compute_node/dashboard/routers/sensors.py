"""
Routers: sensors / ultrasonic / imu / battery / temperature.

Все только-чтение из DashboardState. Изменения приходят с Pi через
MQTT handlers (см. mqtt_handlers.py).

Маппинг старых endpoints → router:

  GET /api/sensors             → /sensors        (router prefix /api/v1)
  GET /api/sensors/ultrasonic  → /sensors/ultrasonic
  GET /api/sensors/imu         → /sensors/imu
  GET /api/battery             → /battery        (отдельный sub-router)
  GET /api/temperature         → /temperature    (отдельный sub-router)
"""
from __future__ import annotations

from fastapi import APIRouter

from ..schemas.sensors import (
    BatteryResponse,
    ImuResponse,
    SensorsResponse,
    TemperatureResponse,
    UltrasonicResponse,
)
from ._deps import StateDep

# /sensors/* group
router = APIRouter()


@router.get('', response_model=SensorsResponse, tags=['sensors'])
async def get_sensors(state: StateDep) -> SensorsResponse:
    """Bundle: ультразвук + IMU. Для batched-запросов с дашборда."""
    with state.lock:
        return SensorsResponse(
            ultrasonic=state.sensors.ultrasonic,
            imu=state.sensors.imu,
        )


@router.get('/ultrasonic', response_model=UltrasonicResponse, tags=['sensors'])
async def get_ultrasonic(state: StateDep) -> UltrasonicResponse:
    with state.lock:
        u = state.sensors.ultrasonic
    return UltrasonicResponse(range_m=u.range_m, age_s=u.age_s)


@router.get('/imu', response_model=ImuResponse, tags=['sensors'])
async def get_imu(state: StateDep) -> ImuResponse:
    with state.lock:
        i = state.sensors.imu
    return ImuResponse(
        yaw=i.yaw, pitch=i.pitch, roll=i.roll,
        gyro=i.gyro, accel=i.accel, ekf=i.ekf,
    )


# /battery (отдельный path — не под /sensors/)
battery_router = APIRouter()


@battery_router.get('', response_model=BatteryResponse, tags=['sensors'])
async def get_battery(state: StateDep) -> BatteryResponse:
    """Voltage, percent, status (ok/low/critical)."""
    with state.lock:
        b = state.sensors.battery
    return BatteryResponse(voltage=b.voltage, percent=b.percent, status=b.status)


# /temperature (отдельный path)
temperature_router = APIRouter()


@temperature_router.get('', response_model=TemperatureResponse, tags=['sensors'])
async def get_temperature(state: StateDep) -> TemperatureResponse:
    """CPU температура Pi."""
    with state.lock:
        t = state.sensors.temperature
    return TemperatureResponse(value=t.value, unit=t.unit)
