"""
IMU, ultrasonic, battery, temperature, watchdog.
"""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field

from .common import OkResponse


# ── Domain models ──────────────────────────────────────────────────────────
class UltrasonicData(BaseModel):
    """HC-SR04 показание дистанции."""
    range_m: float = Field(default=2.0, description='Дистанция в метрах. -1 = sensor error')
    age_s: Optional[float] = Field(
        default=None,
        description='Возраст показания (сек) — если > 1.0, считать stale'
    )


class Vec3(BaseModel):
    """3D вектор (для accel, gyro)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class ImuYpr(BaseModel):
    """Углы Эйлера в радианах. EKF и raw версии."""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0


class ImuData(BaseModel):
    """Полный IMU-снимок с MPU6050."""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    gyro: Vec3 = Field(default_factory=Vec3)
    accel: Vec3 = Field(default_factory=Vec3)
    ekf: Optional[ImuYpr] = Field(
        default=None,
        description='Углы из EKF фильтра (если включен imu.ekf.enabled в config)'
    )


class BatteryStatus(BaseModel):
    """Состояние батареи (2S LiPo через ADS7830)."""
    voltage: float = Field(default=-1.0, description='Напряжение в вольтах. -1 = нет данных')
    percent: int = Field(default=-1, description='0..100, -1 = нет данных')
    status: Optional[str] = Field(
        default=None,
        description='Опциональный статус: ok / low / critical'
    )


class TemperatureData(BaseModel):
    """CPU температура Pi."""
    value: float = Field(default=-1.0, description='Температура. -1 = нет данных')
    unit: str = 'C'


class WatchdogStatus(BaseModel):
    """Статус мониторинга связи (от watchdog_node)."""
    online: bool = False
    last_seen_s: Optional[float] = Field(
        default=None,
        description='Сколько секунд назад был последний heartbeat'
    )
    critical_topics: dict[str, bool] = Field(
        default_factory=dict,
        description='Карта critical-топик → ok? {odom: true, imu: false}'
    )


# ── Composite ──────────────────────────────────────────────────────────────
class SensorsBundle(BaseModel):
    """Все сенсоры одним объектом — для GET /api/sensors."""
    ultrasonic: UltrasonicData = Field(default_factory=UltrasonicData)
    imu: ImuData = Field(default_factory=ImuData)


# ── Response models ────────────────────────────────────────────────────────
class SensorsResponse(OkResponse, SensorsBundle):
    pass


class UltrasonicResponse(OkResponse, UltrasonicData):
    pass


class ImuResponse(OkResponse, ImuData):
    pass


class BatteryResponse(OkResponse, BatteryStatus):
    pass


class TemperatureResponse(OkResponse, TemperatureData):
    pass
