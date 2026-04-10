#!/usr/bin/env python3
"""
test_precision_drive.py — Тест точного движения вперёд / назад N см.

Прямой доступ к железу (без MQTT / ROS2).

Алгоритм точности:
  1. Калибровка IMU: 200 сэмплов → gyro bias + вектор гравитации по всем 3 осям
  2. EkfImu: Kalman-фильтр рысканья + bias-коррекция гироскопа Z, ZUPT-гистерезис
  3. AccelPositionEstimator + VelocityEKF:
       — компенсация гравитации по 3 осям с поправкой на наклон (roll/pitch)
       — Coriolis-поправка (широта Москвы)
       — медианный + IIR-2 фильтры (подавление вибраций двигателей)
       — ZUPT с гистерезисом (детектор нуля скорости)
       — адаптивный масштаб колёсной одометрии
  4. Трапециевидный профиль скорости: плавный разгон → круиз → торможение
  5. Замкнутый контур по позиции, остановка в ±3 мм от цели

Запуск на Pi:
    python3 tests/test_precision_drive.py

Зависимости: smbus2, adafruit-circuitpython-pca9685, adafruit-circuitpython-motor
"""

import math
import os
import struct
import sys
import time

# Корень проекта — чтобы импортировать pi_nodes.*
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

try:
    import smbus2
    _HW_IMU = True
except ImportError:
    _HW_IMU = False
    print('[WARN] smbus2 не найден — IMU работает в симуляции')

from pi_nodes.hardware.motor_driver import MotorDriver
from pi_nodes.filters.accel_position import AccelPositionEstimator
from pi_nodes.filters.ekf_imu import EkfImu

# ─── Константы MPU-6050 ───────────────────────────────────────────────────────
MPU6050_ADDR = 0x68
I2C_BUS      = 1
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG  = 0x1B
DLPF_CFG     = 0x1A   # Digital Low-Pass Filter
SMPLRT_DIV   = 0x19   # Sample Rate Divider

ACCEL_SCALE = 16384.0  # ±2g → 16384 LSB/g
GYRO_SCALE  = 131.0    # ±250°/s → 131 LSB/°/s
DEG2RAD     = math.pi / 180.0
G_STD       = 9.80665  # м/с² (стандартное g)

# ─── Параметры движения ───────────────────────────────────────────────────────
CRUISE_SPEED_PCT  = 28.0   # % от max — медленнее = точнее (>12 — мин для движения)
MIN_SPEED_PCT     = 12.0   # % — минимум при торможении (ниже моторы могут не тронуться)
MAX_SPEED_M_S     = 0.30   # м/с — физический максимум (fast profile)
ACCEL_ZONE_M      = 0.06   # м — зона плавного разгона
DECEL_ZONE_M      = 0.08   # м — зона торможения
STOP_THRESHOLD_M  = 0.003  # м — допустимая ошибка остановки (3 мм)
STOP_WAIT_S       = 1.0    # с — время ожидания ZUPT после остановки

# ─── IMU / цикл ──────────────────────────────────────────────────────────────
LOOP_HZ             = 50
DT                  = 1.0 / LOOP_HZ
CALIBRATION_SAMPLES = 200  # 200 / 50 Гц = 4 с
EMA_ALPHA_ACCEL     = 0.2  # ≈ 8 Гц среза
EMA_ALPHA_GYRO      = 0.3  # ≈ 12 Гц среза


# ─────────────────────────────────────────────────────────────────────────────
class MPU6050:
    """Прямое чтение MPU-6050 через I2C (smbus2)."""

    def __init__(self, bus: int = I2C_BUS, addr: int = MPU6050_ADDR):
        self._addr = addr
        self._bus  = None
        if not _HW_IMU:
            return
        self._bus = smbus2.SMBus(bus)
        self._bus.write_byte_data(addr, PWR_MGMT_1, 0x00)   # выход из сна
        time.sleep(0.1)
        # DLPF код 3 → bandwidth ≈ 44 Гц, задержка 4.9 мс
        self._bus.write_byte_data(addr, DLPF_CFG, 0x03)
        # Sample rate = 1000 / (1 + 19) = 50 Гц
        self._bus.write_byte_data(addr, SMPLRT_DIV, 19)
        # Accel ±2g
        self._bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
        # Gyro ±250°/s
        self._bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
        time.sleep(0.05)

    def read(self):
        """Вернуть (ax, ay, az, gx, gy, gz) в м/с² и рад/с."""
        if self._bus is None:
            # Симуляция: покой на плоском полу
            return 0.0, 0.0, G_STD, 0.0, 0.0, 0.0
        raw = self._bus.read_i2c_block_data(self._addr, ACCEL_XOUT_H, 14)
        v = struct.unpack('>7h', bytes(raw))
        # Порядок: ax, ay, az, temp, gx, gy, gz
        ax = v[0] / ACCEL_SCALE * G_STD
        ay = v[1] / ACCEL_SCALE * G_STD
        az = v[2] / ACCEL_SCALE * G_STD
        gx = v[4] / GYRO_SCALE * DEG2RAD
        gy = v[5] / GYRO_SCALE * DEG2RAD
        gz = v[6] / GYRO_SCALE * DEG2RAD
        return ax, ay, az, gx, gy, gz

    def close(self):
        if self._bus:
            self._bus.close()


# ─────────────────────────────────────────────────────────────────────────────
def calibrate(imu: MPU6050, n: int = CALIBRATION_SAMPLES):
    """
    Статическая калибровка: n сэмплов в покое.

    Возвращает:
        gyro_bias    — средний drift гироскопа (рад/с), 3 оси
        gravity_body — вектор гравитации в системе тела (м/с²), 3 оси
    """
    print(f'\n[CAL] Калибровка — {n} сэмплов ({n / LOOP_HZ:.0f} с). '
          'НЕ ДВИГАЙТЕ робота...')
    sx = sy = sz = 0.0
    sgx = sgy = sgz = 0.0
    for i in range(n):
        ax, ay, az, gx, gy, gz = imu.read()
        sx += ax;   sy += ay;   sz += az
        sgx += gx;  sgy += gy;  sgz += gz
        time.sleep(DT)
        if (i + 1) % 50 == 0:
            print(f'  {i + 1}/{n}...', end='\r', flush=True)

    grav  = (sx / n, sy / n, sz / n)
    gbias = (sgx / n, sgy / n, sgz / n)
    g_mag = math.sqrt(sum(v * v for v in grav))

    print(f'\n[CAL] g_body = ({grav[0]:+.3f}, {grav[1]:+.3f}, {grav[2]:+.3f})'
          f'  |g| = {g_mag:.4f} м/с²')
    print(f'[CAL] bias   = ({gbias[0]:+.5f}, {gbias[1]:+.5f}, {gbias[2]:+.5f}) рад/с')
    return gbias, grav


# ─────────────────────────────────────────────────────────────────────────────
class _Ema3:
    """EMA-фильтр первого порядка по трём осям."""

    __slots__ = ('a', 'x', 'y', 'z')

    def __init__(self, alpha: float, init=(0.0, 0.0, 0.0)):
        self.a = alpha
        self.x, self.y, self.z = init

    def __call__(self, x, y, z):
        self.x += self.a * (x - self.x)
        self.y += self.a * (y - self.y)
        self.z += self.a * (z - self.z)
        return self.x, self.y, self.z


# ─────────────────────────────────────────────────────────────────────────────
def _speed_pct(pos: float, target: float) -> float:
    """
    Трапециевидный профиль скорости.

    pos    — текущая позиция (м)
    target — целевая позиция (м; отрицательная = назад)
    Возвращает: % скорости со знаком направления
    """
    progress  = abs(pos)
    dist_left = abs(target - pos)

    if progress < ACCEL_ZONE_M:
        t = progress / ACCEL_ZONE_M
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    elif dist_left < DECEL_ZONE_M:
        t = max(0.0, dist_left / DECEL_ZONE_M)
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    else:
        spd = CRUISE_SPEED_PCT

    return spd if target >= 0.0 else -spd


# ─────────────────────────────────────────────────────────────────────────────
def _imu_step(imu, ekf, estimator, gbias, ema_a, ema_g, moving: bool):
    """
    Один шаг: читаем IMU → фильтруем → EKF → обновляем оценщик позиции.
    Возвращает текущую позицию (estimator.x).
    """
    ax_r, ay_r, az_r, gx_r, gy_r, gz_r = imu.read()

    # Вычитаем статический gyro bias (EKF дополнительно корректирует residual Z)
    gx_c = gx_r - gbias[0]
    gy_c = gy_r - gbias[1]
    gz_c = gz_r - gbias[2]

    ax, ay, az = ema_a(ax_r, ay_r, az_r)
    gx, gy, gz = ema_g(gx_c, gy_c, gz_c)

    ekf.predict(gx, gy, gz, DT)
    ekf.update(ax, ay, az)

    ts = time.monotonic()  # реальный таймстамп — AccelPositionEstimator
                           # вычислит dt автоматически из двух ts подряд
    estimator.update_imu(ax, ay, az, gx, gy, gz,
                         ekf.roll, ekf.pitch, ekf.yaw, DT, ts)
    return estimator.x


# ─────────────────────────────────────────────────────────────────────────────
def _wait_zupt(imu, motors, estimator, ekf, gbias, ema_a, ema_g,
               timeout: float = STOP_WAIT_S):
    """Держать моторы выключенными, обновлять оценщик — ждать ZUPT."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        t1 = time.monotonic()
        _imu_step(imu, ekf, estimator, gbias, ema_a, ema_g, moving=False)
        estimator.set_cmd_moving(False)
        estimator.update_wheel_odom(0.0, ekf.yaw, DT)
        estimator.blend()
        if estimator.is_stationary:
            break
        elapsed = time.monotonic() - t1
        if elapsed < DT:
            time.sleep(DT - elapsed)


# ─────────────────────────────────────────────────────────────────────────────
def drive_segment(
        imu: MPU6050,
        motors: MotorDriver,
        estimator: AccelPositionEstimator,
        ekf: EkfImu,
        gbias: tuple,
        ema_a: _Ema3,
        ema_g: _Ema3,
        target_m: float,
        label: str) -> float:
    """
    Двигаться до target_m (м) с замкнутым контуром по позиции.
    estimator должен быть сброшен в 0 перед вызовом.
    Возвращает итоговую позицию (м).
    """
    print(f'\n[{label}]  цель = {target_m * 100:+.1f} см')
    t_safety = time.monotonic()

    while True:
        t0 = time.monotonic()

        # ── IMU + позиция ────────────────────────────────────────────────────
        pos = _imu_step(imu, ekf, estimator, gbias, ema_a, ema_g, moving=True)

        # ── Проверка условия остановки ───────────────────────────────────────
        overshoot = (target_m > 0.0 and pos >= target_m) or \
                    (target_m < 0.0 and pos <= target_m)
        dist_left = abs(target_m - pos)

        if overshoot or dist_left <= STOP_THRESHOLD_M:
            motors.stop()
            estimator.set_cmd_moving(False)
            _wait_zupt(imu, motors, estimator, ekf, gbias, ema_a, ema_g)
            err_mm = (pos - target_m) * 1000.0
            print(f'  Позиция: {pos * 100:.2f} см  '
                  f'ошибка: {err_mm:+.1f} мм')
            return pos

        # ── Команда скоростью ────────────────────────────────────────────────
        spd = _speed_pct(pos, target_m)
        v_cmd = spd / 100.0 * MAX_SPEED_M_S   # м/с для wheel_odom

        estimator.set_cmd_moving(True)
        estimator.update_wheel_odom(v_cmd, ekf.yaw, DT)
        estimator.blend()

        motors.move(spd, 0.0)

        # ── Лог каждые 0.5 с ─────────────────────────────────────────────────
        if int((time.monotonic() - t_safety) * 2) % 4 == 0:
            print(f'  pos={pos * 100:6.2f} см  '
                  f'до цели={dist_left * 100:.2f} см  '
                  f'spd={spd:.1f}%',
                  end='\r', flush=True)

        # ── Таймаут безопасности ─────────────────────────────────────────────
        if time.monotonic() - t_safety > 30.0:
            motors.stop()
            print('\n[WARN] Таймаут 30 с — аварийная остановка')
            return pos

        # ── 50 Гц ────────────────────────────────────────────────────────────
        elapsed = time.monotonic() - t0
        if elapsed < DT:
            time.sleep(DT - elapsed)


# ─────────────────────────────────────────────────────────────────────────────
def main():
    dist_cm = float(input('Расстояние N (см): ').strip())
    target_m = dist_cm / 100.0

    print(f'\nПлан: +{dist_cm:.1f} см вперёд → пауза → -{dist_cm:.1f} см назад')

    imu    = MPU6050()
    motors = MotorDriver()

    # ── Калибровка ───────────────────────────────────────────────────────────
    gbias, grav = calibrate(imu)

    # Home roll/pitch из вектора гравитации
    home_roll  = math.atan2(grav[1], grav[2])
    home_pitch = math.atan2(-grav[0],
                            math.sqrt(grav[1] ** 2 + grav[2] ** 2))

    # ── EKF ориентации ───────────────────────────────────────────────────────
    ekf = EkfImu(q_angle=0.001, q_bias=0.0001, r_accel=0.5, accel_gate=0.5)
    ekf.init_from_calibration(home_roll, home_pitch, 0.0, gbias)

    # ── EMA-фильтры (инициализируем из калибровочных значений) ───────────────
    ema_a = _Ema3(EMA_ALPHA_ACCEL, init=grav)
    ema_g = _Ema3(EMA_ALPHA_GYRO,  init=(0.0, 0.0, 0.0))

    # ── AccelPositionEstimator ───────────────────────────────────────────────
    estimator = AccelPositionEstimator()
    estimator.set_calibration(
        gravity_body=tuple(grav),
        home_roll=home_roll,
        home_pitch=home_pitch,
    )

    input('\n[READY] Нажмите Enter для начала...')
    time.sleep(0.5)

    # ── Прогон ВПЕРЁД ────────────────────────────────────────────────────────
    estimator.reset()
    pos_fwd = drive_segment(imu, motors, estimator, ekf, gbias,
                            ema_a, ema_g, target_m, 'ВПЕРЁД')

    print('\n[PAUSE] 1.5 с...')
    time.sleep(1.5)

    # ── Прогон НАЗАД ─────────────────────────────────────────────────────────
    estimator.reset()
    pos_back = drive_segment(imu, motors, estimator, ekf, gbias,
                             ema_a, ema_g, -target_m, 'НАЗАД')

    # ── Итог ─────────────────────────────────────────────────────────────────
    print('\n' + '─' * 50)
    print(f'  Цель:         {dist_cm:.1f} см')
    print(f'  Вперёд:       {pos_fwd * 100:.2f} см  '
          f'(ошибка {(pos_fwd - target_m) * 1000:+.1f} мм)')
    print(f'  Назад:        {abs(pos_back) * 100:.2f} см  '
          f'(ошибка {(abs(pos_back) - target_m) * 1000:+.1f} мм)')
    print('─' * 50)

    motors.shutdown()
    imu.close()


if __name__ == '__main__':
    main()
