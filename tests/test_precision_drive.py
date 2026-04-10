#!/usr/bin/env python3
"""
test_precision_drive.py — Точное движение вперёд / назад N см.

Только колёсная одометрия: pos += v_cmd × dt_реальный
Никакого IMU / MQTT / ROS2.

═══ КАЛИБРОВКА (один раз) ════════════════════════════════════════════════
  1. Оставить WHEEL_SCALE = 1.0
  2. Запустить: python3 tests/test_precision_drive.py  →  N = 50
  3. Рулеткой измерить реальное расстояние X см
  4. Вписать: WHEEL_SCALE = X / 50.0
  5. Повторить — должна быть точность ±1–2 см
═══════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from pi_nodes.hardware.motor_driver import MotorDriver

# ─── Калибровка ──────────────────────────────────────────────────────────────
# Отношение реального расстояния к расчётному при данной скорости.
# Пример: ехали 50 см, реально проехали 45 → WHEEL_SCALE = 45 / 50 = 0.90
WHEEL_SCALE = 1.0           # ← настроить после первого прогона

# ─── Параметры движения ───────────────────────────────────────────────────────
MAX_SPEED_M_S    = 0.30     # м/с при 100% (fast-profile, не трогать)
CRUISE_SPEED_PCT = 50.0     # % — крейсерская скорость
MIN_SPEED_PCT    = 18.0     # % — минимум (ниже гусеницы могут не тронуться)

ACCEL_ZONE_M     = 0.06     # м — зона плавного разгона
DECEL_ZONE_M     = 0.10     # м — зона торможения
STOP_THRESHOLD_M = 0.005    # м — порог остановки (5 мм по одометрии)
BRAKE_SETTLE_S   = 0.25     # с — пауза после stop() для механической дотяжки

PAUSE_BETWEEN_S  = 1.5      # с — пауза между прогонами

LOOP_HZ          = 200      # Гц — частота управляющего цикла
DT_LOOP          = 1.0 / LOOP_HZ
DT_MAX           = 0.05     # с — кэп на dt (защита от первой итерации)


# ─────────────────────────────────────────────────────────────────────────────
def _speed_pct(pos: float, target: float) -> float:
    """
    Трапециевидный профиль скорости.

    pos    — текущая накопленная позиция (м)
    target — цель (м; отрицательная = назад)
    Возвращает % скорости со знаком направления.
    """
    progress  = abs(pos)
    dist_left = abs(target - pos)

    if progress < ACCEL_ZONE_M:
        # Зона разгона: MIN → CRUISE
        t   = progress / ACCEL_ZONE_M
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    elif dist_left < DECEL_ZONE_M:
        # Зона торможения: CRUISE → MIN
        t   = max(0.0, dist_left / DECEL_ZONE_M)
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    else:
        spd = CRUISE_SPEED_PCT

    return spd if target >= 0.0 else -spd


# ─────────────────────────────────────────────────────────────────────────────
def drive_segment(motors: MotorDriver, target_m: float, label: str) -> float:
    """
    Двигаться target_m метров.
    Одометрия: pos += (spd/100 × MAX_SPEED_M_S × WHEEL_SCALE) × dt_реальный
    Возвращает итоговую позицию по одометрии (м).
    """
    print(f'\n[{label}]  цель = {target_m * 100:+.1f} см')

    pos     = 0.0
    t_prev  = time.monotonic()
    t_start = t_prev
    t_log   = t_prev

    while True:
        t_now = time.monotonic()
        # Реальный dt — точнее фиксированного при джиттере ОС
        dt    = min(t_now - t_prev, DT_MAX)
        t_prev = t_now

        # ── 1. Вычисляем скорость для текущей позиции ─────────────────────
        spd       = _speed_pct(pos, target_m)
        v_m_s     = spd / 100.0 * MAX_SPEED_M_S * WHEEL_SCALE

        # ── 2. Проверяем условие остановки (до накопления, чтобы не перелететь)
        dist_left = abs(target_m - pos)
        overshoot = (target_m > 0.0 and pos >= target_m) or \
                    (target_m < 0.0 and pos <= target_m)

        if overshoot or dist_left <= STOP_THRESHOLD_M:
            motors.stop()
            time.sleep(BRAKE_SETTLE_S)
            err_mm = (pos - target_m) * 1000.0
            print(f'\n  Готово: одометрия = {pos * 100:.1f} см  '
                  f'(Δ {err_mm:+.0f} мм по одометрии)')
            return pos

        # ── 3. Накапливаем позицию ─────────────────────────────────────────
        pos += v_m_s * dt

        # ── 4. Команда моторам ─────────────────────────────────────────────
        motors.move(spd, 0.0)

        # ── 5. Лог каждые 0.3 с ───────────────────────────────────────────
        if t_now - t_log >= 0.3:
            t_log = t_now
            print(f'  pos={pos * 100:6.1f} см  '
                  f'осталось={dist_left * 100:5.1f} см  '
                  f'v={abs(spd):.0f}%   ',
                  end='\r', flush=True)

        # ── 6. Таймаут безопасности ────────────────────────────────────────
        if t_now - t_start > 30.0:
            motors.stop()
            print('\n[WARN] Таймаут 30 с — аварийная остановка')
            return pos

        # ── 7. Держим частоту цикла ────────────────────────────────────────
        to_sleep = DT_LOOP - (time.monotonic() - t_now)
        if to_sleep > 0:
            time.sleep(to_sleep)


# ─────────────────────────────────────────────────────────────────────────────
def main():
    dist_cm  = float(input('Расстояние N (см): ').strip())
    target_m = dist_cm / 100.0

    print(f'\nWHEEL_SCALE  = {WHEEL_SCALE}')
    print(f'Скорость     = {CRUISE_SPEED_PCT:.0f}%  '
          f'(торможение с {DECEL_ZONE_M * 100:.0f} см до цели)')
    print(f'План         : +{dist_cm:.1f} см → пауза {PAUSE_BETWEEN_S:.1f} с '
          f'→ -{dist_cm:.1f} см')

    motors = MotorDriver()

    input('\n[READY] Нажмите Enter для начала...')
    time.sleep(0.3)

    # ── Вперёд ───────────────────────────────────────────────────────────────
    pos_fwd = drive_segment(motors, +target_m, 'ВПЕРЁД')

    print(f'\n[PAUSE] {PAUSE_BETWEEN_S:.1f} с...')
    time.sleep(PAUSE_BETWEEN_S)

    # ── Назад ────────────────────────────────────────────────────────────────
    pos_back = drive_segment(motors, -target_m, 'НАЗАД')

    # ── Итог ─────────────────────────────────────────────────────────────────
    print('\n' + '═' * 52)
    print(f'  Цель              : {dist_cm:.1f} см')
    print(f'  Вперёд (одом.)    : {pos_fwd * 100:.1f} см')
    print(f'  Назад  (одом.)    : {abs(pos_back) * 100:.1f} см')
    print()
    if WHEEL_SCALE == 1.0:
        print('  ⚠  WHEEL_SCALE не откалиброван.')
        print(f'     Измерьте реальное расстояние X и задайте:')
        print(f'     WHEEL_SCALE = X / {dist_cm:.1f}')
    else:
        print(f'  WHEEL_SCALE = {WHEEL_SCALE} (откалиброван)')
    print('═' * 52)

    motors.shutdown()


if __name__ == '__main__':
    main()
