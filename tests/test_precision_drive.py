#!/usr/bin/env python3
"""
test_precision_drive.py — Точное движение вперёд / назад N см.

Только колёсная одометрия: pos += v_cmd × dt_реальный
Никакого IMU / MQTT / ROS2.

═══ ПОРЯДОК КАЛИБРОВКИ ══════════════════════════════════════════════════

 Шаг 1 — MOTOR_TRIM (прямолинейность)
   • Запустить режим 'f' (только вперёд), N = 200
   • Измерить боковое смещение X см:
       X > 0 (уехал вправо) → увеличить MOTOR_TRIM на 1–2
       X < 0 (уехал влево)  → уменьшить MOTOR_TRIM на 1–2
   • Повторять пока смещение < 2 см на 200 см

 Шаг 2 — WHEEL_SCALE_FWD
   • Запустить режим 'f', N = 50
   • Измерить реальное расстояние X_fwd см
   • WHEEL_SCALE_FWD = X_fwd / 50.0

 Шаг 3 — WHEEL_SCALE_BWD
   • Запустить режим 'b' (только назад), N = 50
   • Измерить реальное расстояние X_bwd см
   • WHEEL_SCALE_BWD = WHEEL_SCALE_FWD * (X_bwd / 50.0)
   • Из ваших данных: 20/25 = 40/50 = 0.80 → WHEEL_SCALE_BWD = WHEEL_SCALE_FWD * 0.80

 Шаг 4 — Финальная проверка
   • Запустить режим 't' (полный тест), N = 50
   • Ошибка возврата должна быть < 1 см
═══════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from pi_nodes.hardware.motor_driver import MotorDriver

# ═══ КАЛИБРОВОЧНЫЕ КОНСТАНТЫ — менять только здесь ═══════════════════════════

# Угловая поправка для прямолинейного движения (% от max_angular).
# > 0 → корректирует дрейф вправо (добавляет левый поворот)
# < 0 → корректирует дрейф влево
# Диапазон: обычно от -5.0 до +5.0
MOTOR_TRIM = 0.0

# Масштаб вперёд: реальное / одометрическое расстояние при движении вперёд.
# Формула: WHEEL_SCALE_FWD = реальное_см / N_см  (запуск режима 'f')
WHEEL_SCALE_FWD = 1.0

# Масштаб назад: аналогично для движения назад.
# Из ваших данных (25→20, 50→40): BWD-КПД = 0.80
# Формула: WHEEL_SCALE_BWD = WHEEL_SCALE_FWD * (реальное_назад_см / N_см)
WHEEL_SCALE_BWD = WHEEL_SCALE_FWD * 0.80

# ═════════════════════════════════════════════════════════════════════════════

# ─── Параметры движения (не менять без необходимости) ─────────────────────────
MAX_SPEED_M_S    = 0.30     # м/с при 100% (fast-profile)
CRUISE_SPEED_PCT = 50.0     # % — крейсерская скорость
MIN_SPEED_PCT    = 18.0     # % — минимум (ниже гусеницы могут не тронуться)

ACCEL_ZONE_M     = 0.06     # м — зона плавного разгона
DECEL_ZONE_M     = 0.10     # м — зона торможения
STOP_THRESHOLD_M = 0.005    # м — порог остановки по одометрии

BRAKE_SETTLE_S   = 0.25     # с — пауза после stop() (механическая дотяжка)
PAUSE_BETWEEN_S  = 1.5      # с — пауза между прогонами в полном тесте

WHEEL_BASE       = 0.17     # м — колёсная база (из motor_node.py)

LOOP_HZ          = 200      # Гц — частота управляющего цикла
DT_LOOP          = 1.0 / LOOP_HZ
DT_MAX           = 0.05     # с — кэп на dt (защита первой итерации)


# ─────────────────────────────────────────────────────────────────────────────
def _speed_pct(pos: float, target: float) -> float:
    """Трапециевидный профиль. Возвращает % со знаком направления."""
    progress  = abs(pos)
    dist_left = abs(target - pos)

    if progress < ACCEL_ZONE_M:
        t   = progress / ACCEL_ZONE_M
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    elif dist_left < DECEL_ZONE_M:
        t   = max(0.0, dist_left / DECEL_ZONE_M)
        spd = MIN_SPEED_PCT + t * (CRUISE_SPEED_PCT - MIN_SPEED_PCT)
    else:
        spd = CRUISE_SPEED_PCT

    return spd if target >= 0.0 else -spd


# ─────────────────────────────────────────────────────────────────────────────
def drive_segment(motors: MotorDriver, target_m: float, label: str) -> float:
    """
    Двигаться target_m метров (+ = вперёд, − = назад).

    Одометрия: pos += v_cmd × dt_реальный
      v_cmd = spd/100 × MAX_SPEED_M_S × WHEEL_SCALE_FWD/BWD

    MOTOR_TRIM применяется симметрично:
      вперёд:  motors.move(spd, +MOTOR_TRIM)
      назад:   motors.move(spd, −MOTOR_TRIM)
    Это компенсирует один и тот же физический дефект мотора в обоих направлениях.

    Возвращает накопленную позицию (м) на момент остановки.
    """
    going_fwd = target_m >= 0.0
    scale     = WHEEL_SCALE_FWD if going_fwd else WHEEL_SCALE_BWD
    trim      = MOTOR_TRIM      if going_fwd else -MOTOR_TRIM

    print(f'\n[{label}]  цель = {target_m * 100:+.1f} см  '
          f'scale = {scale:.3f}  trim = {trim:+.1f}%')

    pos     = 0.0
    t_prev  = time.monotonic()
    t_start = t_prev
    t_log   = t_prev

    while True:
        t_now = time.monotonic()
        dt    = min(t_now - t_prev, DT_MAX)
        t_prev = t_now

        # ── 1. Скорость для текущей позиции ──────────────────────────────────
        spd   = _speed_pct(pos, target_m)
        v_m_s = spd / 100.0 * MAX_SPEED_M_S * scale

        # ── 2. Условие остановки ──────────────────────────────────────────────
        dist_left = abs(target_m - pos)
        overshoot = (going_fwd  and pos >= target_m) or \
                    (not going_fwd and pos <= target_m)

        if overshoot or dist_left <= STOP_THRESHOLD_M:
            motors.stop()
            time.sleep(BRAKE_SETTLE_S)
            print(f'\n  Готово: одометрия = {pos * 100:.1f} см')
            return pos

        # ── 3. Накапливаем одометрию ──────────────────────────────────────────
        pos += v_m_s * dt

        # ── 4. Команда моторам (с trim) ───────────────────────────────────────
        motors.move(spd, trim)

        # ── 5. Лог ────────────────────────────────────────────────────────────
        if t_now - t_log >= 0.3:
            t_log = t_now
            print(f'  pos={pos * 100:6.1f} см  '
                  f'осталось={dist_left * 100:5.1f} см  '
                  f'v={abs(spd):.0f}%   ',
                  end='\r', flush=True)

        # ── 6. Таймаут безопасности ───────────────────────────────────────────
        if t_now - t_start > 30.0:
            motors.stop()
            print('\n[WARN] Таймаут 30 с — аварийная остановка')
            return pos

        # ── 7. Частота цикла ──────────────────────────────────────────────────
        to_sleep = DT_LOOP - (time.monotonic() - t_now)
        if to_sleep > 0:
            time.sleep(to_sleep)


# ─────────────────────────────────────────────────────────────────────────────
def _print_calibration_hint(dist_cm: float,
                            pos_fwd: float = None, pos_bwd: float = None):
    """Печатает формулы для пересчёта WHEEL_SCALE по результатам прогона."""
    print('\n' + '─' * 52)
    print('  КАЛИБРОВКА — подсказки:')
    if pos_fwd is not None:
        print(f'  Одометрия вперёд = {pos_fwd * 100:.1f} см  '
              f'(команда = {dist_cm:.1f} см)')
        print(f'  Введите реальное расстояние вперёд (рулетка): ', end='')
        try:
            real_fwd = float(input().strip())
            new_fwd = WHEEL_SCALE_FWD * (real_fwd / (pos_fwd * 100))
            print(f'  → WHEEL_SCALE_FWD = {new_fwd:.4f}')
            print(f'  → WHEEL_SCALE_BWD = {new_fwd:.4f} * 0.80 = '
                  f'{new_fwd * 0.80:.4f}  (если коэф. 0.80 не изменился)')
        except (ValueError, ZeroDivisionError):
            pass
    if pos_bwd is not None:
        print(f'  Одометрия назад  = {abs(pos_bwd) * 100:.1f} см  '
              f'(команда = {dist_cm:.1f} см)')
        print(f'  Введите реальное расстояние назад (рулетка): ', end='')
        try:
            real_bwd = float(input().strip())
            ratio = real_bwd / (abs(pos_bwd) * 100)
            new_bwd = WHEEL_SCALE_BWD * ratio
            print(f'  → WHEEL_SCALE_BWD = {new_bwd:.4f}')
            if pos_fwd is not None:
                print(f'  → BWD / FWD коэф. = '
                      f'{new_bwd / WHEEL_SCALE_FWD:.3f}  (ожидалось 0.800)')
        except (ValueError, ZeroDivisionError):
            pass
    print('─' * 52)


# ─────────────────────────────────────────────────────────────────────────────
def calibrate_mode():
    """
    Интерактивный расчёт всех трёх коэффициентов по замерам рулеткой.

    Формулы:
      WHEEL_SCALE_FWD = реальное_вперёд / N
      WHEEL_SCALE_BWD = WHEEL_SCALE_FWD × (реальное_назад / N)
      MOTOR_TRIM      = CRUISE_SPEED_PCT × WHEEL_BASE × X_м / D_м²
                      ≈ 0.021 × X_см  (при D=2 м, CRUISE=50%)
    """
    print('\n' + '═' * 54)
    print('  КАЛЬКУЛЯТОР КОЭФФИЦИЕНТОВ')
    print('═' * 54)

    # ── WHEEL_SCALE_FWD ──────────────────────────────────────────
    print('\n[1] WHEEL_SCALE_FWD')
    print(f'    Запустить режим f, N см, измерить рулеткой.')
    n_fwd   = float(input('    Командованное расстояние N_вперёд (см): '))
    real_fwd = float(input('    Реальное расстояние вперёд (см):        '))
    ws_fwd  = real_fwd / n_fwd
    print(f'    → WHEEL_SCALE_FWD = {real_fwd:.1f} / {n_fwd:.1f} = {ws_fwd:.4f}')

    # ── WHEEL_SCALE_BWD ──────────────────────────────────────────
    print('\n[2] WHEEL_SCALE_BWD')
    print(f'    Запустить режим b с тем же N, измерить рулеткой.')
    n_bwd    = float(input('    Командованное расстояние N_назад (см):  ') or n_fwd)
    real_bwd = float(input('    Реальное расстояние назад (см):         '))
    bwd_ratio = real_bwd / n_bwd
    ws_bwd    = ws_fwd * bwd_ratio
    print(f'    → BWD/FWD коэф. = {real_bwd:.1f} / {n_bwd:.1f} = {bwd_ratio:.4f}')
    print(f'    → WHEEL_SCALE_BWD = {ws_fwd:.4f} × {bwd_ratio:.4f} = {ws_bwd:.4f}')

    # ── MOTOR_TRIM ───────────────────────────────────────────────
    print('\n[3] MOTOR_TRIM  (прямолинейность)')
    print('    Запустить режим f, N ≥ 200 см.')
    print('    Положить рулетку от старта до финиша прямо.')
    print('    Измерить боковое отклонение X:')
    print('      + X если уехал ВПРАВО,  − X если уехал ВЛЕВО')
    d_m = float(input('    Тестовое расстояние D (см): ')) / 100.0
    x_cm = float(input('    Боковое смещение X (см, знак!): '))
    x_m  = x_cm / 100.0
    # MOTOR_TRIM = CRUISE_SPEED_PCT × WHEEL_BASE × X_м / D_м²
    trim = CRUISE_SPEED_PCT * WHEEL_BASE * x_m / (d_m ** 2)
    print(f'    Формула: {CRUISE_SPEED_PCT} × {WHEEL_BASE} × {x_m:.3f} / {d_m:.2f}²'
          f' = {trim:.3f}')
    print(f'    → MOTOR_TRIM = {trim:.3f}')

    # ── Итог ─────────────────────────────────────────────────────
    print('\n' + '═' * 54)
    print('  Вставьте в скрипт (строки в секции КАЛИБРОВОЧНЫЕ КОНСТАНТЫ):')
    print('═' * 54)
    print(f'  MOTOR_TRIM      = {trim:.3f}')
    print(f'  WHEEL_SCALE_FWD = {ws_fwd:.4f}')
    print(f'  WHEEL_SCALE_BWD = {ws_fwd:.4f} * {bwd_ratio:.4f}  # = {ws_bwd:.4f}')
    print('═' * 54)


def main():
    print('Режимы: f = только вперёд | b = только назад | t = полный тест | c = расчёт коэффициентов')
    mode     = input('Режим [f/b/t/c]: ').strip().lower() or 't'

    if mode == 'c':
        calibrate_mode()
        return
    dist_cm  = float(input('Расстояние N (см): ').strip())
    target_m = dist_cm / 100.0

    print(f'\n  MOTOR_TRIM      = {MOTOR_TRIM:+.1f}%')
    print(f'  WHEEL_SCALE_FWD = {WHEEL_SCALE_FWD:.4f}')
    print(f'  WHEEL_SCALE_BWD = {WHEEL_SCALE_BWD:.4f}  '
          f'(BWD/FWD = {WHEEL_SCALE_BWD / WHEEL_SCALE_FWD:.3f})')
    print(f'  Скорость        = {CRUISE_SPEED_PCT:.0f}%')

    motors = MotorDriver()
    input('\n[READY] Нажмите Enter для начала...')
    time.sleep(0.3)

    pos_fwd = pos_bwd = None

    if mode in ('f', 't'):
        pos_fwd = drive_segment(motors, +target_m, 'ВПЕРЁД')

    if mode == 't':
        print(f'\n[PAUSE] {PAUSE_BETWEEN_S:.1f} с...')
        time.sleep(PAUSE_BETWEEN_S)

    if mode in ('b', 't'):
        pos_bwd = drive_segment(motors, -target_m, 'НАЗАД')

    # ── Итог ─────────────────────────────────────────────────────────────────
    print('\n' + '═' * 52)
    print(f'  Цель              : {dist_cm:.1f} см')
    if pos_fwd is not None:
        print(f'  Вперёд (одом.)    : {pos_fwd * 100:.1f} см')
    if pos_bwd is not None:
        print(f'  Назад  (одом.)    : {abs(pos_bwd) * 100:.1f} см')
    if pos_fwd is not None and pos_bwd is not None:
        net_err = (pos_fwd + pos_bwd) * 100   # должно быть ≈ 0
        print(f'  Ошибка возврата   : {net_err:+.1f} см одометрии')
    print('═' * 52)

    motors.shutdown()

    # Подсказки для пересчёта коэффициентов
    _print_calibration_hint(dist_cm, pos_fwd, pos_bwd)


if __name__ == '__main__':
    main()
