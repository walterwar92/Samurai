#!/usr/bin/env python3
"""
calibration_node — Автоматическая калибровка одометрии.

Позволяет вычислить масштабные коэффициенты (scale factors) для линейной
и угловой одометрии, чтобы затем подставить их в motor_node.py
(WHEEL_SCALE_LINEAR, WHEEL_SCALE_ANGULAR).

Subscribes:
    samurai/{robot_id}/calibration/command — "start_linear" | "start_angular"
    samurai/{robot_id}/odom               — текущая одометрия {x, y, theta, ...}
    samurai/{robot_id}/range              — ультразвуковой датчик расстояния (см)

Publishes:
    samurai/{robot_id}/cmd_vel              — команды движения
    samurai/{robot_id}/calibration/status   — прогресс калибровки (текст)
    samurai/{robot_id}/calibration/result   — результат: {type, expected, measured,
                                               scale_factor, ...}

Процедура ЛИНЕЙНОЙ калибровки:
    1. Запоминаем начальные показания odom (x, y) и range (если стена впереди).
    2. Едем прямо со скоростью 0.10 м/с ровно 5 секунд (ожидаемо ~0.5 м).
    3. Останавливаемся, читаем конечные odom и range.
    4. Сравниваем: odom_delta vs expected (0.5 м), range_delta vs expected.
    5. Публикуем масштабный коэффициент.

Процедура УГЛОВОЙ калибровки:
    1. Запоминаем начальный theta из odom.
    2. Вращаемся на месте со скоростью 0.5 рад/с ровно 2π/0.5 ≈ 12.566 с
       (один полный оборот = 2π рад).
    3. Останавливаемся, читаем конечный theta.
    4. Сравниваем theta_delta vs ожидаемых 2π рад.
    5. Публикуем масштабный коэффициент.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# ── Параметры линейной калибровки ──────────────────────────────
LINEAR_SPEED = 0.10          # м/с — скорость прямолинейного движения
LINEAR_DURATION = 5.0        # секунд
LINEAR_EXPECTED = LINEAR_SPEED * LINEAR_DURATION  # 0.5 м

# ── Параметры угловой калибровки ───────────────────────────────
ANGULAR_SPEED = 0.5          # рад/с — скорость вращения
ANGULAR_EXPECTED = 2.0 * math.pi  # один полный оборот (рад)
ANGULAR_DURATION = ANGULAR_EXPECTED / ANGULAR_SPEED  # ~12.566 с

# ── Задержка стабилизации после остановки ──────────────────────
SETTLE_DELAY = 1.0           # секунд — ждём, пока робот полностью остановится


class CalibrationNode(MqttNode):
    """Нода автоматической калибровки одометрии."""

    def __init__(self, **kwargs):
        super().__init__('calibration_node', **kwargs)

        # --- Текущие показания датчиков ---
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_received = False

        self._range_cm = None          # последнее значение с ультразвукового датчика
        self._range_received = False

        # --- Состояние калибровки ---
        self._calibrating = False      # идёт ли калибровка прямо сейчас
        self._cal_type = None          # "linear" | "angular"

        # --- Подписки ---
        self.subscribe('calibration/command', self._command_cb, qos=1)
        self.subscribe('odom', self._odom_cb, qos=0)
        self.subscribe('range', self._range_cb, qos=0)

        self.log_info('CalibrationNode инициализирован. Ожидаю команду на '
                       'calibration/command ("start_linear" / "start_angular")')

    # ================================================================
    #  Коллбэки MQTT
    # ================================================================

    def _command_cb(self, topic, data):
        """Обработка команды калибровки."""
        cmd = str(data).strip().lower()

        if self._calibrating:
            self.log_warn('Калибровка уже запущена (%s), команда "%s" проигнорирована',
                          self._cal_type, cmd)
            self._publish_status('busy', f'Калибровка {self._cal_type} уже выполняется')
            return

        if cmd == 'start_linear':
            self._start_linear_calibration()
        elif cmd == 'start_angular':
            self._start_angular_calibration()
        else:
            self.log_warn('Неизвестная команда калибровки: "%s"', cmd)
            self._publish_status('error', f'Неизвестная команда: {cmd}')

    def _odom_cb(self, topic, data):
        """Получаем актуальные данные одометрии."""
        if not isinstance(data, dict):
            return
        self._odom_x = float(data.get('x', 0.0))
        self._odom_y = float(data.get('y', 0.0))
        self._odom_theta = float(data.get('theta', 0.0))
        self._odom_received = True

    def _range_cb(self, topic, data):
        """Получаем расстояние от ультразвукового датчика (в сантиметрах)."""
        try:
            if isinstance(data, dict):
                self._range_cm = float(data.get('distance', data.get('range', 0.0)))
            else:
                self._range_cm = float(data)
            self._range_received = True
        except (ValueError, TypeError):
            pass

    # ================================================================
    #  Линейная калибровка
    # ================================================================

    def _start_linear_calibration(self):
        """Запускает процедуру линейной калибровки в отдельном потоке."""
        import threading
        self._calibrating = True
        self._cal_type = 'linear'
        t = threading.Thread(target=self._run_linear_calibration,
                             name='cal_linear', daemon=True)
        t.start()

    def _run_linear_calibration(self):
        """Процедура линейной калибровки (выполняется в фоновом потоке)."""
        try:
            self.log_info('=== ЛИНЕЙНАЯ КАЛИБРОВКА: старт ===')
            self._publish_status('running', 'Линейная калибровка: подготовка...')

            # 1. Ждём свежие данные odom
            if not self._wait_for_odom(timeout=3.0):
                self._publish_status('error', 'Нет данных одометрии (odom)')
                return

            # 2. Запоминаем начальные показания
            start_x = self._odom_x
            start_y = self._odom_y
            start_range = self._range_cm  # может быть None, если нет стены
            has_range = self._range_received and start_range is not None

            self.log_info('Начальная позиция: x=%.4f, y=%.4f', start_x, start_y)
            if has_range:
                self.log_info('Начальное расстояние (ультразвук): %.1f см', start_range)
            else:
                self.log_info('Ультразвуковой датчик: нет данных (калибровка только по odom)')

            self._publish_status('running',
                                 f'Еду вперёд {LINEAR_SPEED} м/с на {LINEAR_DURATION} с...')

            # 3. Едем прямо
            start_time = time.monotonic()
            while (time.monotonic() - start_time) < LINEAR_DURATION:
                self.publish('cmd_vel', {
                    'linear_x': LINEAR_SPEED,
                    'angular_z': 0.0,
                })
                elapsed = time.monotonic() - start_time
                progress = min(elapsed / LINEAR_DURATION * 100.0, 100.0)
                if int(elapsed * 4) % 4 == 0:  # каждые ~0.25 с не спамим
                    self._publish_status('running',
                                         f'Линейная: {progress:.0f}% ({elapsed:.1f}/{LINEAR_DURATION:.1f} с)')
                time.sleep(0.05)  # 20 Hz — совпадает с частотой motor_node

            # 4. Останавливаемся
            self._stop_robot()
            self._publish_status('running', 'Остановка, ожидание стабилизации...')
            time.sleep(SETTLE_DELAY)

            # 5. Читаем конечные показания
            end_x = self._odom_x
            end_y = self._odom_y
            end_range = self._range_cm

            # 6. Вычисляем результат
            odom_delta = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)

            result = {
                'type': 'linear',
                'expected_m': round(LINEAR_EXPECTED, 4),
                'odom_delta_m': round(odom_delta, 4),
                'odom_scale_factor': round(LINEAR_EXPECTED / odom_delta, 4) if odom_delta > 0.001 else None,
                'speed': LINEAR_SPEED,
                'duration': LINEAR_DURATION,
                'ts': self.timestamp(),
            }

            # Если есть ультразвук — добавляем данные
            if has_range and end_range is not None:
                range_delta_cm = start_range - end_range  # при движении к стене range уменьшается
                range_delta_m = range_delta_cm / 100.0
                result['range_start_cm'] = round(start_range, 1)
                result['range_end_cm'] = round(end_range, 1)
                result['range_delta_m'] = round(range_delta_m, 4)
                result['range_vs_odom'] = round(range_delta_m / odom_delta, 4) if odom_delta > 0.001 else None
                self.log_info('Ультразвук: начало=%.1f см, конец=%.1f см, дельта=%.4f м',
                              start_range, end_range, range_delta_m)

            self.log_info('=== ЛИНЕЙНАЯ КАЛИБРОВКА: завершена ===')
            self.log_info('Ожидаемо: %.4f м, odom: %.4f м, scale=%.4f',
                          LINEAR_EXPECTED, odom_delta,
                          result['odom_scale_factor'] or 0.0)

            self.publish('calibration/result', result, qos=1)
            self._publish_status('done',
                                 f'Линейная калибровка завершена. '
                                 f'odom={odom_delta:.4f} м, scale={result["odom_scale_factor"]}')

        except Exception as exc:
            self.log_error('Ошибка линейной калибровки: %s', exc)
            self._publish_status('error', f'Ошибка: {exc}')
        finally:
            self._stop_robot()
            self._calibrating = False
            self._cal_type = None

    # ================================================================
    #  Угловая калибровка
    # ================================================================

    def _start_angular_calibration(self):
        """Запускает процедуру угловой калибровки в отдельном потоке."""
        import threading
        self._calibrating = True
        self._cal_type = 'angular'
        t = threading.Thread(target=self._run_angular_calibration,
                             name='cal_angular', daemon=True)
        t.start()

    def _run_angular_calibration(self):
        """Процедура угловой калибровки (выполняется в фоновом потоке)."""
        try:
            self.log_info('=== УГЛОВАЯ КАЛИБРОВКА: старт ===')
            self._publish_status('running', 'Угловая калибровка: подготовка...')

            # 1. Ждём свежие данные odom
            if not self._wait_for_odom(timeout=3.0):
                self._publish_status('error', 'Нет данных одометрии (odom)')
                return

            # 2. Запоминаем начальный угол
            start_theta = self._odom_theta
            self.log_info('Начальный угол: theta=%.4f рад (%.1f°)',
                          start_theta, math.degrees(start_theta))

            self._publish_status('running',
                                 f'Вращаюсь {ANGULAR_SPEED} рад/с на {ANGULAR_DURATION:.1f} с...')

            # 3. Вращаемся на месте (один полный оборот)
            start_time = time.monotonic()
            total_theta_delta = 0.0
            prev_theta = start_theta

            while (time.monotonic() - start_time) < ANGULAR_DURATION:
                self.publish('cmd_vel', {
                    'linear_x': 0.0,
                    'angular_z': ANGULAR_SPEED,
                })

                # Накапливаем изменение угла (с учётом перехода через ±π)
                cur_theta = self._odom_theta
                dtheta = self._normalize_angle(cur_theta - prev_theta)
                total_theta_delta += dtheta
                prev_theta = cur_theta

                elapsed = time.monotonic() - start_time
                progress = min(elapsed / ANGULAR_DURATION * 100.0, 100.0)
                if int(elapsed * 4) % 4 == 0:
                    self._publish_status('running',
                                         f'Угловая: {progress:.0f}% ({elapsed:.1f}/{ANGULAR_DURATION:.1f} с)')
                time.sleep(0.05)

            # 4. Останавливаемся
            self._stop_robot()
            self._publish_status('running', 'Остановка, ожидание стабилизации...')
            time.sleep(SETTLE_DELAY)

            # Финальный учёт последнего приращения
            cur_theta = self._odom_theta
            dtheta = self._normalize_angle(cur_theta - prev_theta)
            total_theta_delta += dtheta

            # 5. Вычисляем результат
            # total_theta_delta — суммарное изменение угла по одометрии
            odom_theta_abs = abs(total_theta_delta)

            scale_factor = (ANGULAR_EXPECTED / odom_theta_abs
                            if odom_theta_abs > 0.01 else None)

            result = {
                'type': 'angular',
                'expected_rad': round(ANGULAR_EXPECTED, 4),
                'odom_theta_delta_rad': round(total_theta_delta, 4),
                'odom_theta_abs_rad': round(odom_theta_abs, 4),
                'odom_scale_factor': round(scale_factor, 4) if scale_factor else None,
                'speed_rad_s': ANGULAR_SPEED,
                'duration': round(ANGULAR_DURATION, 3),
                'ts': self.timestamp(),
            }

            self.log_info('=== УГЛОВАЯ КАЛИБРОВКА: завершена ===')
            self.log_info('Ожидаемо: %.4f рад (360°), odom: %.4f рад (%.1f°), scale=%.4f',
                          ANGULAR_EXPECTED, odom_theta_abs,
                          math.degrees(odom_theta_abs),
                          scale_factor or 0.0)

            self.publish('calibration/result', result, qos=1)
            self._publish_status('done',
                                 f'Угловая калибровка завершена. '
                                 f'odom={math.degrees(odom_theta_abs):.1f}°, '
                                 f'scale={result["odom_scale_factor"]}')

        except Exception as exc:
            self.log_error('Ошибка угловой калибровки: %s', exc)
            self._publish_status('error', f'Ошибка: {exc}')
        finally:
            self._stop_robot()
            self._calibrating = False
            self._cal_type = None

    # ================================================================
    #  Вспомогательные методы
    # ================================================================

    def _stop_robot(self):
        """Отправляем нулевую скорость (остановка)."""
        for _ in range(5):  # несколько раз для надёжности
            self.publish('cmd_vel', {
                'linear_x': 0.0,
                'angular_z': 0.0,
            })
            time.sleep(0.02)

    def _wait_for_odom(self, timeout: float = 3.0) -> bool:
        """Ждём хотя бы одно сообщение odom. Возвращает True если получено."""
        if self._odom_received:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._odom_received:
                return True
            time.sleep(0.05)
        self.log_error('Таймаут: данные odom не получены за %.1f с', timeout)
        return False

    def _publish_status(self, state: str, message: str):
        """Публикуем статус калибровки."""
        self.publish('calibration/status', {
            'state': state,        # "running", "done", "error", "busy"
            'message': message,
            'cal_type': self._cal_type,
            'ts': self.timestamp(),
        })
        self.log_info('[status] %s: %s', state, message)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализуем угол в диапазон [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def on_shutdown(self):
        """Гарантированно останавливаем робота при завершении."""
        self._stop_robot()


# ════════════════════════════════════════════════════════════════
#  Точка входа
# ════════════════════════════════════════════════════════════════

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Автоматическая калибровка одометрии Samurai-робота')
    parser.add_argument('--broker', default='127.0.0.1',
                        help='Адрес MQTT-брокера (по умолчанию 127.0.0.1)')
    parser.add_argument('--port', type=int, default=1883,
                        help='Порт MQTT-брокера (по умолчанию 1883)')
    parser.add_argument('--robot-id', default='robot1',
                        help='Идентификатор робота (по умолчанию robot1)')
    args = parser.parse_args()

    node = CalibrationNode(broker=args.broker, port=args.port,
                           robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
