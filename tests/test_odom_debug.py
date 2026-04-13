#!/usr/bin/env python3
"""
test_odom_debug.py — Диагностика и отладка одометрии.

Standalone скрипт: подключается к MQTT брокеру, подписывается на те же
топики что motor_node, и вычисляет позицию ЧИСТЫМ dead-reckoning.

Позволяет:
  1. Сравнить "чистую" одометрию с тем что публикует motor_node
  2. Увидеть КАЖДОЕ входящее cmd_vel и его влияние на позицию
  3. Проверить что без команд позиция НЕ меняется (фантом = баг)
  4. Публиковать отладочную одометрию на odom/debug для дашборда

Запуск на Pi:
    python3 tests/test_odom_debug.py
    python3 tests/test_odom_debug.py --broker 192.168.1.100

Запуск локально (подключение к Pi):
    python tests/test_odom_debug.py --broker <PI_IP>

Режимы:
    --mode monitor    : только слушать odom + cmd_vel, НЕ публиковать (по умолчанию)
    --mode publish    : публиковать odom/debug с чистым dead-reckoning
    --mode inject     : отправить тестовый cmd_vel и проверить реакцию
"""

import argparse
import json
import math
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt


# ── Калибровочные коэффициенты (из calibration_profiles.yaml) ──
SCALE_FWD = 1.235
SCALE_BWD = 0.988
DEADZONE_LINEAR = 0.01   # м/с
DEADZONE_ANGULAR = 0.05  # rad/s
CMD_VEL_TIMEOUT = 0.5    # с
MANUAL_OVERRIDE_TIMEOUT = 0.5  # с

# Speed profiles (для inject-режима)
SPEED_PROFILES = {
    'slow':   (0.10, 0.8),
    'normal': (0.20, 1.5),
    'fast':   (0.30, 2.0),
}


class OdomDebugger:
    """Чистый dead-reckoning одометр для сравнения с motor_node."""

    def __init__(self, broker: str, port: int, robot_id: str, mode: str):
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._prefix = f'samurai/{robot_id}'
        self._mode = mode
        self._running = False

        # ── Dead-reckoning state (наша чистая одометрия) ──
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0          # heading из IMU
        self._imu_yaw = None       # None = IMU ещё не пришёл

        # Команды
        self._auto_linear = 0.0
        self._auto_angular = 0.0
        self._last_auto_time = 0.0

        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._last_manual_time = 0.0

        # ── То что публикует motor_node (для сравнения) ──
        self._mn_x = 0.0
        self._mn_y = 0.0
        self._mn_theta = 0.0
        self._mn_vx = 0.0
        self._mn_stationary = True
        self._mn_last_ts = 0.0

        # ── Статистика ──
        self._cmd_count = 0
        self._odom_count = 0
        self._imu_count = 0
        self._last_print_time = 0.0
        self._max_drift = 0.0       # макс. расхождение наш vs motor_node

        # ── Timing ──
        self._last_tick = time.monotonic()
        self._tick_rate = 20  # Hz

        # ── MQTT ──
        self._client = mqtt.Client(
            client_id=f'odom_debug_{robot_id}',
            protocol=mqtt.MQTTv311)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f'[ERROR] MQTT connect failed rc={rc}')
            return
        print(f'[OK] MQTT connected to {self._broker}:{self._port}')
        p = self._prefix
        # Подписываемся на все нужные топики
        client.subscribe(f'{p}/cmd_vel', qos=1)
        client.subscribe(f'{p}/cmd_vel/manual', qos=1)
        client.subscribe(f'{p}/imu', qos=0)
        client.subscribe(f'{p}/odom', qos=0)
        # Установка callback'ов
        client.message_callback_add(f'{p}/cmd_vel', self._on_cmd_vel)
        client.message_callback_add(f'{p}/cmd_vel/manual', self._on_cmd_vel_manual)
        client.message_callback_add(f'{p}/imu', self._on_imu)
        client.message_callback_add(f'{p}/odom', self._on_odom)
        print('[OK] Subscribed to: cmd_vel, cmd_vel/manual, imu, odom')
        print()

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(f'[WARN] MQTT disconnected (rc={rc})')

    # ── MQTT callbacks ──────────────────────────────────────────

    def _on_cmd_vel(self, client, userdata, msg):
        """Автономная команда скорости (от FSM, path_recorder)."""
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        lin = float(d.get('linear_x', 0.0))
        ang = float(d.get('angular_z', 0.0))
        self._auto_linear = lin
        self._auto_angular = ang
        self._last_auto_time = time.monotonic()
        self._cmd_count += 1
        print(f'  [CMD auto]  lin={lin:+.3f}  ang={ang:+.3f}')

    def _on_cmd_vel_manual(self, client, userdata, msg):
        """Ручная команда (джойстик, дашборд)."""
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        lin = float(d.get('linear_x', 0.0))
        ang = float(d.get('angular_z', 0.0))
        self._manual_linear = lin
        self._manual_angular = ang
        self._last_manual_time = time.monotonic()
        self._cmd_count += 1
        print(f'  [CMD manual] lin={lin:+.3f}  ang={ang:+.3f}')

    def _on_imu(self, client, userdata, msg):
        """IMU данные — берём только yaw для heading."""
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        ekf = d.get('ekf')
        if ekf:
            yaw = ekf.get('yaw_rad')
            if yaw is not None:
                self._imu_yaw = yaw
                self._imu_count += 1

    def _on_odom(self, client, userdata, msg):
        """Одометрия от motor_node (для сравнения)."""
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        self._mn_x = d.get('x', 0.0)         # уже в см
        self._mn_y = d.get('y', 0.0)         # уже в см
        self._mn_theta = d.get('theta', 0.0)
        self._mn_vx = d.get('vx', 0.0)
        self._mn_stationary = d.get('stationary', True)
        self._mn_last_ts = d.get('ts', 0.0)
        self._odom_count += 1

    # ── Dead-reckoning (чистый, без IMU-position) ───────────────

    def _tick(self):
        """Один тик dead-reckoning (20 Hz)."""
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        if dt <= 0 or dt > 1.0:
            return

        # ── Выбор команды: manual > autonomous ──
        manual_active = (self._last_manual_time > 0 and
                         (now - self._last_manual_time) <= MANUAL_OVERRIDE_TIMEOUT)

        # Таймаут автономной команды
        if self._last_auto_time > 0 and (now - self._last_auto_time) > CMD_VEL_TIMEOUT:
            self._auto_linear = 0.0
            self._auto_angular = 0.0

        if manual_active:
            lin_cmd = self._manual_linear
            ang_cmd = self._manual_angular
        else:
            lin_cmd = self._auto_linear
            ang_cmd = self._auto_angular

        # Dead zone
        if abs(lin_cmd) < DEADZONE_LINEAR:
            lin_cmd = 0.0
        if abs(ang_cmd) < DEADZONE_ANGULAR:
            ang_cmd = 0.0

        # Clamp (normal profile)
        max_lin, max_ang = SPEED_PROFILES['normal']
        lin = max(-max_lin, min(max_lin, lin_cmd))
        ang = max(-max_ang, min(max_ang, ang_cmd))

        # ── Heading: IMU yaw если есть, иначе интеграция ──
        if self._imu_yaw is not None:
            self._theta = self._imu_yaw
        else:
            self._theta += ang * dt

        cy = math.cos(self._theta)
        sy = math.sin(self._theta)

        # ── Dead-reckoning: v = cmd × scale ──
        if abs(lin) > DEADZONE_LINEAR:
            scale = SCALE_FWD if lin >= 0 else SCALE_BWD
            v_actual = lin * scale
        else:
            v_actual = 0.0

        self._x += v_actual * dt * cy
        self._y += v_actual * dt * sy

        # ── Публикация отладочной одометрии ──
        if self._mode == 'publish':
            odom_debug = {
                'x':  round(self._x * 100.0, 2),   # см
                'y':  round(self._y * 100.0, 2),   # см
                'theta': round(self._theta, 4),
                'vx': round(v_actual, 3),
                'vz': round(ang, 3),
                'speed': round(abs(v_actual), 3),
                'stationary': abs(v_actual) < 0.001 and abs(ang) < DEADZONE_ANGULAR,
                'source': 'debug_deadreckoning',
                'ts': time.time(),
            }
            self._client.publish(
                f'{self._prefix}/odom/debug',
                json.dumps(odom_debug, separators=(',', ':')),
                qos=0)

        # ── Расхождение с motor_node ──
        our_x_cm = self._x * 100.0
        our_y_cm = self._y * 100.0
        drift = math.sqrt(
            (our_x_cm - self._mn_x) ** 2 +
            (our_y_cm - self._mn_y) ** 2)
        if drift > self._max_drift:
            self._max_drift = drift

    def _print_status(self):
        """Печать статуса каждую секунду."""
        now = time.monotonic()
        if now - self._last_print_time < 1.0:
            return
        self._last_print_time = now

        our_x = self._x * 100.0
        our_y = self._y * 100.0
        mn_x = self._mn_x
        mn_y = self._mn_y

        drift = math.sqrt((our_x - mn_x) ** 2 + (our_y - mn_y) ** 2)

        # Определяем есть ли фантом
        # Если motor_node показывает движение а мы — нет, это фантом
        phantom = ''
        if abs(mn_x) > 0.5 or abs(mn_y) > 0.5:
            if abs(our_x) < 0.5 and abs(our_y) < 0.5:
                phantom = '  *** PHANTOM DETECTED ***'
            elif drift > 2.0:
                phantom = f'  ** DRIFT {drift:.1f} cm **'

        imu_deg = math.degrees(self._theta) if self._imu_yaw is not None else -999

        print(f'  [t={now - self._start_time:6.1f}s]'
              f'  OUR: x={our_x:+7.2f} y={our_y:+7.2f} cm'
              f'  |  MN: x={mn_x:+7.2f} y={mn_y:+7.2f} cm'
              f'  |  yaw={imu_deg:+6.1f}°'
              f'  |  mn_vx={self._mn_vx:+.3f} stat={self._mn_stationary}'
              f'  |  msgs: cmd={self._cmd_count} imu={self._imu_count} odom={self._odom_count}'
              f'{phantom}')

    # ── Inject-режим: отправить тестовую команду ────────────────

    def _inject_test(self):
        """Отправить тестовую серию cmd_vel и наблюдать реакцию."""
        print()
        print('='*70)
        print('  INJECT MODE: отправляем тестовые команды')
        print('='*70)
        print()

        # Фаза 1: ждём 3 секунды без команд — позиция должна быть 0
        print('[Phase 1] Тишина 3с — позиция должна быть 0...')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            self._tick()
            self._print_status()
            time.sleep(0.05)

        our_x0 = self._x * 100.0
        our_y0 = self._y * 100.0
        mn_x0 = self._mn_x
        mn_y0 = self._mn_y
        print(f'\n  >> Дрифт за тишину: наш=({our_x0:+.2f},{our_y0:+.2f})'
              f'  motor_node=({mn_x0:+.2f},{mn_y0:+.2f}) см')
        if abs(mn_x0) > 1.0 or abs(mn_y0) > 1.0:
            print('  >> !!! MOTOR_NODE ДВИГАЕТСЯ БЕЗ КОМАНД — это фантом !!!')
        print()

        # Фаза 2: отправляем 0.2 м/с вперёд на 2 секунды
        print('[Phase 2] Команда вперёд lin=0.20 на 2с...')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            self._client.publish(
                f'{self._prefix}/cmd_vel/manual',
                json.dumps({'linear_x': 0.20, 'angular_z': 0.0}),
                qos=1)
            self._tick()
            self._print_status()
            time.sleep(0.05)

        our_x1 = self._x * 100.0
        our_y1 = self._y * 100.0
        mn_x1 = self._mn_x
        mn_y1 = self._mn_y
        delta_our = math.sqrt((our_x1 - our_x0)**2 + (our_y1 - our_y0)**2)
        delta_mn = math.sqrt((mn_x1 - mn_x0)**2 + (mn_y1 - mn_y0)**2)
        print(f'\n  >> Пройдено: наш={delta_our:.2f} см  motor_node={delta_mn:.2f} см')
        expected = 0.20 * SCALE_FWD * 2.0 * 100.0  # ~49.4 см
        print(f'  >> Ожидалось: ~{expected:.1f} см')
        print()

        # Фаза 3: стоп + ждём 5 секунд — позиция должна замереть
        print('[Phase 3] Стоп, ждём 5с — позиция должна замереть...')
        # Отправляем явный стоп
        self._client.publish(
            f'{self._prefix}/cmd_vel/manual',
            json.dumps({'linear_x': 0.0, 'angular_z': 0.0}),
            qos=1)

        t0 = time.monotonic()
        frozen_x = self._mn_x
        frozen_y = self._mn_y
        max_jitter = 0.0
        while time.monotonic() - t0 < 5.0:
            self._tick()
            self._print_status()
            jitter = math.sqrt(
                (self._mn_x - frozen_x)**2 +
                (self._mn_y - frozen_y)**2)
            if jitter > max_jitter:
                max_jitter = jitter
            time.sleep(0.05)

        print(f'\n  >> Макс. джиттер motor_node за 5с стопа: {max_jitter:.3f} см')
        if max_jitter > 1.0:
            print(f'  >> !!! ФАНТОМНОЕ ДВИЖЕНИЕ: {max_jitter:.1f} см джиттер !!!')
        else:
            print('  >> OK: позиция стабильна')

        print()
        print('='*70)
        print('  INJECT завершён')
        print('='*70)

    # ── Main loop ───────────────────────────────────────────────

    def run(self):
        self._running = True
        self._start_time = time.monotonic()
        self._last_tick = time.monotonic()
        self._last_print_time = 0.0

        # Graceful shutdown
        stop = threading.Event()
        def _sig(s, f):
            stop.set()
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        # Connect
        print(f'Connecting to MQTT {self._broker}:{self._port}...')
        self._client.connect(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        time.sleep(1.0)  # ждём подключения

        print()
        print('='*70)
        print(f'  ODOM DEBUGGER  |  mode={self._mode}  |  robot={self._robot_id}')
        print(f'  scale_fwd={SCALE_FWD}  scale_bwd={SCALE_BWD}')
        print(f'  deadzone_lin={DEADZONE_LINEAR}  deadzone_ang={DEADZONE_ANGULAR}')
        print('='*70)
        print()
        print('  Формат: OUR = наш чистый dead-reckoning')
        print('          MN  = что публикует motor_node')
        print('          PHANTOM = MN двигается, а мы нет')
        print()

        if self._mode == 'inject':
            self._inject_test()
        else:
            # Monitor / publish mode — бесконечный цикл
            print(f'  [mode={self._mode}] Ctrl+C для выхода')
            print()
            while not stop.is_set():
                self._tick()
                self._print_status()
                stop.wait(timeout=1.0 / self._tick_rate)

        # Итоги
        print()
        print(f'  Итого: max_drift={self._max_drift:.2f} см'
              f'  cmd={self._cmd_count}  imu={self._imu_count}'
              f'  odom={self._odom_count}')

        self._client.loop_stop()
        self._client.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description='Диагностика одометрии — сравнение чистого dead-reckoning с motor_node')
    parser.add_argument('--broker', default='127.0.0.1',
                        help='MQTT broker IP (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT broker port (default: 1883)')
    parser.add_argument('--robot-id', default='robot1',
                        help='Robot ID (default: robot1)')
    parser.add_argument('--mode', choices=['monitor', 'publish', 'inject'],
                        default='monitor',
                        help='monitor=слушать, publish=публиковать odom/debug, '
                             'inject=отправить тест cmd_vel')
    args = parser.parse_args()

    debugger = OdomDebugger(
        broker=args.broker,
        port=args.port,
        robot_id=args.robot_id,
        mode=args.mode,
    )
    debugger.run()


if __name__ == '__main__':
    main()
