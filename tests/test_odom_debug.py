#!/usr/bin/env python3
"""
test_odom_debug.py — Гибридная одометрия: cmd_vel + IMU пассивная детекция.

Два режима определения позиции:
  1. Моторы активны (cmd_vel ≠ 0) → dead-reckoning по командам × scale
     (нет вибрации в расчёте → нет фантома)
  2. Моторы выключены (нет cmd_vel) → IMU акселерометр детектирует
     физическое перемещение (нет вибрации моторов → чистый сигнал)

Запуск:
    python3 tests/test_odom_debug.py --mode monitor   # наблюдение
    python3 tests/test_odom_debug.py --mode publish    # публикация odom/debug
    python3 tests/test_odom_debug.py --mode inject     # авто-тест

Тест физического движения:
    1. Запустить --mode publish
    2. Подвигать робота руками — позиция должна меняться
    3. Отпустить — позиция должна замереть (без дрифта)
"""

import argparse
import json
import math
import signal
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

# ── IMU passive mode (моторы выключены) ─────────────────────
# Bias tracker: EMA alpha — как быстро учим «фон» акселерометра.
IMU_BIAS_ALPHA_SLOW = 0.02    # нормальный режим (~2.5с постоянная)
IMU_BIAS_ALPHA_FAST = 0.15    # быстрая адаптация после толчка (~0.3с)
IMU_FAST_ADAPT_TICKS = 20     # ~1с быстрой адаптации после остановки

# Порог ОТКЛОНЕНИЯ от bias (м/с²).
# Реальный толчок = 0.5-5+ м/с², изменение gravity residual = 0.1-0.3.
# Ставим 0.35 — отсекаем gravity drift, ловим только настоящие толчки.
IMU_DEVIATION_THRESHOLD = 0.35

# Макс. скорость при ручном толчке (м/с, по МАГНИТУДЕ, не по оси).
# 0.10 м/с = 10 см/с. За 1.5с макс. движения → 15 см макс. смещения.
IMU_VELOCITY_CAP = 0.10

# Макс. длительность непрерывного IMU+ режима (тиков @ 20 Hz).
# Рукой толкают <1.5с. Если deviation > threshold дольше — это не движение,
# а изменение gravity residual (тильт робота сменился). → GRAV recovery.
IMU_MAX_MOVE_TICKS = 30  # 1.5с

# Время «уверенности» (сэмплы): нужно N подряд отклонений для IMU+ режима
IMU_CONFIRM_SAMPLES = 3

# Speed profiles
SPEED_PROFILES = {
    'slow':   (0.10, 0.8),
    'normal': (0.20, 1.5),
    'fast':   (0.30, 2.0),
}


class HybridOdometry:
    """Гибридная одометрия: cmd_vel DR + IMU пассивная детекция.

    Принцип:
      - Моторы ON:  позиция = integral(cmd_vel × scale)
                    → IMU не используется для позиции (вибрация!)
      - Моторы OFF: позиция = integral(integral(accel - bias))
                    → динамический bias трекинг убирает постоянное
                      смещение акселерометра (~0.3-0.4 м/с² остаток
                      от неполного удаления гравитации)

    Heading (theta) всегда из IMU EKF yaw.

    Ключевая идея bias трекинга:
      1. EMA (slow) учит «фон» акселерометра когда стоим
      2. Реальное движение = ОТКЛОНЕНИЕ от этого фона
      3. Во время движения bias замораживается (не учит шум движения)
      4. После остановки — снова адаптируется
    """

    def __init__(self):
        # Позиция (метры)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # IMU passive скорость (м/с, world frame)
        self._imu_vx = 0.0
        self._imu_vy = 0.0

        # ── Динамический bias (EMA фона акселерометра) ──
        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_ready = False  # True после первых ~50 сэмплов
        self._bias_samples = 0
        self._BIAS_WARMUP = 40   # ~2с @ 20 Hz — учим начальный фон

        # ── Подтверждение движения ──
        self._deviation_count = 0  # сколько подряд сэмплов > threshold
        self._fast_adapt_remaining = 0  # тиков быстрой bias-адаптации
        self._continuous_move_ticks = 0  # счётчик непрерывного IMU+ режима

        # Текущий режим
        self.motors_active = False
        self._mode_label = 'IDLE'

        # Диагностика
        self.imu_moves_detected = 0
        self.last_deviation = 0.0  # для вывода

    def update(self, lin_cmd: float, ang_cmd: float,
               imu_yaw: float, accel_x: float, accel_y: float,
               dt: float):
        """Один тик обновления.

        Args:
            lin_cmd: скорость из cmd_vel после deadzone+clamp (м/с)
            ang_cmd: угловая скорость из cmd_vel (rad/s)
            imu_yaw: heading из IMU EKF (rad)
            accel_x: gravity-free ускорение X world (м/с²) из odom.accel_x
            accel_y: gravity-free ускорение Y world (м/с²) из odom.accel_y
            dt: шаг времени (с)
        """
        if dt <= 0 or dt > 1.0:
            return

        self.theta = imu_yaw
        self.motors_active = abs(lin_cmd) > DEADZONE_LINEAR

        if self.motors_active:
            # ── РЕЖИМ 1: Dead-reckoning по командам моторов ──
            scale = SCALE_FWD if lin_cmd >= 0 else SCALE_BWD
            v = lin_cmd * scale
            cy = math.cos(self.theta)
            sy = math.sin(self.theta)

            self.x += v * dt * cy
            self.y += v * dt * sy

            # Сброс IMU скорости
            self._imu_vx = 0.0
            self._imu_vy = 0.0
            self._deviation_count = 0
            self._mode_label = 'CMD'
            # Не обновляем bias когда моторы работают (вибрация!)
            return

        # ── РЕЖИМ 2: IMU пассивная детекция (моторы OFF) ──

        # Шаг 1: Вычислить отклонение от bias
        dev_x = accel_x - self._bias_x
        dev_y = accel_y - self._bias_y
        deviation = math.sqrt(dev_x * dev_x + dev_y * dev_y)
        self.last_deviation = deviation

        is_moving = False

        if deviation > IMU_DEVIATION_THRESHOLD:
            self._deviation_count += 1
            if self._deviation_count >= IMU_CONFIRM_SAMPLES:
                is_moving = True
        else:
            self._deviation_count = 0

        if is_moving:
            self._continuous_move_ticks += 1

            if self._continuous_move_ticks > IMU_MAX_MOVE_TICKS:
                # ── GRAV RECOVERY: deviation держится > 1.5с ──
                # Это НЕ движение — это изменение gravity residual
                # после тильта робота. Принудительно учим новый bias.
                self._imu_vx = 0.0
                self._imu_vy = 0.0
                alpha = IMU_BIAS_ALPHA_FAST
                self._bias_x += alpha * (accel_x - self._bias_x)
                self._bias_y += alpha * (accel_y - self._bias_y)
                self._mode_label = 'GRAV'
            else:
                # ── Настоящее движение: интегрируем ОТКЛОНЕНИЕ ──
                self._imu_vx += dev_x * dt
                self._imu_vy += dev_y * dt

                # Velocity capping по МАГНИТУДЕ (не per-axis)
                vel_mag = math.sqrt(self._imu_vx**2 + self._imu_vy**2)
                if vel_mag > IMU_VELOCITY_CAP:
                    s = IMU_VELOCITY_CAP / vel_mag
                    self._imu_vx *= s
                    self._imu_vy *= s

                self.imu_moves_detected += 1
                self._mode_label = 'IMU+'
                # Запланировать быструю адаптацию на момент остановки
                self._fast_adapt_remaining = IMU_FAST_ADAPT_TICKS
        else:
            # ── Нет движения: HARD ZUPT + обновление bias ──
            self._continuous_move_ticks = 0
            self._imu_vx = 0.0
            self._imu_vy = 0.0

            # Обновляем bias:
            #   - warmup: быстрая инициализация (~2с при старте)
            #   - fast adapt: быстрая EMA после толчка (0.15, ~1с)
            #   - slow: медленная EMA для тонкой подстройки (0.02)
            if self._bias_samples < self._BIAS_WARMUP:
                alpha = 0.1
                self._bias_samples += 1
                if self._bias_samples >= self._BIAS_WARMUP:
                    self._bias_ready = True
            elif self._fast_adapt_remaining > 0:
                alpha = IMU_BIAS_ALPHA_FAST
                self._fast_adapt_remaining -= 1
            else:
                alpha = IMU_BIAS_ALPHA_SLOW

            self._bias_x += alpha * (accel_x - self._bias_x)
            self._bias_y += alpha * (accel_y - self._bias_y)

            self._mode_label = 'BIAS' if not self._bias_ready else 'IDLE'

        # Интегрируем позицию из IMU скорости
        # (is_moving → capped velocity, !is_moving → нулевая скорость)
        self.x += self._imu_vx * dt
        self.y += self._imu_vy * dt

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self._imu_vx = 0.0
        self._imu_vy = 0.0
        self._bias_x = 0.0
        self._bias_y = 0.0
        self._bias_ready = False
        self._bias_samples = 0
        self._deviation_count = 0
        self._fast_adapt_remaining = 0
        self._continuous_move_ticks = 0

    @property
    def velocity_magnitude(self):
        if self.motors_active:
            return 0.0
        return math.sqrt(self._imu_vx**2 + self._imu_vy**2)


class OdomDebugger:
    """Тестовый стенд для гибридной одометрии."""

    def __init__(self, broker: str, port: int, robot_id: str, mode: str):
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._prefix = f'samurai/{robot_id}'
        self._mode = mode

        # ── Гибридная одометрия ──
        self._odom = HybridOdometry()

        # ── Команды (для определения motors_active) ──
        self._auto_linear = 0.0
        self._auto_angular = 0.0
        self._last_auto_time = 0.0
        self._manual_linear = 0.0
        self._manual_angular = 0.0
        self._last_manual_time = 0.0

        # IMU yaw
        self._imu_yaw = None

        # Gravity-free accel из odom (motor_node → AccelPositionEstimator)
        self._accel_x = 0.0
        self._accel_y = 0.0

        # ── Motor_node odom для сравнения ──
        self._mn_x = 0.0
        self._mn_y = 0.0
        self._mn_vx = 0.0
        self._mn_stationary = True

        # ── Статистика ──
        self._cmd_count = 0
        self._odom_count = 0
        self._imu_count = 0
        self._last_print_time = 0.0
        self._start_time = 0.0

        # ── Timing ──
        self._last_tick = time.monotonic()
        self._tick_rate = 20

        # ── MQTT ──
        self._client = mqtt.Client(
            client_id=f'odom_debug_{robot_id}',
            protocol=mqtt.MQTTv311)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

    # ── MQTT ────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f'[ERROR] MQTT connect failed rc={rc}')
            return
        print(f'[OK] MQTT connected to {self._broker}:{self._port}')
        p = self._prefix
        client.subscribe(f'{p}/cmd_vel', qos=1)
        client.subscribe(f'{p}/cmd_vel/manual', qos=1)
        client.subscribe(f'{p}/imu', qos=0)
        client.subscribe(f'{p}/odom', qos=0)
        client.message_callback_add(f'{p}/cmd_vel', self._on_cmd_vel)
        client.message_callback_add(f'{p}/cmd_vel/manual', self._on_cmd_vel_manual)
        client.message_callback_add(f'{p}/imu', self._on_imu)
        client.message_callback_add(f'{p}/odom', self._on_odom)
        print('[OK] Subscribed to: cmd_vel, cmd_vel/manual, imu, odom')

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(f'[WARN] MQTT disconnected (rc={rc})')

    def _on_cmd_vel(self, client, userdata, msg):
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        self._auto_linear = float(d.get('linear_x', 0.0))
        self._auto_angular = float(d.get('angular_z', 0.0))
        self._last_auto_time = time.monotonic()
        self._cmd_count += 1

    def _on_cmd_vel_manual(self, client, userdata, msg):
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        self._manual_linear = float(d.get('linear_x', 0.0))
        self._manual_angular = float(d.get('angular_z', 0.0))
        self._last_manual_time = time.monotonic()
        self._cmd_count += 1

    def _on_imu(self, client, userdata, msg):
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
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        self._mn_x = d.get('x', 0.0)
        self._mn_y = d.get('y', 0.0)
        self._mn_vx = d.get('vx', 0.0)
        self._mn_stationary = d.get('stationary', True)
        # Gravity-free acceleration из AccelPositionEstimator
        self._accel_x = d.get('accel_x', 0.0)
        self._accel_y = d.get('accel_y', 0.0)
        self._odom_count += 1

    # ── Tick ────────────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0 or dt > 1.0:
            return

        # Выбор команды: manual > autonomous
        manual_active = (self._last_manual_time > 0 and
                         (now - self._last_manual_time) <= MANUAL_OVERRIDE_TIMEOUT)
        if self._last_auto_time > 0 and (now - self._last_auto_time) > CMD_VEL_TIMEOUT:
            self._auto_linear = 0.0
            self._auto_angular = 0.0

        if manual_active:
            lin_cmd = self._manual_linear
            ang_cmd = self._manual_angular
        else:
            lin_cmd = self._auto_linear
            ang_cmd = self._auto_angular

        # Dead zone + clamp
        if abs(lin_cmd) < DEADZONE_LINEAR:
            lin_cmd = 0.0
        if abs(ang_cmd) < DEADZONE_ANGULAR:
            ang_cmd = 0.0
        max_lin, max_ang = SPEED_PROFILES['normal']
        lin = max(-max_lin, min(max_lin, lin_cmd))
        ang = max(-max_ang, min(max_ang, ang_cmd))

        # Heading
        if self._imu_yaw is not None:
            yaw = self._imu_yaw
        else:
            yaw = self._odom.theta + ang * dt

        # ── Гибридная одометрия ──
        self._odom.update(
            lin_cmd=lin,
            ang_cmd=ang,
            imu_yaw=yaw,
            accel_x=self._accel_x,
            accel_y=self._accel_y,
            dt=dt,
        )

        # Публикация
        if self._mode == 'publish':
            msg = {
                'x':  round(self._odom.x * 100.0, 2),
                'y':  round(self._odom.y * 100.0, 2),
                'theta': round(self._odom.theta, 4),
                'vx': round(self._odom.velocity_magnitude, 3),
                'vz': round(ang, 3),
                'speed': round(self._odom.velocity_magnitude, 3),
                'stationary': self._odom._mode_label == 'IDLE',
                'source': 'hybrid',
                'mode': self._odom._mode_label,
                'ts': time.time(),
            }
            self._client.publish(
                f'{self._prefix}/odom/debug',
                json.dumps(msg, separators=(',', ':')),
                qos=0)

    def _print_status(self):
        now = time.monotonic()
        if now - self._last_print_time < 1.0:
            return
        self._last_print_time = now

        ox = self._odom.x * 100.0
        oy = self._odom.y * 100.0
        mx = self._mn_x
        my = self._mn_y
        yaw_deg = math.degrees(self._odom.theta) if self._imu_yaw is not None else -999
        mode = self._odom._mode_label
        accel_mag = math.sqrt(self._accel_x**2 + self._accel_y**2)
        dev = self._odom.last_deviation
        bias_mag = math.sqrt(self._odom._bias_x**2 + self._odom._bias_y**2)
        imu_vel = self._odom.velocity_magnitude * 100.0  # cm/s

        print(f'  [{now - self._start_time:6.1f}s]'
              f'  [{mode:4s}]'
              f'  OUR: x={ox:+8.2f} y={oy:+8.2f}'
              f'  |  MN: x={mx:+8.2f} y={my:+8.2f}'
              f'  |  acc={accel_mag:.3f} bias={bias_mag:.3f} dev={dev:.3f}'
              f'  |  vel={imu_vel:5.1f}cm/s'
              f'  |  det={self._odom.imu_moves_detected}')

    # ── Inject ──────────────────────────────────────────────────

    def _inject_test(self):
        print()
        print('='*70)
        print('  INJECT: авто-тест гибридной одометрии')
        print('='*70)

        # Phase 1: тишина
        print('\n[Phase 1] Тишина 3с — позиция = 0...')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            self._tick()
            self._print_status()
            time.sleep(0.05)
        ox0, oy0 = self._odom.x * 100, self._odom.y * 100
        print(f'  >> Дрифт: ({ox0:+.2f}, {oy0:+.2f}) см')
        if abs(ox0) > 1.0 or abs(oy0) > 1.0:
            print('  >> !!! ДРИФТ БЕЗ ДВИЖЕНИЯ !!!')

        # Phase 2: команда вперёд
        print('\n[Phase 2] cmd_vel lin=0.20 на 2с...')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            self._client.publish(
                f'{self._prefix}/cmd_vel/manual',
                json.dumps({'linear_x': 0.20, 'angular_z': 0.0}), qos=1)
            self._tick()
            self._print_status()
            time.sleep(0.05)
        ox1, oy1 = self._odom.x * 100, self._odom.y * 100
        dist = math.sqrt((ox1 - ox0)**2 + (oy1 - oy0)**2)
        expected = 0.20 * SCALE_FWD * 2.0 * 100.0
        print(f'  >> Пройдено: {dist:.1f} см (ожидалось ~{expected:.1f})')

        # Phase 3: стоп
        print('\n[Phase 3] Стоп 5с — позиция замирает...')
        self._client.publish(
            f'{self._prefix}/cmd_vel/manual',
            json.dumps({'linear_x': 0.0, 'angular_z': 0.0}), qos=1)
        fx, fy = self._odom.x, self._odom.y
        max_j = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            self._tick()
            self._print_status()
            j = math.sqrt((self._odom.x - fx)**2 + (self._odom.y - fy)**2) * 100
            if j > max_j:
                max_j = j
            time.sleep(0.05)
        print(f'  >> Джиттер за 5с стопа: {max_j:.2f} см')
        if max_j > 1.0:
            print(f'  >> !!! ФАНТОМ: {max_j:.1f} см !!!')
        else:
            print('  >> OK: стабильно')

        print('\n' + '='*70)
        print(f'  IMU движений обнаружено: {self._odom.imu_moves_detected}')
        print('='*70)

    # ── Main ────────────────────────────────────────────────────

    def run(self):
        self._start_time = time.monotonic()
        self._last_tick = time.monotonic()
        self._last_print_time = 0.0

        stop = threading.Event()
        def _sig(s, f):
            stop.set()
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(f'Connecting to MQTT {self._broker}:{self._port}...')
        self._client.connect(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        time.sleep(1.0)

        print()
        print('='*70)
        print(f'  HYBRID ODOM DEBUG  |  mode={self._mode}  |  robot={self._robot_id}')
        print(f'  CMD: scale_fwd={SCALE_FWD}  scale_bwd={SCALE_BWD}')
        print(f'  IMU: dev_thr={IMU_DEVIATION_THRESHOLD}  vel_cap={IMU_VELOCITY_CAP} m/s'
              f'  max_move={IMU_MAX_MOVE_TICKS} ticks')
        print(f'       bias_slow={IMU_BIAS_ALPHA_SLOW}  bias_fast={IMU_BIAS_ALPHA_FAST}'
              f'  fast_ticks={IMU_FAST_ADAPT_TICKS}')
        print('='*70)
        print()
        print('  Режимы:  CMD  = позиция по cmd_vel (моторы ON)')
        print('           IMU+ = движение по акселерометру (моторы OFF)')
        print('           GRAV = gravity recovery (bias переучивается)')
        print('           IDLE = стоим (bias выучен, отклонений нет)')
        print('           BIAS = начальная калибровка bias (~2с)')
        print()

        if self._mode == 'inject':
            self._inject_test()
        else:
            print(f'  Ctrl+C для выхода')
            print()
            while not stop.is_set():
                self._tick()
                self._print_status()
                stop.wait(timeout=1.0 / self._tick_rate)

        print()
        print(f'  Итого: imu_moves={self._odom.imu_moves_detected}'
              f'  cmd={self._cmd_count}  imu={self._imu_count}'
              f'  odom={self._odom_count}')
        self._client.loop_stop()
        self._client.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description='Гибридная одометрия: cmd_vel + IMU passive')
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    parser.add_argument('--mode', choices=['monitor', 'publish', 'inject'],
                        default='monitor',
                        help='monitor / publish / inject')
    args = parser.parse_args()

    OdomDebugger(args.broker, args.port, args.robot_id, args.mode).run()


if __name__ == '__main__':
    main()
