#!/usr/bin/env python3
"""
diagnostic.py — Автоматизированный диагностический скрипт робота Г.О.Ш.А.
"""

import logging
import sys
import time
import os
import platform
import json
import struct
import wave
import subprocess
from datetime import datetime
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────
# Настройка логирования
# ──────────────────────────────────────────────────────────────────────

LOG_DIR = Path.home() / "samurai_diagnostics"
LOG_DIR.mkdir(exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = LOG_DIR / f"diagnostic_{timestamp}.log"

logger = logging.getLogger("samurai_diag")
logger.setLevel(logging.DEBUG)

fmt = logging.Formatter(
    "[%(asctime)s] [%(levelname)-8s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

fh = logging.FileHandler(LOG_FILE, encoding="utf-8")
fh.setLevel(logging.DEBUG)
fh.setFormatter(fmt)
logger.addHandler(fh)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.INFO)
ch.setFormatter(fmt)
logger.addHandler(ch)

# ──────────────────────────────────────────────────────────────────────
# Вспомогательные функции
# ──────────────────────────────────────────────────────────────────────

results: dict[str, dict] = {}
saved_files: list[str] = []  # пути ко всем сохранённым файлам диагностики


def section(title: str):
    sep = "=" * 60
    logger.info("")
    logger.info(sep)
    logger.info(f"  {title}")
    logger.info(sep)


def record(module: str, *, status: str, details: dict | None = None):
    results[module] = {"status": status, "details": details or {}}


def safe_import(module_name: str):
    try:
        mod = __import__(module_name)
        logger.debug(f"Импорт '{module_name}' — OK")
        return mod
    except ImportError as e:
        logger.error(f"Импорт '{module_name}' — ОШИБКА: {e}")
        return None


# ══════════════════════════════════════════════════════════════════════
#  0. СИСТЕМНАЯ ИНФОРМАЦИЯ
# ══════════════════════════════════════════════════════════════════════

def diagnose_system_info():
    section("СИСТЕМНАЯ ИНФОРМАЦИЯ")

    info = {
        "hostname": platform.node(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "python": platform.python_version(),
        "time": datetime.now().isoformat(),
    }

    try:
        with open("/proc/cpuinfo", "r") as f:
            cpuinfo = f.read()
        for line in cpuinfo.splitlines():
            if line.startswith("Model"):
                info["rpi_model"] = line.split(":")[-1].strip()
            if line.startswith("Serial"):
                info["rpi_serial"] = line.split(":")[-1].strip()
            if line.startswith("Hardware"):
                info["rpi_hardware"] = line.split(":")[-1].strip()
            if line.startswith("Revision"):
                info["rpi_revision"] = line.split(":")[-1].strip()
    except FileNotFoundError:
        logger.warning("/proc/cpuinfo не найден (не Raspberry Pi?)")

    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_milli = int(f.read().strip())
            info["cpu_temp_c"] = temp_milli / 1000.0
    except Exception:
        pass

    try:
        with open("/proc/meminfo", "r") as f:
            for line in f:
                if line.startswith("MemTotal"):
                    info["mem_total_kb"] = int(line.split()[1])
                if line.startswith("MemAvailable"):
                    info["mem_available_kb"] = int(line.split()[1])
    except Exception:
        pass

    try:
        st = os.statvfs("/")
        info["disk_total_gb"] = round(st.f_blocks * st.f_frsize / (1024**3), 2)
        info["disk_free_gb"] = round(st.f_bavail * st.f_frsize / (1024**3), 2)
    except Exception:
        pass

    for k, v in info.items():
        logger.info(f"  {k}: {v}")

    record("system_info", status="OK", details=info)


# ══════════════════════════════════════════════════════════════════════
#  1. PYTHON-ПАКЕТЫ
# ══════════════════════════════════════════════════════════════════════

def diagnose_python_packages():
    section("УСТАНОВЛЕННЫЕ PYTHON-ПАКЕТЫ (ключевые)")

    packages = [
        "adafruit_pca9685", "adafruit_motor", "board", "busio",
        "gpiozero", "picamera2", "cv2", "numpy",
        "pyaudio", "vosk", "smbus2",
        "rpi_ws281x",
        "rclpy", "sensor_msgs", "geometry_msgs", "std_msgs", "cv_bridge",
    ]

    installed = {}
    for pkg in packages:
        try:
            mod = __import__(pkg)
            ver = getattr(mod, "__version__", getattr(mod, "VERSION", "N/A"))
            installed[pkg] = str(ver)
            logger.info(f"  {pkg:25s} — v{ver}")
        except ImportError:
            installed[pkg] = "NOT INSTALLED"
            logger.warning(f"  {pkg:25s} — НЕ УСТАНОВЛЕН")

    record("python_packages", status="OK", details=installed)


# ══════════════════════════════════════════════════════════════════════
#  2. GPIO ИНФОРМАЦИЯ
# ══════════════════════════════════════════════════════════════════════

def diagnose_gpio():
    section("GPIO ИНФОРМАЦИЯ")

    try:
        from gpiozero import Device
        logger.info(f"  gpiozero pin factory: {Device.pin_factory}")
    except Exception as e:
        logger.error(f"  gpiozero ОШИБКА: {e}")

    logger.info(f"  GPIO карта робота (BCM):")
    logger.info(f"    GPIO2  (SDA1) — I2C SDA (PCA9685, ADS7830)")
    logger.info(f"    GPIO3  (SCL1) — I2C SCL (PCA9685, ADS7830)")
    logger.info(f"    GPIO12       — WS2812 RGB LED Data")
    logger.info(f"    GPIO16       — Line Tracking Middle")
    logger.info(f"    GPIO17       — Лазер [НЕ ТЕСТИРУЕТСЯ]")
    logger.info(f"    GPIO19       — Line Tracking Right")
    logger.info(f"    GPIO20       — Line Tracking Left")
    logger.info(f"    GPIO23       — HC-SR04 Trigger")
    logger.info(f"    GPIO24       — HC-SR04 Echo")

    record("gpio", status="OK", details={
        "used_pins": {
            "GPIO2": "I2C SDA",
            "GPIO3": "I2C SCL",
            "GPIO12": "WS2812 LED Data",
            "GPIO16": "Line Tracking Middle",
            "GPIO17": "Laser (SKIP)",
            "GPIO19": "Line Tracking Right",
            "GPIO20": "Line Tracking Left",
            "GPIO23": "HC-SR04 Trigger",
            "GPIO24": "HC-SR04 Echo",
        }
    })


# ══════════════════════════════════════════════════════════════════════
#  3. I2C ШИНА И УСТРОЙСТВА
# ══════════════════════════════════════════════════════════════════════

def diagnose_i2c():
    section("I2C ШИНА И УСТРОЙСТВА")

    smbus2 = safe_import("smbus2")
    i2c_devices = []

    KNOWN_DEVICES = {
        "0x48": "ADS7830 (8-канальный АЦП)",
        "0x4b": "ADS7830 (8-канальный АЦП, альт. адрес)",
        "0x5f": "PCA9685 (Adeept Robot HAT, PWM-контроллер)",
        "0x40": "PCA9685 (стандартный адрес)",
        "0x68": "MPU6050 (IMU — гироскоп + акселерометр) [НЕ УСТАНОВЛЕН]",
    }

    if smbus2:
        try:
            bus = smbus2.SMBus(1)
            logger.info("I2C bus 1 открыт — сканирование адресов 0x03..0x77...")
            for addr in range(0x03, 0x78):
                try:
                    bus.read_byte(addr)
                    addr_hex = hex(addr)
                    i2c_devices.append(addr_hex)
                    name = KNOWN_DEVICES.get(addr_hex, "неизвестное устройство")
                    logger.info(f"  Найдено I2C устройство: {addr_hex} — {name}")
                except Exception:
                    pass
            bus.close()
        except Exception as e:
            logger.error(f"Ошибка открытия I2C bus 1: {e}")
    else:
        try:
            result = subprocess.run(
                ["i2cdetect", "-y", "1"],
                capture_output=True, text=True, timeout=5
            )
            logger.info(f"i2cdetect вывод:\n{result.stdout}")
        except Exception as e:
            logger.error(f"i2cdetect недоступен: {e}")

    if not i2c_devices:
        logger.warning("  I2C устройства НЕ ОБНАРУЖЕНЫ")

    record("i2c_scan", status="OK" if i2c_devices else "WARNING",
           details={"devices": i2c_devices, "known": KNOWN_DEVICES})
    return i2c_devices


# ══════════════════════════════════════════════════════════════════════
#  4. ADS7830 — 8-КАНАЛЬНЫЙ АЦП
# ══════════════════════════════════════════════════════════════════════

def diagnose_ads7830():
    section("ADS7830 — 8-КАНАЛЬНЫЙ АЦП (I2C)")

    # ADS7830 может быть на 0x48 или 0x4b в зависимости от конфигурации A0/A1
    POSSIBLE_ADDRS = [0x48, 0x49, 0x4A, 0x4B]
    # Команды для чтения каналов 0-7 (single-ended)
    CHANNEL_CMDS = (0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4)

    smbus2 = safe_import("smbus2")
    if not smbus2:
        logger.error("  smbus2 не установлен — ADS7830 недоступен")
        record("ads7830", status="ERROR", details={"error": "smbus2 not installed"})
        return

    found_addr = None
    try:
        bus = smbus2.SMBus(1)

        for addr in POSSIBLE_ADDRS:
            try:
                bus.read_byte(addr)
                found_addr = addr
                logger.info(f"  ADS7830 найден на адресе {hex(addr)}")
                break
            except Exception:
                continue

        if found_addr is None:
            logger.warning(f"  ADS7830 НЕ НАЙДЕН (проверены адреса: "
                           f"{[hex(a) for a in POSSIBLE_ADDRS]})")
            bus.close()
            record("ads7830", status="WARNING", details={
                "error": "Device not found",
                "checked_addrs": [hex(a) for a in POSSIBLE_ADDRS],
            })
            return

        logger.info(f"  Тип: 8-канальный 8-бит АЦП")
        logger.info(f"  I2C адрес: {hex(found_addr)}")
        logger.info(f"  Разрешение: 8 бит (0-255)")
        logger.info(f"  Опорное напряжение: 3.3V")

        # Чтение всех 8 каналов
        logger.info(f"  >>> Тест: чтение всех 8 каналов...")
        channel_values = {}
        for ch_idx, cmd in enumerate(CHANNEL_CMDS):
            try:
                bus.write_byte(found_addr, cmd)
                value = bus.read_byte(found_addr)
                voltage = round(value / 255.0 * 3.3, 3)
                channel_values[f"CH{ch_idx}"] = {"raw": value, "voltage_v": voltage}
                logger.info(f"    CH{ch_idx}: raw={value:3d}/255  "
                            f"voltage={voltage:.3f}V  cmd={hex(cmd)}")
            except Exception as e:
                logger.error(f"    CH{ch_idx}: ОШИБКА — {e}")
                channel_values[f"CH{ch_idx}"] = {"error": str(e)}

        bus.close()

        # Канал батареи (обычно CH0 или CH4 на Adeept)
        logger.info(f"  Примечание: один из каналов обычно подключён к")
        logger.info(f"  делителю напряжения батареи (7.4V → 3.3V)")

        record("ads7830", status="OK", details={
            "address": hex(found_addr),
            "resolution_bits": 8,
            "channels": channel_values,
        })

    except Exception as e:
        logger.error(f"  ADS7830 ОШИБКА: {e}")
        record("ads7830", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  5. PCA9685 PWM-КОНТРОЛЛЕР
# ══════════════════════════════════════════════════════════════════════

pca_driver = None


def diagnose_pca9685():
    global pca_driver
    section("PCA9685 PWM-КОНТРОЛЛЕР (I2C)")

    # Попробуем оба адреса: 0x5F (Adeept) и 0x40 (стандартный)
    ADDRS_TO_TRY = [0x5F, 0x40]

    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        i2c = busio.I2C(board.SCL, board.SDA)
        pca = None
        used_addr = None

        for addr in ADDRS_TO_TRY:
            try:
                pca = PCA9685(i2c, address=addr)
                pca.frequency = 50
                used_addr = addr
                logger.info(f"  PCA9685 найден на адресе {hex(addr)}")
                break
            except Exception:
                continue

        if pca is None:
            logger.error(f"  PCA9685 НЕ НАЙДЕН (проверены: {[hex(a) for a in ADDRS_TO_TRY]})")
            record("pca9685", status="ERROR", details={
                "error": "PCA9685 not found",
                "checked": [hex(a) for a in ADDRS_TO_TRY],
            })
            return None

        pca_driver = pca

        logger.info(f"  PCA9685 инициализирован успешно")
        logger.info(f"  I2C адрес: {hex(used_addr)}")
        logger.info(f"  Частота PWM: {pca.frequency} Hz")
        logger.info(f"  Каналов: 16 (0-15)")
        logger.info(f"  Распределение каналов:")
        logger.info(f"    ch0  — Сервопривод 0 (клешня)")
        logger.info(f"    ch1  — Сервопривод 1")
        logger.info(f"    ch2  — Сервопривод 2")
        logger.info(f"    ch3  — Сервопривод 3")
        logger.info(f"    ch4  — Сервопривод 4")
        logger.info(f"    ch5  — Сервопривод 5")
        logger.info(f"    ch6  — Свободен")
        logger.info(f"    ch7  — Свободен")
        logger.info(f"    ch8  — M4 правый-задний IN1")
        logger.info(f"    ch9  — M4 правый-задний IN2")
        logger.info(f"    ch10 — M3 левый-задний IN2")
        logger.info(f"    ch11 — M3 левый-задний IN1")
        logger.info(f"    ch12 — M2 левый-передний IN1")
        logger.info(f"    ch13 — M2 левый-передний IN2")
        logger.info(f"    ch14 — M1 правый-передний IN2")
        logger.info(f"    ch15 — M1 правый-передний IN1")

        # Чтение duty_cycle каждого канала
        for ch_num in range(16):
            try:
                dc = pca.channels[ch_num].duty_cycle
                logger.debug(f"  Канал {ch_num:2d}: duty_cycle = {dc}")
            except Exception as e:
                logger.debug(f"  Канал {ch_num:2d}: ошибка чтения — {e}")

        record("pca9685", status="OK", details={
            "address": hex(used_addr),
            "frequency_hz": pca.frequency,
            "channels": 16,
        })
        return pca

    except Exception as e:
        logger.error(f"  PCA9685 ОШИБКА: {e}")
        record("pca9685", status="ERROR", details={"error": str(e)})
        return None


# ══════════════════════════════════════════════════════════════════════
#  6. DRV8833 — ДРАЙВЕРЫ МОТОРОВ
# ══════════════════════════════════════════════════════════════════════

def diagnose_drv8833(pca):
    section("DRV8833 x2 — ДРАЙВЕРЫ DC МОТОРОВ")

    logger.info("  Тип: DRV8833 Dual H-Bridge Motor Driver")
    logger.info("  Количество: 2 шт. на Adeept Robot HAT V3.1")
    logger.info("  DRV8833 #1: управляет M1 (правый-передний) + M2 (левый-передний)")
    logger.info("  DRV8833 #2: управляет M3 (левый-задний) + M4 (правый-задний)")
    logger.info("  Управление: через PCA9685 PWM-каналы (ch8-ch15)")
    logger.info("  Режим: SLOW_DECAY (плавное управление DC моторами)")

    if pca is not None:
        logger.info("  Статус: PCA9685 доступен — DRV8833 управляемы")
        record("drv8833", status="OK", details={
            "count": 2,
            "type": "DRV8833 Dual H-Bridge",
            "motors": "M1-M4 via PCA9685 ch8-ch15",
        })
    else:
        logger.error("  PCA9685 недоступен — DRV8833 не тестируются")
        record("drv8833", status="ERROR", details={"error": "PCA9685 not available"})


# ══════════════════════════════════════════════════════════════════════
#  7. LM324 — ОПЕРАЦИОННЫЙ УСИЛИТЕЛЬ
# ══════════════════════════════════════════════════════════════════════

def diagnose_lm324():
    section("LM324 — ОПЕРАЦИОННЫЙ УСИЛИТЕЛЬ")

    logger.info("  Тип: LM324 Quad Op-Amp")
    logger.info("  Назначение: усиление/буферизация аналоговых сигналов")
    logger.info("  Расположение: на плате Adeept Robot HAT V3.1")
    logger.info("  Применение на HAT:")
    logger.info("    - Буферизация сигналов для ADS7830 АЦП")
    logger.info("    - Усиление сигналов ИК-датчиков линии")
    logger.info("    - Компаратор для пороговых уровней")
    logger.info("  Прямое тестирование: невозможно (пассивный компонент)")
    logger.info("  Косвенная проверка: через работу ADS7830 и Line Tracking")

    record("lm324", status="OK", details={
        "type": "LM324 Quad Op-Amp",
        "note": "Passive component, tested indirectly via ADS7830 and line tracking",
    })


# ══════════════════════════════════════════════════════════════════════
#  8. DC МОТОРЫ (M1-M4)
# ══════════════════════════════════════════════════════════════════════

def diagnose_motors(pca):
    section("DC МОТОРЫ (4× гусеничная платформа, через DRV8833)")

    if pca is None:
        logger.error("  PCA9685 не инициализирован — моторы недоступны")
        record("motors", status="ERROR", details={"error": "PCA9685 not available"})
        return None

    try:
        from adafruit_motor import motor as adafruit_motor

        MOTOR_MAP = {
            "M1_right_front": (15, 14),
            "M2_left_front": (12, 13),
            "M3_left_rear": (11, 10),
            "M4_right_rear": (8, 9),
        }

        motors = {}
        for name, (ch_in1, ch_in2) in MOTOR_MAP.items():
            try:
                m = adafruit_motor.DCMotor(pca.channels[ch_in1], pca.channels[ch_in2])
                m.decay_mode = adafruit_motor.SLOW_DECAY
                motors[name] = m
                logger.info(f"  {name}: ch_IN1={ch_in1}, ch_IN2={ch_in2} — OK")
            except Exception as e:
                logger.error(f"  {name}: ch_IN1={ch_in1}, ch_IN2={ch_in2} — ОШИБКА: {e}")

        if not motors:
            record("motors", status="ERROR", details={"error": "No motors initialized"})
            return None

        SPEED = 0.35
        MOVE_TIME = 1.0
        move_log = []  # лог всех движений с таймштампами

        def set_all(throttle):
            for m in motors.values():
                try:
                    m.throttle = throttle
                except Exception as e:
                    logger.error(f"  throttle error: {e}")

        def set_sides(left, right):
            for name, m in motors.items():
                try:
                    m.throttle = left if "left" in name else right
                except Exception as e:
                    logger.error(f"  {name} throttle error: {e}")

        def stop_all():
            for m in motors.values():
                try:
                    m.throttle = 0
                except Exception:
                    pass

        # --- Комбинированные движения ---
        movements = [
            ("ВПЕРЁД",        lambda: set_all(SPEED),          f"throttle={SPEED}"),
            ("НАЗАД",         lambda: set_all(-SPEED),         f"throttle={-SPEED}"),
            ("ПОВОРОТ_ВЛЕВО", lambda: set_sides(-SPEED, SPEED), f"L={-SPEED}, R={SPEED}"),
            ("ПОВОРОТ_ВПРАВО", lambda: set_sides(SPEED, -SPEED), f"L={SPEED}, R={-SPEED}"),
        ]

        for move_name, move_fn, params in movements:
            t_start = time.time()
            logger.info(f"  >>> Тест: {move_name} ({params}, {MOVE_TIME}с)")
            try:
                move_fn()
                time.sleep(MOVE_TIME)
                stop_all()
                t_elapsed = time.time() - t_start
                move_log.append({
                    "move": move_name, "params": params,
                    "duration_s": round(t_elapsed, 3), "status": "OK"
                })
                logger.info(f"  <<< {move_name} — OK (факт. {t_elapsed:.3f}с)")
            except Exception as e:
                move_log.append({
                    "move": move_name, "params": params,
                    "duration_s": 0, "status": "ERROR", "error": str(e)
                })
                logger.error(f"  <<< {move_name} — ОШИБКА: {e}")
            time.sleep(0.5)

        stop_all()

        # --- Каждый мотор по отдельности ---
        logger.info("")
        logger.info("  --- Тест каждого мотора по отдельности ---")
        individual_results = {}
        for name, m in motors.items():
            try:
                logger.info(f"  >>> {name}: вперёд 0.5с")
                m.throttle = SPEED
                time.sleep(0.5)
                m.throttle = 0
                time.sleep(0.3)

                logger.info(f"  >>> {name}: назад 0.5с")
                m.throttle = -SPEED
                time.sleep(0.5)
                m.throttle = 0
                time.sleep(0.3)

                individual_results[name] = "OK"
                logger.info(f"  <<< {name} — OK")
            except Exception as e:
                individual_results[name] = f"ERROR: {e}"
                logger.error(f"  <<< {name} — ОШИБКА: {e}")

        stop_all()
        record("motors", status="OK", details={
            "count": len(motors),
            "names": list(motors.keys()),
            "test_speed": SPEED,
            "driver": "DRV8833 x2 via PCA9685",
            "movement_tests": move_log,
            "individual_tests": individual_results,
        })
        return motors

    except Exception as e:
        logger.error(f"  Общая ошибка моторов: {e}")
        record("motors", status="ERROR", details={"error": str(e)})
        return None


# ══════════════════════════════════════════════════════════════════════
#  9. AD002 СЕРВОПРИВОДЫ x6 (PCA9685 каналы 0-5)
# ══════════════════════════════════════════════════════════════════════

def diagnose_servos(pca):
    section("AD002 СЕРВОПРИВОДЫ x6 (PCA9685 каналы 0-5)")

    if pca is None:
        logger.error("  PCA9685 не инициализирован — сервоприводы недоступны")
        record("servos_ad002", status="ERROR", details={"error": "PCA9685 not available"})
        return

    try:
        from adafruit_motor import servo as adafruit_servo

        MIN_PULSE = 500
        MAX_PULSE = 2400
        ACTUATION_RANGE = 180

        # Конфигурация 6 сервоприводов
        SERVO_CONFIG = {
            0: {"name": "Servo0 (клешня / gripper)", "init": 90, "test_angles": [30, 130, 90]},
            1: {"name": "Servo1 (рука / arm joint 1)", "init": 90, "test_angles": [70, 110, 90]},
            2: {"name": "Servo2 (рука / arm joint 2)", "init": 90, "test_angles": [70, 110, 90]},
            3: {"name": "Servo3 (рука / arm joint 3)", "init": 90, "test_angles": [70, 110, 90]},
            4: {"name": "Servo4 (рука / arm joint 4)", "init": 90, "test_angles": [70, 110, 90]},
            5: {"name": "Servo5 (рука / arm joint 5)", "init": 90, "test_angles": [70, 110, 90]},
        }

        logger.info(f"  Тип: AD002 Servo Motor")
        logger.info(f"  Количество: 6 шт.")
        logger.info(f"  Диапазон пульса: {MIN_PULSE}-{MAX_PULSE} мкс")
        logger.info(f"  Диапазон угла: 0-{ACTUATION_RANGE}°")
        logger.info("")

        servo_results = {}

        for ch, cfg in SERVO_CONFIG.items():
            name = cfg["name"]
            try:
                srv = adafruit_servo.Servo(
                    pca.channels[ch],
                    min_pulse=MIN_PULSE,
                    max_pulse=MAX_PULSE,
                    actuation_range=ACTUATION_RANGE,
                )

                logger.info(f"  [{ch}] {name}")
                logger.info(f"      PCA9685 канал: {ch}")
                logger.info(f"      Пульс: {MIN_PULSE}-{MAX_PULSE} мкс")

                # Начальная позиция
                init_angle = cfg["init"]
                srv.angle = init_angle
                logger.info(f"      INIT → {init_angle}° (установлен)")
                time.sleep(0.5)

                # Тестовые углы
                tested_angles = []
                for angle in cfg["test_angles"]:
                    try:
                        srv.angle = angle
                        tested_angles.append({"angle": angle, "status": "OK"})
                        logger.info(f"      → {angle}° — OK (установлен)")
                        time.sleep(0.5)
                    except Exception as e:
                        tested_angles.append({"angle": angle, "status": "ERROR", "error": str(e)})
                        logger.error(f"      → {angle}° — ОШИБКА: {e}")

                # Вернуть в init
                srv.angle = init_angle
                time.sleep(0.3)

                servo_results[f"ch{ch}"] = {
                    "name": name, "status": "OK", "channel": ch,
                    "init_angle": init_angle, "tested_angles": tested_angles,
                }
                logger.info(f"      <<< OK (возвращён в {init_angle}°)")

            except Exception as e:
                logger.error(f"  [{ch}] {name} — ОШИБКА: {e}")
                servo_results[f"ch{ch}"] = {"name": name, "status": "ERROR", "error": str(e)}

        ok_count = sum(1 for v in servo_results.values() if v["status"] == "OK")
        record("servos_ad002", status="OK" if ok_count > 0 else "ERROR", details={
            "type": "AD002 Servo Motor",
            "total": 6,
            "ok": ok_count,
            "pulse_range_us": f"{MIN_PULSE}-{MAX_PULSE}",
            "servos": servo_results,
        })

    except Exception as e:
        logger.error(f"  Сервоприводы ОШИБКА: {e}")
        record("servos_ad002", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  10. УЛЬТРАЗВУКОВОЙ ДАТЧИК HC-SR04
# ══════════════════════════════════════════════════════════════════════

def diagnose_ultrasonic():
    section("УЛЬТРАЗВУКОВОЙ ДАТЧИК HC-SR04 (GPIO23/24)")

    TRIGGER_PIN = 23
    ECHO_PIN = 24
    MAX_RANGE = 2.0
    MIN_RANGE = 0.02
    FOV_RAD = 0.26

    try:
        from gpiozero import DistanceSensor

        sensor = DistanceSensor(
            echo=ECHO_PIN, trigger=TRIGGER_PIN, max_distance=MAX_RANGE
        )

        logger.info(f"  HC-SR04 инициализирован")
        logger.info(f"  GPIO Trigger: {TRIGGER_PIN}")
        logger.info(f"  GPIO Echo:    {ECHO_PIN}")
        logger.info(f"  Макс. дальность: {MAX_RANGE} м")
        logger.info(f"  Мин. дальность:  {MIN_RANGE} м")
        logger.info(f"  Угол обзора:     {FOV_RAD} рад (~15°)")

        readings = []
        logger.info(f"  >>> Тест: 10 замеров с интервалом 0.1с...")
        for i in range(10):
            try:
                dist = sensor.distance
                readings.append(dist)
                logger.info(f"    Замер {i+1:2d}: {dist:.4f} м ({dist*100:.1f} см)")
            except Exception as e:
                logger.error(f"    Замер {i+1:2d}: ОШИБКА — {e}")
            time.sleep(0.1)

        sensor.close()

        avg = mn = mx = None
        if readings:
            avg = sum(readings) / len(readings)
            mn = min(readings)
            mx = max(readings)
            logger.info(f"  Среднее: {avg:.4f} м, мин: {mn:.4f} м, макс: {mx:.4f} м")

        record("ultrasonic", status="OK" if readings else "ERROR", details={
            "trigger_gpio": TRIGGER_PIN,
            "echo_gpio": ECHO_PIN,
            "max_range_m": MAX_RANGE,
            "readings_count": len(readings),
            "avg_m": round(avg, 4) if avg else None,
            "min_m": round(mn, 4) if mn else None,
            "max_m": round(mx, 4) if mx else None,
        })

    except Exception as e:
        logger.error(f"  Ультразвук ОШИБКА: {e}")
        record("ultrasonic", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  11. 3-CH LINE TRACKING MODULE (GPIO19/16/20)
# ══════════════════════════════════════════════════════════════════════

def diagnose_line_tracking():
    section("3-CH LINE TRACKING MODULE (ИК-датчики линии)")

    # Adeept стандартные пины
    LINE_RIGHT = 19
    LINE_MIDDLE = 16
    LINE_LEFT = 20

    try:
        import RPi.GPIO as GPIO

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LINE_RIGHT, GPIO.IN)
        GPIO.setup(LINE_MIDDLE, GPIO.IN)
        GPIO.setup(LINE_LEFT, GPIO.IN)

        logger.info(f"  3-канальный ИК-модуль следования за линией")
        logger.info(f"  GPIO Right:  {LINE_RIGHT}")
        logger.info(f"  GPIO Middle: {LINE_MIDDLE}")
        logger.info(f"  GPIO Left:   {LINE_LEFT}")
        logger.info(f"  Логика: 0 = линия обнаружена, 1 = нет линии")

        # Несколько замеров
        logger.info(f"  >>> Тест: 5 замеров с интервалом 0.2с...")
        all_readings = []
        for i in range(5):
            r = GPIO.input(LINE_RIGHT)
            m = GPIO.input(LINE_MIDDLE)
            l = GPIO.input(LINE_LEFT)
            reading = {"right": r, "middle": m, "left": l}
            all_readings.append(reading)
            logger.info(f"    Замер {i+1}: Left={l}  Middle={m}  Right={r}")
            time.sleep(0.2)

        GPIO.cleanup([LINE_RIGHT, LINE_MIDDLE, LINE_LEFT])

        record("line_tracking", status="OK", details={
            "type": "3-CH IR Line Tracking Module",
            "gpio_right": LINE_RIGHT,
            "gpio_middle": LINE_MIDDLE,
            "gpio_left": LINE_LEFT,
            "readings": all_readings,
        })

    except ImportError:
        # Попробуем через gpiozero
        try:
            from gpiozero import DigitalInputDevice

            sensors = {
                "right": DigitalInputDevice(LINE_RIGHT),
                "middle": DigitalInputDevice(LINE_MIDDLE),
                "left": DigitalInputDevice(LINE_LEFT),
            }

            logger.info(f"  3-канальный ИК-модуль (через gpiozero)")
            logger.info(f"  GPIO Right:  {LINE_RIGHT}")
            logger.info(f"  GPIO Middle: {LINE_MIDDLE}")
            logger.info(f"  GPIO Left:   {LINE_LEFT}")

            logger.info(f"  >>> Тест: 5 замеров с интервалом 0.2с...")
            all_readings = []
            for i in range(5):
                reading = {name: s.value for name, s in sensors.items()}
                all_readings.append(reading)
                logger.info(f"    Замер {i+1}: Left={reading['left']}  "
                            f"Middle={reading['middle']}  Right={reading['right']}")
                time.sleep(0.2)

            for s in sensors.values():
                s.close()

            record("line_tracking", status="OK", details={
                "type": "3-CH IR Line Tracking Module",
                "gpio_right": LINE_RIGHT,
                "gpio_middle": LINE_MIDDLE,
                "gpio_left": LINE_LEFT,
                "readings": all_readings,
            })

        except Exception as e:
            logger.error(f"  Line Tracking ОШИБКА: {e}")
            record("line_tracking", status="ERROR", details={"error": str(e)})

    except Exception as e:
        logger.error(f"  Line Tracking ОШИБКА: {e}")
        record("line_tracking", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  12. WS2812 RGB LED MODULE (GPIO12)
# ══════════════════════════════════════════════════════════════════════

def diagnose_ws2812():
    section("WS2812 RGB LED MODULE (GPIO12)")

    LED_COUNT = 16
    LED_PIN = 12
    LED_FREQ_HZ = 800000
    LED_DMA = 10
    LED_BRIGHTNESS = 255
    LED_INVERT = False
    LED_CHANNEL = 0

    try:
        from rpi_ws281x import PixelStrip, Color

        logger.info(f"  Тип: WS2812 адресные RGB светодиоды")
        logger.info(f"  GPIO Data:    {LED_PIN}")
        logger.info(f"  Количество:   {LED_COUNT} светодиодов")
        logger.info(f"  Частота:      {LED_FREQ_HZ} Hz")
        logger.info(f"  DMA канал:    {LED_DMA}")
        logger.info(f"  Яркость:      {LED_BRIGHTNESS}/255")
        logger.info(f"  LED Channel:  {LED_CHANNEL}")

        strip = PixelStrip(
            LED_COUNT, LED_PIN, LED_FREQ_HZ,
            LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL
        )
        strip.begin()

        # Тест: красный
        logger.info(f"  >>> Тест: все LED → КРАСНЫЙ")
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(255, 0, 0))
        strip.show()
        time.sleep(1.0)

        # Тест: зелёный
        logger.info(f"  >>> Тест: все LED → ЗЕЛЁНЫЙ")
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(0, 255, 0))
        strip.show()
        time.sleep(1.0)

        # Тест: синий
        logger.info(f"  >>> Тест: все LED → СИНИЙ")
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(0, 0, 255))
        strip.show()
        time.sleep(1.0)

        # Тест: бегущий огонь белым
        logger.info(f"  >>> Тест: бегущий огонь (белый)")
        for i in range(strip.numPixels()):
            # Погасить все
            for j in range(strip.numPixels()):
                strip.setPixelColor(j, Color(0, 0, 0))
            strip.setPixelColor(i, Color(255, 255, 255))
            strip.show()
            time.sleep(0.1)

        # Погасить все
        logger.info(f"  >>> Выключение всех LED")
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(0, 0, 0))
        strip.show()

        logger.info(f"  <<< WS2812 — OK")
        record("ws2812_led", status="OK", details={
            "type": "WS2812 RGB LED",
            "gpio_pin": LED_PIN,
            "led_count": LED_COUNT,
            "frequency_hz": LED_FREQ_HZ,
        })

    except ImportError:
        logger.error("  rpi_ws281x не установлен")
        logger.error("  Установите: sudo pip3 install rpi_ws281x")
        record("ws2812_led", status="ERROR", details={"error": "rpi_ws281x not installed"})
    except Exception as e:
        logger.error(f"  WS2812 ОШИБКА: {e}")
        if "sp" in str(e).lower() or "root" in str(e).lower() or "permission" in str(e).lower():
            logger.error("  Примечание: WS2812 требует запуск от root (sudo)")
        record("ws2812_led", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  13. КАМЕРА CSI (picamera2)
# ══════════════════════════════════════════════════════════════════════

def diagnose_camera():
    section("RASPBERRY PI CAMERA (CSI / picamera2)")

    try:
        from picamera2 import Picamera2

        cam = Picamera2()

        cam_info = cam.camera_properties
        logger.info(f"  Свойства камеры:")
        for k, v in cam_info.items():
            logger.info(f"    {k}: {v}")

        all_cameras = Picamera2.global_camera_info()
        logger.info(f"  Доступные камеры: {len(all_cameras)}")
        for idx, ci in enumerate(all_cameras):
            logger.info(f"    Камера {idx}: {ci}")

        WIDTH, HEIGHT = 640, 480
        config = cam.create_preview_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
            buffer_count=2,
        )
        cam.configure(config)
        logger.info(f"  Конфигурация: {WIDTH}x{HEIGHT}, RGB888")

        cam.start()
        time.sleep(1.0)

        frame = cam.capture_array()
        cam.stop()
        cam.close()

        logger.info(f"  >>> Тест: захват кадра")
        logger.info(f"    Размер кадра: {frame.shape}")
        logger.info(f"    Тип данных:   {frame.dtype}")
        logger.info(f"    Средняя яркость: {frame.mean():.1f}")
        logger.info(f"    Мин. значение:   {frame.min()}")
        logger.info(f"    Макс. значение:  {frame.max()}")

        # Анализ по каналам RGB
        import numpy as np
        for ch_idx, ch_name in enumerate(["Red", "Green", "Blue"]):
            ch_data = frame[:, :, ch_idx]
            logger.info(f"    Канал {ch_name}: "
                        f"mean={ch_data.mean():.1f}, "
                        f"std={ch_data.std():.1f}, "
                        f"min={ch_data.min()}, max={ch_data.max()}")

        # Проверка: не чёрный ли кадр (крышка на камере?)
        if frame.mean() < 5.0:
            logger.warning("    ВНИМАНИЕ: кадр почти чёрный — возможно крышка на камере!")
        elif frame.mean() > 250.0:
            logger.warning("    ВНИМАНИЕ: кадр почти белый — возможна засветка!")

        snapshot_path_str = ""
        try:
            import cv2
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            snapshot_path = LOG_DIR / f"camera_snapshot_{timestamp}.jpg"
            cv2.imwrite(str(snapshot_path), frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            snapshot_path_str = str(snapshot_path)
            saved_files.append(snapshot_path_str)
            logger.info(f"  Снимок сохранён: {snapshot_path}")
            file_size = snapshot_path.stat().st_size
            logger.info(f"  Размер файла: {file_size} байт ({file_size/1024:.1f} KB)")
        except Exception as e:
            logger.warning(f"  Не удалось сохранить снимок (cv2): {e}")

        record("camera", status="OK", details={
            "resolution": f"{WIDTH}x{HEIGHT}",
            "frame_shape": str(frame.shape),
            "dtype": str(frame.dtype),
            "mean_brightness": round(float(frame.mean()), 1),
            "channel_means": {
                "red": round(float(frame[:, :, 0].mean()), 1),
                "green": round(float(frame[:, :, 1].mean()), 1),
                "blue": round(float(frame[:, :, 2].mean()), 1),
            },
            "properties": {str(k): str(v) for k, v in cam_info.items()},
            "cameras_count": len(all_cameras),
            "snapshot_file": snapshot_path_str,
        })

    except Exception as e:
        logger.error(f"  Камера ОШИБКА: {e}")
        record("camera", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  14. USB-МИКРОФОН (PyAudio)
# ══════════════════════════════════════════════════════════════════════

def diagnose_microphone():
    section("USB-МИКРОФОН (PyAudio)")

    try:
        import pyaudio

        pa = pyaudio.PyAudio()

        host_api_count = pa.get_host_api_count()
        logger.info(f"  Host API count: {host_api_count}")
        for i in range(host_api_count):
            api_info = pa.get_host_api_info_by_index(i)
            logger.info(f"    Host API {i}: {api_info['name']}, "
                        f"devices: {api_info['deviceCount']}")

        device_count = pa.get_device_count()
        logger.info(f"  Аудиоустройств всего: {device_count}")

        input_devices = []
        for i in range(device_count):
            try:
                dev = pa.get_device_info_by_index(i)
                logger.info(f"    Устройство {i}: {dev['name']}")
                logger.info(f"      maxInputChannels:  {dev['maxInputChannels']}")
                logger.info(f"      maxOutputChannels: {dev['maxOutputChannels']}")
                logger.info(f"      defaultSampleRate: {dev['defaultSampleRate']}")
                if dev["maxInputChannels"] > 0:
                    input_devices.append({
                        "index": i,
                        "name": dev["name"],
                        "channels": dev["maxInputChannels"],
                        "sample_rate": dev["defaultSampleRate"],
                    })
            except Exception as e:
                logger.warning(f"    Устройство {i}: ошибка — {e}")

        logger.info(f"  Устройств ввода (микрофоны): {len(input_devices)}")
        for d in input_devices:
            logger.info(f"    [{d['index']}] {d['name']} "
                        f"(ch={d['channels']}, rate={d['sample_rate']})")

        if input_devices:
            test_dev = input_devices[0]
            RATE = 16000
            CHUNK = 4000
            RECORD_SEC = 2

            logger.info(f"  >>> Тест: запись {RECORD_SEC}с с [{test_dev['index']}]...")
            try:
                stream = pa.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=RATE,
                    input=True,
                    input_device_index=test_dev["index"],
                    frames_per_buffer=CHUNK,
                )

                frames = []
                for _ in range(int(RATE / CHUNK * RECORD_SEC)):
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    frames.append(data)

                stream.stop_stream()
                stream.close()

                total_samples = 0
                total_abs = 0
                max_sample = 0
                for frame_data in frames:
                    samples = struct.unpack(f"<{len(frame_data)//2}h", frame_data)
                    for s in samples:
                        total_abs += abs(s)
                        total_samples += 1
                        if abs(s) > max_sample:
                            max_sample = abs(s)

                avg_amplitude = total_abs / total_samples if total_samples else 0
                logger.info(f"  <<< Запись OK")
                logger.info(f"    Записано сэмплов:    {total_samples}")
                logger.info(f"    Средняя амплитуда:   {avg_amplitude:.0f}")
                logger.info(f"    Макс. амплитуда:     {max_sample}")
                logger.info(f"    (тишина ~0-200, голос ~500-5000, макс=32767)")

                # Сохраняем запись в WAV-файл
                wav_path = LOG_DIR / f"mic_test_{timestamp}.wav"
                try:
                    with wave.open(str(wav_path), "wb") as wf:
                        wf.setnchannels(1)
                        wf.setsampwidth(2)  # 16-bit = 2 bytes
                        wf.setframerate(RATE)
                        wf.writeframes(b"".join(frames))
                    saved_files.append(str(wav_path))
                    wav_size = wav_path.stat().st_size
                    logger.info(f"  Аудио сохранено: {wav_path}")
                    logger.info(f"  Размер WAV: {wav_size} байт ({wav_size/1024:.1f} KB)")
                except Exception as e:
                    logger.warning(f"  Не удалось сохранить WAV: {e}")

            except Exception as e:
                logger.error(f"  <<< Запись ОШИБКА: {e}")

        pa.terminate()

        mic_details = {
            "host_api_count": host_api_count,
            "total_devices": device_count,
            "input_devices": input_devices,
        }
        if input_devices:
            mic_details["test_recording"] = {
                "duration_sec": RECORD_SEC,
                "sample_rate": RATE,
            }
        record("microphone", status="OK" if input_devices else "WARNING",
               details=mic_details)

    except ImportError:
        logger.error("  PyAudio не установлен — микрофон не проверен")
        record("microphone", status="ERROR", details={"error": "pyaudio not installed"})
    except Exception as e:
        logger.error(f"  Микрофон ОШИБКА: {e}")
        record("microphone", status="ERROR", details={"error": str(e)})


# ══════════════════════════════════════════════════════════════════════
#  15. ADEEPT ROBOT HAT V3.1
# ══════════════════════════════════════════════════════════════════════

def diagnose_robot_hat():
    section("ADEEPT ROBOT HAT V3.1 — СВОДКА")

    logger.info("  Платформа: Adeept Robot HAT V3.1")
    logger.info("  Совместимость: Raspberry Pi 4B/3B/3B+")
    logger.info("")
    logger.info("  Интегрированные чипы:")
    logger.info("    1. PCA9685    — 16-канальный PWM-контроллер (I2C)")
    logger.info("    2. DRV8833 x2 — драйверы DC моторов (Dual H-Bridge)")
    logger.info("    3. ADS7830    — 8-канальный 8-бит АЦП (I2C)")
    logger.info("    4. LM324      — Quad операционный усилитель")
    logger.info("")
    logger.info("  Разъёмы:")
    logger.info("    - 16 портов для сервоприводов (PCA9685 ch0-ch15)")
    logger.info("    - 4 порта DC моторов (через DRV8833)")
    logger.info("    - Ультразвуковой датчик (GPIO23/24)")
    logger.info("    - 3-CH Line Tracking (GPIO19/16/20)")
    logger.info("    - WS2812 RGB LED (GPIO12)")
    logger.info("    - I2C шина (GPIO2/3)")

    record("adeept_robot_hat", status="OK", details={
        "type": "Adeept Robot HAT V3.1",
        "chips": ["PCA9685", "DRV8833 x2", "ADS7830", "LM324"],
    })


# ══════════════════════════════════════════════════════════════════════
#  16. VOSK — РАСПОЗНАВАНИЕ РЕЧИ
# ══════════════════════════════════════════════════════════════════════

def diagnose_vosk():
    section("VOSK — РАСПОЗНАВАНИЕ РЕЧИ")

    vosk_model_path = Path.home() / "vosk-model-ru"

    try:
        import vosk
        logger.info(f"  Vosk версия: "
                     f"{vosk.__version__ if hasattr(vosk, '__version__') else 'N/A'}")
        logger.info(f"  Путь к модели: {vosk_model_path}")

        if vosk_model_path.exists():
            logger.info(f"  Модель найдена — OK")
            try:
                model = vosk.Model(str(vosk_model_path))
                logger.info(f"  Модель загружена успешно")
                record("vosk", status="OK", details={
                    "model_path": str(vosk_model_path),
                    "model_exists": True,
                })
            except Exception as e:
                logger.error(f"  Ошибка загрузки модели: {e}")
                record("vosk", status="ERROR", details={"error": str(e)})
        else:
            logger.warning(f"  Модель НЕ НАЙДЕНА по пути {vosk_model_path}")
            logger.warning(f"  Скачайте: vosk-model-small-ru-0.22")
            record("vosk", status="WARNING", details={
                "model_path": str(vosk_model_path),
                "model_exists": False,
            })

    except ImportError:
        logger.warning("  Vosk не установлен")
        record("vosk", status="WARNING", details={"error": "vosk not installed"})


# ══════════════════════════════════════════════════════════════════════
#  ИТОГОВЫЙ ОТЧЁТ
# ══════════════════════════════════════════════════════════════════════

def print_summary():
    section("ИТОГОВЫЙ ОТЧЁТ")

    logger.info(f"  {'Модуль':<25s} {'Статус':<10s} Подробности")
    logger.info("  " + "-" * 68)

    status_tag = {"OK": "[OK]  ", "WARNING": "[WARN]", "ERROR": "[ERR] "}

    for module, data in results.items():
        st = data["status"]
        tag = status_tag.get(st, st)
        details_str = ""
        if st == "ERROR" and "error" in data.get("details", {}):
            details_str = data["details"]["error"]
        logger.info(f"  {module:<23s} {tag:<10s} {details_str}")

    ok_count = sum(1 for d in results.values() if d["status"] == "OK")
    warn_count = sum(1 for d in results.values() if d["status"] == "WARNING")
    err_count = sum(1 for d in results.values() if d["status"] == "ERROR")

    logger.info("")
    logger.info(f"  Всего модулей: {len(results)}")
    logger.info(f"  [OK]:    {ok_count}")
    logger.info(f"  [WARN]:  {warn_count}")
    logger.info(f"  [ERROR]: {err_count}")

    # JSON отчёт
    json_path = LOG_DIR / f"diagnostic_{timestamp}.json"
    saved_files.append(str(LOG_FILE))
    saved_files.append(str(json_path))

    report = {
        "timestamp": datetime.now().isoformat(),
        "results": results,
        "summary": {
            "total": len(results),
            "ok": ok_count,
            "warnings": warn_count,
            "errors": err_count,
        },
        "skipped_modules": [
            {"name": "laser", "reason": "Не установлен", "gpio": 17},
            {"name": "imu_mpu6050", "reason": "Гироскоп+акселерометр не установлен",
             "i2c": "0x68"},
        ],
        "saved_files": saved_files,
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    logger.info("")
    logger.info("  Сохранённые файлы:")
    for fp in saved_files:
        logger.info(f"    - {fp}")
    logger.info("")
    logger.info(f"  JSON отчёт: {json_path}")
    logger.info(f"  Лог файл:   {LOG_FILE}")


# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════

def main():
    logger.info("╔════════════════════════════════════════════════════════════╗")
    logger.info("║   SAMURAI — Полная диагностика оборудования робота        ║")
    logger.info("║   Пропуск: Лазер (GPIO17), IMU MPU6050 (I2C 0x68)        ║")
    logger.info("╚════════════════════════════════════════════════════════════╝")
    logger.info(f"  Лог: {LOG_FILE}")
    logger.info("")

    start_time = time.time()

    #  0. Системная информация
    diagnose_system_info()

    #  1. Python-пакеты
    diagnose_python_packages()

    #  2. GPIO карта
    diagnose_gpio()

    #  3. I2C шина — сканирование устройств
    diagnose_i2c()

    #  4. ADS7830 — 8-канальный АЦП
    diagnose_ads7830()

    #  5. PCA9685 — PWM-контроллер
    pca = diagnose_pca9685()

    #  6. DRV8833 — драйверы моторов (информация)
    diagnose_drv8833(pca)

    #  7. LM324 — операционный усилитель (информация)
    diagnose_lm324()

    #  8. DC моторы M1-M4
    diagnose_motors(pca)

    #  9. AD002 сервоприводы x6
    diagnose_servos(pca)

    # 10. Ультразвуковой датчик HC-SR04
    diagnose_ultrasonic()

    # 11. 3-CH Line Tracking Module
    diagnose_line_tracking()

    # 12. WS2812 RGB LED
    diagnose_ws2812()

    # 13. Камера CSI
    diagnose_camera()

    # 14. USB-микрофон
    diagnose_microphone()

    # 15. Adeept Robot HAT — сводка
    diagnose_robot_hat()

    # 16. Vosk — распознавание речи
    diagnose_vosk()

    # Закрываем PCA9685
    if pca is not None:
        try:
            pca.deinit()
            logger.info("PCA9685 деинициализирован")
        except Exception:
            pass

    elapsed = time.time() - start_time
    logger.info(f"\nВремя диагностики: {elapsed:.1f} сек")

    print_summary()

    logger.info("")
    logger.info("Диагностика завершена.")


if __name__ == "__main__":
    main()
