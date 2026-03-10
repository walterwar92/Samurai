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
import subprocess
from datetime import datetime
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────
# Настройка логирования — всё сохраняется рядом со скриптом
# ──────────────────────────────────────────────────────────────────────

SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR / "diagnostics_output"
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

# Флаг: I2C был выключен до запуска диагностики и мы его включили
_i2c_was_disabled: bool = False


# ──────────────────────────────────────────────────────────────────────
# Проверка и включение I2C
# ──────────────────────────────────────────────────────────────────────

def ensure_i2c() -> bool:
    """Проверяет, что I2C включён. Если выключен — включает на время диагностики.

    Проверяет наличие /dev/i2c-1, загружает модуль ядра i2c-dev,
    при необходимости включает I2C через raspi-config.
    Запоминает исходное состояние, чтобы отключить в конце (restore_i2c).
    Возвращает True если I2C доступен, False если требуется перезагрузка.
    """
    global _i2c_was_disabled
    section("ПРОВЕРКА I2C ИНТЕРФЕЙСА")

    # 1. Быстрая проверка — /dev/i2c-1 уже есть?
    if os.path.exists('/dev/i2c-1'):
        logger.info("  /dev/i2c-1 — найден, I2C уже активен")
        _i2c_was_disabled = False
        record("i2c_enable", status="OK", details={"action": "already enabled"})
        return True

    logger.warning("  /dev/i2c-1 НЕ НАЙДЕН — попытка включения I2C...")

    # 2. Загрузка модуля ядра
    try:
        result = subprocess.run(
            ['modprobe', 'i2c-dev'],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            logger.info("  modprobe i2c-dev — выполнено")
        else:
            logger.warning(f"  modprobe i2c-dev — ошибка: {result.stderr.strip()}")
    except Exception as e:
        logger.warning(f"  modprobe i2c-dev — не удалось: {e}")

    if os.path.exists('/dev/i2c-1'):
        logger.info("  /dev/i2c-1 появился после modprobe — OK")
        _i2c_was_disabled = True
        record("i2c_enable", status="OK", details={"action": "modprobe i2c-dev"})
        return True

    # 3. Проверяем raspi-config
    try:
        result = subprocess.run(
            ['raspi-config', 'nonint', 'get_i2c'],
            capture_output=True, text=True, timeout=5,
        )
        i2c_status = result.stdout.strip()
        # raspi-config get_i2c: "1" = выключен, "0" = включён
        if i2c_status == '1':
            logger.warning("  I2C выключен в raspi-config — включаем...")
            _i2c_was_disabled = True
            enable_result = subprocess.run(
                ['raspi-config', 'nonint', 'do_i2c', '0'],
                capture_output=True, text=True, timeout=10,
            )
            if enable_result.returncode == 0:
                logger.info("  raspi-config do_i2c 0 — выполнено")
            else:
                logger.error(f"  raspi-config do_i2c — ошибка: "
                             f"{enable_result.stderr.strip()}")

            # Повторная загрузка модуля
            subprocess.run(
                ['modprobe', 'i2c-dev'],
                capture_output=True, timeout=5,
            )

            if os.path.exists('/dev/i2c-1'):
                logger.info("  I2C включён успешно (будет выключен после диагностики)")
                record("i2c_enable", status="OK",
                       details={"action": "enabled via raspi-config + modprobe"})
                return True
            else:
                logger.error("  I2C включён в raspi-config, но /dev/i2c-1 "
                             "не появился — ТРЕБУЕТСЯ ПЕРЕЗАГРУЗКА")
                logger.error("  Выполните: sudo reboot")
                record("i2c_enable", status="ERROR",
                       details={"action": "enabled, reboot required"})
                return False
        else:
            logger.warning("  raspi-config: I2C включён, но /dev/i2c-1 "
                           "отсутствует — ТРЕБУЕТСЯ ПЕРЕЗАГРУЗКА")
            record("i2c_enable", status="ERROR",
                   details={"action": "enabled in config, reboot required"})
            return False

    except FileNotFoundError:
        logger.warning("  raspi-config не найден (не Raspberry Pi?)")
        record("i2c_enable", status="WARNING",
               details={"action": "raspi-config not found"})
        return False
    except Exception as e:
        logger.error(f"  Ошибка проверки raspi-config: {e}")
        record("i2c_enable", status="ERROR", details={"error": str(e)})
        return False


def restore_i2c():
    """Выключает I2C если он был выключен до запуска диагностики."""
    if not _i2c_was_disabled:
        logger.info("I2C: был включён до диагностики — оставляем как есть")
        return

    section("ВОССТАНОВЛЕНИЕ СОСТОЯНИЯ I2C")
    logger.info("  I2C был выключен до диагностики — выключаем обратно...")

    # Выгружаем модуль ядра
    try:
        subprocess.run(
            ['rmmod', 'i2c-dev'],
            capture_output=True, text=True, timeout=5,
        )
        logger.info("  rmmod i2c-dev — выполнено")
    except Exception as e:
        logger.warning(f"  rmmod i2c-dev — не удалось: {e}")

    # Выключаем I2C через raspi-config
    try:
        result = subprocess.run(
            ['raspi-config', 'nonint', 'do_i2c', '1'],  # 1 = disable
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode == 0:
            logger.info("  raspi-config do_i2c 1 (disable) — выполнено")
        else:
            logger.warning(f"  raspi-config do_i2c 1 — ошибка: "
                           f"{result.stderr.strip()}")
    except FileNotFoundError:
        logger.warning("  raspi-config не найден — пропуск выключения")
    except Exception as e:
        logger.warning(f"  Ошибка выключения I2C: {e}")

    logger.info("  I2C восстановлен в исходное состояние (выключен)")


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
        "smbus2",
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

# ── smbus2-based PCA9685 fallback ────────────────────────────────────

class PCA9685Smbus:
    """Минимальная реализация PCA9685 через smbus2 (без adafruit-blinka)."""

    _MODE1 = 0x00
    _MODE2 = 0x01
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self._bus = bus
        self._address = address
        self.frequency = 50
        # Reset
        self._bus.write_byte_data(self._address, self._MODE1, 0x00)

    @property
    def frequency(self):
        return self._freq

    @frequency.setter
    def frequency(self, freq):
        self._freq = freq
        prescale = int(round(25_000_000.0 / (4096 * freq)) - 1)
        old_mode = self._bus.read_byte_data(self._address, self._MODE1)
        # Sleep
        self._bus.write_byte_data(self._address, self._MODE1, (old_mode & 0x7F) | 0x10)
        self._bus.write_byte_data(self._address, self._PRESCALE, prescale)
        self._bus.write_byte_data(self._address, self._MODE1, old_mode)
        time.sleep(0.005)
        self._bus.write_byte_data(self._address, self._MODE1, old_mode | 0xA0)

    def set_pwm(self, channel, on, off):
        reg = self._LED0_ON_L + 4 * channel
        self._bus.write_byte_data(self._address, reg, on & 0xFF)
        self._bus.write_byte_data(self._address, reg + 1, on >> 8)
        self._bus.write_byte_data(self._address, reg + 2, off & 0xFF)
        self._bus.write_byte_data(self._address, reg + 3, off >> 8)

    def get_pwm(self, channel):
        reg = self._LED0_ON_L + 4 * channel
        on_l = self._bus.read_byte_data(self._address, reg)
        on_h = self._bus.read_byte_data(self._address, reg + 1)
        off_l = self._bus.read_byte_data(self._address, reg + 2)
        off_h = self._bus.read_byte_data(self._address, reg + 3)
        return (on_h << 8 | on_l, off_h << 8 | off_l)

    def set_throttle(self, channel_in1, channel_in2, throttle):
        """Управление DC-мотором через 2 PWM-канала (DRV8833 slow decay)."""
        throttle = max(-1.0, min(1.0, throttle))
        duty = int(abs(throttle) * 0xFFFF)
        duty12 = duty >> 4  # 16-bit → 12-bit
        if throttle > 0:
            self.set_pwm(channel_in1, 0, duty12)
            self.set_pwm(channel_in2, 0, 0)
        elif throttle < 0:
            self.set_pwm(channel_in1, 0, 0)
            self.set_pwm(channel_in2, 0, duty12)
        else:
            self.set_pwm(channel_in1, 0, 0)
            self.set_pwm(channel_in2, 0, 0)

    def set_servo_angle(self, channel, angle, min_pulse=500, max_pulse=2400):
        """Установка угла сервопривода (0-180°)."""
        pulse_us = min_pulse + (max_pulse - min_pulse) * angle / 180.0
        # При частоте 50 Hz период = 20000 мкс, 12-bit = 4096 шагов
        duty = int(pulse_us / 20000.0 * 4096)
        self.set_pwm(channel, 0, duty)

    def deinit(self):
        # Все каналы выключить
        for ch in range(16):
            self.set_pwm(ch, 0, 0)


def diagnose_pca9685():
    global pca_driver
    section("PCA9685 PWM-КОНТРОЛЛЕР (I2C)")

    ADDRS_TO_TRY = [0x5F, 0x40]

    # Попытка 1: adafruit-blinka (полная библиотека)
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
                logger.info(f"  PCA9685 найден на адресе {hex(addr)} (adafruit)")
                break
            except Exception:
                continue

        if pca is None:
            raise RuntimeError("PCA9685 not found via adafruit")

        pca_driver = pca
        _log_pca9685_info(hex(used_addr), pca.frequency, "adafruit-blinka")

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
            "driver": "adafruit-blinka",
        })
        return pca

    except Exception as e_adafruit:
        logger.warning(f"  adafruit-blinka недоступен: {e_adafruit}")
        logger.info(f"  Попытка инициализации через smbus2...")

    # Попытка 2: smbus2 fallback
    try:
        import smbus2

        bus = smbus2.SMBus(1)
        pca = None
        used_addr = None

        for addr in ADDRS_TO_TRY:
            try:
                bus.read_byte(addr)
                pca = PCA9685Smbus(bus, address=addr)
                used_addr = addr
                logger.info(f"  PCA9685 найден на адресе {hex(addr)} (smbus2)")
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
        _log_pca9685_info(hex(used_addr), pca.frequency, "smbus2")

        for ch_num in range(16):
            try:
                on_val, off_val = pca.get_pwm(ch_num)
                logger.debug(f"  Канал {ch_num:2d}: ON={on_val}, OFF={off_val}")
            except Exception as e:
                logger.debug(f"  Канал {ch_num:2d}: ошибка чтения — {e}")

        record("pca9685", status="OK", details={
            "address": hex(used_addr),
            "frequency_hz": pca.frequency,
            "channels": 16,
            "driver": "smbus2-fallback",
        })
        return pca

    except Exception as e:
        logger.error(f"  PCA9685 ОШИБКА: {e}")
        record("pca9685", status="ERROR", details={"error": str(e)})
        return None


def _log_pca9685_info(addr_hex: str, freq: int, driver: str):
    logger.info(f"  PCA9685 инициализирован успешно (драйвер: {driver})")
    logger.info(f"  I2C адрес: {addr_hex}")
    logger.info(f"  Частота PWM: {freq} Hz")
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

    MOTOR_MAP = {
        "M1_right_front": (15, 14),
        "M2_left_front": (12, 13),
        "M3_left_rear": (11, 10),
        "M4_right_rear": (8, 9),
    }

    is_smbus = isinstance(pca, PCA9685Smbus)

    try:
        if is_smbus:
            # smbus2-фоллбэк: используем PCA9685Smbus напрямую
            motor_channels = {}
            for name, (ch_in1, ch_in2) in MOTOR_MAP.items():
                motor_channels[name] = (ch_in1, ch_in2)
                logger.info(f"  {name}: ch_IN1={ch_in1}, ch_IN2={ch_in2} — OK (smbus2)")
        else:
            from adafruit_motor import motor as adafruit_motor

            motor_channels = {}
            motors_adafruit = {}
            for name, (ch_in1, ch_in2) in MOTOR_MAP.items():
                try:
                    m = adafruit_motor.DCMotor(pca.channels[ch_in1], pca.channels[ch_in2])
                    m.decay_mode = adafruit_motor.SLOW_DECAY
                    motors_adafruit[name] = m
                    motor_channels[name] = (ch_in1, ch_in2)
                    logger.info(f"  {name}: ch_IN1={ch_in1}, ch_IN2={ch_in2} — OK")
                except Exception as e:
                    logger.error(f"  {name}: ch_IN1={ch_in1}, ch_IN2={ch_in2} — ОШИБКА: {e}")

        if not motor_channels:
            record("motors", status="ERROR", details={"error": "No motors initialized"})
            return None

        SPEED = 0.35
        MOVE_TIME = 1.0
        move_log = []

        def set_throttle(name, throttle):
            if is_smbus:
                ch_in1, ch_in2 = MOTOR_MAP[name]
                pca.set_throttle(ch_in1, ch_in2, throttle)
            else:
                motors_adafruit[name].throttle = throttle

        def set_all(throttle):
            for name in motor_channels:
                try:
                    set_throttle(name, throttle)
                except Exception as e:
                    logger.error(f"  throttle error: {e}")

        def set_sides(left, right):
            for name in motor_channels:
                try:
                    set_throttle(name, left if "left" in name else right)
                except Exception as e:
                    logger.error(f"  {name} throttle error: {e}")

        def stop_all():
            for name in motor_channels:
                try:
                    set_throttle(name, 0)
                except Exception:
                    pass

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

        logger.info("")
        logger.info("  --- Тест каждого мотора по отдельности ---")
        individual_results = {}
        for name in motor_channels:
            try:
                logger.info(f"  >>> {name}: вперёд 0.5с")
                set_throttle(name, SPEED)
                time.sleep(0.5)
                set_throttle(name, 0)
                time.sleep(0.3)

                logger.info(f"  >>> {name}: назад 0.5с")
                set_throttle(name, -SPEED)
                time.sleep(0.5)
                set_throttle(name, 0)
                time.sleep(0.3)

                individual_results[name] = "OK"
                logger.info(f"  <<< {name} — OK")
            except Exception as e:
                individual_results[name] = f"ERROR: {e}"
                logger.error(f"  <<< {name} — ОШИБКА: {e}")

        stop_all()
        record("motors", status="OK", details={
            "count": len(motor_channels),
            "names": list(motor_channels.keys()),
            "test_speed": SPEED,
            "driver": f"DRV8833 x2 via PCA9685 ({'smbus2' if is_smbus else 'adafruit'})",
            "movement_tests": move_log,
            "individual_tests": individual_results,
        })
        return motor_channels

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

    MIN_PULSE = 500
    MAX_PULSE = 2400
    ACTUATION_RANGE = 180

    # Для клешни: открытие (30°) → закрытие (130°) → исходное (90°)
    # Для остальных: отклонение вперёд (70°) → назад (110°) → исходное (90°)
    SERVO_CONFIG = {
        0: {"name": "Servo0 (клешня / gripper)", "init": 90,
            "test_steps": [(30,  "ОТКРЫТИЕ клешни"),
                           (130, "ЗАКРЫТИЕ клешни"),
                           (90,  "возврат в исходное")]},
        1: {"name": "Servo1 (рука / arm joint 1)", "init": 90,
            "test_steps": [(70,  "отклонение −20°"),
                           (110, "отклонение +20°"),
                           (90,  "возврат в исходное")]},
        2: {"name": "Servo2 (рука / arm joint 2)", "init": 90,
            "test_steps": [(70,  "отклонение −20°"),
                           (110, "отклонение +20°"),
                           (90,  "возврат в исходное")]},
        3: {"name": "Servo3 (рука / arm joint 3)", "init": 90,
            "test_steps": [(70,  "отклонение −20°"),
                           (110, "отклонение +20°"),
                           (90,  "возврат в исходное")]},
        4: {"name": "Servo4 (рука / arm joint 4)", "init": 90,
            "test_steps": [(70,  "отклонение −20°"),
                           (110, "отклонение +20°"),
                           (90,  "возврат в исходное")]},
        5: {"name": "Servo5 (рука / arm joint 5)", "init": 90,
            "test_steps": [(70,  "отклонение −20°"),
                           (110, "отклонение +20°"),
                           (90,  "возврат в исходное")]},
    }

    is_smbus = isinstance(pca, PCA9685Smbus)

    logger.info(f"  Тип: AD002 Servo Motor")
    logger.info(f"  Количество: 6 шт.")
    logger.info(f"  Диапазон пульса: {MIN_PULSE}-{MAX_PULSE} мкс")
    logger.info(f"  Диапазон угла: 0-{ACTUATION_RANGE}°")
    logger.info("")

    try:
        servo_results = {}

        # Adafruit: создаём объекты Servo ОДИН РАЗ на канал и держим живыми весь тест.
        # Иначе __del__ вызывает _pwm.deinit() и PWM-сигнал обрывается сразу после
        # каждого set_angle — серво двигается и тут же отпускается.
        adafruit_servos = {}
        if not is_smbus:
            try:
                from adafruit_motor import servo as adafruit_servo
                for ch in SERVO_CONFIG:
                    adafruit_servos[ch] = adafruit_servo.Servo(
                        pca.channels[ch],
                        min_pulse=MIN_PULSE,
                        max_pulse=MAX_PULSE,
                        actuation_range=ACTUATION_RANGE,
                    )
            except Exception as e:
                logger.error(f"  Не удалось создать объекты Servo: {e}")

        def set_angle(ch, angle):
            if is_smbus:
                pca.set_servo_angle(ch, angle, MIN_PULSE, MAX_PULSE)
            else:
                if ch in adafruit_servos:
                    adafruit_servos[ch].angle = angle

        for ch, cfg in SERVO_CONFIG.items():
            name = cfg["name"]
            try:
                logger.info(f"  [{ch}] {name}")
                logger.info(f"      PCA9685 канал: {ch}")
                logger.info(f"      Пульс: {MIN_PULSE}-{MAX_PULSE} мкс")

                init_angle = cfg["init"]
                set_angle(ch, init_angle)
                logger.info(f"      INIT → {init_angle}° (установлен)")
                time.sleep(0.5)

                tested_angles = []
                for angle, label in cfg["test_steps"]:
                    try:
                        set_angle(ch, angle)
                        tested_angles.append({"angle": angle, "label": label, "status": "OK"})
                        logger.info(f"      → {angle}°  [{label}] — OK")
                        time.sleep(0.6)
                    except Exception as e:
                        tested_angles.append({"angle": angle, "label": label,
                                              "status": "ERROR", "error": str(e)})
                        logger.error(f"      → {angle}°  [{label}] — ОШИБКА: {e}")

                # Явный возврат в исходное (на случай если последний шаг упал)
                set_angle(ch, init_angle)
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
        # gpiozero — работает на Trixie (lgpio) и Bookworm (pigpio/rpigpio)
        from gpiozero import DigitalInputDevice

        sensors = {
            "right": DigitalInputDevice(LINE_RIGHT),
            "middle": DigitalInputDevice(LINE_MIDDLE),
            "left": DigitalInputDevice(LINE_LEFT),
        }

        logger.info(f"  3-канальный ИК-модуль следования за линией")
        logger.info(f"  GPIO Right:  {LINE_RIGHT}")
        logger.info(f"  GPIO Middle: {LINE_MIDDLE}")
        logger.info(f"  GPIO Left:   {LINE_LEFT}")
        logger.info(f"  Логика: 0 = линия обнаружена, 1 = нет линии")

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

    except Exception as e_gpiozero:
        # Фоллбэк: RPi.GPIO (только Bookworm/старые версии)
        try:
            import RPi.GPIO as GPIO

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(LINE_RIGHT, GPIO.IN)
            GPIO.setup(LINE_MIDDLE, GPIO.IN)
            GPIO.setup(LINE_LEFT, GPIO.IN)

            logger.info(f"  3-канальный ИК-модуль (через RPi.GPIO)")
            logger.info(f"  GPIO Right:  {LINE_RIGHT}")
            logger.info(f"  GPIO Middle: {LINE_MIDDLE}")
            logger.info(f"  GPIO Left:   {LINE_LEFT}")
            logger.info(f"  Логика: 0 = линия обнаружена, 1 = нет линии")

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
    logger.info("║   Пропуск: Лазер (GPIO17), IMU MPU6050, микрофон, Vosk   ║")
    logger.info("╚════════════════════════════════════════════════════════════╝")
    logger.info(f"  Лог: {LOG_FILE}")
    logger.info("")

    start_time = time.time()

    #  Проверка I2C — без него I2C-устройства не заработают
    i2c_ok = ensure_i2c()
    if not i2c_ok:
        logger.error("I2C НЕДОСТУПЕН! I2C-зависимые тесты могут завершиться "
                     "с ошибками.")
        logger.error("Рекомендация: sudo reboot и повторный запуск диагностики.")

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

    # 14. Adeept Robot HAT — сводка
    diagnose_robot_hat()

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

    # Восстанавливаем I2C в исходное состояние (выключаем если был выключен)
    restore_i2c()

    logger.info("")
    logger.info("Диагностика завершена.")


if __name__ == "__main__":
    main()
