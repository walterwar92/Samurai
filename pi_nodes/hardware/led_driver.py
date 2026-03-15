"""
WS2812B NeoPixel LED driver для Adeept Robot HAT V3.1.

GPIO 10 (SPI0_MOSI, board pin 19) — пин для данного HAT (проверено на Adeept HAT V3.1).
По умолчанию 8 светодиодов, адресуемые RGB.

Зависимости (на Raspberry Pi):
    sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel
    # или только rpi_ws281x (если не нужна адафрут-обёртка)

Библиотека rpi_ws281x требует запуска от root или настройки:
    sudo setcap cap_sys_rawio+ep /usr/bin/python3
    echo 'SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules
"""

import logging
import time
from typing import Tuple

_log = logging.getLogger(__name__)

# ---------- Попытка импорта аппаратных библиотек ----------
try:
    import board
    import neopixel
    _HW_NEOPIXEL = True
except (ImportError, NotImplementedError, RuntimeError):
    _HW_NEOPIXEL = False

# GPIO пин и количество светодиодов (Adeept HAT V3.1)
LED_PIN_BOARD = 'D10'   # GPIO 10 = board.D10 (Adeept HAT V3.1 LED разъём, SPI0_MOSI)
LED_COUNT_DEFAULT = 12  # 4 панели × 3 диода
LED_BRIGHTNESS_DEFAULT = 0.3  # 0.0–1.0 (держим умеренно, не слепит)

Color = Tuple[int, int, int]  # (R, G, B), каждый 0–255

# ---------- Предустановленные цвета ----------
COLORS = {
    'red':     (255, 0,   0),
    'green':   (0,   255, 0),
    'blue':    (0,   0,   255),
    'yellow':  (255, 255, 0),
    'cyan':    (0,   255, 255),
    'magenta': (255, 0,   255),
    'white':   (255, 255, 255),
    'orange':  (255, 80,  0),
    'off':     (0,   0,   0),
}


class _FakeLEDs:
    """Stub-реализация для запуска без аппаратуры."""

    def __init__(self, count: int):
        self._pixels = [(0, 0, 0)] * count
        self.n = count

    def __setitem__(self, idx, color):
        if isinstance(idx, slice):
            for i in range(*idx.indices(self.n)):
                self._pixels[i] = color
        else:
            self._pixels[idx] = color

    def __getitem__(self, idx):
        return self._pixels[idx]

    def fill(self, color):
        for i in range(self.n):
            self._pixels[i] = color

    def show(self):
        colors_str = ', '.join(f'#{r:02x}{g:02x}{b:02x}' for r, g, b in self._pixels)
        _log.debug('LED [sim] %s', colors_str)

    def deinit(self):
        pass


class LedDriver:
    """Управляет адресуемыми RGB-светодиодами WS2812B."""

    def __init__(
        self,
        count: int = LED_COUNT_DEFAULT,
        brightness: float = LED_BRIGHTNESS_DEFAULT,
        auto_write: bool = False,
    ):
        self._count = count
        self._simulated = False

        if _HW_NEOPIXEL:
            try:
                pin = getattr(board, LED_PIN_BOARD)
                self._pixels = neopixel.NeoPixel(
                    pin,
                    count,
                    brightness=brightness,
                    auto_write=auto_write,
                    pixel_order=neopixel.GRB,  # WS2812B — GRB порядок
                )
                _log.info('LED: инициализировано %d WS2812B на %s', count, LED_PIN_BOARD)
            except Exception as exc:
                _log.warning('LED: аппаратура недоступна (%s) — симуляция', exc)
                self._pixels = _FakeLEDs(count)
                self._simulated = True
        else:
            _log.warning('LED: библиотека neopixel не найдена — симуляция')
            self._pixels = _FakeLEDs(count)
            self._simulated = True

    # ------------------------------------------------------------------
    @property
    def simulated(self) -> bool:
        return self._simulated

    @property
    def count(self) -> int:
        return self._count

    # ------------------------------------------------------------------
    def set_all(self, color: Color):
        """Установить один цвет для всех диодов и обновить."""
        self._pixels.fill(color)
        self._pixels.show()

    def set_one(self, index: int, color: Color):
        """Установить цвет одного диода (0-based)."""
        if 0 <= index < self._count:
            self._pixels[index] = color
            self._pixels.show()

    def set_range(self, start: int, end: int, color: Color):
        """Установить цвет диодов с start до end (не включая end)."""
        for i in range(max(0, start), min(end, self._count)):
            self._pixels[i] = color
        self._pixels.show()

    def set_colors(self, colors: list[Color]):
        """Установить список цветов (len может быть меньше count)."""
        for i, c in enumerate(colors[:self._count]):
            self._pixels[i] = c
        self._pixels.show()

    def off(self):
        """Выключить все диоды."""
        self.set_all((0, 0, 0))

    # ------------------------------------------------------------------
    # Встроенные эффекты
    # ------------------------------------------------------------------
    def blink(self, color: Color, times: int = 3, on_sec: float = 0.2, off_sec: float = 0.2):
        """Мигнуть указанным цветом N раз."""
        for _ in range(times):
            self.set_all(color)
            time.sleep(on_sec)
            self.off()
            time.sleep(off_sec)

    def pulse(self, color: Color, steps: int = 20, delay: float = 0.03):
        """Плавное нарастание/угасание (только аппаратный brightness)."""
        if self._simulated:
            self.set_all(color)
            time.sleep(steps * delay * 2)
            self.off()
            return

        r, g, b = color
        # Нарастание
        for i in range(steps + 1):
            k = i / steps
            self._pixels.fill((int(r * k), int(g * k), int(b * k)))
            self._pixels.show()
            time.sleep(delay)
        # Угасание
        for i in range(steps, -1, -1):
            k = i / steps
            self._pixels.fill((int(r * k), int(g * k), int(b * k)))
            self._pixels.show()
            time.sleep(delay)

    def rainbow_cycle(self, wait: float = 0.02, cycles: int = 1):
        """Проход по цветовому кругу для всех диодов."""
        for _ in range(cycles):
            for j in range(256):
                for i in range(self._count):
                    hue = (i * 256 // self._count + j) & 255
                    self._pixels[i] = _wheel(hue)
                self._pixels.show()
                time.sleep(wait)

    # ------------------------------------------------------------------
    def deinit(self):
        self.off()
        self._pixels.deinit()


# ------------------------------------------------------------------
def _wheel(pos: int) -> Color:
    """Преобразует 0-255 в цвет радуги (RGB)."""
    pos = pos & 255
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)
