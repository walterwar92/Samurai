#!/usr/bin/env python3
"""
find_led_pin.py — перебор GPIO пинов для WS2812B (rpi_ws281x).

rpi_ws281x работает только на пинах с PWM/PCM/SPI:
    GPIO 12  (PWM0, board pin 32)  <-- Adeept HAT V3.1 по умолчанию
    GPIO 18  (PWM0, board pin 12)
    GPIO 13  (PWM1, board pin 33)
    GPIO 19  (PWM1, board pin 35)
    GPIO 10  (SPI MOSI, board pin 19)
    GPIO 21  (PCM DOUT, board pin 40)

Запуск (нужен root):
    sudo python3 find_led_pin.py [количество_диодов]

Скрипт включает все диоды БЕЛЫМ на каждом пине и ждёт Enter.
Как только диоды загорятся — запомните номер GPIO и нажмите 'y'.
"""

import sys
import time

LED_COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 8
BRIGHTNESS = 0.3
TEST_COLOR = (255, 255, 255)   # белый — виден при любом типе пикселей
HOLD_SEC   = 0.5               # сколько держать после включения перед паузой

# Все GPIO, которые поддерживает rpi_ws281x
CANDIDATE_PINS = [
    (12, 'PWM0', 32),
    (18, 'PWM0 alt', 12),
    (13, 'PWM1', 33),
    (19, 'PWM1 alt', 35),
    (10, 'SPI MOSI', 19),
    (21, 'PCM DOUT', 40),
]

# --------------------------------------------------------------------------
try:
    from rpi_ws281x import PixelStrip, Color as WsColor, WS2811_STRIP_GRB
except ImportError:
    print('ОШИБКА: rpi_ws281x не установлена.')
    print('Установите: sudo pip3 install rpi-ws281x')
    sys.exit(1)

# --------------------------------------------------------------------------
def try_pin(gpio: int, label: str, board_pin: int) -> bool:
    """
    Пробует инициализировать ленту на gpio, включает белый цвет.
    Возвращает True если пользователь подтвердил что диоды зажглись.
    """
    print()
    print(f'--- GPIO {gpio:2d}  ({label}, board pin {board_pin}) ---')

    strip = None
    try:
        strip = PixelStrip(LED_COUNT, gpio, 800000, 10, False, BRIGHTNESS, 0,
                           WS2811_STRIP_GRB)
        strip.begin()

        # Включить все диоды белым
        r, g, b = TEST_COLOR
        for i in range(LED_COUNT):
            strip.setPixelColor(i, WsColor(r, g, b))
        strip.show()
        print(f'  Лента инициализирована. Диоды должны светиться БЕЛЫМ.')
        time.sleep(HOLD_SEC)

    except Exception as exc:
        print(f'  Ошибка инициализации: {exc}')
        return False

    finally:
        # Выключить независимо от результата
        if strip is not None:
            try:
                for i in range(LED_COUNT):
                    strip.setPixelColor(i, WsColor(0, 0, 0))
                strip.show()
            except Exception:
                pass

    answer = input('  Диоды загорались? [y/n/q]: ').strip().lower()
    if answer == 'q':
        print('Выход.')
        sys.exit(0)
    return answer == 'y'


# --------------------------------------------------------------------------
def main():
    print('=' * 55)
    print(f'Поиск пина WS2812B. Диодов: {LED_COUNT}, яркость: {BRIGHTNESS}')
    print('На каждом пине лента включится на БЕЛЫЙ, затем выключится.')
    print("Нажмите 'y' если диоды зажглись, 'n' — нет, 'q' — выход.")
    print('=' * 55)

    found = []

    for gpio, label, board_pin in CANDIDATE_PINS:
        if try_pin(gpio, label, board_pin):
            found.append((gpio, label, board_pin))
            print(f'  [OK] GPIO {gpio} работает!')

    print()
    print('=' * 55)
    if found:
        print('Рабочие пины:')
        for gpio, label, board_pin in found:
            print(f'  GPIO {gpio:2d}  ({label}, board pin {board_pin})')
        best = found[0]
        print()
        print(f'Используйте в led_driver.py:')
        print(f"  LED_PIN_BOARD = 'D{best[0]}'")
    else:
        print('Ни один пин не сработал. Проверьте:')
        print('  1. Запущен ли скрипт через sudo?')
        print('  2. Подключены ли диоды к питанию 5V?')
        print('  3. Правильно ли подключён DATA-провод к Pi?')
        print('  4. Количество диодов (сейчас: %d)?' % LED_COUNT)


if __name__ == '__main__':
    main()
