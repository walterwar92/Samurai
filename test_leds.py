#!/usr/bin/env python3
"""
Тестовый скрипт для светодиодов WS2812B на Samurai Robot.

Запуск на Raspberry Pi (нужен root для rpi_ws281x):
    sudo python3 test_leds.py [команда] [аргументы]

Команды:
    color <имя|hex>    — залить все диоды цветом (пример: color red, color #ff8000)
    blink <имя|hex>    — мигнуть 3 раза
    pulse <имя|hex>    — плавное нарастание/угасание
    rainbow            — радужный цикл
    scan               — проверка всех цветов: все диоды, затем по одному
    demo               — демонстрация всех эффектов
    off                — выключить

Без аргументов — запускает scan.
"""

import sys
import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s — %(message)s',
)

# Добавляем корень проекта в путь
import os
sys.path.insert(0, os.path.dirname(__file__))

from pi_nodes.hardware.led_driver import LedDriver, COLORS, Color

LED = LedDriver(count=8, brightness=0.3)


# ------------------------------------------------------------------
def parse_color(arg: str) -> Color:
    """Разбирает имя цвета или hex (#rrggbb / rrggbb)."""
    arg = arg.lower().strip()
    if arg in COLORS:
        return COLORS[arg]
    # hex
    arg = arg.lstrip('#')
    if len(arg) == 6:
        try:
            r = int(arg[0:2], 16)
            g = int(arg[2:4], 16)
            b = int(arg[4:6], 16)
            return (r, g, b)
        except ValueError:
            pass
    print(f'Неизвестный цвет: {arg!r}')
    print(f'Доступные имена: {", ".join(COLORS.keys())}')
    sys.exit(1)


def scan(hold: float = 2.0):
    """Проверка аппаратуры: каждый цвет на всех диодах, потом диоды по одному."""
    sim = ' [СИМУЛЯЦИЯ]' if LED.simulated else ''
    print(f'=== Проверка LED{sim} ({LED.count} диодов) ===')
    print()

    color_names = [n for n in COLORS if n != 'off']

    # --- Шаг 1: все диоды одним цветом ---
    print('Шаг 1: все диоды — каждый цвет по очереди')
    print(f'       задержка {hold:.0f}с, Ctrl+C для пропуска шага\n')
    try:
        for name in color_names:
            color = COLORS[name]
            print(f'  >> {name:10s}  RGB{color}')
            LED.set_all(color)
            time.sleep(hold)
    except KeyboardInterrupt:
        print('  (пропущено)')
    LED.off()
    time.sleep(0.5)

    # --- Шаг 2: диоды по одному ---
    print()
    print('Шаг 2: каждый диод по очереди (белый)')
    print(f'       убедитесь что все {LED.count} диодов светятся\n')
    try:
        for i in range(LED.count):
            print(f'  >> диод #{i}')
            LED.off()
            LED.set_one(i, COLORS['white'])
            time.sleep(0.6)
    except KeyboardInterrupt:
        print('  (пропущено)')
    LED.off()
    time.sleep(0.3)

    # --- Шаг 3: бегущий огонь всеми цветами ---
    print()
    print('Шаг 3: бегущий огонь — все цвета\n')
    try:
        for name in color_names:
            color = COLORS[name]
            print(f'  >> {name}')
            for i in range(LED.count):
                LED.off()
                LED.set_one(i, color)
                time.sleep(0.08)
        LED.off()
    except KeyboardInterrupt:
        print('  (пропущено)')
    LED.off()

    print()
    print('Готово. Если какой-то диод не светился — проверьте питание/пайку.')


def demo():
    print('=== LED Demo ===')

    print('Все цвета по очереди...')
    for name, color in COLORS.items():
        if name == 'off':
            continue
        print(f'  → {name}')
        LED.set_all(color)
        time.sleep(0.5)
    LED.off()
    time.sleep(0.3)

    print('Blink красным...')
    LED.blink(COLORS['red'], times=3)
    time.sleep(0.3)

    print('Pulse синим...')
    LED.pulse(COLORS['blue'], steps=15, delay=0.02)
    time.sleep(0.3)

    print('Радуга...')
    LED.rainbow_cycle(wait=0.01, cycles=2)

    print('Диоды по одному (зелёный)...')
    for i in range(LED.count):
        LED.set_one(i, COLORS['green'])
        time.sleep(0.1)
    time.sleep(0.3)
    LED.off()

    print('Готово.')


# ------------------------------------------------------------------
def main():
    cmd = sys.argv[1].lower() if len(sys.argv) > 1 else 'scan'

    try:
        if cmd == 'color':
            color = parse_color(sys.argv[2] if len(sys.argv) > 2 else 'white')
            print(f'Цвет: RGB{color}')
            LED.set_all(color)
            print('Ctrl+C для выхода...')
            while True:
                time.sleep(1)

        elif cmd == 'blink':
            color = parse_color(sys.argv[2] if len(sys.argv) > 2 else 'white')
            print(f'Мигаем: RGB{color}')
            LED.blink(color, times=5, on_sec=0.3, off_sec=0.2)

        elif cmd == 'pulse':
            color = parse_color(sys.argv[2] if len(sys.argv) > 2 else 'cyan')
            print(f'Пульс: RGB{color}')
            try:
                while True:
                    LED.pulse(color, steps=20, delay=0.03)
            except KeyboardInterrupt:
                pass

        elif cmd == 'rainbow':
            print('Радуга (Ctrl+C для остановки)...')
            try:
                while True:
                    LED.rainbow_cycle(wait=0.01, cycles=1)
            except KeyboardInterrupt:
                pass

        elif cmd == 'scan':
            hold = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
            scan(hold=hold)

        elif cmd == 'demo':
            demo()

        elif cmd == 'off':
            LED.off()
            print('Выключено.')

        else:
            print(__doc__)
            sys.exit(1)

    except KeyboardInterrupt:
        print('\nОстановлено.')
    finally:
        LED.off()
        LED.deinit()


if __name__ == '__main__':
    main()
