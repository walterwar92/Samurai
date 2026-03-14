"""
Юнит-тесты для pi_nodes/hardware/led_driver.py

Запуск:
    pip install pytest
    pytest tests/test_led_driver.py -v

Все тесты работают без аппаратуры Raspberry Pi.
Hardware-path тесты мокируют модули `board` и `neopixel`.
"""

import sys
import time
import types
import pytest
from unittest.mock import MagicMock, patch, call


# ---------------------------------------------------------------------------
# Хелперы для создания мок-модулей board/neopixel
# ---------------------------------------------------------------------------

def _make_board_module():
    """Возвращает фейковый модуль board с атрибутом D12."""
    mod = types.ModuleType('board')
    mod.D12 = object()  # любой sentinel-объект
    return mod


def _make_neopixel_module(pixels_instance=None):
    """Возвращает фейковый модуль neopixel."""
    mod = types.ModuleType('neopixel')
    mock_pixels = pixels_instance or MagicMock()
    mod.NeoPixel = MagicMock(return_value=mock_pixels)
    mod.GRB = 'GRB'
    return mod


def _import_fresh(board_mod=None, neopixel_mod=None):
    """
    Импортирует led_driver заново с нужным состоянием sys.modules.
    Возвращает модуль led_driver.
    """
    # Убираем старый импорт
    for key in list(sys.modules):
        if 'led_driver' in key:
            del sys.modules[key]

    patched = {}
    if board_mod is not None:
        patched['board'] = board_mod
        patched['neopixel'] = neopixel_mod
    else:
        # Убираем board/neopixel, чтобы симуляция сработала
        patched['board'] = None      # ImportError при import board
        patched['neopixel'] = None

    with patch.dict(sys.modules, patched):
        import pi_nodes.hardware.led_driver as m
        return m


# ---------------------------------------------------------------------------
# Тесты _FakeLEDs
# ---------------------------------------------------------------------------

class TestFakeLEDs:
    """_FakeLEDs — stub без аппаратуры, используется в симуляции."""

    def setup_method(self):
        # Импортируем через симуляцию (без board/neopixel)
        mod = _import_fresh()
        self.FakeLEDs = mod._FakeLEDs

    def test_init_count(self):
        leds = self.FakeLEDs(8)
        assert leds.n == 8
        assert len(leds._pixels) == 8

    def test_initial_color_is_black(self):
        leds = self.FakeLEDs(4)
        for i in range(4):
            assert leds[i] == (0, 0, 0)

    def test_setitem_single(self):
        leds = self.FakeLEDs(4)
        leds[2] = (255, 0, 0)
        assert leds[2] == (255, 0, 0)
        assert leds[0] == (0, 0, 0)

    def test_setitem_slice(self):
        leds = self.FakeLEDs(6)
        leds[1:4] = (0, 255, 0)
        assert leds[0] == (0, 0, 0)
        assert leds[1] == (0, 255, 0)
        assert leds[2] == (0, 255, 0)
        assert leds[3] == (0, 255, 0)
        assert leds[4] == (0, 0, 0)

    def test_fill(self):
        leds = self.FakeLEDs(4)
        leds.fill((10, 20, 30))
        for i in range(4):
            assert leds[i] == (10, 20, 30)

    def test_show_does_not_raise(self):
        leds = self.FakeLEDs(4)
        leds.fill((1, 2, 3))
        leds.show()  # не должно бросать исключение

    def test_deinit_does_not_raise(self):
        leds = self.FakeLEDs(4)
        leds.deinit()


# ---------------------------------------------------------------------------
# Тесты LedDriver в режиме симуляции (нет аппаратных библиотек)
# ---------------------------------------------------------------------------

class TestLedDriverSimulated:
    """LedDriver падает в симуляцию при отсутствии board/neopixel."""

    def setup_method(self):
        self.mod = _import_fresh()  # без board/neopixel → симуляция

    def _make(self, count=8, brightness=0.3):
        return self.mod.LedDriver(count=count, brightness=brightness)

    def test_simulated_flag(self):
        drv = self._make()
        assert drv.simulated is True

    def test_count_property(self):
        drv = self._make(count=12)
        assert drv.count == 12

    def test_set_all(self):
        drv = self._make(count=4)
        drv.set_all((255, 0, 0))
        for i in range(4):
            assert drv._pixels[i] == (255, 0, 0)

    def test_set_one_valid(self):
        drv = self._make(count=4)
        drv.set_one(2, (0, 255, 0))
        assert drv._pixels[2] == (0, 255, 0)
        assert drv._pixels[0] == (0, 0, 0)

    def test_set_one_out_of_range_ignored(self):
        drv = self._make(count=4)
        drv.set_one(10, (255, 0, 0))  # не должно бросать
        for i in range(4):
            assert drv._pixels[i] == (0, 0, 0)

    def test_set_range(self):
        drv = self._make(count=8)
        drv.set_range(2, 5, (0, 0, 255))
        assert drv._pixels[1] == (0, 0, 0)
        assert drv._pixels[2] == (0, 0, 255)
        assert drv._pixels[4] == (0, 0, 255)
        assert drv._pixels[5] == (0, 0, 0)

    def test_set_range_clamped(self):
        drv = self._make(count=4)
        drv.set_range(-5, 100, (1, 2, 3))  # выход за границы → не падает
        for i in range(4):
            assert drv._pixels[i] == (1, 2, 3)

    def test_set_colors_full(self):
        drv = self._make(count=3)
        colors = [(10, 0, 0), (0, 20, 0), (0, 0, 30)]
        drv.set_colors(colors)
        for i, c in enumerate(colors):
            assert drv._pixels[i] == c

    def test_set_colors_partial(self):
        drv = self._make(count=4)
        drv.set_colors([(1, 1, 1), (2, 2, 2)])
        assert drv._pixels[0] == (1, 1, 1)
        assert drv._pixels[1] == (2, 2, 2)
        assert drv._pixels[2] == (0, 0, 0)  # нетронуто

    def test_set_colors_excess_truncated(self):
        drv = self._make(count=2)
        drv.set_colors([(1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0)])
        # только первые 2 применяются
        assert drv._pixels[0] == (1, 0, 0)
        assert drv._pixels[1] == (2, 0, 0)

    def test_off(self):
        drv = self._make(count=4)
        drv.set_all((255, 255, 255))
        drv.off()
        for i in range(4):
            assert drv._pixels[i] == (0, 0, 0)

    def test_deinit_turns_off(self):
        drv = self._make(count=4)
        drv.set_all((100, 100, 100))
        drv.deinit()
        for i in range(4):
            assert drv._pixels[i] == (0, 0, 0)

    def test_blink_ends_off(self):
        drv = self._make(count=4)
        drv.blink((255, 0, 0), times=2, on_sec=0.001, off_sec=0.001)
        for i in range(4):
            assert drv._pixels[i] == (0, 0, 0)

    def test_blink_count(self):
        """Blink N раз → set_all вызывался N раз с цветом и N раз off."""
        drv = self._make(count=4)
        calls_on = []
        calls_off = []
        original_set_all = drv.set_all

        def track_set_all(color):
            if color == (0, 0, 0):
                calls_off.append(color)
            else:
                calls_on.append(color)
            original_set_all(color)

        drv.set_all = track_set_all
        drv.blink((255, 0, 0), times=3, on_sec=0.001, off_sec=0.001)
        assert len(calls_on) == 3
        assert len(calls_off) == 3

    def test_pulse_simulated_no_raise(self):
        """В симуляции pulse не меняет яркость, но не падает."""
        drv = self._make()
        assert drv.simulated
        drv.pulse((0, 255, 0), steps=3, delay=0.001)

    def test_rainbow_cycle_no_raise(self):
        drv = self._make(count=4)
        drv.rainbow_cycle(wait=0.001, cycles=1)


# ---------------------------------------------------------------------------
# Тесты утилиты _wheel
# ---------------------------------------------------------------------------

class TestWheel:
    def setup_method(self):
        self.mod = _import_fresh()

    def test_wheel_0(self):
        r, g, b = self.mod._wheel(0)
        assert r == 255 and g == 0 and b == 0

    def test_wheel_85(self):
        r, g, b = self.mod._wheel(85)
        assert r == 0 and g == 255 and b == 0

    def test_wheel_170(self):
        r, g, b = self.mod._wheel(170)
        assert r == 0 and g == 0 and b == 255

    def test_wheel_255_wraps(self):
        # 255 & 255 == 255 → третья ветка: pos=85 → (255, 0, 0)
        r, g, b = self.mod._wheel(255)
        assert 0 <= r <= 255
        assert 0 <= g <= 255
        assert 0 <= b <= 255

    def test_wheel_components_in_range(self):
        for pos in range(256):
            r, g, b = self.mod._wheel(pos)
            assert 0 <= r <= 255
            assert 0 <= g <= 255
            assert 0 <= b <= 255


# ---------------------------------------------------------------------------
# Тесты предустановленных цветов (COLORS)
# ---------------------------------------------------------------------------

class TestColors:
    def setup_method(self):
        self.mod = _import_fresh()

    def test_colors_keys_present(self):
        required = {'red', 'green', 'blue', 'yellow', 'cyan', 'magenta',
                    'white', 'orange', 'off'}
        assert required.issubset(self.mod.COLORS.keys())

    def test_off_is_black(self):
        assert self.mod.COLORS['off'] == (0, 0, 0)

    def test_colors_values_valid_rgb(self):
        for name, (r, g, b) in self.mod.COLORS.items():
            assert 0 <= r <= 255, f'{name}: r={r}'
            assert 0 <= g <= 255, f'{name}: g={g}'
            assert 0 <= b <= 255, f'{name}: b={b}'


# ---------------------------------------------------------------------------
# Тесты LedDriver с мок-аппаратурой (board + neopixel)
# ---------------------------------------------------------------------------

class TestLedDriverHardwareMocked:
    """Проверяем hardware-path: LedDriver корректно вызывает neopixel.NeoPixel."""

    def setup_method(self):
        self.mock_pixels = MagicMock()
        self.mock_pixels.n = 8
        self.board_mod = _make_board_module()
        self.neopixel_mod = _make_neopixel_module(self.mock_pixels)
        self.mod = _import_fresh(self.board_mod, self.neopixel_mod)

    def _make(self, count=8, brightness=0.3):
        with patch.dict(sys.modules, {'board': self.board_mod,
                                       'neopixel': self.neopixel_mod}):
            return self.mod.LedDriver(count=count, brightness=brightness)

    def test_not_simulated(self):
        drv = self._make()
        assert drv.simulated is False

    def test_neopixel_constructed_correctly(self):
        drv = self._make(count=8, brightness=0.5)
        self.neopixel_mod.NeoPixel.assert_called_once_with(
            self.board_mod.D12,
            8,
            brightness=0.5,
            auto_write=False,
            pixel_order='GRB',
        )

    def test_set_all_calls_fill_and_show(self):
        drv = self._make()
        drv.set_all((10, 20, 30))
        self.mock_pixels.fill.assert_called_once_with((10, 20, 30))
        self.mock_pixels.show.assert_called()

    def test_off_fills_black(self):
        drv = self._make()
        drv.off()
        self.mock_pixels.fill.assert_called_with((0, 0, 0))

    def test_set_one_sets_and_shows(self):
        drv = self._make(count=8)
        drv.set_one(3, (255, 128, 0))
        self.mock_pixels.__setitem__.assert_called_with(3, (255, 128, 0))
        self.mock_pixels.show.assert_called()

    def test_deinit_calls_hw_deinit(self):
        drv = self._make()
        drv.deinit()
        self.mock_pixels.deinit.assert_called_once()
