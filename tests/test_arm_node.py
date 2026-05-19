"""Tests for pi_nodes.nodes.arm_node — фокус на интерполяции углов
и миграции дефолтных пресетов.

Контекст: текущий ServoDriver выставляет PWM моментально — слайдеры
и пресеты ощущаются «резко». Меняем подход: arm_node ведёт _target/
_current модель, фоновый таймер 50Гц шагает current к target с
max_speed_deg_per_sec. Все источники (single joint, joints[],
load_preset, home, FSM-команды) ставят только target — сглаживание
становится сквозным.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


@pytest.fixture
def arm_node_factory(tmp_path):
    """Фабрика ArmNode с замоканной MQTT-связью, ServoDriver и presets-файлом.

    Возвращает функцию `_factory(max_speed=120, presets_seed=None)`.
    Каждый тест может задать стартовый presets.json через `presets_seed`
    (dict с ключом 'arm').
    """
    def _factory(max_speed: float = 120.0, presets_seed: dict | None = None):
        from pi_nodes.nodes import arm_node as arm_node_module

        # Засеваем presets-файл
        presets_path = tmp_path / 'servo_presets.json'
        if presets_seed is not None:
            import json
            presets_path.write_text(json.dumps(presets_seed), encoding='utf-8')

        def fake_cfg(key, default=None):
            return {
                'servos.arm.channels': [0, 1, 2, 3],
                'servos.arm.home_angles': [0, 120, 0, 0],
                'servos.arm.min_angles': [0, 0, 0, 0],
                'servos.arm.max_angles': [160, 145, 180, 180],
                'servos.arm.invert_angles': [False, False, False, True],
                'servos.arm.labels': ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня'],
                'servos.arm.locked': False,    # unlocked для удобства тестов
                'servos.arm.max_speed_deg_per_sec': max_speed,
            }.get(key, default)

        with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
            MockClient.return_value = MagicMock()
            with patch.object(arm_node_module.ArmNode, 'create_timer',
                              lambda self, period, cb: None):
                with patch.object(arm_node_module.ArmNode, 'subscribe',
                                  lambda *a, **kw: None):
                    with patch('pi_nodes.nodes.arm_node.ServoDriver') as MockServo:
                        # Каждый Servo — MagicMock с .frozen=False и .set_angle
                        instances = [MagicMock(frozen=False, simulated=False)
                                     for _ in range(4)]
                        MockServo.side_effect = instances
                        with patch.object(arm_node_module, 'cfg', fake_cfg):
                            def _init_presets(self, path=None):
                                import json
                                import threading
                                self._path = str(presets_path)
                                self._lock = threading.Lock()
                                try:
                                    with open(self._path, 'r', encoding='utf-8') as f:
                                        self._data = json.load(f)
                                except (FileNotFoundError, json.JSONDecodeError):
                                    self._data = {}
                            with patch.object(
                                arm_node_module.ServoPresets,
                                '__init__', _init_presets):
                                node = arm_node_module.ArmNode()

        node._mock_servos = instances
        node._published: list[tuple[str, object]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        # Сброс mock-счётчиков: _unlock() в __init__ уже вызвал set_angle
        # для всех серво, тестам интереснее то, что произошло ПОСЛЕ старта.
        for m in instances:
            m.reset_mock()
        return node

    return _factory


def test_factory_imports():
    """Smoke: модуль импортируется и фабрика готова."""
    from pi_nodes.nodes import arm_node as _  # noqa: F401


def test_init_target_equals_current_equals_home(arm_node_factory):
    """На старте _target_angles и _current_angles совпадают с home_angles.

    Это гарантия что без команд интерполятор не двигает руку — она остаётся
    в home (или в физическом положении, если locked=True).
    """
    node = arm_node_factory()
    assert node._current_angles == [0.0, 120.0, 0.0, 0.0]
    assert node._target_angles == [0.0, 120.0, 0.0, 0.0]


def test_single_joint_sets_target_only_no_immediate_move(arm_node_factory):
    """`{joint:1, angle:90}` → _target_angles[0]=90, но _current_angles[0]=0
    (home). Сервопривод не дёрнется без вызова _interpolate_tick.
    """
    node = arm_node_factory()
    node._cmd_cb('arm/command', {'joint': 1, 'angle': 90.0})

    assert node._target_angles[0] == 90.0
    assert node._current_angles[0] == 0.0    # home, не двигались
    # ServoDriver.set_angle не вызывался для CH0
    node._mock_servos[0].set_angle.assert_not_called()


def test_interpolate_tick_steps_max_speed(arm_node_factory):
    """Tick @ 50Гц с max_speed=120°/с даёт шаг 2.4° за вызов.

    Цель — 90°, current=0°. После 1 тика current=2.4°.
    """
    node = arm_node_factory(max_speed=120.0)
    node._target_angles[0] = 90.0

    node._interpolate_tick()

    assert node._current_angles[0] == pytest.approx(2.4, abs=1e-6)
    # ServoDriver получил physical-угол (для CH0 invert=False → physical=logical)
    node._mock_servos[0].set_angle.assert_called_once()
    args, _ = node._mock_servos[0].set_angle.call_args
    assert args[0] == pytest.approx(2.4, abs=1e-6)


def test_interpolate_tick_snaps_when_delta_smaller_than_step(arm_node_factory):
    """Если |target - current| < max_step, current = target (не overshoot)."""
    node = arm_node_factory(max_speed=120.0)
    node._current_angles[0] = 89.0
    node._target_angles[0] = 90.0    # delta=1°, max_step=2.4° → snap

    node._interpolate_tick()

    assert node._current_angles[0] == 90.0


def test_interpolate_tick_handles_negative_direction(arm_node_factory):
    """Target меньше current — шагаем вниз."""
    node = arm_node_factory(max_speed=120.0)
    node._current_angles[0] = 50.0
    node._target_angles[0] = 0.0

    node._interpolate_tick()

    assert node._current_angles[0] == pytest.approx(47.6, abs=1e-6)


def test_interpolate_tick_moves_frozen_joint_to_target(arm_node_factory):
    """Frozen-сустав теперь интерполируется к target с set_angle(force=True).
    Это позволяет UI-слайдеру двигать frozen-сустав, оставляя его под PWM.
    Set_angle вызывается с force=True, иначе ServoDriver проигнорирует.
    """
    node = arm_node_factory(max_speed=120.0)
    node._target_angles[0] = 90.0
    node._mock_servos[0].frozen = True

    node._interpolate_tick()

    # Шаг 120°/с * 0.02с = 2.4°
    assert node._current_angles[0] == pytest.approx(2.4, abs=1e-6)
    node._mock_servos[0].set_angle.assert_called_once()
    # force=True для frozen — обходит фильтр в драйвере
    _args, kwargs = node._mock_servos[0].set_angle.call_args
    assert kwargs.get('force') is True


def test_interpolate_tick_uses_force_false_for_unfrozen(arm_node_factory):
    """Незамороженный сустав получает set_angle(force=False) — обычный путь."""
    node = arm_node_factory(max_speed=120.0)
    node._target_angles[0] = 90.0
    node._mock_servos[0].frozen = False

    node._interpolate_tick()

    node._mock_servos[0].set_angle.assert_called_once()
    _args, kwargs = node._mock_servos[0].set_angle.call_args
    assert kwargs.get('force') is False


def test_publish_state_uses_current(arm_node_factory):
    """`arm/state` публикует _current_angles — то, где сервопривод РЕАЛЬНО
    сейчас, а не target. Так UI видит плавное движение слайдеров, не скачки.
    """
    import json
    node = arm_node_factory()
    node._target_angles = [90.0, 60.0, 45.0, 180.0]
    node._current_angles = [10.0, 110.0, 0.0, 0.0]    # ещё едем

    node._publish_state()

    pub = next(p for p in node._published if p[0] == 'arm/state')
    payload = json.loads(pub[1])
    assert payload['j1'] == 10.0
    assert payload['j2'] == 110.0
    assert payload['j3'] == 0.0
    assert payload['j4'] == 0.0


def test_interpolate_tick_converges_after_n_ticks(arm_node_factory):
    """После ceil(delta / max_step) тиков current точно равно target.

    Защита от регрессии "одна команда поставила target, рука «зависла»
    на полпути". Тест прогоняет интерполятор до сходимости и
    проверяет финальное равенство.
    """
    import math as _math
    node = arm_node_factory(max_speed=120.0)
    node._target_angles[0] = 100.0   # delta=100°, max_step=2.4° → ~42 тика

    expected_ticks = int(_math.ceil(100.0 / 2.4))
    for _ in range(expected_ticks + 2):    # +2 для запаса
        node._interpolate_tick()

    assert node._current_angles[0] == 100.0
    # Последний шаг — snap. Все промежуточные были по max_step.


def test_interpolate_tick_inverted_joint_sends_physical_angle(arm_node_factory):
    """CH3 (клешня) имеет invert=True в фикстуре (max=180, min=0).
    Логическое target=180 (закрыто) → physical = 180 - 180 + 0 = 0.
    Логическое target=0 (открыто) → physical = 180 - 0 + 0 = 180.

    Фиксирует контракт _to_physical: интерполятор работает в логических
    координатах, физическая инверсия применяется ТОЛЬКО на границе
    set_angle. Регресс здесь = клешня будет открываться когда
    логически закрывается.
    """
    node = arm_node_factory(max_speed=120.0)
    # Логически "закрыть клешню" — target=180
    node._target_angles[3] = 180.0
    node._current_angles[3] = 178.0   # уже почти доехали → snap

    node._interpolate_tick()

    # Logical current=180, but ServoDriver получает physical=0
    assert node._current_angles[3] == 180.0
    args, _ = node._mock_servos[3].set_angle.call_args
    assert args[0] == pytest.approx(0.0, abs=1e-6)


def test_init_migrates_default_presets_when_empty(arm_node_factory):
    """При первом запуске (presets.json не существует или пуст) arm_node
    создаёт два дефолтных пресета — grab_ready и grab_hold. Это даёт FSM
    готовые позы без ручной настройки пользователем.
    """
    node = arm_node_factory(presets_seed=None)

    assert node._presets.load_preset('arm', 'grab_ready') == [110.0, 100.0, 180.0, 0.0]
    assert node._presets.load_preset('arm', 'grab_hold') == [10.0, 30.0, 180.0, 180.0]


def test_init_does_not_overwrite_user_presets(arm_node_factory):
    """Если пользователь сохранил свой grab_ready — миграция НЕ перетирает.

    Сценарий: пользователь скорректировал углы под живое железо и сохранил.
    После рестарта Pi (или service restart) — должны остаться пользовательские
    значения, не сброситься в дефолт.
    """
    custom = {
        'arm': {
            'grab_ready': [150.0, 95.0, 175.0, 5.0],   # пользовательские
            # grab_hold пользователь НЕ сохранял — должен создаться дефолт
        }
    }
    node = arm_node_factory(presets_seed=custom)

    # Пользовательский сохраняется
    assert node._presets.load_preset('arm', 'grab_ready') == [150.0, 95.0, 175.0, 5.0]
    # Отсутствующий — создаётся из дефолта
    assert node._presets.load_preset('arm', 'grab_hold') == [10.0, 30.0, 180.0, 180.0]


def test_set_joint_allow_frozen_true_updates_target(arm_node_factory):
    """_set_joint(idx, X, allow_frozen=True) обновляет target даже для frozen."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0

    node._set_joint(0, 75.0, allow_frozen=True)

    assert node._target_angles[0] == 75.0


def test_set_joint_default_skips_frozen_target(arm_node_factory):
    """_set_joint(idx, X) без allow_frozen НЕ обновляет target для frozen."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0

    node._set_joint(0, 75.0)

    assert node._target_angles[0] == 30.0   # не изменился


def test_cmd_cb_single_joint_moves_frozen(arm_node_factory):
    """arm/command {joint:1, angle:50} двигает frozen-сустав (target обновлён)."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 0.0

    node._cmd_cb('arm/command', {'joint': 1, 'angle': 50.0})

    assert node._target_angles[0] == 50.0


def test_cmd_cb_home_skips_frozen(arm_node_factory):
    """home команда обновляет target только для unfrozen суставов."""
    node = arm_node_factory()
    # home_angles = [0, 120, 0, 0], текущие targets такие же после _unlock()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0   # frozen-сустав в нестандартной позе
    node._target_angles[1] = 60.0   # unfrozen — должен уехать в home=120

    node._cmd_cb('arm/command', {'command': 'home'})

    assert node._target_angles[0] == 30.0   # frozen не изменился
    assert node._target_angles[1] == 120.0  # unfrozen уехал в home


def test_cmd_cb_load_preset_skips_frozen(arm_node_factory):
    """load_preset обновляет target только для unfrozen суставов."""
    node = arm_node_factory(presets_seed={'arm': {'foo': [10.0, 20.0, 30.0, 40.0]}})
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'command': 'load_preset', 'name': 'foo'})

    assert node._target_angles[0] == 100.0  # frozen не изменился
    assert node._target_angles[1] == 20.0   # unfrozen загрузил из preset


def test_cmd_cb_joints_array_skips_frozen(arm_node_factory):
    """{joints:[...]} обновляет target только для unfrozen суставов."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'joints': [10.0, 20.0, 30.0, 40.0]})

    assert node._target_angles[0] == 100.0  # frozen не изменился
    assert node._target_angles[1] == 20.0   # unfrozen загрузил
