# Захват цветного объекта — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Робот центруется на яркий объект (R/G/B), подъезжает, выезжает рукой в позу `grab_ready=[160,100,180,0]`, захватывает с `grab_hold=[10,30,180,180]` и удерживает с заморозкой пока клешня закрыта. Все переходы углов плавные через интерполяцию в Pi `arm_node` @ 50Гц.

**Architecture:** В `arm_node` появляется target/current модель — все команды (`joint+angle`, `joints[]`, `load_preset`, `home`) ставят `_target_angles[i]`; фоновый таймер 50Гц шагает `_current_angles[i]` к target с `max_speed_deg_per_sec`. FSM `_do_approach` шлёт `grab_ready`, новый `_do_grab` шлёт `grab_hold` → ждёт 1.5с settle → `freeze`. В `set_claw(open)` параллельно публикуется `unfreeze`. UI расширяет лимит CH0 до 160.

**Tech Stack:** Python (pi_nodes, compute_node, pytest, paho-mqtt), TypeScript/React (Vite), YAML (config), MQTT.

**Spec:** [docs/superpowers/specs/2026-05-17-arm-grab-sequence-design.md](../specs/2026-05-17-arm-grab-sequence-design.md)

---

## File Structure

**Создаются:**
- `tests/test_arm_node.py` — юнит-тесты интерполяции + миграции пресетов в `arm_node`. Mocked `ServoDriver` и `MqttNode`.
- `tests/test_fsm_grab_sequence.py` — юнит-тесты FSM-перехода в APPROACHING/GRABBING и публикаций `arm/command`.
- `tests/test_actuators_router.py` — тесты `set_claw` endpoint, проверка авто-`unfreeze` при `state="open"`.

**Модифицируются:**
- `pi_nodes/nodes/arm_node.py` — поля `_target_angles`/`_current_angles`/`_max_speed`, метод `_interpolate_tick`, миграция пресетов в `__init__`, `_publish_state` читает current, `_set_joint` пишет в target.
- `pi_nodes/nodes/fsm_node.py` — `_transition` сбрасывает `_approach_arm_sent` и `_grab_t`; `_do_approach` шлёт `grab_ready` один раз; `_do_grab` переписан под пресет `grab_hold`+freeze; убираются `claw/command` публикации.
- `compute_node/dashboard/routers/actuators.py` — `set_claw` дополнительно публикует `arm/command unfreeze` при `state="open"` или `angle < 90`.
- `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx` — `ARM_JOINTS[0].max: 120 → 160`, обновить шапку-комментарий.
- `config.yaml` — `servos.arm.max_angles[0]: 120 → 160`, `servos.arm.max_speed_deg_per_sec: 120`.
- `compute_node/static/index.html` + `compute_node/static/assets/*` — авто после `npm run build`.

---

## Task 1: arm_node — модель target/current и интерполяция

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py:47-100` (`__init__`), `:118-126` (`_set_joint`), `:274-280` (`_publish_state`)
- Test: `tests/test_arm_node.py`

- [ ] **Step 1.1: Создать test-файл с failing-импортом и fixture**

`tests/test_arm_node.py`:
```python
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
                            with patch.object(
                                arm_node_module.ServoPresets, '__init__',
                                lambda self, path=None: setattr(
                                    self, '_path', str(presets_path)) or
                                    setattr(self, '_data', {}) or
                                    setattr(self, '_lock',
                                            __import__('threading').Lock())):
                                node = arm_node_module.ArmNode()

        node._mock_servos = instances
        node._published: list[tuple[str, object]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        return node

    return _factory


def test_factory_imports():
    """Smoke: модуль импортируется и фабрика готова."""
    from pi_nodes.nodes import arm_node as _  # noqa: F401
```

- [ ] **Step 1.2: Запустить test-файл — должен пройти smoke-тест**

Run: `pytest tests/test_arm_node.py::test_factory_imports -v`
Expected: PASS (просто проверяем что arm_node импортируется без ошибок)

- [ ] **Step 1.3: Добавить failing-тест: при старте target=current=home**

В конец `tests/test_arm_node.py`:
```python
def test_init_target_equals_current_equals_home(arm_node_factory):
    """На старте _target_angles и _current_angles совпадают с home_angles.

    Это гарантия что без команд интерполятор не двигает руку — она остаётся
    в home (или в физическом положении, если locked=True).
    """
    node = arm_node_factory()
    assert node._current_angles == [0.0, 120.0, 0.0, 0.0]
    assert node._target_angles == [0.0, 120.0, 0.0, 0.0]
```

Run: `pytest tests/test_arm_node.py::test_init_target_equals_current_equals_home -v`
Expected: FAIL — `AttributeError: 'ArmNode' object has no attribute '_current_angles'` (или `_target_angles`)

- [ ] **Step 1.4: Реализовать поля target/current в `arm_node.__init__`**

Заменить в `pi_nodes/nodes/arm_node.py:73-80`:
```python
        self._servos: list[ServoDriver] = []
        self._angles: list[float] = list(map(float, self._home_angles))
        for i in range(self._num_joints):
            init_phys = self._to_physical(i, float(self._home_angles[i]))
            s = ServoDriver(channel=self._channels[i],
                            init_angle=init_phys,
                            start_disabled=True)
            self._servos.append(s)
```
На:
```python
        self._servos: list[ServoDriver] = []
        # Target — куда хотим. Current — где реально сервопривод сейчас.
        # Интерполятор шагает current → target с max_speed_deg_per_sec.
        # Все источники углов (joint+angle, joints[], load_preset, home)
        # ставят только target; сглаживание сквозное.
        self._target_angles: list[float] = list(map(float, self._home_angles))
        self._current_angles: list[float] = list(map(float, self._home_angles))
        # Максимальная угловая скорость для интерполяции (°/сек).
        # Защита: max_speed=0 «зависил» бы руку — clamp до 1.0.
        self._max_speed: float = max(
            1.0, float(cfg('servos.arm.max_speed_deg_per_sec', 120.0)))
        for i in range(self._num_joints):
            init_phys = self._to_physical(i, float(self._home_angles[i]))
            s = ServoDriver(channel=self._channels[i],
                            init_angle=init_phys,
                            start_disabled=True)
            self._servos.append(s)
```

Также удалить старое поле `self._angles` и заменить все ссылки на него:
- `pi_nodes/nodes/arm_node.py:115` (`self._angles[i] = float(self._home_angles[i])`) → `self._target_angles[i] = self._current_angles[i] = float(self._home_angles[i])`
- `pi_nodes/nodes/arm_node.py:126` (`self._angles[idx] = angle`) → `self._target_angles[idx] = angle` (см. Step 1.6)
- `pi_nodes/nodes/arm_node.py:186` (`self._angles[idx]` в log) → `self._target_angles[idx]`
- `pi_nodes/nodes/arm_node.py:211` (`save_preset` шлёт `list(self._angles)`) → `list(self._current_angles)` (сохраняем то, где реально стоит — current)
- `pi_nodes/nodes/arm_node.py:227` (log) → `self._target_angles`
- `pi_nodes/nodes/arm_node.py:255` (log) → `self._target_angles[idx]`
- `pi_nodes/nodes/arm_node.py:264` (log) → `self._target_angles`
- `pi_nodes/nodes/arm_node.py:277` (`_publish_state` читает `self._angles[i]`) → `self._current_angles[i]`

- [ ] **Step 1.5: Запустить тест — должен пройти**

Run: `pytest tests/test_arm_node.py::test_init_target_equals_current_equals_home -v`
Expected: PASS

- [ ] **Step 1.6: Failing-тест: команда `joint+angle` ставит target, current не двигается без тика**

В конец `tests/test_arm_node.py`:
```python
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
```

Run: `pytest tests/test_arm_node.py::test_single_joint_sets_target_only_no_immediate_move -v`
Expected: FAIL (текущий `_set_joint` вызывает `set_angle` сразу)

- [ ] **Step 1.7: Переделать `_set_joint` так, чтобы он писал в target**

Заменить `pi_nodes/nodes/arm_node.py:118-126`:
```python
    def _set_joint(self, idx: int, angle: float):
        """Set joint angle (логический) с лимитами. Инверсия применяется внутри."""
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        self._servos[idx].set_angle(self._to_physical(idx, angle))
        if not self._servos[idx].frozen:
            self._angles[idx] = angle
```
На:
```python
    def _set_joint(self, idx: int, angle: float):
        """Set joint TARGET angle (логический) с лимитами.

        Реальный PWM шлёт _interpolate_tick @ 50Гц, плавно шагая current
        к target с max_speed_deg_per_sec. Если сустав frozen — target всё
        равно обновляется (чтобы после unfreeze сразу поехать к нему).
        """
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        self._target_angles[idx] = angle
```

- [ ] **Step 1.8: Запустить тест — PASS**

Run: `pytest tests/test_arm_node.py::test_single_joint_sets_target_only_no_immediate_move -v`
Expected: PASS

- [ ] **Step 1.9: Failing-тест: `_interpolate_tick` шагает current → target ровно на max_step**

В конец:
```python
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


def test_interpolate_tick_skips_frozen_joints(arm_node_factory):
    """Если servo.frozen=True — interpolate_tick НЕ двигает current и НЕ
    шлёт set_angle. Это эквивалентно «freeze ставит current=target и стоп».
    """
    node = arm_node_factory(max_speed=120.0)
    node._target_angles[0] = 90.0
    node._mock_servos[0].frozen = True

    node._interpolate_tick()

    assert node._current_angles[0] == 0.0   # не двигались
    node._mock_servos[0].set_angle.assert_not_called()
```

Run: `pytest tests/test_arm_node.py -v -k interpolate_tick`
Expected: FAIL (метод `_interpolate_tick` не существует)

- [ ] **Step 1.10: Реализовать `_interpolate_tick` + регистрацию таймера в `__init__`**

В `arm_node.py`, добавить импорт сверху (если ещё нет):
```python
import math
```

Добавить в `__init__` после `self.create_timer(0.1, self._publish_state)`:
```python
        # Интерполятор: шагаем _current → _target @ 50Гц.
        # PCA9685 I²C-команды принимаются без проблем; ServoDriver.set_angle
        # уже клампит 0..180 и обрабатывает frozen.
        self.create_timer(0.02, self._interpolate_tick)
```

Добавить метод (после `_set_joint` или перед `_publish_state`):
```python
    def _interpolate_tick(self):
        """Шаг интерполяции: current → target со скоростью _max_speed.

        Вызывается таймером @ 50Гц. Для каждого сустава:
        - если frozen → пропускаем (PWM-удержание уже у драйвера);
        - если |delta| <= max_step → snap current к target (без overshoot);
        - иначе current += sign(delta) * max_step.
        Реальный PWM выставляется через ServoDriver.set_angle (с учётом
        инверсии — _to_physical).
        """
        dt = 0.02
        max_step = self._max_speed * dt
        for i in range(self._num_joints):
            if self._servos[i].frozen:
                continue
            delta = self._target_angles[i] - self._current_angles[i]
            if delta == 0.0:
                continue
            if abs(delta) <= max_step:
                self._current_angles[i] = self._target_angles[i]
            else:
                self._current_angles[i] += math.copysign(max_step, delta)
            phys = self._to_physical(i, self._current_angles[i])
            self._servos[i].set_angle(phys)
```

- [ ] **Step 1.11: Запустить все тесты интерполяции — PASS**

Run: `pytest tests/test_arm_node.py -v -k interpolate_tick`
Expected: 4 PASS

- [ ] **Step 1.12: Failing-тест: `_publish_state` отдаёт current, не target**

```python
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
```

Run: `pytest tests/test_arm_node.py::test_publish_state_uses_current -v`
Expected: либо PASS (если Step 1.4 правильно заменил `_angles` на `_current_angles` в `_publish_state`), либо FAIL — если PASS, всё ОК; если FAIL — исправить.

Если FAIL — заменить в `pi_nodes/nodes/arm_node.py:274-280`:
```python
    def _publish_state(self):
        state = {}
        for i in range(self._num_joints):
            state[f'j{i+1}'] = round(self._angles[i], 1)
        state['frozen'] = [s.frozen for s in self._servos]
        state['locked'] = self._locked
        self.publish('arm/state', json.dumps(state))
```
На:
```python
    def _publish_state(self):
        state = {}
        for i in range(self._num_joints):
            state[f'j{i+1}'] = round(self._current_angles[i], 1)
        state['frozen'] = [s.frozen for s in self._servos]
        state['locked'] = self._locked
        self.publish('arm/state', json.dumps(state))
```

- [ ] **Step 1.13: Запустить все тесты `test_arm_node.py` — все PASS**

Run: `pytest tests/test_arm_node.py -v`
Expected: 7 PASS (5 интерполяционных + 1 init + 1 import smoke)

- [ ] **Step 1.14: Commit**

```bash
git add tests/test_arm_node.py pi_nodes/nodes/arm_node.py
git commit -m "feat(arm): target/current interpolation @ 50Hz — плавный переход углов"
```

---

## Task 2: arm_node — авто-миграция пресетов `grab_ready` и `grab_hold`

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py` (`__init__`, добавить миграцию после создания `ServoPresets`)
- Test: `tests/test_arm_node.py` (добавить тесты миграции)

- [ ] **Step 2.1: Failing-тест: при пустом presets-файле создаются grab_ready и grab_hold**

В `tests/test_arm_node.py` добавить в конец:
```python
def test_init_migrates_default_presets_when_empty(arm_node_factory):
    """При первом запуске (presets.json не существует или пуст) arm_node
    создаёт два дефолтных пресета — grab_ready и grab_hold. Это даёт FSM
    готовые позы без ручной настройки пользователем.
    """
    node = arm_node_factory(presets_seed=None)

    assert node._presets.load_preset('arm', 'grab_ready') == [160.0, 100.0, 180.0, 0.0]
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
```

Run: `pytest tests/test_arm_node.py -v -k migrate`
Expected: FAIL — пресетов нет, поскольку миграция не реализована.

⚠ **Tip:** fixture `arm_node_factory` патчит `ServoPresets.__init__` — поэтому пишет/читает в `tmp_path/servo_presets.json`. Если `presets_seed` передан, он будет загружен через стандартный `_load()` (надо обновить fixture, чтобы `_data` инициализировалась из файла, а не пустым dict). Подправить fixture:

В `arm_node_factory._factory`, заменить блок патча `ServoPresets.__init__`:
```python
                            with patch.object(
                                arm_node_module.ServoPresets, '__init__',
                                lambda self, path=None: setattr(
                                    self, '_path', str(presets_path)) or
                                    setattr(self, '_data', {}) or
                                    setattr(self, '_lock',
                                            __import__('threading').Lock())):
```
На:
```python
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
```

- [ ] **Step 2.2: Реализовать миграцию в `ArmNode.__init__`**

В `pi_nodes/nodes/arm_node.py`, после строки `self._presets = ServoPresets()`:
```python
        # Авто-миграция дефолтных поз. Если пользователь уже сохранил свой
        # вариант пресета — НЕ перетираем (load_preset вернёт его).
        # Цель: FSM хант мяча (см. docs/superpowers/specs/2026-05-17-arm-grab-
        # sequence-design.md) получает готовые grab_ready и grab_hold без
        # ручных кликов в UI.
        _DEFAULT_ARM_PRESETS = {
            'grab_ready': [160.0, 100.0, 180.0, 0.0],
            'grab_hold':  [10.0,  30.0,  180.0, 180.0],
        }
        for _name, _angles in _DEFAULT_ARM_PRESETS.items():
            if self._presets.load_preset('arm', _name) is None:
                self._presets.save_preset('arm', _name, _angles)
                self.log_info('Migration: created arm preset "%s"=%s',
                              _name, _angles)
```

- [ ] **Step 2.3: Запустить миграционные тесты — PASS**

Run: `pytest tests/test_arm_node.py -v -k migrate`
Expected: 2 PASS

- [ ] **Step 2.4: Запустить весь `test_arm_node.py` — все PASS**

Run: `pytest tests/test_arm_node.py -v`
Expected: 9 PASS

- [ ] **Step 2.5: Commit**

```bash
git add tests/test_arm_node.py pi_nodes/nodes/arm_node.py
git commit -m "feat(arm): авто-миграция пресетов grab_ready / grab_hold при старте"
```

---

## Task 3: config.yaml — лимит CH0=160 и max_speed_deg_per_sec

**Files:**
- Modify: `config.yaml` (секция `servos.arm`)

- [ ] **Step 3.1: Прочитать текущую секцию `servos.arm` в `config.yaml`**

Run: `grep -n -A 12 "^servos:" config.yaml | head -20`

(Ожидаемый вывод включает `max_angles: [120, 145, 180, 180]` и `invert_angles: [false, false, false, true]`.)

- [ ] **Step 3.2: Поменять `max_angles[0]` на 160 и добавить `max_speed_deg_per_sec: 120`**

В `config.yaml`, найти строку:
```yaml
    max_angles:  [120, 145, 180, 180]
```
Заменить на:
```yaml
    max_angles:  [160, 145, 180, 180]   # CH0 расширен до 160° под позу grab_ready
```

Добавить после `invert_angles:` (или перед `labels:`):
```yaml
    # Максимальная угловая скорость для интерполяции в arm_node.
    # ServoDriver сам по себе моментальный — мы делаем плавный переход
    # шагами _interpolate_tick @ 50Гц с этим лимитом (°/сек).
    # Общий для всех 4 суставов; если CH3 (клешня) визуально нужно
    # отличать — расширим до per-joint списка.
    max_speed_deg_per_sec: 120
```

- [ ] **Step 3.3: Проверить что YAML валидный + arm_node поднимает скорость из config**

Run: `python -c "from config_loader import cfg; print(cfg('servos.arm.max_angles'), cfg('servos.arm.max_speed_deg_per_sec'))"`
Expected: `[160, 145, 180, 180] 120` (или подобное)

- [ ] **Step 3.4: Запустить регрессионный набор pi_nodes тестов**

Run: `pytest tests/test_arm_node.py tests/test_imu_node.py tests/test_config_loader.py -v`
Expected: все PASS (никаких регрессий от config-изменений)

- [ ] **Step 3.5: Commit**

```bash
git add config.yaml
git commit -m "config(arm): CH0 max 120→160, добавлен max_speed_deg_per_sec=120"
```

---

## Task 4: FSM — `_do_approach` шлёт `grab_ready` один раз при входе

**Files:**
- Modify: `pi_nodes/nodes/fsm_node.py:335-369` (`_transition`), `:433-471` (`_do_approach`)
- Test: `tests/test_fsm_grab_sequence.py` (новый)

- [ ] **Step 4.1: Создать test-файл с fixture для FSM**

`tests/test_fsm_grab_sequence.py`:
```python
"""Tests for fsm_node — последовательность APPROACHING/GRABBING с новыми
позами руки (grab_ready / grab_hold) и заморозкой.

Контекст: до этого PR FSM `_do_grab` слал `claw/command "close"` и не
управлял углами 1-3. Теперь рука выезжает в `grab_ready` при входе в
APPROACHING и закрывается в `grab_hold` с заморозкой в GRABBING. Тесты
фиксируют этот контракт публикаций.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


@pytest.fixture
def fsm_node_factory():
    """Фабрика FSMNode с замоканной MQTT-связью и таймерами."""
    def _factory():
        from pi_nodes.nodes import fsm_node as fsm_module

        with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
            MockClient.return_value = MagicMock()
            with patch.object(fsm_module.FSMNode, 'create_timer',
                              lambda self, period, cb: None):
                with patch.object(fsm_module.FSMNode, 'subscribe',
                                  lambda *a, **kw: None):
                    node = fsm_module.FSMNode()

        node._published: list[tuple[str, object]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        return node

    return _factory


def test_factory_smoke(fsm_node_factory):
    """Smoke: FSMNode инстанцируется и в IDLE."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    assert node._state == State.IDLE
```

Run: `pytest tests/test_fsm_grab_sequence.py::test_factory_smoke -v`
Expected: PASS

- [ ] **Step 4.2: Failing-тест: вход в APPROACHING шлёт `grab_ready` один раз**

Добавить в `tests/test_fsm_grab_sequence.py`:
```python
def test_approach_sends_grab_ready_once_on_entry(fsm_node_factory):
    """Первый _do_approach(...) при свежем входе в APPROACHING публикует
    arm/command {"command":"load_preset","name":"grab_ready"}. Повторные
    тики НЕ шлют (один раз за state).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    # Симулируем вход в APPROACHING из TARGETING (центрировались по мячу)
    node._transition(State.APPROACHING)
    node._target_colour = 'red'
    fake_det = {'colour': 'red', 'x': 300, 'y': 200, 'w': 40, 'h': 40}

    # Tick 1: должно быть load_preset grab_ready
    node._do_approach(fake_det, range_m=0.50)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1
    assert arm_pubs[0][1]['name'] == 'grab_ready'

    # Tick 2-5: НЕ шлём повторно
    for _ in range(4):
        node._do_approach(fake_det, range_m=0.40)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1   # всё ещё ровно 1
```

Run: `pytest tests/test_fsm_grab_sequence.py::test_approach_sends_grab_ready_once_on_entry -v`
Expected: FAIL — `_do_approach` пока не шлёт `arm/command`.

- [ ] **Step 4.3: Failing-тест: повторный вход в APPROACHING после выхода → ещё раз шлёт**

```python
def test_approach_resends_grab_ready_after_state_exit(fsm_node_factory):
    """Если FSM ушёл в TARGETING (потерял мяч) и вернулся в APPROACHING —
    grab_ready должен послаться снова. Флаг `_approach_arm_sent`
    сбрасывается в _transition.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.APPROACHING)
    fake_det = {'colour': 'red', 'x': 300, 'y': 200, 'w': 40, 'h': 40}

    node._do_approach(fake_det, range_m=0.50)
    # Потеряли мяч → вернулись в TARGETING → снова в APPROACHING
    node._transition(State.TARGETING)
    node._transition(State.APPROACHING)
    node._do_approach(fake_det, range_m=0.50)

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 2
    assert all(p[1]['name'] == 'grab_ready' for p in arm_pubs)
```

Run: `pytest tests/test_fsm_grab_sequence.py::test_approach_resends_grab_ready_after_state_exit -v`
Expected: FAIL

- [ ] **Step 4.4: Реализовать поле `_approach_arm_sent` и сброс в `_transition`**

В `pi_nodes/nodes/fsm_node.py:86-103`, в конец `__init__` добавить:
```python
        # Флаг «послали grab_ready при входе в APPROACHING» (см. _do_approach).
        # Сбрасывается в _transition при любом изменении state.
        self._approach_arm_sent = False
        # Локальный таймер фазы GRABBING (см. _do_grab).
        self._grab_t = 0.0
```

В `_transition` (строка 335), добавить ПОСЛЕ `self._lost_frames = 0`:
```python
        self._approach_arm_sent = False
        self._grab_t = 0.0
```

- [ ] **Step 4.5: Реализовать публикацию `grab_ready` в `_do_approach`**

В `pi_nodes/nodes/fsm_node.py:433-471`, в самое начало `_do_approach` (после `def`-строки и docstring если есть, перед `self._approach_timeout += 0.1`):
```python
        if not self._approach_arm_sent:
            self.publish('arm/command',
                         {'command': 'load_preset', 'name': 'grab_ready'},
                         qos=1)
            self._approach_arm_sent = True
            self.log_info('Arm → grab_ready (approach start)')
```

- [ ] **Step 4.6: Запустить approach-тесты — PASS**

Run: `pytest tests/test_fsm_grab_sequence.py -v -k approach`
Expected: 2 PASS

- [ ] **Step 4.7: Commit**

```bash
git add tests/test_fsm_grab_sequence.py pi_nodes/nodes/fsm_node.py
git commit -m "feat(fsm): _do_approach шлёт arm preset grab_ready при входе"
```

---

## Task 5: FSM — `_do_grab` переписан под `grab_hold` + freeze + settle

**Files:**
- Modify: `pi_nodes/nodes/fsm_node.py:473-484` (`_do_grab`)
- Test: `tests/test_fsm_grab_sequence.py`

- [ ] **Step 5.1: Failing-тест: первый тик GRABBING шлёт `grab_hold`**

Добавить в `tests/test_fsm_grab_sequence.py`:
```python
def test_grab_first_tick_sends_grab_hold_preset(fsm_node_factory):
    """В первый тик _do_grab публикуется arm/command
    {"command":"load_preset","name":"grab_hold"}.
    Это разом ставит CH0/CH1/CH2 в позу захвата и CH3=180 (закрывает клешню).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)
                and p[1].get('command') == 'load_preset']
    assert len(arm_pubs) == 1
    assert arm_pubs[0][1]['name'] == 'grab_hold'


def test_grab_does_not_publish_claw_command(fsm_node_factory):
    """Новая логика НЕ использует topic claw/command (legacy для servo_node,
    который работает с CH0=Основание — неправильный канал для клешни).
    Всё идёт через arm/command (CH3 — клешня по новой логике).
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(20):    # 2 секунды эмулируем
        node._do_grab()

    claw_pubs = [p for p in node._published if p[0] == 'claw/command']
    assert claw_pubs == []
```

Run: `pytest tests/test_fsm_grab_sequence.py -v -k grab_first_tick`
Expected: FAIL (текущий `_do_grab` шлёт `claw/command 'open'`)

- [ ] **Step 5.2: Failing-тест: после settle (1.5с) шлётся freeze и FSM в RETURNING**

```python
def test_grab_after_settle_sends_freeze_and_transitions_to_returning(
        fsm_node_factory):
    """Через ~1.5с после первой команды grab_hold (интерполятор успевает
    доехать) FSM шлёт arm/command {"command":"freeze"} и переходит в
    RETURNING. Рука остаётся frozen в grab_hold, корпус едет домой.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    # 15 тиков = 1.5с (tick=0.1с). Точно равно settle_s, должен запуститься
    # переход после 16-го тика (>1.5).
    for _ in range(16):
        node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)]
    freeze_pubs = [p for p in arm_pubs if p[1].get('command') == 'freeze']
    assert len(freeze_pubs) == 1
    # Переход в RETURNING
    assert node._state == State.RETURNING


def test_grab_during_settle_does_not_freeze_yet(fsm_node_factory):
    """Между t=0.1с и t=1.5с — никакие freeze/новые preset-команды не шлются.
    Только один grab_hold на старте, потом ждём.
    """
    from pi_nodes.nodes.fsm_node import State

    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(10):    # 1.0с
        node._do_grab()

    arm_pubs = [p for p in node._published
                if p[0] == 'arm/command'
                and isinstance(p[1], dict)]
    # Один load_preset grab_hold и НИ ОДНОГО freeze
    preset_pubs = [p for p in arm_pubs if p[1].get('command') == 'load_preset']
    freeze_pubs = [p for p in arm_pubs if p[1].get('command') == 'freeze']
    assert len(preset_pubs) == 1
    assert len(freeze_pubs) == 0
    assert node._state == State.GRABBING    # ещё не перешли
```

Run: `pytest tests/test_fsm_grab_sequence.py -v -k grab_after_settle`
Expected: FAIL

- [ ] **Step 5.3: Переписать `_do_grab`**

Заменить `pi_nodes/nodes/fsm_node.py:473-484` целиком:
```python
    def _do_grab(self):
        """Захват объекта новой 3-фазной логикой (заменяет старую с
        claw/command). См. spec 2026-05-17-arm-grab-sequence.

        Phase 1 (t<=0.1c): один раз публикуем arm/command load_preset grab_hold.
        Phase 2 (0.1c < t < 1.5c): ждём пока _interpolate_tick доедет до позы.
        Phase 3 (t >= 1.5c): freeze всех суставов → RETURNING.

        Settle 1.5с обоснован: самая длинная дельта при переходе
        grab_ready (160,100,180,0) → grab_hold (10,30,180,180) — это CH0
        (150°). При max_speed_deg_per_sec=120 это 1.25с + 0.25с jitter.
        """
        self._grab_t += 0.1

        if self._grab_t <= 0.1:
            # Phase 1 — единичная команда
            self.publish('arm/command',
                         {'command': 'load_preset', 'name': 'grab_hold'},
                         qos=1)
            self.log_info('Arm → grab_hold (closing claw)')
            return

        GRAB_SETTLE_S = 1.5
        if self._grab_t < GRAB_SETTLE_S:
            return

        # Phase 3 — freeze + переход
        self.publish('arm/command', {'command': 'freeze'}, qos=1)
        self.log_info('Arm FROZEN — holding object')
        self._transition(State.RETURNING)
```

- [ ] **Step 5.4: Запустить все grab-тесты — PASS**

Run: `pytest tests/test_fsm_grab_sequence.py -v -k grab`
Expected: 4 PASS

- [ ] **Step 5.5: Запустить весь `test_fsm_grab_sequence.py`**

Run: `pytest tests/test_fsm_grab_sequence.py -v`
Expected: 5 PASS (1 smoke + 2 approach + 2 grab… в total — 5? 1 smoke + 2 approach + 4 grab = 7)
Expected: 7 PASS

- [ ] **Step 5.6: Commit**

```bash
git add tests/test_fsm_grab_sequence.py pi_nodes/nodes/fsm_node.py
git commit -m "feat(fsm): _do_grab — load_preset grab_hold → settle 1.5с → freeze → RETURNING"
```

---

## Task 6: actuators.py — авто-`unfreeze` при открытии клешни

**Files:**
- Modify: `compute_node/dashboard/routers/actuators.py:77-89` (`set_claw`)
- Test: `tests/test_actuators_router.py` (новый)

- [ ] **Step 6.1: Создать test-файл `tests/test_actuators_router.py`**

```python
"""Tests for compute_node/dashboard/routers/actuators.py — фокус на
авто-unfreeze при открытии клешни.

Контекст: после захвата объекта FSM шлёт arm/command freeze. Когда
пользователь/voice/UI открывает клешню — рука должна сама размораживаться,
иначе суставы 1-3 останутся жёстко зафиксированными. Реализация — в
set_claw endpoint: при state="open" (или angle<90) дополнительно
публикуется arm/command {"command":"unfreeze"}.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

fastapi = pytest.importorskip('fastapi')

from fastapi.testclient import TestClient   # noqa: E402

from compute_node.dashboard.app import create_app   # noqa: E402
from compute_node.dashboard.state import DashboardState   # noqa: E402


@pytest.fixture
def fake_mqtt():
    m = MagicMock()
    m.connected = True
    m.publish.return_value = True
    return m


@pytest.fixture
def client(fake_mqtt):
    state = DashboardState()
    app = create_app(state, mqtt=fake_mqtt, ros2=None, enable_socketio=False)
    return TestClient(app)


# ── Tests ──────────────────────────────────────────────────────────────
def test_claw_open_publishes_joint4_and_unfreeze(client, fake_mqtt):
    """POST /api/actuators/claw {state:open} — публикует ДВЕ команды:
    arm/command {joint:4, angle:0} (открыть клешню) и
    arm/command {"command":"unfreeze"} (снять заморозку, потому что
    объект больше не в руке)."""
    r = client.post('/api/v1/actuators/claw', json={'state': 'open'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    topics_and_payloads = [(call.args[0], call.args[1]) for call in pub_calls]

    # joint=4 angle=0
    assert any(t.endswith('arm/command') and p == {'joint': 4, 'angle': 0.0}
               for t, p in topics_and_payloads)
    # unfreeze
    assert any(t.endswith('arm/command') and p == {'command': 'unfreeze'}
               for t, p in topics_and_payloads)


def test_claw_close_does_not_unfreeze(client, fake_mqtt):
    """POST {state:close} — публикуется только joint=4 angle=180.
    Никакого unfreeze (рука как раз должна оставаться frozen для удержания).
    """
    r = client.post('/api/v1/actuators/claw', json={'state': 'close'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    payloads = [call.args[1] for call in pub_calls]

    assert {'joint': 4, 'angle': 180.0} in payloads
    assert {'command': 'unfreeze'} not in payloads


def test_claw_angle_below_90_triggers_unfreeze(client, fake_mqtt):
    """POST {angle: 30} — клешня всё ещё в «открытой» зоне (логически
    меньше середины). Должно сработать как open: + unfreeze.
    """
    r = client.post('/api/v1/actuators/claw', json={'angle': 30.0})
    assert r.status_code == 200

    payloads = [call.args[1] for call in fake_mqtt.publish.call_args_list]
    assert {'joint': 4, 'angle': 30.0} in payloads
    assert {'command': 'unfreeze'} in payloads


def test_claw_angle_above_90_does_not_unfreeze(client, fake_mqtt):
    """POST {angle: 150} — клешня близко к закрытой. Никакого unfreeze."""
    r = client.post('/api/v1/actuators/claw', json={'angle': 150.0})
    assert r.status_code == 200

    payloads = [call.args[1] for call in fake_mqtt.publish.call_args_list]
    assert {'joint': 4, 'angle': 150.0} in payloads
    assert {'command': 'unfreeze'} not in payloads
```

- [ ] **Step 6.2: Запустить — должны быть FAIL для open и angle<90 тестов**

Run: `pytest tests/test_actuators_router.py -v`
Expected: 2 PASS (close, angle>90 — текущая логика уже корректна), 2 FAIL (open, angle<90 — нет unfreeze)

- [ ] **Step 6.3: Реализовать авто-unfreeze в `set_claw`**

В `compute_node/dashboard/routers/actuators.py:77-89`, заменить:
```python
@router.post('/claw', response_model=CommandAck, tags=['actuators'])
async def set_claw(cmd: ClawCommand, mqtt: MQTTDep) -> CommandAck:
    """Клешня = arm joint 4 (1-indexed). open=0°, close=180°."""
    if cmd.angle is not None:
        angle = max(0.0, min(180.0, float(cmd.angle)))
    elif cmd.state == 'open':
        angle = 0.0
    elif cmd.state == 'close':
        angle = 180.0
    else:
        raise HTTPException(400, 'state ("open"/"close") or angle required')
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    return CommandAck()
```
На:
```python
@router.post('/claw', response_model=CommandAck, tags=['actuators'])
async def set_claw(cmd: ClawCommand, mqtt: MQTTDep) -> CommandAck:
    """Клешня = arm joint 4 (1-indexed). open=0°, close=180°.

    При открытии клешни (state=open или angle<90) дополнительно публикуем
    arm/command unfreeze — снимаем заморозку, поставленную FSM в _do_grab.
    Прокси «объект в руке» = «клешня закрыта»; открытие → объект отпущен,
    рука может двигаться. Unfreeze идемпотентен на arm_node (no-op если уже
    разморожен).
    """
    if cmd.angle is not None:
        angle = max(0.0, min(180.0, float(cmd.angle)))
    elif cmd.state == 'open':
        angle = 0.0
    elif cmd.state == 'close':
        angle = 180.0
    else:
        raise HTTPException(400, 'state ("open"/"close") or angle required')
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    if angle < 90.0:
        mqtt.publish('arm/command', {'command': 'unfreeze'}, qos=1)
    return CommandAck()
```

- [ ] **Step 6.4: Запустить — все PASS**

Run: `pytest tests/test_actuators_router.py -v`
Expected: 4 PASS

- [ ] **Step 6.5: Commit**

```bash
git add tests/test_actuators_router.py compute_node/dashboard/routers/actuators.py
git commit -m "feat(actuators): авто-unfreeze руки при открытии клешни (state=open или angle<90)"
```

---

## Task 7: Frontend — `ARM_JOINTS[0].max = 160`

**Files:**
- Modify: `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx:9-22`

- [ ] **Step 7.1: Поднять лимит CH0 в `ARM_JOINTS` и обновить комментарий шапки**

В `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`, заменить шапку-комментарий и массив:
```typescript
/**
 * Servo mapping (PCA9685):
 *   CH0 — Основание   home=0   [0; 120]
 *   CH1 — Сустав 1    home=120 [0; 145]
 *   CH2 — Сустав 2    home=0   [0; 180]
 *   CH3 — Клешня      home=0   [0; 180]  (0=открыта, 180=закрыта)
 *   CH4 — Голова       home=90  [0; 180]
 */

const ARM_JOINTS = [
  { label: 'Основание',  min: 0, max: 120, home: 0   },
  { label: 'Сустав 1',   min: 0, max: 145, home: 120 },
  { label: 'Сустав 2',   min: 0, max: 180, home: 0   },
  { label: 'Клешня',     min: 0, max: 180, home: 0   },
]
```
На:
```typescript
/**
 * Servo mapping (PCA9685):
 *   CH0 — Основание   home=0   [0; 160]
 *   CH1 — Сустав 1    home=120 [0; 145]
 *   CH2 — Сустав 2    home=0   [0; 180]
 *   CH3 — Клешня      home=0   [0; 180]  (0=открыта, 180=закрыта)
 *   CH4 — Голова       home=90  [0; 180]
 *
 * Pre-grab поза grab_ready=[160, 100, 180, 0] требует CH0=160°,
 * поэтому верхняя граница слайдера расширена с 120° до 160°.
 */

const ARM_JOINTS = [
  { label: 'Основание',  min: 0, max: 160, home: 0   },
  { label: 'Сустав 1',   min: 0, max: 145, home: 120 },
  { label: 'Сустав 2',   min: 0, max: 180, home: 0   },
  { label: 'Клешня',     min: 0, max: 180, home: 0   },
]
```

- [ ] **Step 7.2: Запустить TypeScript check (если есть test-команда у фронта)**

Run: `cd compute_node/frontend && npm run build`
Expected: build PASS — Vite ругаться не должен (изменился только числовой литерал в массиве).

- [ ] **Step 7.3: Commit**

```bash
git add compute_node/frontend/src/components/actuators/ServoControlPanel.tsx
git commit -m "feat(ui): расширить лимит CH0 в ServoControlPanel до 160 (под grab_ready)"
```

---

## Task 8: Frontend — закоммитить пересобранные dist-артефакты

**Files:**
- Modify: `compute_node/static/index.html`, `compute_node/static/assets/index-*.js`
- (артефакты Vite после `npm run build`)

⚠ Эта задача делается уже автоматически в Task 7.2. Но фронт-артефакты лежат в репо (см. `compute_node/static/assets/*.js` в git status начала сессии), поэтому отдельный коммит для них = чище история.

- [ ] **Step 8.1: Просмотреть git status в compute_node/static**

Run: `git status compute_node/static/`
Expected: `index.html` modified (изменился hash ссылки), `assets/index-*.js` modified.

- [ ] **Step 8.2: Удалить старые index-*.js / *-chunk.js которые больше не linked**

Run:
```bash
# Список текущих linked-файлов:
grep -oE 'index-[A-Za-z0-9_-]+\.(js|css)' compute_node/static/index.html | sort -u

# Удалить все остальные index-*.js из assets/ кроме этих
ls compute_node/static/assets/index-*.js | head
```
Удалить вручную те, что НЕ linked в `index.html` (избегаем мусора).

- [ ] **Step 8.3: Commit dist**

```bash
git add compute_node/static/
git commit -m "build(frontend): rebuild — ARM_JOINTS CH0 max=160"
```

---

## Task 9: Интеграционный smoke (ручной)

Эта задача — чек-лист verification без кода. Использовать после merge на железе.

- [ ] **Step 9.1: Запустить весь Pi-стек локально (или на роботе)**

Run: `./samurai.sh robot`
Ожидание: arm_node стартует, в логах появляется:
- `Migration: created arm preset "grab_ready"=[160.0, 100.0, 180.0, 0.0]` (если первый запуск)
- `Migration: created arm preset "grab_hold"=...`
- `Arm node ready (4 joints, channels=[0, 1, 2, 3], locked=true)`

- [ ] **Step 9.2: Поднять compute-стек и открыть UI**

Run: `./samurai.sh compute` → `http://localhost:5000/`
Перейти на вкладку Hardware → секция Сервоприводы.

- [ ] **Step 9.3: Разблокировать руку**

В UI: кнопка «Разблокировать» (Arm). Ожидание: рука переходит в home плавно (CH1: 0→120°). Это уже видно как «плавный» переход — не рывок.

- [ ] **Step 9.4: Слайдер CH0 — проверить лимит 160°**

Тянуть CH0 от 0 до максимума. Ожидание: слайдер допускает 160° (не зажимается на 120°). Серво едет плавно, без рывков.

- [ ] **Step 9.5: Загрузить пресет grab_ready**

В Presets-секции Arm: жмём «Загр.» возле `grab_ready`. Ожидание: все 4 сустава плавно идут в [160, 100, 180, 0]. Время — около 1.5с (определяется max_speed=120°/с и самой длинной дельтой).

- [ ] **Step 9.6: Загрузить пресет grab_hold**

Жмём «Загр.» возле `grab_hold`. Ожидание: рука плавно идёт в [10, 30, 180, 180]. CH3 уходит в 180° (логически — клешня закрыта; физически с инверсией это 0° — клешня сжата).

- [ ] **Step 9.7: Открыть клешню — проверить auto-unfreeze**

Сценарий полного цикла:
1. Загрузить `grab_hold` → жмём «Замор. все» (или ждём пока FSM сам).
2. Жмём UI-кнопку «Open» возле клешни (или POST `/api/v1/actuators/claw {"state":"open"}` через curl).
3. Ожидание: клешня плавно открывается (CH3 → 0°), параллельно StatusBadge меняется с FROZEN на ACTIVE.

- [ ] **Step 9.8: Полный FSM-захват (если есть мяч под руку)**

Положить красный мяч в поле зрения. Через UI/voice: «возьми красный мяч».
Ожидание лог-цепочка:
- `FSM: IDLE → SEARCHING`
- `Ball spotted: red — targeting` → `FSM: SEARCHING → TARGETING`
- `Ball centred — approaching red` → `FSM: TARGETING → APPROACHING`
- `Arm → grab_ready (approach start)` (один раз)
- При доезде → `FSM: APPROACHING → GRABBING`
- `Arm → grab_hold (closing claw)`
- Через ~1.5с: `Arm FROZEN — holding object`, `FSM: GRABBING → RETURNING`

- [ ] **Step 9.9: Регрессионный pytest полный набор**

Run: `pytest tests/ -v --tb=short -x`
Expected: все существующие тесты PASS (никаких регрессий от наших изменений).

---

## Self-Review (после написания плана)

**Spec coverage:**

| Spec секция | Реализуется в Task |
|---|---|
| §3.1 Структурное | Tasks 1-7 (диаграмма) |
| §3.2 Target/current модель | Task 1 |
| §3.3 Авто-миграция пресетов | Task 2 |
| §3.4 FSM `_transition` + `_do_approach` + `_do_grab` | Tasks 4, 5 |
| §3.5 Авто-unfreeze в `set_claw` | Task 6 |
| §3.6 Конфиг (`max_angles[0]=160`, `max_speed`) | Task 3 |
| §3.7 UI `ARM_JOINTS[0].max=160` | Task 7 |
| §3.8 `arm/state` публикует current | Task 1 (Step 1.12) |
| §4 Edge cases | Покрытие в тестах Tasks 1, 4, 5, 6 |
| §5 Тестирование | Tasks 1-6 (unit), Task 9 (manual) |

Все секции спека покрыты.

**Placeholder scan:** Проверены все «TBD», «TODO», «implement later» — отсутствуют.

**Type consistency:**
- Поля FSM: `_approach_arm_sent: bool`, `_grab_t: float` — единообразно во всех тестах и в `_transition`.
- Поля arm_node: `_target_angles: list[float]`, `_current_angles: list[float]`, `_max_speed: float`.
- MQTT payloads: `{'command': 'load_preset', 'name': 'grab_ready'}`, `{'command': 'freeze'}`, `{'command': 'unfreeze'}`, `{'joint': 4, 'angle': 0.0}` — единообразно во всех Task'ах.
- Имена пресетов: `grab_ready`, `grab_hold` — единообразно.
- Константа `GRAB_SETTLE_S = 1.5` локальна в `_do_grab` (не глобал).

Всё согласовано.
