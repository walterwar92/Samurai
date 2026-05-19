# Default-Frozen CH0/1/2 Arm Joints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Сделать CH0/CH1/CH2 (основание + 2 сустава руки) frozen по умолчанию; команды `home`/`load_preset`/`{"joints":[…]}` сначала двигают суставы, затем снова морозят CH0/1/2. CH3 (клешня) и CH4 (голова) не затрагиваются.

**Architecture:** На бэке `arm_node.py` вызываем `_freeze_all_except_claw()` в 4 точках: после `_unlock()` (стартовая инициализация), и после установки целевых углов в каждой из трёх mass-команд (home / load_preset / joints-array). Все эти mass-команды переходят на `_set_joint(allow_frozen=True)`, чтобы переопределять предыдущее frozen-состояние. Single-joint drag, ServoDriver и фронтенд не меняются — UI уже умеет показывать frozen-состояние из `arm/state`.

**Tech Stack:** Python 3 (pi_nodes/), pytest, MagicMock-фикстура.

---

## File Structure

- **Modify:** `pi_nodes/nodes/arm_node.py` — `_unlock`, `_cmd_cb` (string-home, dict-home, load_preset, joints-array), docstring `_set_joint`.
- **Modify:** `tests/test_arm_node.py` — добавить параметр `reset_after_init` в фикстуру, переписать 4 существующих теста с инвертированной семантикой, добавить 4 новых.
- **Spec (already committed):** `docs/superpowers/specs/2026-05-19-default-frozen-arm-ch012-design.md`.
- **Без изменений:** `pi_nodes/hardware/servo_driver.py`, `pi_nodes/nodes/fsm_node.py`, фронтенд, Android.

---

### Task 1: Extend test fixture with `reset_after_init` parameter

**Why:** Новый тест `test_unlock_auto_freezes_ch0_ch1_ch2` должен проверить, что `_unlock()` (вызванный в `__init__`) дёрнул `s.freeze()` на CH0/1/2. Сейчас фикстура безусловно делает `m.reset_mock()` в конце — мы стираем эти вызовы. Добавляем флаг для одного теста, не ломая остальные.

**Files:**
- Modify: `tests/test_arm_node.py` (фикстура `arm_node_factory`, строки ~23-92)

- [ ] **Step 1: Add `reset_after_init` kwarg to `_factory`**

Найти в `tests/test_arm_node.py` строку ~31:

```python
    def _factory(max_speed: float = 120.0, presets_seed: dict | None = None):
```

Заменить на:

```python
    def _factory(max_speed: float = 120.0, presets_seed: dict | None = None,
                 reset_after_init: bool = True):
```

Найти блок reset_mock (строки ~86-89):

```python
        # Сброс mock-счётчиков: _unlock() в __init__ уже вызвал set_angle
        # для всех серво, тестам интереснее то, что произошло ПОСЛЕ старта.
        for m in instances:
            m.reset_mock()
```

Заменить на:

```python
        # Сброс mock-счётчиков: _unlock() в __init__ уже вызвал set_angle
        # для всех серво, тестам интереснее то, что произошло ПОСЛЕ старта.
        # Тесты, проверяющие саму инициализацию (__init__ → _unlock), могут
        # передать reset_after_init=False, чтобы увидеть set_angle/freeze
        # вызовы из стартового unlock'а.
        if reset_after_init:
            for m in instances:
                m.reset_mock()
```

- [ ] **Step 2: Run existing tests to confirm no regression**

Run: `pytest tests/test_arm_node.py -v`
Expected: все тесты PASS (новый параметр имеет default `True`, поведение фикстуры не меняется).

- [ ] **Step 3: Commit**

```bash
git add tests/test_arm_node.py
git commit -m "test(arm_node): add reset_after_init param to fixture"
```

---

### Task 2: Auto-freeze CH0/1/2 in `_unlock()` (TDD)

**Why:** Дать арм-ноде дефолтное состояние «руки висят» сразу после инициализации, без явной команды freeze от UI/FSM.

**Files:**
- Test: `tests/test_arm_node.py` (новый тест)
- Modify: `pi_nodes/nodes/arm_node.py:178-187` (метод `_unlock`)

- [ ] **Step 1: Add failing test `test_unlock_auto_freezes_ch0_ch1_ch2`**

Добавить в конец `tests/test_arm_node.py` (перед строкой с следующим разделом, либо просто в конец):

```python
def test_unlock_auto_freezes_ch0_ch1_ch2(arm_node_factory):
    """`_unlock()` в __init__ морозит CH0/1/2 (default frozen-state).
    Клешня (CH3) остаётся свободной — общая freeze-семантика
    «всё кроме клешни», уже зашитая в `_freeze_all_except_claw`.
    """
    node = arm_node_factory(reset_after_init=False)

    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_arm_node.py::test_unlock_auto_freezes_ch0_ch1_ch2 -v`
Expected: FAIL с `AssertionError: Expected 'freeze' to have been called once. Called 0 times.` (т.к. `_unlock` пока не морозит).

- [ ] **Step 3: Implement auto-freeze in `_unlock`**

Открыть `pi_nodes/nodes/arm_node.py`. Найти метод `_unlock` (строки ~178-187):

```python
    def _unlock(self):
        """Unlock arm and initialize servos to home angles."""
        self._locked = False
        if not self._servo_initialized:
            with self._state_lock:
                for i in range(self._num_joints):
                    phys = self._to_physical(i, self._home_angles[i])
                    self._servos[i].set_angle(phys, force=True)
                    self._target_angles[i] = self._current_angles[i] = float(self._home_angles[i])
            self._servo_initialized = True
```

Заменить на:

```python
    def _unlock(self):
        """Unlock arm and initialize servos to home angles.

        На выходе CH0/1/2 заморожены (default-state «руки висят»),
        CH3 свободна. Пользователь может разморозить через UI; следующий
        home/load_preset/{joints:[…]} снова их заморозит.
        """
        self._locked = False
        if not self._servo_initialized:
            with self._state_lock:
                for i in range(self._num_joints):
                    phys = self._to_physical(i, self._home_angles[i])
                    self._servos[i].set_angle(phys, force=True)
                    self._target_angles[i] = self._current_angles[i] = float(self._home_angles[i])
            self._servo_initialized = True
            self._freeze_all_except_claw()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_arm_node.py::test_unlock_auto_freezes_ch0_ch1_ch2 -v`
Expected: PASS.

- [ ] **Step 5: Run full arm_node test suite for regressions**

Run: `pytest tests/test_arm_node.py -v`
Expected: все тесты PASS. Если `test_init_target_equals_current_equals_home` или другие падают — проверь логику.

- [ ] **Step 6: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): auto-freeze CH0/1/2 on _unlock"
```

---

### Task 3: `home` command overrides frozen + re-freezes (TDD)

**Why:** Команда `home` теперь должна двигать руку в home независимо от текущего frozen-state, и затем заморозить CH0/1/2. Это касается обеих форм — JSON `{"command":"home"}` и строки `"home"`.

**Files:**
- Modify: `tests/test_arm_node.py` (тесты на строках 321 и 361, плюс новый)
- Modify: `pi_nodes/nodes/arm_node.py:256-261` (string-ветка) и `pi_nodes/nodes/arm_node.py:289-294` (dict-ветка)

- [ ] **Step 1: Flip existing test `test_cmd_cb_home_skips_frozen` to new semantics**

Найти в `tests/test_arm_node.py` тест `test_cmd_cb_home_skips_frozen` (строки 321-332):

```python
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
```

Заменить целиком на:

```python
def test_cmd_cb_home_overrides_frozen_and_refreezes(arm_node_factory):
    """home переопределяет frozen: target всех суставов → home_angles,
    после чего CH0/1/2 морозятся снова. Семантика: «возврат в известное
    положение всегда морозит base+joints»."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0   # frozen-сустав в нестандартной позе
    node._target_angles[1] = 60.0

    node._cmd_cb('arm/command', {'command': 'home'})

    # Все суставы уехали в home_angles=[0, 120, 0, 0]
    assert node._target_angles == [0.0, 120.0, 0.0, 0.0]
    # CH0/1/2 заморожены через _freeze_all_except_claw, CH3 не дёргали
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 2: Flip `test_cmd_cb_string_home_skips_frozen` similarly**

Найти `test_cmd_cb_string_home_skips_frozen` (строки 361-371):

```python
def test_cmd_cb_string_home_skips_frozen(arm_node_factory):
    """Legacy string-form 'home' тоже уважает frozen (не двигает заморож. сустав)."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0
    node._target_angles[1] = 60.0

    node._cmd_cb('arm/command', 'home')

    assert node._target_angles[0] == 30.0    # frozen не изменился
    assert node._target_angles[1] == 120.0   # unfrozen уехал в home
```

Заменить на:

```python
def test_cmd_cb_string_home_overrides_frozen_and_refreezes(arm_node_factory):
    """Legacy string-form 'home' тоже переопределяет frozen и морозит снова."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 30.0
    node._target_angles[1] = 60.0

    node._cmd_cb('arm/command', 'home')

    assert node._target_angles == [0.0, 120.0, 0.0, 0.0]
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_home_overrides_frozen_and_refreezes tests/test_arm_node.py::test_cmd_cb_string_home_overrides_frozen_and_refreezes -v`
Expected: FAIL — текущая реализация всё ещё пропускает frozen-сустав в home.

- [ ] **Step 4: Update implementation — string-ветка `home`**

Открыть `pi_nodes/nodes/arm_node.py`. Найти строки ~256-261 (внутри `_cmd_cb`):

```python
            if cmd_lower == 'home':
                self._unlock_if_needed()
                for i in range(self._num_joints):
                    self._set_joint(i, self._home_angles[i])
                self.log_info('Arm → HOME')
                return
```

Заменить на:

```python
            if cmd_lower == 'home':
                self._unlock_if_needed()
                for i in range(self._num_joints):
                    self._set_joint(i, self._home_angles[i], allow_frozen=True)
                self._freeze_all_except_claw()
                self.log_info('Arm → HOME (CH0/1/2 re-frozen)')
                return
```

- [ ] **Step 5: Update implementation — dict-ветка `home`**

Найти строки ~289-294 (внутри того же `_cmd_cb`, ниже `cmd = d.get('command', '')`):

```python
        if cmd == 'home':
            self._unlock_if_needed()
            for i in range(self._num_joints):
                self._set_joint(i, self._home_angles[i])
            self.log_info('Arm → HOME')
            return
```

Заменить на:

```python
        if cmd == 'home':
            self._unlock_if_needed()
            for i in range(self._num_joints):
                self._set_joint(i, self._home_angles[i], allow_frozen=True)
            self._freeze_all_except_claw()
            self.log_info('Arm → HOME (CH0/1/2 re-frozen)')
            return
```

- [ ] **Step 6: Run home tests to verify they pass**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_home_overrides_frozen_and_refreezes tests/test_arm_node.py::test_cmd_cb_string_home_overrides_frozen_and_refreezes -v`
Expected: PASS.

- [ ] **Step 7: Run full suite for regressions**

Run: `pytest tests/test_arm_node.py -v`
Expected: все PASS.

- [ ] **Step 8: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): home overrides frozen and re-freezes CH0/1/2"
```

---

### Task 4: `load_preset` overrides frozen + re-freezes (TDD)

**Why:** Без этого FSM grab после default-freeze просто не отработает: `load_preset grab_ready` улетит в no-op для CH0/1/2.

**Files:**
- Modify: `tests/test_arm_node.py:335-345` (`test_cmd_cb_load_preset_skips_frozen`)
- Modify: `pi_nodes/nodes/arm_node.py:338-351`

- [ ] **Step 1: Flip existing test**

Найти `test_cmd_cb_load_preset_skips_frozen` (строки 335-345):

```python
def test_cmd_cb_load_preset_skips_frozen(arm_node_factory):
    """load_preset обновляет target только для unfrozen суставов."""
    node = arm_node_factory(presets_seed={'arm': {'foo': [10.0, 20.0, 30.0, 40.0]}})
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'command': 'load_preset', 'name': 'foo'})

    assert node._target_angles[0] == 100.0  # frozen не изменился
    assert node._target_angles[1] == 20.0   # unfrozen загрузил из preset
```

Заменить целиком на:

```python
def test_cmd_cb_load_preset_overrides_frozen_and_refreezes(arm_node_factory):
    """load_preset переопределяет frozen и морозит CH0/1/2 заново.

    Это критично для FSM grab: load_preset grab_hold вызывается ПОСЛЕ
    load_preset grab_ready (который уже заморозил суставы), и без
    allow_frozen=True hot-target никогда бы не обновился.
    """
    node = arm_node_factory(presets_seed={'arm': {'foo': [10.0, 20.0, 30.0, 40.0]}})
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'command': 'load_preset', 'name': 'foo'})

    # Все суставы взяли значения из preset, включая frozen CH0
    assert node._target_angles == [10.0, 20.0, 30.0, 40.0]
    # И сразу заморожены CH0/1/2 снова
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_load_preset_overrides_frozen_and_refreezes -v`
Expected: FAIL.

- [ ] **Step 3: Update implementation**

Открыть `pi_nodes/nodes/arm_node.py`. Найти строки ~338-351:

```python
        if cmd == 'load_preset':
            name = d.get('name', '').strip()
            if not name:
                self.log_warn('load_preset: name required')
                return
            angles = self._presets.load_preset('arm', name)
            if angles is None:
                self.log_warn('Preset not found: arm/%s', name)
                return
            self._unlock_if_needed()
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Preset loaded: arm/%s → %s', name, self._target_angles)
            return
```

Заменить на:

```python
        if cmd == 'load_preset':
            name = d.get('name', '').strip()
            if not name:
                self.log_warn('load_preset: name required')
                return
            angles = self._presets.load_preset('arm', name)
            if angles is None:
                self.log_warn('Preset not found: arm/%s', name)
                return
            self._unlock_if_needed()
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a), allow_frozen=True)
            self._freeze_all_except_claw()
            self.log_info('Preset loaded: arm/%s → %s (CH0/1/2 re-frozen)',
                          name, self._target_angles)
            return
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_load_preset_overrides_frozen_and_refreezes -v`
Expected: PASS.

- [ ] **Step 5: Run full suite for regressions**

Run: `pytest tests/test_arm_node.py -v`
Expected: все PASS.

- [ ] **Step 6: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): load_preset overrides frozen and re-freezes CH0/1/2"
```

---

### Task 5: `{"joints":[…]}` mass-command overrides frozen + re-freezes (TDD)

**Why:** Симметрично с home/load_preset — единая семантика для всех «mass-команд».

**Files:**
- Modify: `tests/test_arm_node.py:348-358` (`test_cmd_cb_joints_array_skips_frozen`)
- Modify: `pi_nodes/nodes/arm_node.py:386-393`

- [ ] **Step 1: Flip existing test**

Найти `test_cmd_cb_joints_array_skips_frozen` (строки 348-358):

```python
def test_cmd_cb_joints_array_skips_frozen(arm_node_factory):
    """{joints:[...]} обновляет target только для unfrozen суставов."""
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'joints': [10.0, 20.0, 30.0, 40.0]})

    assert node._target_angles[0] == 100.0  # frozen не изменился
    assert node._target_angles[1] == 20.0   # unfrozen загрузил
```

Заменить на:

```python
def test_cmd_cb_joints_array_overrides_frozen_and_refreezes(arm_node_factory):
    """{joints:[...]} тоже переопределяет frozen и морозит CH0/1/2 снова.
    Симметрично с home/load_preset — все mass-команды ведут себя одинаково.
    """
    node = arm_node_factory()
    node._mock_servos[0].frozen = True
    node._target_angles[0] = 100.0
    node._target_angles[1] = 100.0

    node._cmd_cb('arm/command', {'joints': [10.0, 20.0, 30.0, 40.0]})

    assert node._target_angles == [10.0, 20.0, 30.0, 40.0]
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_joints_array_overrides_frozen_and_refreezes -v`
Expected: FAIL.

- [ ] **Step 3: Update implementation**

Открыть `pi_nodes/nodes/arm_node.py`. Найти строки ~386-393:

```python
        # All joints: {"joints": [90, 90, 90, 90]}
        if 'joints' in d:
            self._unlock_if_needed()
            angles = d['joints']
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a))
            self.log_info('Arm all joints → %s', self._target_angles)
            return
```

Заменить на:

```python
        # All joints: {"joints": [90, 90, 90, 90]}
        if 'joints' in d:
            self._unlock_if_needed()
            angles = d['joints']
            for i, a in enumerate(angles[:self._num_joints]):
                self._set_joint(i, float(a), allow_frozen=True)
            self._freeze_all_except_claw()
            self.log_info('Arm all joints → %s (CH0/1/2 re-frozen)',
                          self._target_angles)
            return
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_arm_node.py::test_cmd_cb_joints_array_overrides_frozen_and_refreezes -v`
Expected: PASS.

- [ ] **Step 5: Run full suite for regressions**

Run: `pytest tests/test_arm_node.py -v`
Expected: все PASS.

- [ ] **Step 6: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): joints-array overrides frozen and re-freezes CH0/1/2"
```

---

### Task 6: Update `_set_joint` docstring

**Why:** Docstring сейчас утверждает, что mass-команды НЕ двигают frozen-сустав. После Task 3-5 это уже не так — все mass-команды передают `allow_frozen=True`. Чтобы не путать будущих контрибьюторов, обновляем формулировку.

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py:209-228` (docstring `_set_joint`)

- [ ] **Step 1: Update docstring**

Открыть `pi_nodes/nodes/arm_node.py`. Найти метод `_set_joint` (строки ~209-228):

```python
    def _set_joint(self, idx: int, angle: float, allow_frozen: bool = False):
        """Set joint TARGET angle (логический) с лимитами.

        Реальный PWM шлёт _interpolate_tick @ 50Гц, плавно шагая current
        к target с max_speed_deg_per_sec.

        Если сустав frozen и allow_frozen=False — target НЕ обновляется
        (mass-команды home/preset/joints-array не двигают замороженный
        сустав, чтобы случайно не сорвать захват мяча после FSM grab).
        Single-joint команды от UI слайдера передают allow_frozen=True —
        пользователь явно целится в конкретный сустав, разрешаем.
        """
```

Заменить на:

```python
    def _set_joint(self, idx: int, angle: float, allow_frozen: bool = False):
        """Set joint TARGET angle (логический) с лимитами.

        Реальный PWM шлёт _interpolate_tick @ 50Гц, плавно шагая current
        к target с max_speed_deg_per_sec.

        Если сустав frozen и allow_frozen=False — target НЕ обновляется.
        В текущем коде все источники (single-joint от UI слайдера,
        mass-команды home/load_preset/joints-array) передают
        allow_frozen=True. Default False оставлен как защитная семантика
        на случай новых внутренних вызовов.
        """
```

- [ ] **Step 2: Run full suite (sanity-check)**

Run: `pytest tests/test_arm_node.py -v`
Expected: все PASS (docstring изменение не влияет на behavior).

- [ ] **Step 3: Commit**

```bash
git add pi_nodes/nodes/arm_node.py
git commit -m "docs(arm_node): refresh _set_joint docstring for new mass-cmd semantics"
```

---

### Task 7: Add unfreeze-persistence + integration test (TDD)

**Why:** Проверить два сценария, не покрытых индивидуальными task'ами:
1. После ручной разморозки single-joint командой `frozen` остаётся False (не происходит «auto-refreeze на каждой команде»).
2. End-to-end FSM grab sequence работает на default-frozen ноде.

**Files:**
- Modify: `tests/test_arm_node.py` (добавить два теста)

- [ ] **Step 1: Add `test_unfreeze_persists_through_single_joint_command`**

Добавить в конец `tests/test_arm_node.py`:

```python
def test_unfreeze_persists_through_single_joint_command(arm_node_factory):
    """После arm/command "unfreeze" одиночная команда {joint:1, angle:50}
    НЕ морозит CH0 обратно. Авто-refreeze случается только в mass-командах
    (home/load_preset/joints-array)."""
    node = arm_node_factory()
    # Старт: CH0/1/2 заморожены автоматически. Эмулируем это в моках.
    for i in range(3):
        node._mock_servos[i].frozen = True

    # Пользователь жмёт "Разм. все"
    node._cmd_cb('arm/command', {'command': 'unfreeze'})
    for s in node._mock_servos:
        s.frozen = False   # эмулируем эффект unfreeze() на моках

    # Очищаем call-history после unfreeze, чтобы видеть только последующие freeze'ы.
    for m in node._mock_servos:
        m.reset_mock()

    # Single-joint drag не должен ничего морозить
    node._cmd_cb('arm/command', {'joint': 1, 'angle': 50.0})

    assert node._target_angles[0] == 50.0
    for m in node._mock_servos:
        m.freeze.assert_not_called()
```

- [ ] **Step 2: Add `test_home_after_unfreeze_refreezes_again`**

Добавить следом:

```python
def test_home_after_unfreeze_refreezes_again(arm_node_factory):
    """После unfreeze всех суставов команда home должна снова заморозить
    CH0/1/2 — «возврат в известное положение всегда морозит»."""
    node = arm_node_factory()

    # Эмуляция unfreeze
    node._cmd_cb('arm/command', {'command': 'unfreeze'})
    for s in node._mock_servos:
        s.frozen = False

    # Сброс счётчиков
    for m in node._mock_servos:
        m.reset_mock()

    # home
    node._cmd_cb('arm/command', {'command': 'home'})

    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 3: Add `test_fsm_grab_sequence_end_to_end_with_default_frozen`**

Добавить следом:

```python
def test_fsm_grab_sequence_end_to_end_with_default_frozen(arm_node_factory):
    """E2E: arm_node стартует с default-frozen CH0/1/2 (после _unlock).
    FSM шлёт load_preset grab_ready → load_preset grab_hold → freeze.
    Никаких прямых unfreeze. Target должен в итоге доехать до grab_hold,
    финальный freeze на CH0/1/2 идёт через _freeze_all_except_claw
    (FSM сам отдельно делает freeze joint=4 для клешни — не тестируем).
    """
    node = arm_node_factory(reset_after_init=False)
    # _unlock уже отморозил CH0/1/2 в __init__
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    # Эмулируем эффект freeze() на моках
    for i in range(3):
        node._mock_servos[i].frozen = True

    # Сброс mock-call-history для чистого подсчёта в последующих шагах
    for m in node._mock_servos:
        m.reset_mock()

    # FSM шаг 1: grab_ready
    node._cmd_cb('arm/command', {'command': 'load_preset', 'name': 'grab_ready'})
    # grab_ready preset из миграции: [110, 100, 180, 0]
    assert node._target_angles == [110.0, 100.0, 180.0, 0.0]
    # CH0/1/2 заморожены снова
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()

    for m in node._mock_servos:
        m.reset_mock()

    # FSM шаг 2: grab_hold
    node._cmd_cb('arm/command', {'command': 'load_preset', 'name': 'grab_hold'})
    # grab_hold preset: [10, 30, 180, 180]
    assert node._target_angles == [10.0, 30.0, 180.0, 180.0]
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()

    for m in node._mock_servos:
        m.reset_mock()

    # FSM шаг 3: финальный freeze (CH0/1/2 уже frozen, но команда идемпотентна)
    node._cmd_cb('arm/command', {'command': 'freeze'})
    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 4: Run new tests**

Run: `pytest tests/test_arm_node.py::test_unfreeze_persists_through_single_joint_command tests/test_arm_node.py::test_home_after_unfreeze_refreezes_again tests/test_arm_node.py::test_fsm_grab_sequence_end_to_end_with_default_frozen -v`
Expected: все PASS (импл уже готова после Task 2-5).

- [ ] **Step 5: Run full arm_node + fsm regression**

Run: `pytest tests/test_arm_node.py tests/test_fsm_grab_sequence.py -v`
Expected: все PASS. Если что-то падает в FSM-тестах — открой spec, проверь, что FSM публикует те же arm/command, что и раньше (он публикует — мы не трогали fsm_node.py).

- [ ] **Step 6: Commit**

```bash
git add tests/test_arm_node.py
git commit -m "test(arm_node): unfreeze-persistence + FSM grab e2e with default frozen"
```

---

### Task 8: Final regression + push

**Why:** Финальная проверка на весь тест-сьют, затем пуш.

**Files:** —

- [ ] **Step 1: Full test suite**

Run: `pytest tests/ -v 2>&1 | tail -50`
Expected: все тесты зелёные. Если что-то ломается вне arm_node/fsm — это unrelated, проверь.

- [ ] **Step 2: Manual smoke check (опционально, если есть Pi доступ)**

На Pi: `./samurai.sh robot` → дашборд `:5000` → секция «Сервоприводы»:
- Сразу после старта arm-секция показывает HOLD-бейджи на ch0/1/2, без бейджа на ch3.
- Тяну слайдер ch1 — слайдер двигается, сустав едет, HOLD остаётся.
- Жму «Разм. все» → бейджи HOLD исчезают со всех суставов.
- Жму «Домой» → рука едет в home, HOLD появляется на ch0/1/2 снова.
- Жму «Тест захвата» → последовательность отрабатывает, в финале HOLD на ch0/1/2 (ch3 — отдельно от FSM).

Если живого железа нет — пропусти этот шаг, всё покрыто unit-тестами.

- [ ] **Step 3: Push**

```bash
git push origin <текущая-ветка>
```

Если ветка `dev` — пуш по правилу пользователя (см. memory: feedback_push_after_phase).

---

## Self-Review

**Spec coverage:**

| Spec section | Plan task |
|---|---|
| Goal — CH0/1/2 default frozen | Task 2 |
| `_unlock` → freeze | Task 2 |
| `home` (string + dict) | Task 3 |
| `load_preset` | Task 4 |
| `{"joints":[…]}` | Task 5 |
| `_set_joint` docstring | Task 6 |
| Test: unlock auto-freezes | Task 2 |
| Test: home overrides | Task 3 |
| Test: load_preset overrides | Task 4 |
| Test: joints-array overrides | Task 5 |
| Test: unfreeze persists | Task 7 |
| Test: FSM grab end-to-end | Task 7 |
| Risk: race set_target → freeze | покрыт существующим `test_interpolate_tick_moves_frozen_joint_to_target` |
| Acceptance — no Frontend changes | подтверждено в плане (no-op) |

**Placeholder scan:** проверено — все code blocks полные, exact paths, exact commands. Никаких «add error handling» или «similar to Task N».

**Type consistency:**
- `_freeze_all_except_claw()` используется во всех 4 точках одинаково (вызов без аргументов).
- `_set_joint(idx, angle, allow_frozen=True)` — везде одна сигнатура.
- Тест-фикстура `reset_after_init=False` — одинаковый kwarg во всех новых тестах.

**Сценарии out-of-scope:** Head (CH4), ServoDriver, frontend, fsm_node.py, Android. Все явно вынесены в spec §«Что НЕ делаем».

---

Plan complete and saved to `docs/superpowers/plans/2026-05-19-default-frozen-arm-ch012.md`. Two execution options:

**1. Subagent-Driven (recommended)** — я диспатчу свежий subagent на каждый Task, ревью между task'ами, быстрая итерация.

**2. Inline Execution** — выполняю task'и в этой сессии через executing-plans, batch с checkpoint'ами для ревью.

Какой подход?
