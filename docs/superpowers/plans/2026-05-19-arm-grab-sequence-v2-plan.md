# Arm Grab v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Заменить 3-фазный `_do_grab` на 5-фазную последовательность (open → 1s → grab_hold → settle+1s → grab_return → RETURNING) с auto-unfreeze клешни через 20с; фикс бага «открой клешню → размораживаются все суставы».

**Architecture:** В `arm_node` расширяем команду `freeze` параметром `duration` и ведём `_freeze_timers[]` per-joint (`threading.Timer`). Авто-миграция нового пресета `grab_return = [30, 60, 0, 180]`. В `fsm_node._do_grab` пишем 5-фазную логику на флагах `_grab_open_sent`, `_grab_hold_sent`. В `actuators.py` фиксим bare unfreeze → `unfreeze joint=4`.

**Tech Stack:** Python 3.11 (Pi/compute), `threading.Timer`, `paho-mqtt`, `pytest`, FastAPI TestClient.

**Spec:** [docs/superpowers/specs/2026-05-19-arm-grab-sequence-v2-design.md](../specs/2026-05-19-arm-grab-sequence-v2-design.md)

---

## File Structure

**Modify:**
- `pi_nodes/nodes/arm_node.py` — add `_freeze_timers`, `_freeze_joint`, `_auto_unfreeze`, `_unfreeze_joint`; extend `_cmd_cb` freeze/unfreeze; refactor `_freeze_all_except_claw`; add `grab_return` to `_DEFAULT_ARM_PRESETS`.
- `pi_nodes/nodes/fsm_node.py` — add `_grab_open_sent`, `_grab_hold_sent` state fields; reset in `_transition`; rewrite `_do_grab` to 5 phases.
- `compute_node/dashboard/routers/actuators.py` — fix bare unfreeze → `unfreeze joint=4` in `set_claw` (line 97).
- `tests/test_arm_node.py` — add tests for duration timer + grab_return migration.
- `tests/test_fsm_grab_sequence.py` — replace existing grab tests with v2-phase tests (existing tests will break — they test old 3-phase API).
- `tests/test_actuators_router.py` — update existing tests to expect `unfreeze joint=4` payload.

**No new files.** All changes inline in existing modules.

---

## Task 1: arm_node — `_freeze_timers` field + `_freeze_joint`/`_auto_unfreeze` helpers

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py`
- Test: `tests/test_arm_node.py`

**Context:** Сейчас `freeze`-команда в `_cmd_cb` напрямую дёргает `self._servos[idx].freeze()`. Нам нужно добавить промежуточный слой с per-joint таймером для auto-unfreeze. Этот таск — только новые helpers + state, БЕЗ изменения `_cmd_cb` (это в Task 2).

- [ ] **Step 1.1: Write failing test for `_freeze_joint` with duration**

Добавь в конец `tests/test_arm_node.py`:

```python
def test_freeze_joint_with_duration_starts_timer(arm_node_factory):
    """_freeze_joint(idx, duration) вызывает _servos[idx].freeze() и
    создаёт активный threading.Timer в _freeze_timers[idx].
    """
    import threading
    node = arm_node_factory()
    assert node._freeze_timers == [None, None, None, None]

    node._freeze_joint(3, duration=10.0)

    node._mock_servos[3].freeze.assert_called_once()
    assert node._freeze_timers[3] is not None
    assert isinstance(node._freeze_timers[3], threading.Timer)
    # Cleanup: cancel timer (real Timer, иначе процесс ждёт 10с)
    node._freeze_timers[3].cancel()


def test_freeze_joint_without_duration_no_timer(arm_node_factory):
    """_freeze_joint(idx) без duration вызывает freeze, но не создаёт таймер."""
    node = arm_node_factory()
    node._freeze_joint(3, duration=None)

    node._mock_servos[3].freeze.assert_called_once()
    assert node._freeze_timers[3] is None


def test_freeze_joint_duration_zero_no_timer(arm_node_factory):
    """duration=0 — не создаём таймер (degenerate case)."""
    node = arm_node_factory()
    node._freeze_joint(3, duration=0.0)

    node._mock_servos[3].freeze.assert_called_once()
    assert node._freeze_timers[3] is None


def test_freeze_joint_restarts_timer(arm_node_factory):
    """Повторный freeze с duration отменяет предыдущий таймер и стартует новый."""
    import threading
    node = arm_node_factory()
    node._freeze_joint(3, duration=10.0)
    first_timer = node._freeze_timers[3]
    assert first_timer.is_alive()

    node._freeze_joint(3, duration=10.0)
    second_timer = node._freeze_timers[3]

    assert first_timer is not second_timer
    assert not first_timer.is_alive()    # cancelled
    assert isinstance(second_timer, threading.Timer)
    second_timer.cancel()


def test_freeze_joint_timer_calls_auto_unfreeze(arm_node_factory):
    """После duration секунд таймер вызывает _servos[idx].unfreeze().

    Используем маленький duration (50ms) + sleep чтобы реально дождаться.
    """
    import time
    node = arm_node_factory()
    node._freeze_joint(3, duration=0.05)

    time.sleep(0.15)    # запас на jitter timer-thread

    node._mock_servos[3].unfreeze.assert_called_once()
    assert node._freeze_timers[3] is None    # очищен в _auto_unfreeze
```

- [ ] **Step 1.2: Run tests — expect FAIL (no _freeze_timers / _freeze_joint)**

```bash
pytest tests/test_arm_node.py::test_freeze_joint_with_duration_starts_timer tests/test_arm_node.py::test_freeze_joint_without_duration_no_timer tests/test_arm_node.py::test_freeze_joint_duration_zero_no_timer tests/test_arm_node.py::test_freeze_joint_restarts_timer tests/test_arm_node.py::test_freeze_joint_timer_calls_auto_unfreeze -v
```

Expected: 5 FAILS, типа `AttributeError: 'ArmNode' object has no attribute '_freeze_timers'` / `'_freeze_joint'`.

- [ ] **Step 1.3: Add `_freeze_timers` state field in `ArmNode.__init__`**

В `pi_nodes/nodes/arm_node.py`, найди строку `self._state_lock = threading.Lock()` (~line 118) и СРАЗУ ПОСЛЕ неё добавь:

```python
        # Per-joint таймеры auto-unfreeze для freeze c duration. None если
        # таймера нет. См. _freeze_joint / _auto_unfreeze.
        self._freeze_timers: list[threading.Timer | None] = [None] * self._num_joints
```

- [ ] **Step 1.4: Add `_freeze_joint` and `_auto_unfreeze` methods**

В `pi_nodes/nodes/arm_node.py`, найди метод `_freeze_all_except_claw` (~line 414) и СРАЗУ ПЕРЕД ним добавь:

```python
    def _freeze_joint(self, idx: int, duration: float | None = None):
        """Заморозить сустав idx. Если duration > 0 — schedule auto-unfreeze.

        Идемпотентно по таймерам: повторный freeze с duration на том же
        суставе отменяет предыдущий Timer и стартует новый. Это нужно
        FSM grab v2 — каждый новый цикл захвата перезапускает 20с timer.
        """
        if idx < 0 or idx >= self._num_joints:
            return
        # Cancel previous timer (если был)
        if self._freeze_timers[idx] is not None:
            self._freeze_timers[idx].cancel()
            self._freeze_timers[idx] = None

        self._servos[idx].freeze()

        if duration is not None and duration > 0:
            t = threading.Timer(float(duration),
                                self._auto_unfreeze, args=(idx,))
            t.daemon = True
            t.start()
            self._freeze_timers[idx] = t
            self.log_info('Arm joint %d FROZEN at %.1f° (auto-unfreeze in %.1fs)',
                          idx + 1, self._target_angles[idx], duration)
        else:
            self.log_info('Arm joint %d FROZEN at %.1f°',
                          idx + 1, self._target_angles[idx])

    def _auto_unfreeze(self, idx: int):
        """Колбэк threading.Timer: разморозить сустав idx по истечении
        duration. Очищает _freeze_timers[idx] чтобы commands могли
        отличать «таймер ещё активен» от «таймер отработал».
        """
        self._freeze_timers[idx] = None
        self._servos[idx].unfreeze()
        self.log_info('Arm joint %d AUTO-UNFROZEN (timer expired)', idx + 1)
```

- [ ] **Step 1.5: Run tests — expect PASS**

```bash
pytest tests/test_arm_node.py::test_freeze_joint_with_duration_starts_timer tests/test_arm_node.py::test_freeze_joint_without_duration_no_timer tests/test_arm_node.py::test_freeze_joint_duration_zero_no_timer tests/test_arm_node.py::test_freeze_joint_restarts_timer tests/test_arm_node.py::test_freeze_joint_timer_calls_auto_unfreeze -v
```

Expected: 5 PASS.

- [ ] **Step 1.6: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): _freeze_joint helper with optional duration timer"
```

---

## Task 2: arm_node — `_unfreeze_joint` helper + `_cmd_cb` integration

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py`
- Test: `tests/test_arm_node.py`

**Context:** Теперь подключаем helpers к `_cmd_cb`. `freeze` принимает `duration`, `unfreeze` отменяет активный таймер.

- [ ] **Step 2.1: Write failing tests for `_unfreeze_joint` and `_cmd_cb` integration**

Добавь в `tests/test_arm_node.py`:

```python
def test_unfreeze_joint_cancels_active_timer(arm_node_factory):
    """_unfreeze_joint(idx) отменяет активный таймер и вызывает unfreeze.

    Регрессия: если таймер не отменить, через duration сек он
    дёрнет unfreeze ещё раз (no-op на size серво, но lognoise).
    """
    node = arm_node_factory()
    node._freeze_joint(3, duration=10.0)
    timer = node._freeze_timers[3]
    assert timer.is_alive()

    node._unfreeze_joint(3)

    assert not timer.is_alive()    # cancelled
    assert node._freeze_timers[3] is None
    node._mock_servos[3].unfreeze.assert_called_once()


def test_cmd_cb_freeze_with_duration(arm_node_factory):
    """{command:freeze, joint:4, duration:0.05} вызывает _freeze_joint(3, 0.05)
    — freeze + Timer стартует.
    """
    import time
    node = arm_node_factory()

    node._cmd_cb('arm/command',
                 {'command': 'freeze', 'joint': 4, 'duration': 0.05})

    node._mock_servos[3].freeze.assert_called_once()
    assert node._freeze_timers[3] is not None

    time.sleep(0.15)
    node._mock_servos[3].unfreeze.assert_called_once()


def test_cmd_cb_freeze_without_duration_no_timer(arm_node_factory):
    """{command:freeze, joint:4} без duration — таймера нет (current behavior)."""
    node = arm_node_factory()

    node._cmd_cb('arm/command', {'command': 'freeze', 'joint': 4})

    node._mock_servos[3].freeze.assert_called_once()
    assert node._freeze_timers[3] is None


def test_cmd_cb_unfreeze_joint_cancels_timer(arm_node_factory):
    """{command:unfreeze, joint:4} отменяет активный 20s таймер на клешне.
    Кейс: пользователь жмёт «открой клешню» в UI пока FSM grab держит её.
    """
    import time
    node = arm_node_factory()
    node._cmd_cb('arm/command',
                 {'command': 'freeze', 'joint': 4, 'duration': 10.0})
    assert node._freeze_timers[3] is not None

    node._cmd_cb('arm/command', {'command': 'unfreeze', 'joint': 4})

    assert node._freeze_timers[3] is None
    node._mock_servos[3].unfreeze.assert_called_once()

    # Через короткое время unfreeze НЕ вызывается повторно (таймер отменён)
    time.sleep(0.05)
    assert node._mock_servos[3].unfreeze.call_count == 1


def test_cmd_cb_unfreeze_all_cancels_all_timers(arm_node_factory):
    """{command:unfreeze} без joint отменяет все активные таймеры
    и размораживает все серво.
    """
    node = arm_node_factory()
    node._cmd_cb('arm/command',
                 {'command': 'freeze', 'joint': 4, 'duration': 10.0})

    node._cmd_cb('arm/command', {'command': 'unfreeze'})

    for i in range(4):
        assert node._freeze_timers[i] is None
        node._mock_servos[i].unfreeze.assert_called_once()
```

- [ ] **Step 2.2: Run tests — expect FAIL**

```bash
pytest tests/test_arm_node.py::test_unfreeze_joint_cancels_active_timer tests/test_arm_node.py::test_cmd_cb_freeze_with_duration tests/test_arm_node.py::test_cmd_cb_freeze_without_duration_no_timer tests/test_arm_node.py::test_cmd_cb_unfreeze_joint_cancels_timer tests/test_arm_node.py::test_cmd_cb_unfreeze_all_cancels_all_timers -v
```

Expected: 5 FAILS — `_unfreeze_joint` не существует; freeze/unfreeze пока не используют новые helpers.

- [ ] **Step 2.3: Add `_unfreeze_joint` method**

В `pi_nodes/nodes/arm_node.py`, СРАЗУ ПОСЛЕ `_auto_unfreeze` (добавленного в Task 1) добавь:

```python
    def _unfreeze_joint(self, idx: int):
        """Разморозить сустав idx. Отменяет активный auto-unfreeze таймер
        если был. Не падает на out-of-range.
        """
        if idx < 0 or idx >= self._num_joints:
            return
        if self._freeze_timers[idx] is not None:
            self._freeze_timers[idx].cancel()
            self._freeze_timers[idx] = None
        self._servos[idx].unfreeze()
```

- [ ] **Step 2.4: Modify `_cmd_cb` freeze handler to support duration**

В `pi_nodes/nodes/arm_node.py`, найди существующий блок (~line 309):

```python
        if cmd == 'freeze':
            self._unlock_if_needed()
            joint = d.get('joint')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._servos[idx].freeze()
                    self.log_info('Arm joint %d FROZEN at %.1f°',
                                  idx + 1, self._target_angles[idx])
            else:
                self._freeze_all_except_claw()
            return
```

Замени на:

```python
        if cmd == 'freeze':
            self._unlock_if_needed()
            joint = d.get('joint')
            duration = d.get('duration')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._freeze_joint(idx, duration)
            else:
                self._freeze_all_except_claw(duration)
            return
```

- [ ] **Step 2.5: Modify `_cmd_cb` unfreeze handler to use `_unfreeze_joint`**

В `pi_nodes/nodes/arm_node.py`, найди существующий блок (~line 322):

```python
        if cmd == 'unfreeze':
            joint = d.get('joint')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._servos[idx].unfreeze()
                    self.log_info('Arm joint %d UNFROZEN', idx + 1)
            else:
                for s in self._servos:
                    s.unfreeze()
                self.log_info('Arm ALL joints UNFROZEN')
            return
```

Замени на:

```python
        if cmd == 'unfreeze':
            joint = d.get('joint')
            if joint is not None:
                idx = int(joint) - 1
                if 0 <= idx < self._num_joints:
                    self._unfreeze_joint(idx)
                    self.log_info('Arm joint %d UNFROZEN', idx + 1)
            else:
                for i in range(self._num_joints):
                    self._unfreeze_joint(i)
                self.log_info('Arm ALL joints UNFROZEN')
            return
```

- [ ] **Step 2.6: Modify string 'unfreeze' branch in `_cmd_cb`**

В `pi_nodes/nodes/arm_node.py`, найди (~line 277):

```python
            if cmd_lower == 'unfreeze':
                for s in self._servos:
                    s.unfreeze()
                self.log_info('Arm ALL joints UNFROZEN')
                return
```

Замени на:

```python
            if cmd_lower == 'unfreeze':
                for i in range(self._num_joints):
                    self._unfreeze_joint(i)
                self.log_info('Arm ALL joints UNFROZEN')
                return
```

- [ ] **Step 2.7: Run tests — expect PASS**

Сначала Task 2 тесты:

```bash
pytest tests/test_arm_node.py::test_unfreeze_joint_cancels_active_timer tests/test_arm_node.py::test_cmd_cb_freeze_with_duration tests/test_arm_node.py::test_cmd_cb_freeze_without_duration_no_timer tests/test_arm_node.py::test_cmd_cb_unfreeze_joint_cancels_timer tests/test_arm_node.py::test_cmd_cb_unfreeze_all_cancels_all_timers -v
```

Expected: 5 PASS.

Но `_freeze_all_except_claw(duration)` ещё не принимает аргумент — это Task 3. Тест `test_cmd_cb_unfreeze_all_cancels_all_timers` шлёт `unfreeze` без joint, который вызывает loop `_unfreeze_joint(i)` для всех — это работает. Проверь что также не сломались СУЩЕСТВУЮЩИЕ тесты:

```bash
pytest tests/test_arm_node.py -v
```

Expected: все PASS, кроме возможно `test_cmd_cb_freeze_all_skips_claw_dict_form` / `test_cmd_cb_freeze_all_skips_claw_string_form` если они падают из-за того что `_freeze_all_except_claw` пока без `duration`. Если PASS — значит вызов в Task 2.4 правильно прокидывает `None`, и Task 3 сделает signature.

Если падает с `TypeError: _freeze_all_except_claw() takes 1 positional argument but 2 were given` — продолжай в Task 3, там это починим.

- [ ] **Step 2.8: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): freeze command supports duration param; unfreeze cancels timer"
```

---

## Task 3: arm_node — refactor `_freeze_all_except_claw` to accept duration

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py`
- Test: `tests/test_arm_node.py`

**Context:** `_freeze_all_except_claw` сейчас принимает 0 аргументов. Task 2.4 вызывает её с `duration`. Делаем сигнатуру совместимой + унифицируем путь через `_freeze_joint`.

- [ ] **Step 3.1: Write failing test for `_freeze_all_except_claw(duration)`**

Добавь в `tests/test_arm_node.py`:

```python
def test_freeze_all_except_claw_supports_duration(arm_node_factory):
    """_freeze_all_except_claw(duration) морозит CH0/1/2 с auto-unfreeze
    таймером для каждого. CH3 (клешня) не тронут.
    """
    node = arm_node_factory()

    node._freeze_all_except_claw(duration=10.0)

    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
        assert node._freeze_timers[i] is not None
        node._freeze_timers[i].cancel()    # cleanup
    node._mock_servos[3].freeze.assert_not_called()
    assert node._freeze_timers[3] is None


def test_freeze_all_except_claw_without_duration_no_timers(arm_node_factory):
    """_freeze_all_except_claw() (без duration) — current behavior:
    морозит CH0/1/2 без таймеров (frozen indefinite).
    """
    node = arm_node_factory()

    node._freeze_all_except_claw()

    for i in range(3):
        node._mock_servos[i].freeze.assert_called_once()
        assert node._freeze_timers[i] is None
    node._mock_servos[3].freeze.assert_not_called()
```

- [ ] **Step 3.2: Run tests — expect FAIL**

```bash
pytest tests/test_arm_node.py::test_freeze_all_except_claw_supports_duration tests/test_arm_node.py::test_freeze_all_except_claw_without_duration_no_timers -v
```

Expected: FAIL — TypeError или freeze без таймера.

- [ ] **Step 3.3: Refactor `_freeze_all_except_claw`**

В `pi_nodes/nodes/arm_node.py`, найди текущий метод (~line 414):

```python
    def _freeze_all_except_claw(self):
        """Freeze всех суставов руки, КРОМЕ клешни (последний канал).

        Клешня (CH3) морозится только явной командой joint=N через личную
        ❄ кнопку слайдера в UI — пара к servos.arm.claw_init_on_startup,
        который тоже выделяет клешню в отдельную дисциплину. Это позволяет
        держать руку в позе захвата (CH0..CH2 frozen), а клешню оставлять
        под прямым ручным управлением слайдером без лишних разморозок.
        """
        if self._num_joints <= 1:
            return
        for s in self._servos[:-1]:
            s.freeze()
        self.log_info('Arm joints FROZEN (claw excluded)')
```

Замени на:

```python
    def _freeze_all_except_claw(self, duration: float | None = None):
        """Freeze всех суставов руки, КРОМЕ клешни (последний канал).

        Клешня (CH3) морозится только явной командой joint=N через личную
        ❄ кнопку слайдера в UI — пара к servos.arm.claw_init_on_startup,
        который тоже выделяет клешню в отдельную дисциплину. Это позволяет
        держать руку в позе захвата (CH0..CH2 frozen), а клешню оставлять
        под прямым ручным управлением слайдером без лишних разморозок.

        duration: если задан и > 0 — каждый из CH0/1/2 получает свой
        auto-unfreeze Timer на duration секунд. По умолчанию None — freeze
        indefinite (старое поведение).
        """
        if self._num_joints <= 1:
            return
        for i in range(self._num_joints - 1):
            self._freeze_joint(i, duration)
        self.log_info('Arm joints FROZEN (claw excluded)')
```

- [ ] **Step 3.4: Run tests — expect PASS**

```bash
pytest tests/test_arm_node.py -v
```

Expected: все PASS. Особое внимание на `test_unfreeze_persists_through_single_joint_command` и существующие freeze-тесты.

- [ ] **Step 3.5: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "refactor(arm_node): _freeze_all_except_claw routes through _freeze_joint, accepts duration"
```

---

## Task 4: arm_node — `grab_return` preset migration

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py`
- Test: `tests/test_arm_node.py`

**Context:** Новый preset `grab_return = [grab_ready[0..2], 180]` = `[30, 60, 0, 180]`. Клешня закрыта (держит мяч), CH0/1/2 как у grab_ready.

- [ ] **Step 4.1: Write failing test for `grab_return` migration**

Добавь в `tests/test_arm_node.py`:

```python
def test_init_migrates_grab_return_preset(arm_node_factory):
    """При пустом presets.json arm_node автомигрирует grab_return.
    Значения: CH0/1/2 как у grab_ready, CH3=180 (клешня закрыта).
    Это нужно FSM grab v2 Phase 5 — возврат в позу grab_ready
    с удержанием объекта.
    """
    node = arm_node_factory(presets_seed=None)

    assert node._presets.load_preset('arm', 'grab_return') == [30.0, 60.0, 0.0, 180.0]


def test_init_does_not_overwrite_user_grab_return(arm_node_factory):
    """Пользовательский grab_return не перезаписывается миграцией."""
    custom = {
        'arm': {
            'grab_return': [55.0, 75.0, 5.0, 175.0],    # user-edited
        }
    }
    node = arm_node_factory(presets_seed=custom)

    assert node._presets.load_preset('arm', 'grab_return') == [55.0, 75.0, 5.0, 175.0]
```

- [ ] **Step 4.2: Run tests — expect FAIL**

```bash
pytest tests/test_arm_node.py::test_init_migrates_grab_return_preset tests/test_arm_node.py::test_init_does_not_overwrite_user_grab_return -v
```

Expected: FAIL — `grab_return` preset не существует.

- [ ] **Step 4.3: Add `grab_return` to `_DEFAULT_ARM_PRESETS`**

В `pi_nodes/nodes/arm_node.py`, найди (~line 147):

```python
        _DEFAULT_ARM_PRESETS = {
            'grab_ready': [30.0, 60.0, 0.0, 0.0],
            'grab_hold':  [0.0,  100.0,  0.0, 180.0],
        }
```

Замени на:

```python
        _DEFAULT_ARM_PRESETS = {
            'grab_ready': [30.0, 60.0, 0.0, 0.0],
            'grab_hold':  [0.0,  100.0,  0.0, 180.0],
            # Возврат после grab_hold в позу grab_ready, НО с закрытой клешнёй.
            # FSM grab v2 Phase 5 загружает этот пресет — рука едет в grab_ready
            # CH0/1/2 углы, клешня (CH3) остаётся 180 (объект удержан).
            # Через 20с timer в arm_node размораживает клешню → объект освобождается.
            'grab_return': [30.0, 60.0, 0.0, 180.0],
        }
```

- [ ] **Step 4.4: Run tests — expect PASS**

```bash
pytest tests/test_arm_node.py::test_init_migrates_grab_return_preset tests/test_arm_node.py::test_init_does_not_overwrite_user_grab_return -v
```

Expected: PASS.

- [ ] **Step 4.5: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm_node): migrate grab_return preset (grab_ready angles + closed claw)"
```

---

## Task 5: fsm_node — new state fields + `_transition` reset

**Files:**
- Modify: `pi_nodes/nodes/fsm_node.py`
- Test: `tests/test_fsm_grab_sequence.py`

**Context:** Готовим FSM к 5-фазной логике. Добавляем флаги `_grab_open_sent`, `_grab_hold_sent` и сбрасываем их в `_transition`. Существующий `_grab_t` остаётся.

- [ ] **Step 5.1: Write failing test for state reset**

Добавь в `tests/test_fsm_grab_sequence.py`:

```python
def test_grab_state_flags_initial_false(fsm_node_factory):
    """Новые поля _grab_open_sent и _grab_hold_sent инициализируются False."""
    node = fsm_node_factory()
    assert node._grab_open_sent is False
    assert node._grab_hold_sent is False


def test_grab_state_flags_reset_on_transition(fsm_node_factory):
    """После _transition в любой state флаги сбрасываются в False.
    Это гарантирует что повторный заход в GRABBING запустит все фазы заново.
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()

    node._grab_open_sent = True
    node._grab_hold_sent = True

    node._transition(State.IDLE)

    assert node._grab_open_sent is False
    assert node._grab_hold_sent is False
```

- [ ] **Step 5.2: Run tests — expect FAIL**

```bash
pytest tests/test_fsm_grab_sequence.py::test_grab_state_flags_initial_false tests/test_fsm_grab_sequence.py::test_grab_state_flags_reset_on_transition -v
```

Expected: FAIL — атрибутов не существует.

- [ ] **Step 5.3: Add state fields in `FSMNode.__init__`**

В `pi_nodes/nodes/fsm_node.py`, найди (~line 113-114):

```python
        # Локальный таймер фазы GRABBING (см. _do_grab).
        self._grab_t = 0.0
```

Замени на:

```python
        # Локальный таймер фазы GRABBING (см. _do_grab).
        self._grab_t = 0.0
        # Флаги «фаза отстрелила» для 5-фазного _do_grab v2.
        # Сбрасываются в _transition при любой смене state.
        self._grab_open_sent = False    # Phase 1 (open claw + freeze 20s) done
        self._grab_hold_sent = False    # Phase 3 (load grab_hold) done
```

- [ ] **Step 5.4: Reset flags in `_transition`**

В `pi_nodes/nodes/fsm_node.py`, найди (~line 353):

```python
        self._grab_t = 0.0
```

(внутри `_transition`)

И ПОСЛЕ этой строки добавь:

```python
        self._grab_open_sent = False
        self._grab_hold_sent = False
```

- [ ] **Step 5.5: Run tests — expect PASS**

```bash
pytest tests/test_fsm_grab_sequence.py::test_grab_state_flags_initial_false tests/test_fsm_grab_sequence.py::test_grab_state_flags_reset_on_transition -v
```

Expected: PASS.

- [ ] **Step 5.6: Commit**

```bash
git add pi_nodes/nodes/fsm_node.py tests/test_fsm_grab_sequence.py
git commit -m "feat(fsm_node): add _grab_open_sent/_grab_hold_sent state for grab v2 phases"
```

---

## Task 6: fsm_node — new 5-phase `_do_grab`

**Files:**
- Modify: `pi_nodes/nodes/fsm_node.py`
- Test: `tests/test_fsm_grab_sequence.py`

**Context:** Полная замена существующего `_do_grab`. Существующие тесты `_do_grab` (старая 3-фазная логика) сломаются — их обновляем здесь.

- [ ] **Step 6.1: Remove obsolete grab tests**

В `tests/test_fsm_grab_sequence.py` УДАЛИ следующие три теста (они написаны под старую 3-фазную логику и теперь несовместимы):

- `test_grab_first_tick_sends_grab_hold_preset` — старая: первый тик = `load_preset grab_hold`. Новая: первый тик = `joint=4 angle=0` + `freeze joint=4 duration=20`.
- `test_grab_does_not_publish_claw_command` — оставляем, всё ещё валидно.
- `test_grab_after_settle_sends_freeze_and_transitions_to_returning` — старая: после settle публикует 2 freeze (общий + joint=4). Новая: после settle публикует `load_preset grab_return`.

Удали полностью `test_grab_first_tick_sends_grab_hold_preset` и `test_grab_after_settle_sends_freeze_and_transitions_to_returning`.

`test_grab_does_not_publish_claw_command` ОСТАВЬ — он по-прежнему валиден.

- [ ] **Step 6.2: Write new failing tests for v2 phases**

Добавь в `tests/test_fsm_grab_sequence.py`:

```python
def test_grab_phase1_publishes_open_and_freeze_duration(fsm_node_factory):
    """Phase 1 (первый тик GRABBING): два arm/command publishes:
    1) {joint:4, angle:0} (открыть клешню)
    2) {command:freeze, joint:4, duration:20.0} (freeze клешня на 20с)
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    node._do_grab()

    arm_pubs = [p for p in node._published if p[0] == 'arm/command']
    assert {'joint': 4, 'angle': 0.0} in [p[1] for p in arm_pubs]
    assert {'command': 'freeze', 'joint': 4, 'duration': 20.0} in [p[1] for p in arm_pubs]
    assert node._grab_open_sent is True
    assert node._grab_hold_sent is False


def test_grab_phase1_only_once(fsm_node_factory):
    """Phase 1 публикуется ровно один раз за вход в GRABBING."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    for _ in range(5):
        node._do_grab()    # tick 1-5

    open_pubs = [p for p in node._published
                 if p[0] == 'arm/command'
                 and isinstance(p[1], dict)
                 and p[1].get('joint') == 4
                 and 'angle' in p[1]]
    assert len(open_pubs) == 1


def test_grab_phase2_waits_no_new_publishes(fsm_node_factory):
    """Phase 2 (t < 1.1с): после Phase 1 нет новых публикаций до t≥1.1."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)
    node._do_grab()    # Phase 1 fires
    pubs_after_phase1 = len(node._published)

    # Тики 2..10 (_grab_t = 0.2..1.0) — Phase 2 wait
    for _ in range(9):
        node._do_grab()

    assert len(node._published) == pubs_after_phase1   # никаких новых


def test_grab_phase3_publishes_grab_hold(fsm_node_factory):
    """Phase 3 (~t=1.1с): публикуется load_preset grab_hold."""
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    # 11 тиков → _grab_t = 1.1 (CPython float может дать 1.10000...01)
    for _ in range(11):
        node._do_grab()

    hold_pubs = [p for p in node._published
                 if p[0] == 'arm/command'
                 and isinstance(p[1], dict)
                 and p[1].get('command') == 'load_preset'
                 and p[1].get('name') == 'grab_hold']
    assert len(hold_pubs) == 1
    assert node._grab_hold_sent is True


def test_grab_phase5_publishes_grab_return_and_transitions(fsm_node_factory):
    """Phase 5 (после settle + 1с): load_preset grab_return + переход в RETURNING.

    С max_speed=120 (fixture) settle = 100/120 ≈ 0.83с.
    phase_5_t = 1.1 + 0.83 + 1.0 ≈ 2.93с → 30 тиков.
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)

    for _ in range(30):
        node._do_grab()

    return_pubs = [p for p in node._published
                   if p[0] == 'arm/command'
                   and isinstance(p[1], dict)
                   and p[1].get('command') == 'load_preset'
                   and p[1].get('name') == 'grab_return']
    assert len(return_pubs) == 1
    assert node._state == State.RETURNING


def test_grab_does_not_publish_old_freeze_pattern(fsm_node_factory):
    """В новой логике НЕ публикуются bare freeze / freeze joint=4 как в v1.
    Замораживание идёт через freeze joint=4 duration=20 (Phase 1) и
    load_preset _freeze_all_except_claw (Phase 3/5 неявно).
    """
    from pi_nodes.nodes.fsm_node import State
    node = fsm_node_factory()
    node._transition(State.GRABBING)
    for _ in range(35):
        node._do_grab()

    arm_pubs = [p for p in node._published if p[0] == 'arm/command']
    payloads = [p[1] for p in arm_pubs if isinstance(p[1], dict)]
    # Старые паттерны должны отсутствовать
    assert {'command': 'freeze'} not in payloads
    assert {'command': 'freeze', 'joint': 4} not in payloads
```

- [ ] **Step 6.3: Run tests — expect FAIL**

```bash
pytest tests/test_fsm_grab_sequence.py -v
```

Expected: новые тесты FAIL (логика ещё старая). Старые удалённые тесты не запускаются.

- [ ] **Step 6.4: Replace `_do_grab` body**

В `pi_nodes/nodes/fsm_node.py`, найди существующий `_do_grab` (~line 494-544):

```python
    def _do_grab(self):
        """Захват объекта новой 3-фазной логикой (заменяет старую с
        claw/command). См. spec 2026-05-17-arm-grab-sequence.

        Phase 1 (t<=0.1c): один раз публикуем arm/command load_preset grab_hold.
        Phase 2 (0.1c < t < settle): ждём пока _interpolate_tick доедет до позы.
        Phase 3 (t >= settle): freeze всех суставов → RETURNING.

        Settle выводится из max_speed_deg_per_sec: самая длинная дельта при
        переходе grab_ready (110,100,180,0) → grab_hold (10,30,180,180) —
        это CH0 (100°). settle = 100° / min(max_speeds) + 0.25с jitter.
        При scalar 120°/с это ~1.08с; при текущем списке [45,45,9999,9999]
        — ~2.47с (лимитирует медленный сустав). При изменении конфига
        пересчитывается автоматически.
        """
        self._grab_t += 0.1

        if self._grab_t <= 0.1:
            # Phase 1 — единичная команда
            self.publish('arm/command',
                         {'command': 'load_preset', 'name': 'grab_hold'},
                         qos=1)
            self.log_info('Arm → grab_hold (closing claw)')
            return

        _GRAB_DELTA_DEG = 100.0   # CH0: grab_ready[0]=110 → grab_hold[0]=10
        # max_speed_deg_per_sec может быть скаляром или списком per-joint.
        # Settle ограничивается самым медленным суставом, т.к. CH0 (100°)
        # — самая длинная дельта; берём min() по списку. Клешня (CH3)
        # обычно instant (9999°/с), но min отбросит её и оставит реалистичную
        # скорость основания/суставов 1-2 (~45°/с).
        _raw_speed = cfg('servos.arm.max_speed_deg_per_sec', 120.0)
        if isinstance(_raw_speed, (list, tuple)) and _raw_speed:
            _max_speed = max(1.0, min(float(v) for v in _raw_speed))
        else:
            _max_speed = max(1.0, float(_raw_speed))
        grab_settle_s = _GRAB_DELTA_DEG / _max_speed + 0.25
        if self._grab_t < grab_settle_s:
            return

        # Phase 3 — freeze + переход.
        # arm_node freeze-all исключает клешню (CH3) из общей заморозки
        # (только личная ❄ кнопка слайдера её морозит), поэтому FSM шлёт
        # ДВА freeze: общий для CH0..CH2 и явный joint=4 для клешни —
        # иначе после grab_hold PWM на CH3 отключится через HOLD_TIME
        # и захваченный мяч выпадет.
        self.publish('arm/command', {'command': 'freeze'}, qos=1)
        self.publish('arm/command',
                     {'command': 'freeze', 'joint': 4}, qos=1)
        self.log_info('Arm FROZEN — holding object (incl. claw)')
        self._transition(State.RETURNING)
```

Замени на:

```python
    def _do_grab(self):
        """Захват объекта 5-фазной логикой v2 (spec 2026-05-19-arm-grab-sequence-v2).

        Phase 1 (one-shot, ~t=0.1с):
            publish arm/command {joint:4, angle:0}                    # open claw
            publish arm/command {command:freeze, joint:4, duration:20.0}
                                                                       # freeze + 20s timer
        Phase 2 (wait): t < 1.1с — задержка 1с после открытия клешни.
        Phase 3 (one-shot, ~t=1.1с):
            publish arm/command {command:load_preset, name:grab_hold}  # closes claw, moves arm
        Phase 4 (wait): t < 1.1 + settle + 1.0 — settle интерполятора + 1с задержка.
        Phase 5 (one-shot, finally):
            publish arm/command {command:load_preset, name:grab_return}
                                                                       # CH0/1/2 → grab_ready,
                                                                       # claw stays 180 (closed)
            _transition(State.RETURNING)

        Через ~20с от Phase 1 arm_node auto-unfreeze клешню → PWM release
        через HOLD_TIME → объект освобождается.

        Settle: max_delta=100° (консервативный upper bound; реальные
        дефолтные пресеты дают max=40° для CH1, но пользователь может
        отредактировать пресеты) / min(max_speeds).
        """
        self._grab_t += 0.1

        # Phase 1: open claw + freeze duration=20
        if not self._grab_open_sent:
            self.publish('arm/command', {'joint': 4, 'angle': 0.0}, qos=1)
            self.publish('arm/command',
                         {'command': 'freeze', 'joint': 4, 'duration': 20.0},
                         qos=1)
            self._grab_open_sent = True
            self.log_info('Grab Phase 1: open claw + freeze claw 20s')
            return

        # Phase 2: wait 1s after opening claw
        if self._grab_t < 1.1:
            return

        # Phase 3: load grab_hold (closes claw on object, moves CH0/1/2)
        if not self._grab_hold_sent:
            self.publish('arm/command',
                         {'command': 'load_preset', 'name': 'grab_hold'},
                         qos=1)
            self._grab_hold_sent = True
            self.log_info('Grab Phase 3: → grab_hold (closing claw on object)')
            return

        # Phase 4: wait for grab_hold settle + 1s
        # max_speed_deg_per_sec может быть скаляром или списком per-joint;
        # min() выбирает самый медленный CH0/1/2 (клешня обычно 9999°/с
        # — instant — но среди CH0/1/2 берётся реальная скорость).
        _GRAB_DELTA_DEG = 100.0
        _raw_speed = cfg('servos.arm.max_speed_deg_per_sec', 120.0)
        if isinstance(_raw_speed, (list, tuple)) and _raw_speed:
            _max_speed = max(1.0, min(float(v) for v in _raw_speed))
        else:
            _max_speed = max(1.0, float(_raw_speed))
        grab_settle_s = _GRAB_DELTA_DEG / _max_speed
        phase_5_t = 1.1 + grab_settle_s + 1.0
        if self._grab_t < phase_5_t:
            return

        # Phase 5: return to grab_return pose + transition
        self.publish('arm/command',
                     {'command': 'load_preset', 'name': 'grab_return'},
                     qos=1)
        self.log_info('Grab Phase 5: → grab_return (initial pose, claw stays closed)'
                      ' → RETURNING')
        self._transition(State.RETURNING)
```

- [ ] **Step 6.5: Run tests — expect PASS**

```bash
pytest tests/test_fsm_grab_sequence.py -v
```

Expected: все PASS, включая новые v2 тесты и сохранённый `test_grab_does_not_publish_claw_command` + `test_approach_*`.

- [ ] **Step 6.6: Commit**

```bash
git add pi_nodes/nodes/fsm_node.py tests/test_fsm_grab_sequence.py
git commit -m "feat(fsm_node): 5-phase _do_grab v2 (open->1s->grab_hold->settle+1s->grab_return)"
```

---

## Task 7: actuators.py — fix bare unfreeze → `unfreeze joint=4`

**Files:**
- Modify: `compute_node/dashboard/routers/actuators.py:97`
- Test: `tests/test_actuators_router.py`

**Context:** Existing tests passed bare `{command:unfreeze}`. После фикса должно быть `{command:unfreeze, joint:4}`. Существующие тесты сломаются — обновляем их.

- [ ] **Step 7.1: Update existing test `test_claw_open_publishes_joint4_and_unfreeze`**

В `tests/test_actuators_router.py`, найди тест (~line 45):

```python
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
```

Замени на:

```python
def test_claw_open_publishes_joint4_and_unfreeze_claw_only(client, fake_mqtt):
    """POST /api/actuators/claw {state:open} — публикует ДВЕ команды:
    arm/command {joint:4, angle:0} (открыть клешню) и
    arm/command {command:unfreeze, joint:4} (снять заморозку ТОЛЬКО клешни,
    не CH0/1/2). До фикса bare unfreeze без joint размораживал все
    суставы — это был баг.
    """
    r = client.post('/api/v1/actuators/claw', json={'state': 'open'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    topics_and_payloads = [(call.args[0], call.args[1]) for call in pub_calls]

    # joint=4 angle=0 (открыть клешню)
    assert any(t.endswith('arm/command') and p == {'joint': 4, 'angle': 0.0}
               for t, p in topics_and_payloads)
    # unfreeze joint=4 — ТОЛЬКО клешня
    assert any(t.endswith('arm/command') and p == {'command': 'unfreeze', 'joint': 4}
               for t, p in topics_and_payloads)
    # Bare unfreeze (без joint) — НЕ должно быть, это размораживало бы CH0/1/2
    assert not any(t.endswith('arm/command') and p == {'command': 'unfreeze'}
                   for t, p in topics_and_payloads)
```

- [ ] **Step 7.2: Update existing test `test_claw_close_does_not_unfreeze`**

В `tests/test_actuators_router.py` найди:

```python
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
```

Замени на:

```python
def test_claw_close_does_not_unfreeze(client, fake_mqtt):
    """POST {state:close} — публикуется только joint=4 angle=180.
    Никакого unfreeze (рука как раз должна оставаться frozen для удержания).
    Ни bare {command:unfreeze}, ни joint-specific {command:unfreeze, joint:4}.
    """
    r = client.post('/api/v1/actuators/claw', json={'state': 'close'})
    assert r.status_code == 200

    pub_calls = fake_mqtt.publish.call_args_list
    payloads = [call.args[1] for call in pub_calls]

    assert {'joint': 4, 'angle': 180.0} in payloads
    assert {'command': 'unfreeze'} not in payloads
    assert {'command': 'unfreeze', 'joint': 4} not in payloads
```

- [ ] **Step 7.3: Check remaining tests in file**

Прочти `tests/test_actuators_router.py` целиком (`cat tests/test_actuators_router.py` или Read tool):

```bash
pytest tests/test_actuators_router.py -v --collect-only
```

Если есть другие тесты которые ожидают bare `{command:unfreeze}` (например `test_claw_angle_below_90_triggers_unfreeze`), обнови их аналогично: ожидать `{command:unfreeze, joint:4}` вместо bare.

Для `test_claw_angle_below_90_triggers_unfreeze` (~line 78), замени проверку:

```python
    assert {'command': 'unfreeze'} in payloads
```

на:

```python
    assert {'command': 'unfreeze', 'joint': 4} in payloads
    assert {'command': 'unfreeze'} not in payloads
```

- [ ] **Step 7.4: Run tests — expect FAIL (код ещё не пофиксен)**

```bash
pytest tests/test_actuators_router.py -v
```

Expected: FAIL на обновлённых тестах (ожидают `unfreeze joint=4`, получают bare unfreeze).

- [ ] **Step 7.5: Fix `set_claw` in actuators.py**

В `compute_node/dashboard/routers/actuators.py`, найди (~line 95-97):

```python
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    if angle < 90.0:
        mqtt.publish('arm/command', {'command': 'unfreeze'}, qos=1)
    return CommandAck()
```

Замени на:

```python
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    if angle < 90.0:
        # Размораживаем ТОЛЬКО клешню. Раньше bare {command:unfreeze}
        # размораживал все CH0/1/2 — баг: «открой клешню» в UI ломал
        # удержание руки в позе захвата. Также эта команда отменяет
        # активный 20s auto-unfreeze таймер из FSM grab v2 — пользователь
        # явно требует «отпусти мяч сейчас» вместо ожидания таймера.
        mqtt.publish('arm/command',
                     {'command': 'unfreeze', 'joint': 4}, qos=1)
    return CommandAck()
```

Также обнови docstring `set_claw` (~line 79-85):

```python
async def set_claw(cmd: ClawCommand, mqtt: MQTTDep) -> CommandAck:
    """Клешня = arm joint 4 (1-indexed). open=0°, close=180°.

    При открытии клешни (state=open или angle<90) дополнительно публикуем
    arm/command unfreeze joint=4 — снимаем заморозку ТОЛЬКО клешни (CH0/1/2
    остаются под FSM-контролем). Если был активен 20s auto-unfreeze таймер
    из FSM grab v2 — он отменяется (пользовательский override).
    """
```

- [ ] **Step 7.6: Run tests — expect PASS**

```bash
pytest tests/test_actuators_router.py -v
```

Expected: все PASS.

- [ ] **Step 7.7: Commit**

```bash
git add compute_node/dashboard/routers/actuators.py tests/test_actuators_router.py
git commit -m "fix(actuators): claw open publishes unfreeze joint=4, not bare unfreeze"
```

---

## Task 8: Full test suite sanity check

**Files:** все тестовые файлы.

**Context:** Убедимся что мы ничего не сломали в смежных тестах.

- [ ] **Step 8.1: Run full pi_nodes test suite**

```bash
pytest tests/test_arm_node.py tests/test_fsm_grab_sequence.py tests/test_fsm_node.py tests/test_actuators_router.py -v
```

Expected: все PASS.

- [ ] **Step 8.2: Run full test suite — sanity check**

```bash
pytest tests/ -v --tb=short -x 2>&1 | head -100
```

Если есть FAIL — обычно это:
- Тесты ожидают bare `unfreeze` (мы могли пропустить файл) — починить аналогично Task 7.
- Тесты ожидают старый payload `_do_grab` — починить аналогично Task 6.

Если FAIL есть — fix inline, прежде чем продолжать. НЕ коммить до полного зелёного.

- [ ] **Step 8.3: Commit if any extra fixes needed**

Если в Step 8.2 нашлись и поправились тесты:

```bash
git add tests/
git commit -m "test: align stale tests with arm grab v2 + unfreeze joint=4 API"
```

Если в Step 8.2 всё было зелёное — пропустить этот шаг.

---

## Task 9: Manual verification on hardware (optional, not required for plan completion)

**Files:** none (manual smoke).

**Context:** План считается выполненным после Task 8 (все unit-тесты зелёные). Этот Task — рекомендации для ручной проверки на живом роботе, выполняется пользователем отдельно.

- [ ] **Step 9.1: Document manual test plan (optional, skip if no hardware access)**

Если есть доступ к Pi с подключённым PCA9685:

1. Запусти Pi: `./samurai.sh robot` (или systemctl start).
2. Запусти compute: `./samurai.sh compute`.
3. Открой UI `http://localhost:5000/`.
4. **Проверка фикса unfreeze** (Task 7):
   - В UI: вручную замёрзь CH0/1/2 (через слайдеры/кнопки).
   - В UI: нажми «Открыть клешню».
   - **Ожидание**: клешня открывается, **CH0/1/2 остаются frozen** (lock icon).
   - Если CH0/1/2 разморозились — фикс не сработал.
5. **Проверка FSM grab v2** (Task 6):
   - Голос: «возьми красный мяч».
   - **Ожидание fазы**:
     - APPROACHING: рука едет в grab_ready (CH0=30, CH1=60, CH2=0, claw=0 открыта).
     - GRABBING Phase 1 (~0с): клешня → 0 (если ещё не была), freeze 20s timer стартует.
     - Phase 2 (1с пауза).
     - Phase 3 (1.1с): рука едет в grab_hold (CH0=0, CH1=100, CH2=0, claw=180 закрывается).
     - Phase 4 (settle + 1с).
     - Phase 5: рука едет в grab_return (CH0=30, CH1=60, CH2=0, claw=180 — мяч удержан).
     - State = RETURNING (робот едет к origin).
   - Через 20с от Phase 1: claw разморозится автоматически (log "AUTO-UNFROZEN"), PWM release → мяч падает.

Если все шаги совпадают с ожиданием — план верифицирован. Если нет — открыть issue с логами.

---

## Self-Review (writing-plans checklist)

**Spec coverage:**
- Section 3.2 (freeze with duration) → Task 1 (helpers) + Task 2 (_cmd_cb integration) + Task 3 (refactor _freeze_all_except_claw)
- Section 3.3 (grab_return migration) → Task 4
- Section 3.4 (_do_grab v2) → Task 5 (state) + Task 6 (logic)
- Section 3.5 (actuators.py fix) → Task 7
- Section 5 (tests) → tests in Tasks 1-7, sanity check in Task 8

**Placeholder scan:** проверил, нет TBD/TODO. Все шаги содержат конкретный код или команды.

**Type consistency:** `_freeze_joint(idx: int, duration: float | None)`, `_auto_unfreeze(idx: int)`, `_unfreeze_joint(idx: int)`, `_freeze_all_except_claw(duration: float | None = None)` — единая сигнатура.

**Edge case coverage:** Task 1 покрывает duration=None и duration=0; Task 2 покрывает unfreeze отмену timer; Task 7 покрывает что bare unfreeze не публикуется.

**No gaps.** План готов к исполнению.
