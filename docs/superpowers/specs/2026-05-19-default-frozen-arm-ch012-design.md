# Default-frozen CH0/1/2 arm joints

**Status:** draft
**Date:** 2026-05-19
**Scope:** `pi_nodes/nodes/arm_node.py`, `tests/test_arm_node.py`

## Goal

CH0/CH1/CH2 (основание, сустав 1, сустав 2) находятся в состоянии `frozen=True`
по умолчанию. Клешня (CH3) и голова (CH4) не затрагиваются — клешня остаётся
свободно управляемой пользователем, голова живёт своей жизнью.

Семантика: «известная позиция = frozen». Каждая команда «возврат в известное
положение» (`_unlock`, `home`, `load_preset`, `{joints:[…]}`) сначала двигает
все суставы (включая ранее замороженные), затем морозит CH0/1/2. Ручная
разморозка (global `unfreeze` или per-channel 🔓) держится только до
следующей такой команды.

## Зачем

Рука после захвата мяча должна удерживать позу под PWM, иначе суставы
проседают под весом. Сейчас пользователю приходится каждый раз руками жать
«Замор. все» — теперь это default-state. Single-joint drag (spec
2026-05-19-frozen-servo-slider-drag-design) продолжает работать как точная
ручная корректировка без разморозки.

## Текущая семантика vs новая

| Точка | Сейчас | После |
|---|---|---|
| `_unlock()` после `_servo_initialized=True` | CH0/1/2 не заморожены | Вызывается `_freeze_all_except_claw()` |
| `home` (string + dict) | `_set_joint(i, home[i])` (allow_frozen=False) | `_set_joint(i, home[i], allow_frozen=True)` → `_freeze_all_except_claw()` |
| `load_preset` | `_set_joint(i, a)` | `_set_joint(i, a, allow_frozen=True)` → `_freeze_all_except_claw()` |
| `{"joints":[…]}` mass | `_set_joint(i, a)` | `_set_joint(i, a, allow_frozen=True)` → `_freeze_all_except_claw()` |
| single-joint `{joint, angle}` | `_set_joint(idx, a, allow_frozen=True)`, без re-freeze | без изменений |
| `unfreeze` global / per-channel | clears freeze | без изменений |
| `freeze` global / per-channel | sets freeze | без изменений |
| `_interpolate_tick` | шагает frozen-сустав с `force=frozen` | без изменений |
| `ServoDriver`, frontend | — | без изменений |

`_freeze_all_except_claw()` уже корректно морозит `_servos[:-1]` (CH0/1/2) и
логирует факт — переиспользуется без правок.

## Архитектура и data flow

```
arm_node.__init__
  │  config: locked=false, claw_init_on_startup=true
  ▼
_unlock()  ─────────────────────────────────────────────────────┐
  ├─ for each servo: set_angle(home, force=True)                │
  ├─ _target_angles[i] = _current_angles[i] = home              │
  ├─ _servo_initialized = True                                  │
  └─ _freeze_all_except_claw()  ◀── НОВОЕ                       │
       └─ _servos[0..2].freeze() — PWM держит home               │
                                                                │
arm/state публикуется @10Гц: frozen=[T,T,T,F]                   │
UI на старте показывает HOLD-бейджи на ch0/1/2                  │
                                                                │
                                                                ▼
USER: «Разм. все» → arm/command {"command":"unfreeze"}
  └─ for s in _servos: s.unfreeze() — все unfrozen
     frozen=[F,F,F,F]

USER: тянет ch1 слайдер до 110°
  └─ single-joint → _set_joint(1, 110, allow_frozen=True) → target=110
     интерполятор: current→target, frozen остаётся False
     UI: HOLD-бейдж не показан

USER: кнопка «Домой» → arm/command "home"
  └─ for i: _set_joint(i, home[i], allow_frozen=True)  ◀── ИЗМЕНЕНО (+allow_frozen)
  └─ _freeze_all_except_claw()                          ◀── НОВОЕ
     интерполятор: каждый сустав едет к home, CH0/1/2 frozen=True
     CH3 free (preset/claw_init), CH0/1/2 holds home via _freeze_refresh

FSM: APPROACHING → load_preset grab_ready (без предварит. unfreeze!)
  └─ _set_joint(i, grab_ready[i], allow_frozen=True)    ◀── ИЗМЕНЕНО
  └─ _freeze_all_except_claw()                          ◀── НОВОЕ
     рука доходит до grab_ready, замораживается
FSM: GRABBING → load_preset grab_hold
  └─ то же
FSM: финальная команда freeze (line 540 fsm_node.py)
  └─ _freeze_all_except_claw() — идемпотентно no-op
```

## Изменения

### `pi_nodes/nodes/arm_node.py`

**`_unlock`** — после `_servo_initialized = True` добавить вызов:

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

**`home` (string-ветка, line ~256-261):**

```python
if cmd_lower == 'home':
    self._unlock_if_needed()
    for i in range(self._num_joints):
        self._set_joint(i, self._home_angles[i], allow_frozen=True)
    self._freeze_all_except_claw()
    self.log_info('Arm → HOME (CH0/1/2 re-frozen)')
    return
```

**`home` (dict-ветка, line ~289-294):** аналогично.

**`load_preset` (line ~338-351):**

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
    self.log_info('Preset loaded: arm/%s → %s (CH0/1/2 re-frozen)', name, self._target_angles)
    return
```

**`{"joints":[…]}` (line ~386-393):**

```python
if 'joints' in d:
    self._unlock_if_needed()
    angles = d['joints']
    for i, a in enumerate(angles[:self._num_joints]):
        self._set_joint(i, float(a), allow_frozen=True)
    self._freeze_all_except_claw()
    self.log_info('Arm all joints → %s (CH0/1/2 re-frozen)', self._target_angles)
    return
```

**`_set_joint`** — без изменений. Docstring уже корректен; обновим только
последнее предложение, чтобы убрать упоминание «mass-команды не двигают
frozen» (теперь это не так).

```python
def _set_joint(self, idx: int, angle: float, allow_frozen: bool = False):
    """Set joint TARGET angle (логический) с лимитами.

    Реальный PWM шлёт _interpolate_tick @ 50Гц, плавно шагая current
    к target с max_speed_deg_per_sec.

    Если сустав frozen и allow_frozen=False — target НЕ обновляется.
    Сейчас все mass-команды (home/load_preset/joints-array) и single-joint
    UI-команды передают allow_frozen=True, оставляя False только для
    защитных вызовов извне.
    """
```

### `pi_nodes/hardware/servo_driver.py`

Без изменений.

### `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`

Без изменений. `arm.frozen` приходит из `arm/state` MQTT — UI отрисует
новое default-state автоматически.

### `pi_nodes/nodes/fsm_node.py`

Без изменений. Финальный `freeze` (line 540) становится идемпотентным,
но оставляем для явности.

## Гонки и тонкости

### `freeze` после `set_target`, пока интерполятор ещё не доехал

Сценарий: `load_preset grab_ready` шлёт target=110° для CH0, текущая
позиция 30°. Сразу после `_set_joint` вызывается `_freeze_all_except_claw`.

В момент freeze: `_current_angles[0]=30`, `_target_angles[0]=110`,
`ServoDriver._angle=30` (последнее физически записанное).
`ServoDriver.freeze()` фиксирует `_angle=30`, помечает `frozen=True`.

`_interpolate_tick` @50Гц видит `delta != 0`, шагает current на
`max_step = 45°/с × 0.02с = 0.9°`. Вызывает
`set_angle(phys=30.9, force=self._servos[i].frozen=True)` — PWM пишется
несмотря на frozen, ServoDriver обновляет `_angle=30.9`.

`_freeze_refresh` в драйвере периодически дёргает `_angle` — после первого
tick'а уже видит свежее значение. Гонок нет, потому что `_state_lock`
закрывает и `_interpolate_tick`, и `_set_joint`; freeze не использует
state_lock, но и не пишет `_target_angles`/`_current_angles`.

### `_unlock_if_needed` повторно после ручной разморозки

Сценарий: пользователь жмёт «Разм. все» → `_servos[*].unfreeze()`.
`_servo_initialized` остаётся True, `_locked=False`. Следующая команда
вызывает `_unlock_if_needed` — но `if self._locked` False, `_unlock` НЕ
запускается. Авто-freeze в `_unlock` не срабатывает повторно. Хорошо: мы
не хотим, чтобы любая команда повторно морозила; только home/preset/joints
явно re-freeze'ят.

### `locked=True` + `claw_init_on_startup=True` (другая ветка конфига)

В `__init__` идёт `_init_claw_only()` — только CH3 получает PWM в home.
CH0/1/2 без PWM (start_disabled=True). `_servo_initialized` остаётся False.

Первая arm/command → `_unlock_if_needed` → `_unlock` → ставит все 4 в home
(force=True) → ставит `_servo_initialized=True` → авто-freeze CH0/1/2.
Корректно ✓.

### Race FSM publish vs auto-freeze

FSM публикует `load_preset grab_ready` в `_do_approach`. На бэке `_cmd_cb`
обрабатывает синхронно: set_target → freeze. Дальше FSM ждёт settle и
шлёт `load_preset grab_hold` — то же.

Если бы между двумя publish'ами кто-то ещё дёрнул unfreeze — race возможен,
но в текущей архитектуре FSM единственный авторитет команд во время grab.
Не митигируем.

## Тесты `tests/test_arm_node.py`

### Удалить/переписать

**`test_cmd_cb_load_preset_skips_frozen`** (line 335) — семантический флип:
новое имя `test_load_preset_overrides_frozen_and_refreezes`, проверяет
обратное.

### Добавить

1. **`test_unlock_auto_freezes_ch0_ch1_ch2`** — после `node._unlock()`:
   `[s.frozen for s in node._servos] == [True, True, True, False]`.

2. **`test_home_overrides_frozen_then_refreezes`** — заморозить все,
   послать `arm/command "home"`, протикать интерполятор → суставы
   доезжают до `home_angles`, `frozen == [T, T, T, F]`.

3. **`test_load_preset_overrides_frozen_and_refreezes`** — сохранить preset
   `[a, b, c, d]`, заморозить все, `load_preset` → `_target_angles ==
   [a, b, c, d]`, `frozen == [T, T, T, F]`.

4. **`test_joints_array_overrides_frozen_and_refreezes`** — то же через
   `{"joints": [a, b, c, d]}`.

5. **`test_unfreeze_persists_until_home_or_preset`** — unfreeze всех,
   single-joint command → `_target_angles[i]` обновился, `frozen[i]
   == False`. После `home` → `frozen[0..2] == True` снова.

6. **`test_fsm_grab_sequence_with_default_frozen`** — стартовое состояние
   после `_unlock` (frozen=[T,T,T,F]), последовательно
   `load_preset grab_ready` → settle → `load_preset grab_hold` → settle →
   `freeze`. Проверить: финальные `_target_angles == grab_hold`, финальный
   `frozen == [T, T, T, F]` (CH3 свободна, freeze-команда без joint
   морозит только CH0..CH2; явный `freeze joint=4` от FSM морозит и
   клешню).

7. **`test_freeze_after_set_target_holds_old_angle`** — поставить target
   CH0 в 100°, current=30°, вызвать `_freeze_all_except_claw`,
   протикать раз → `_servos[0]._angle ≈ 30.9` (current шагнул), frozen
   остался True. Защита от регресса: интерполятор должен продолжать
   шагать frozen-сустав после re-freeze.

## Что НЕ делаем (out of scope)

- Head (CH4) — отдельный subsystem, пользователь явно не упомянул.
- Новый config-флаг `auto_freeze_on_home: bool` — YAGNI.
- Изменение `ServoDriver` или `_freeze_refresh`.
- Изменение `ServoControlPanel.tsx`.
- Android-клиент — он шлёт те же arm/command MQTT, поведение применится
  автоматически.
- Отдельная обработка для `_init_claw_only` ветки конфига — там CH0/1/2
  без PWM, freeze их не имеет смысла (включит PWM на потенциально
  неверной позе). `_unlock` всё равно произойдёт на первой команде.

## Риски

1. **Безопасность FSM grab при внешних `load_preset`:** ранее
   frozen-сустав был «броней» от случайного `load_preset` после захвата
   мяча. Теперь любой preset принудительно сдвинет CH0/1/2 и заморозит
   на новой позиции. Если кто-то по ошибке шлёт `load_preset rest_pose`
   после grab — мяч выпадет. Пользователь явно выбрал жёсткий вариант
   (option B) — принимаем.

2. **Регресс с FSM `unfreeze`-шагом:** FSM grab не шлёт `unfreeze` перед
   `load_preset grab_ready` (см. `fsm_node.py:449`). Раньше это работало,
   потому что суставы не были frozen на старте. С новой семантикой это
   тоже работает — `load_preset` использует `allow_frozen=True`.

3. **Тёплый pwm-frame между set_target и freeze:** см. секцию «Гонки».
   Интерполятор корректно продолжает шагать после re-freeze благодаря
   `force=frozen` (spec 2026-05-19-frozen-servo-slider-drag).

4. **Совместимость с runGrabTest (frontend):** запускается
   `unfreeze → grab_ready → wait → grab_hold → wait → freeze`. С новой
   семантикой первый `unfreeze` избыточен (load_preset сам переопределит),
   но не вреден. Не трогаем.

## Acceptance criteria

- После запуска `arm_node` `arm/state.frozen == [True, True, True, False]`
  без каких-либо команд.
- После `arm/command "home"` (с любым предыдущим frozen-состоянием)
  суставы достигают `home_angles`, `frozen == [T, T, T, F]`.
- После `arm/command load_preset grab_hold` (с любым frozen-состоянием)
  суставы достигают grab_hold, `frozen == [T, T, T, F]`.
- После `arm/command "unfreeze"` `frozen == [F, F, F, F]`; следующая
  single-joint команда не меняет frozen; следующий `home` → frozen[0..2]
  снова True.
- FSM grab sequence end-to-end проходит без падений и оставляет
  `frozen == [T, T, T, T]` (явный `freeze joint=4` от FSM морозит CH3).
- Все существующие тесты `tests/test_arm_node.py` зелёные после
  переписывания `test_cmd_cb_load_preset_skips_frozen` и добавления
  новых.
