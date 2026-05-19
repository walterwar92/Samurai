# Захват v2 — open-первый, 20-секундный hold клешни, возврат в grab_ready

> Меняем `_do_grab` FSM с текущей 3-фазной (load grab_hold → settle → freeze)
> на 5-фазную (open → 1s → grab_hold → settle+1s → grab_return → RETURNING).
> Клешня замораживается с 20-секундным auto-unfreeze таймером в `arm_node` —
> через 20с после открытия клешни она автоматически разморозится (мяч
> освободится после release PWM). Параллельно фиксим баг в
> `actuators.py`: кнопка «открыть клешню» в UI размораживает ТОЛЬКО клешню,
> не CH0/1/2.
>
> **Базируется на:** `main` (ветка для реализации выберется при writing-plans).
> Переиспользует существующий `arm_node` (target/current интерполятор,
> ServoPresets), FSM (state machine, _do_approach без изменений) и систему
> авто-миграции пресетов.
>
> **Скоуп:** `pi_nodes/nodes/arm_node.py`, `pi_nodes/nodes/fsm_node.py`,
> `compute_node/dashboard/routers/actuators.py`, тесты в `tests/`. Без
> изменений в `ServoDriver`, в UI компонентах, в схемах MQTT (только
> расширяется payload существующих команд).

## 1. Контекст и проблема

Текущая логика захвата (см. `2026-05-17-arm-grab-sequence-design.md`):

1. APPROACHING → `arm/command load_preset grab_ready` (рука выдвигается, клешня открыта)
2. GRABBING phase 1 → `load_preset grab_hold` (клешня закрывается на объекте)
3. GRABBING phase 2 → ждём `settle = max_delta / min_speed + 0.25с`
4. GRABBING phase 3 → `freeze` (CH0/1/2) + `freeze joint=4` → RETURNING
5. Открытие клешни через UI → `unfreeze` (все суставы) → ручное управление

Проблемы:

1. **Нет «возврата в исходную позу» после захвата** — рука остаётся в `grab_hold`
   (прижатая к корпусу, CH0=0). Робот едет в RETURNING в неудобной для
   повторного захвата позе. Хотим возвращаться в `grab_ready` (вытянутая
   вперёд, готова к следующему циклу).
2. **Клешня замораживается «навсегда»** — пока не пришло явное unfreeze. На
   практике объект нужно удерживать только пока робот доедет домой
   (~10-30с). После этого автоматическое освобождение проще, чем оркестрация
   из FSM.
3. **Нет «открыть клешню перед захватом»** — текущая логика предполагает
   что `grab_ready` уже открыл клешню (CH3=0). Но если робот делал второй
   подряд захват без полного цикла, клешня могла остаться закрытой.
   Явная фаза «открыть и подождать» делает поведение детерминированным.
4. **Баг в `actuators.py:97`** — открытие клешни через UI публикует
   `{'command': 'unfreeze'}` без `joint` параметра. `arm_node._cmd_cb`
   при отсутствии `joint` размораживает ВСЕ серво (CH0/1/2/3). Пользователь
   ожидает, что кнопка «открой клешню» трогает только клешню.

Пользователь хочет:

1. Фаза «открой клешню» в начале захвата, freeze с 20-секундным таймером,
   автоматический unfreeze после.
2. Задержка 1с после открытия клешни перед движением.
3. Возврат в позу `grab_ready` (но с закрытой клешнёй, удерживая объект)
   после захвата, перед RETURNING.
4. Фикс UI-кнопки клешни: размораживать только её.

## 2. Решения брейншторма (2026-05-19)

| # | Вопрос | Решение |
|---|---|---|
| 1 | Что значит «принять текущее начальное положение перед захватом»? | Использовать `grab_ready` как initial. APPROACHING уже ставит руку в grab_ready; grab-последовательность считает его исходной позой. |
| 2 | Как реализовать «возврат в исходную позу»? | `load_preset grab_ready`-эквивалент через новый preset `grab_return` (= grab_ready[CH0/1/2] + claw=180 closed). Не зависит от текущих углов, проще и стабильнее. |
| 3 | Где живёт 20-секундный таймер? | В `arm_node` (расширение команды freeze параметром `duration`). Орк FSM фаз остаётся видимым; таймер изоляции — там, где живёт состояние клешни. FSM выходит в RETURNING сразу, не блокируется. |
| 4 | Клешня при возврате — открыта или закрыта? | **Закрыта** — держит мяч ещё 20с. Через 20с (auto-unfreeze) PWM освободится и мяч упадёт. Поэтому grab_return ≠ grab_ready: CH3=180 (closed). |

## 3. Архитектура

### 3.1 Структурное

```
┌─── Compute (laptop) ────────────────────────────────────────────────────┐
│ FastAPI /api/actuators/claw ── set_claw(state="open")                    │
│   └─> publish arm/command {joint:4, angle:0}                              │
│   └─> publish arm/command {command:"unfreeze", joint:4}  ← FIX            │ ← Bug fix
│                                                                           │
│ FastAPI /api/actuators (без изменений)                                    │
│ Frontend (без изменений)                                                  │
└──────────────────────────────────┬───────────────────────────────────────┘
                                   │ MQTT
┌─── Pi (raspberry) ──────────────────────────────────────────────────────┐
│ fsm_node.py                                                              │
│   _do_approach (без изменений) → load_preset grab_ready                  │
│   _do_grab — НОВАЯ 5-фазная логика:                                      │
│     Phase 1 (one-shot): open claw + freeze claw duration=20             │ ← NEW
│     Phase 2: wait 1.0с                                                   │ ← NEW
│     Phase 3 (one-shot): load_preset grab_hold                            │
│     Phase 4: wait settle + 1.0с                                          │ ← NEW
│     Phase 5: load_preset grab_return → RETURNING                         │ ← NEW
│                                                                          │
│ arm_node.py                                                              │
│   _freeze_timers: list[Timer|None] @ per-joint                          │ ← NEW
│   freeze command: новый optional param "duration"                        │ ← NEW
│     joint=N + duration=X → freeze(joint) + Timer(X, unfreeze(joint))     │
│   unfreeze command: отменяет таймер если был                             │ ← MOD
│   Авто-миграция grab_return = [grab_ready[0..2], 180.0]                  │ ← NEW
└──────────────────────────────────────────────────────────────────────────┘
```

### 3.2 `arm_node` — freeze with duration

Расширяем существующую freeze-команду:

```python
# Текущий API (без изменений):
{"command": "freeze"}                   # freeze CH0/1/2 (except claw)
{"command": "freeze", "joint": 4}       # freeze CH3 only
{"command": "unfreeze"}                 # unfreeze all
{"command": "unfreeze", "joint": 4}     # unfreeze CH3 only

# Новый API (расширение):
{"command": "freeze", "joint": 4, "duration": 20}  # freeze CH3 + auto-unfreeze в 20с
{"command": "freeze", "duration": 20}              # freeze CH0/1/2 + auto-unfreeze в 20с (не используется FSM, но валидно)
```

Состояние:

```python
# В ArmNode.__init__:
self._freeze_timers: list[threading.Timer | None] = [None] * self._num_joints
```

Обработка freeze:

```python
def _freeze_joint(self, idx: int, duration: float | None = None):
    """Заморозить сустав idx. Если duration задан — schedule auto-unfreeze.
    
    Идемпотентно по таймерам: повторный freeze с duration на том же
    суставе отменяет предыдущий таймер и стартует новый.
    """
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
    """Колбэк таймера: разморозить сустав idx."""
    self._freeze_timers[idx] = None
    self._servos[idx].unfreeze()
    self.log_info('Arm joint %d AUTO-UNFROZEN (timer expired)', idx + 1)
```

Обработка unfreeze (отменяет таймер):

```python
def _unfreeze_joint(self, idx: int):
    if self._freeze_timers[idx] is not None:
        self._freeze_timers[idx].cancel()
        self._freeze_timers[idx] = None
    self._servos[idx].unfreeze()
```

В `_cmd_cb`:

```python
if cmd == 'freeze':
    self._unlock_if_needed()
    joint = d.get('joint')
    duration = d.get('duration')  # Optional[float]
    if joint is not None:
        idx = int(joint) - 1
        if 0 <= idx < self._num_joints:
            self._freeze_joint(idx, duration)
    else:
        # Mass freeze (CH0/1/2). Если duration задан — применяем ко всем CH0/1/2.
        for i in range(self._num_joints - 1):  # exclude claw
            self._freeze_joint(i, duration)
        self.log_info('Arm joints FROZEN (claw excluded)')
    return

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

`_freeze_all_except_claw` остаётся, но внутри использует `_freeze_joint(i, None)`
вместо прямого `_servos[i].freeze()` — единый путь сквозь таймер-кэш.

### 3.3 `arm_node` — авто-миграция `grab_return`

В существующем словаре `_DEFAULT_ARM_PRESETS`:

```python
_DEFAULT_ARM_PRESETS = {
    'grab_ready': [30.0, 60.0, 0.0, 0.0],
    'grab_hold':  [0.0, 100.0, 0.0, 180.0],
    'grab_return': [30.0, 60.0, 0.0, 180.0],  # NEW: grab_ready CH0/1/2 + claw closed
}
```

Логика миграции уже существует (создаёт пресет если отсутствует, не
перетирает пользовательский). Никаких изменений в коде миграции, только
расширение словаря.

### 3.4 `fsm_node._do_grab` — 5-фазная логика

Новые поля состояния:

```python
# В FSM.__init__:
self._grab_open_sent = False    # Phase 1 done
self._grab_hold_sent = False    # Phase 3 done
# Существующий self._grab_t остаётся.
```

Сброс в `_transition`:

```python
def _transition(self, new_state):
    ...
    self._grab_open_sent = False
    self._grab_hold_sent = False
    self._grab_t = 0.0
    ...
```

`_do_grab`:

```python
def _do_grab(self):
    """Захват v2 — 5-фазная последовательность.
    
    Phase 1 (one-shot, ~t=0):
        publish arm/command {joint:4, angle:0}              # open claw target
        publish arm/command {command:"freeze", joint:4, duration:20.0}
                                                            # freeze + 20s timer
    Phase 2 (wait): t < 1.1с — ждём пока клешня откроется + 1с задержка.
    Phase 3 (one-shot, ~t=1.1с):
        publish arm/command {command:"load_preset", name:"grab_hold"}
                                                            # CH0/1/2 → grab_hold,
                                                            # claw target → 180
    Phase 4 (wait): t < 1.1 + settle + 1.0с — ждём settle + 1с задержка после.
    Phase 5 (one-shot, finally):
        publish arm/command {command:"load_preset", name:"grab_return"}
                                                            # CH0/1/2 → grab_ready,
                                                            # claw stays 180
        _transition(State.RETURNING)
    
    Через ~20с от Phase 1 arm_node auto-unfreeze клешню → PWM release
    через HOLD_TIME → мяч освобождается.
    """
    self._grab_t += 0.1  # tick 100ms
    
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
    # settle = max_delta / min_speed. Консервативный upper bound 100°:
    # текущие дефолтные пресеты дают max(CH0/1/2)=40° (CH1: 60→100),
    # но пользователь может отредактировать пресеты — 100° запас.
    # Берём min(max_speeds) чтобы учесть самый медленный сустав.
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

Расчёт `phase_5_t`:
- Phase 1: t=0.0 (одна команда, ~0.1с реально)
- Phase 2 end: t=1.1с (1с после Phase 1)
- Phase 3: t=1.1с (одна команда)
- Phase 4 end: t=1.1 + grab_settle_s + 1.0с
- Phase 5: t=phase_5_t

При scalar max_speed=120°/с: settle=0.83с, phase_5_t≈2.93с.
При config `[45,45,9999,9999]` (текущий список): min_speed=45°/с, settle=2.22с, phase_5_t≈4.32с.
При scalar max_speed=9999: settle=0.01с, phase_5_t≈2.11с.

### 3.5 Bug fix в `actuators.py`

`compute_node/dashboard/routers/actuators.py:97`:

```python
# БЫЛО:
if angle < 90.0:
    mqtt.publish('arm/command', {'command': 'unfreeze'}, qos=1)

# СТАНЕТ:
if angle < 90.0:
    mqtt.publish('arm/command',
                 {'command': 'unfreeze', 'joint': 4}, qos=1)
```

Эффект:
1. Открытие клешни через UI размораживает ТОЛЬКО клешню (не трогает CH0/1/2).
2. Если активен 20s-таймер из FSM grab — он отменяется (через
   `_unfreeze_joint(3)` → `cancel()` таймера).

Обновить docstring `set_claw`: «При открытии клешни (state=open или angle<90)
дополнительно публикуем `arm/command unfreeze joint=4` — снимаем заморозку
ТОЛЬКО клешни (CH0/1/2 остаются под FSM-контролем). Если был активен
20s auto-unfreeze таймер (FSM grab) — он отменяется».

### 3.6 Что НЕ меняется

- `_do_approach` (по-прежнему ставит `grab_ready` при входе в APPROACHING)
- `ServoDriver` (low-level PWM)
- Frontend компоненты (`ServoControlPanel`, `PresetSection` etc.)
- MQTT топики и схемы (только расширяется payload существующих команд)
- `_freeze_all_except_claw`: остаётся семантически (морозит CH0/1/2), но
  внутри использует `_freeze_joint(i, None)` для единства

## 4. Edge cases и инварианты

| Случай | Поведение |
|---|---|
| Пользователь нажимает «Открыть клешню» в UI **во время** FSM grab (когда активен 20s таймер) | `set_claw(open)` шлёт `joint=4 angle=0` (target=0) + `unfreeze joint=4` (cancel timer, frozen=false). Клешня плавно открывается. Объект освобождён сразу, не через 20с. CH0/1/2 не размораживаются. FSM продолжает фазы grab — если ещё не дошли до Phase 5, рука доедет в grab_return, клешня будет открыта. |
| FSM делает второй grab подряд (RETURNING → IDLE → SEARCHING → ... → GRABBING) пока ещё идёт 20s таймер от предыдущего grab | Phase 1 нового grab отменяет старый таймер (через cancel в `_freeze_joint`), стартует новый 20s от текущего момента. Корректно. |
| FSM transition в IDLE/PATROLLING пока 20s таймер активен | Таймер продолжает работать (живёт в arm_node, не зависит от FSM state). Через 20с от Phase 1 клешня разморозится. Если пользователь не хочет этого — нужно вручную `freeze joint=4` или `arm/command "home"` в UI. |
| Кнопка «Заморозить» в UI вызвана пока активен 20s таймер на клешне | UI шлёт `{command:freeze, joint:4}` без duration. `_freeze_joint` отменяет старый таймер + freeze без нового таймера (frozen indefinite). Семантически: пользователь явно перешёл от автоматического hold к ручному. |
| Pi reboot во время 20s hold | Таймер потерян, но и PWM-state потерян (серво обесточен). При старте `claw_init_on_startup=true` клешня уйдёт в home (открыта). Объект потерян, но это OK для аппаратного reset. |
| `freeze` mass-команда с duration={20, ...} | По спеке: морозит CH0/1/2 каждый с своим 20s таймером. Не используется FSM, но валидный API. Все 3 таймера независимы, отменяются индивидуально. |
| `_freeze_all_except_claw` (вызван load_preset/home/joints[]) пока активен 20s таймер на клешне | `_freeze_all_except_claw` морозит CH0/1/2 (без duration). Клешня (CH3) не трогается → её таймер продолжает работать. Корректно. |
| Phase 3 (`load_preset grab_hold`) вызывает `_freeze_all_except_claw` — морозит ли это CH3? | Нет — `_freeze_all_except_claw` исключает последний канал (claw). Клешня остаётся под управлением 20s таймера. Target обновляется до 180 (через `_set_joint(3, 180, allow_frozen=True)` в load_preset) → интерполятор закроет клешню. |
| Phase 5 (`load_preset grab_return`) — клешня уже frozen с timer. Что с target=180? | `load_preset` вызывает `_set_joint(3, 180, allow_frozen=True)` → target[3]=180 (уже было 180 после Phase 3). No change. `_freeze_all_except_claw` морозит CH0/1/2. Клешня frozen остаётся. |
| Таймер успел разморозить клешню ДО Phase 5 (нереалистично, но теоретически) | Если max_speed очень маленький и settle очень долгий, теоретически phase_5_t > 20с. Таймер сработает, клешня unfreeze, мяч может выпасть до возврата. Защита: при scalar speed≥10°/с phase_5_t≤12.1с. На текущем конфиге [45,45,9999,9999] phase_5_t≈4.3с << 20с. При экстремально низком max_speed — нужно увеличить `duration` (20с — параметр FSM, не arm_node; меняется константой). |

## 5. Тестирование

### Unit (`tests/test_arm_node.py`)

Расширить существующий test-файл (используется фабрика `arm_node_factory`):

- `test_freeze_with_duration_starts_timer` — `{command:freeze, joint:4, duration:0.05}` → через 100мс `_servos[3].unfreeze` вызван.
- `test_freeze_duration_restarts_timer` — два подряд freeze с duration → старый таймер отменён.
- `test_unfreeze_cancels_active_timer` — freeze duration → unfreeze → таймер отменён, второй unfreeze (через ожидание duration) не происходит.
- `test_unfreeze_joint_only_targets_one` — `unfreeze joint=4` → только `_servos[3].unfreeze` вызван, CH0/1/2 не тронуты.
- `test_freeze_no_duration_keeps_indefinite` — `freeze joint=4` без duration → нет таймера → `_freeze_timers[3] is None`.
- `test_migration_creates_grab_return` — на пустом presets.json → `grab_return = [30, 60, 0, 180]`.
- `test_migration_preserves_user_grab_return` — пользователь сохранил свой → не перетирается.

Использование real `threading.Timer` с маленьким duration (50-100мс) + `time.sleep(0.15)` в тестах для проверки срабатывания. Альтернатива — мокать `threading.Timer`, но прямой sleep проще и надёжнее.

### Unit (`tests/test_fsm_node.py` или новый `test_fsm_grab_v2.py`)

- `test_grab_phase1_publishes_open_and_freeze_duration` — на первом тике GRABBING → published содержит `{joint:4, angle:0}` и `{command:"freeze", joint:4, duration:20.0}`.
- `test_grab_phase2_waits_1s_no_publishes` — тики 0.1...1.0 → новых публикаций нет.
- `test_grab_phase3_publishes_grab_hold` — на тике 1.1 → `{command:"load_preset", name:"grab_hold"}`.
- `test_grab_phase4_waits_settle` — между Phase 3 и Phase 5 нет публикаций.
- `test_grab_phase5_publishes_grab_return_and_transitions` — после settle+1s → `grab_return` published + state = RETURNING.
- `test_grab_flags_reset_on_transition` — после transition `_grab_open_sent=False, _grab_hold_sent=False`.

### Unit (`tests/test_actuators_router.py` — если не существует, создать)

- `test_set_claw_open_publishes_unfreeze_with_joint` — POST `/api/v1/actuators/claw` body `{"state":"open"}` → published содержит `{command:"unfreeze", joint:4}` (НЕ просто `{command:"unfreeze"}`).
- `test_set_claw_close_does_not_publish_unfreeze` — `state:"close"` → только `joint=4 angle=180`, без unfreeze.

### Manual (на железе)

1. Voice «возьми красный мяч»:
   - Робот ищет, центрируется, едет.
   - Рука выезжает в `grab_ready` (CH0=30, CH1=60, CH2=0, claw open).
   - При доезде: клешня открывается ещё (если была не до конца), freeze 20s timer стартует.
   - 1с задержка.
   - Рука движется в `grab_hold` (CH0=0, CH1=100, CH2=0, claw=180 закрывается на мяче).
   - 1с задержка после settle.
   - Рука едет в `grab_return` (CH0=30, CH1=60, CH2=0, claw=180 — мяч удержан).
   - Робот переходит в RETURNING (едет домой), рука удерживает.
   - Через 20с от Phase 1 → клешня unfreeze → PWM release через HOLD_TIME=0.5с → мяч падает.
2. Мануальная отмена hold: в момент когда рука в grab_return и мяч ещё удерживается, нажать «Открыть клешню» в UI → клешня плавно открывается (target=0), 20s таймер отменён, мяч выпадает сразу. CH0/1/2 остаются в grab_return (НЕ размораживаются — фикс бага).
3. UI смоук-тест:
   - Загрузить `grab_return` через PresetSection → рука едет в [30, 60, 0, 180]. Если клешня была frozen — она остаётся в 180 (target updated, current animates).
   - Загрузить `grab_ready` после → рука едет в [30, 60, 0, 0] — клешня открывается.

## 6. Что НЕ входит в scope

- Не меняем APPROACHING (по-прежнему ставит grab_ready).
- Не меняем low-level PWM (`ServoDriver`).
- Не добавляем UI-кнопку «Захват» (FSM-only, через voice/gesture).
- Не делаем per-joint custom `duration` для mass freeze (только same duration для всех CH0/1/2 при `freeze` без joint — но FSM этим не пользуется).
- Не меняем 20с константу через config — пока хардкод в FSM. Если потребуется тюнить — простой YAGNI, выделим параметр после первого живого теста.
- Не трогаем Samcan / Android / MPS / Voice / Detector.

## 7. Открытые вопросы

Нет. Все ключевые развилки закрыты в брейншторме 2026-05-19.
