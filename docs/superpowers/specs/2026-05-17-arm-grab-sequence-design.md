# Захват цветного объекта — две позы руки, плавный переход, freeze-удержание

> Робот ищет яркий объект (красный / синий / зелёный), центрируется на нём,
> подъезжает и захватывает с помощью 4-DOF руки. Две именованные позы:
> `grab_ready` (рука вытянута вперёд, клешня открыта) и `grab_hold`
> (рука прижата к корпусу, клешня закрыта). Переход между углами —
> плавный (текущий моментальный `set_angle` ощущается резко). Заморозка
> руки активна, пока объект в клешне (прокси-сигнал — «клешня закрыта»).
>
> **Базируется на:** `main` (ветка изменений выберется при writing-plans).
> Переиспользует существующий FSM (`fsm_node.py`), HSV-детектор
> (`compute_node/detector.py`) и систему пресетов (`servo_presets.json`)
> без изменения их интерфейсов.
>
> **Скоуп:** `pi_nodes/nodes/arm_node.py`, `pi_nodes/nodes/fsm_node.py`,
> `compute_node/dashboard/routers/actuators.py`,
> `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`,
> `config.yaml`, `servo_presets.json` (миграция). Без изменений в
> `ServoDriver` (low-level PWM), в детекторе, в схемах MQTT.

## 1. Контекст и проблема

В проекте уже работает автономный хант мяча:

1. **HSV-детектор** (`compute_node/detector.py:143`) публикует
   `samurai/{robot_id}/ball_detection` с `{colour, x, y, w, h, conf}`.
2. **FSM** (`pi_nodes/nodes/fsm_node.py`) проходит цепочку
   `IDLE → SEARCHING → TARGETING → APPROACHING → GRABBING → RETURNING`.
3. **`_do_grab`** (`fsm_node.py:473`) — фиксированный 3-секундный сценарий:
   - `t<1.0с`: едем вперёд `0.05 м/с`,
   - `t<2.0с`: стоп,
   - `t<3.0с`: `claw/command "close"`,
   - `t≥3.0с`: `RETURNING`.

Сценарий **не использует углы суставов 1-3** — клешня просто закрывается
в той позе, в которой рука оказалась. Никаких именованных поз для
«вытянутая вперёд» / «прижата с объектом» нет.

`ServoDriver.set_angle()` (`pi_nodes/hardware/servo_driver.py:91`)
выставляет PWM **моментально**. UI-слайдер дёргает серво при каждом
шаге; пресет переключает все 4 угла одновременно — наблюдаемый эффект
«резкий рывок». Никакой интерполяции / easing / ramp на уровне Pi нет
(подтверждено поиском по `pi_nodes/`).

В UI слайдер CH0 (`ARM_JOINTS[0].max`) сейчас зажат на 120°
(`ServoControlPanel.tsx:18`). Пользователь хочет позу с CH0=160° —
слайдер не позволяет.

Пользователь хочет:

1. Две именованные позы: `grab_ready = [160, 100, 180, 0]` и
   `grab_hold = [10, 30, 180, 180]` (логические углы, CH3: 0=open,
   180=closed; инверсия CH3 уже настроена в `config.yaml`).
2. Плавный переход между любыми углами — для всех источников (UI-слайдер,
   пресет, FSM).
3. Freeze руки во время захвата и пока объект в клешне; авто-unfreeze при
   открытии клешни.
4. UI-лимит CH0 = 160° (вместо текущих 120°).

## 2. Решения брейншторма (2026-05-17)

| # | Вопрос | Решение |
|---|---|---|
| 1 | Когда рука едет в `grab_ready`? | В начале `APPROACHING` — рука выезжает «клешнёй наготове» одновременно с подъездом. |
| 2 | Как долго freeze после grab? | **Пока объект в руке**. Прокси-сигнал — «клешня закрыта»; разморозка авто-триггерится открытием клешни. (Отвязали от исходных «10с».) |
| 3 | Где реализовать сглаживание? | Pi `arm_node` — интерполяция target/current @ 50Гц. Одно место в коде — все источники углов плавные. |
| 4 | Что считается «открытием клешни»? | Команда `set_claw(state="open")` через REST/UI. Voice-команда тоже идёт через тот же endpoint (`fsm_node` уже использует общую шину). |
| 5 | Что с UI-кнопкой «Захват»? | Не добавляем отдельную. Пресеты `grab_ready` / `grab_hold` появятся в существующем `PresetSection`, тест вручную доступен. Боевой сценарий — через FSM (`SEARCHING`-цикл). |
| 6 | Что делает робот после grab? | `→ RETURNING` (как сейчас). Рука остаётся frozen в `grab_hold`. Корпус едет домой, объект удержан клешнёй. |

## 3. Архитектура

### 3.1 Структурное

```
┌──────────────────────────────── Compute (laptop) ────────────────────────────────┐
│ detector.py ──HSV──> MQTT ball_detection                                          │
│ FastAPI /api/actuators/claw  ── set_claw(state="open")                            │
│        └─> publish arm/command joint=4 angle=0                                     │
│        └─> publish arm/command {"command":"unfreeze"}  ← НОВОЕ                     │
│ FastAPI /api/actuators/arm  — без изменений                                        │
│ Frontend ServoControlPanel.tsx — ARM_JOINTS[0].max: 120 → 160  ← НОВОЕ             │
└──────────────────────────────────┬──────────────────────────────────────────────┘
                                   │ MQTT
┌──────────────────────────────── Pi (raspberry) ─────────────────────────────────┐
│ fsm_node.py                                                                       │
│   _do_approach:                                                                   │
│     при первом входе → arm/command {"command":"load_preset","name":"grab_ready"}  │ ← НОВОЕ
│   _do_grab (новая логика):                                                        │
│     t=0:   arm/command {"command":"load_preset","name":"grab_hold"}               │ ← было: claw close
│     t=settle: arm/command {"command":"freeze"}                                    │ ← НОВОЕ
│     → RETURNING (рука остаётся frozen)                                            │
│                                                                                   │
│ arm_node.py                                                                       │
│   _target_angles[i] / _current_angles[i] — новые поля                             │ ← НОВОЕ
│   _interpolate_tick() @ 50 Гц — шагает current → target с max_speed              │ ← НОВОЕ
│   _set_joint / load_preset / joints[] — ставят только _target                     │ ← ИЗМ.
│   freeze: _target = _current (стоп шага), PWM-удержание                          │ ← ИЗМ.
│   На старте — migrate: если нет grab_ready/grab_hold → создать                    │ ← НОВОЕ
│                                                                                   │
│ servo_driver.py — БЕЗ ИЗМЕНЕНИЙ                                                   │
└───────────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Состояние `arm_node`

Добавляются поля:

```python
self._target_angles: list[float] = list(self._home_angles)
self._current_angles: list[float] = list(self._home_angles)
self._max_speed: float = cfg('servos.arm.max_speed_deg_per_sec', 120.0)
```

Существующее `self._angles` устраняем — его роль разделяется:

- **Команды извне (writes)** — `_set_joint`, `joints[]`, `load_preset`,
  `home`, `unlock` — ставят значения в `_target_angles[i]`.
- **Публикация и логи (reads)** — `_publish_state` (`arm/state`),
  `log_info` после команды — читают `_current_angles[i]` (где реально
  стоит сервопривод сейчас, плавно сходящееся к target).
- **Исключение для logs «куда пошла команда»** — в `log_info` после
  команды показываем `_target_angles[i]` (то значение, которое
  запрашивали), чтобы лог не «врал» о моментальной позиции при
  длинных переходах.

Интерполяционный таймер @ 50 Гц:

```python
self.create_timer(0.02, self._interpolate_tick)
```

Шаг (псевдокод):

```python
def _interpolate_tick(self):
    dt = 0.02
    max_step = self._max_speed * dt  # 120 * 0.02 = 2.4°/tick
    for i in range(self._num_joints):
        if self._servos[i].frozen:
            continue
        delta = self._target_angles[i] - self._current_angles[i]
        if abs(delta) <= max_step:
            self._current_angles[i] = self._target_angles[i]
        else:
            self._current_angles[i] += math.copysign(max_step, delta)
        phys = self._to_physical(i, self._current_angles[i])
        self._servos[i].set_angle(phys)
```

`ServoDriver.set_angle` уже клампит 0..180 и обрабатывает `frozen`.
`_schedule_release` в драйвере (HOLD_TIME=0.5с) продолжает работать —
PWM держится 0.5с после каждого шага, что при 50Гц = всегда активен,
пока идёт движение, и отпускается при достижении target.

### 3.3 Авто-миграция пресетов

При старте `ArmNode.__init__`, после создания `ServoPresets`:

```python
_DEFAULT_PRESETS = {
    'grab_ready': [160.0, 100.0, 180.0, 0.0],
    'grab_hold':  [10.0, 30.0, 180.0, 180.0],
}
for name, angles in _DEFAULT_PRESETS.items():
    if self._presets.load_preset('arm', name) is None:
        self._presets.save_preset('arm', name, angles)
        self.log_info('Migration: created arm preset "%s"', name)
```

Если пользователь сохранит свой вариант (например, скорректирует CH1 на
живом железе) — миграция его НЕ перезатрёт (есть → пропускаем).

### 3.4 Изменения в FSM

Новые поля состояния:

```python
self._approach_arm_sent = False   # сбрасывается при _transition
self._grab_t = 0.0                # локальный таймер фазы GRABBING
```

В `_transition` сбрасываем оба:

```python
def _transition(self, new_state):
    self._approach_arm_sent = False
    self._grab_t = 0.0
    self._approach_timeout = 0.0
    ...
```

`_do_approach` — добавляем в начале:

```python
if not self._approach_arm_sent:
    self.publish('arm/command',
                 {'command': 'load_preset', 'name': 'grab_ready'}, qos=1)
    self._approach_arm_sent = True
    self.log_info('Arm → grab_ready (approach start)')
```

`_do_grab` — полная замена:

```python
def _do_grab(self):
    self._grab_t += 0.1  # tick = 100ms

    # Phase 1: послать grab_hold (один раз)
    if self._grab_t <= 0.1:
        self.publish('arm/command',
                     {'command': 'load_preset', 'name': 'grab_hold'}, qos=1)
        self.log_info('Arm → grab_hold (closing claw)')
        return

    # Phase 2: подождать settle (1.5с — запас на интерполяцию)
    GRAB_SETTLE_S = 1.5
    if self._grab_t < GRAB_SETTLE_S:
        return

    # Phase 3: freeze + переход в RETURNING (один раз)
    self.publish('arm/command', {'command': 'freeze'}, qos=1)
    self.log_info('Arm FROZEN — holding object')
    self._transition(State.RETURNING)
```

Расчёт `GRAB_SETTLE_S`: самая длинная дельта при переходе `grab_ready →
grab_hold` — это CH0 (160→10 = 150°). При `max_speed=120°/с` это
**1.25с**. Округлено вверх до 1.5с (запас на jitter таймера).

### 3.5 Авто-unfreeze при открытии клешни

В `compute_node/dashboard/routers/actuators.py:77` `set_claw`:

```python
@router.post('/claw', ...)
async def set_claw(cmd: ClawCommand, mqtt: MQTTDep) -> CommandAck:
    ...
    mqtt.publish('arm/command', {'joint': 4, 'angle': angle}, qos=1)
    if cmd.state == 'open' or (cmd.angle is not None and cmd.angle < 90.0):
        mqtt.publish('arm/command', {'command': 'unfreeze'}, qos=1)
    return CommandAck()
```

Идемпотентно: `arm_node._cmd_cb` корректно обрабатывает unfreeze на
уже-размороженной руке (no-op).

Альтернативный путь — `claw/command "open"` (отдельный legacy topic от
FSM `_do_grab` старой версии) — больше не используется в нашей версии
`_do_grab` (мы выкидываем `publish('claw/command', 'open')` целиком, всё
идёт через `arm/command`).

### 3.6 Конфиг

`config.yaml`:

```yaml
servos:
  arm:
    home_angles: [0, 120, 0, 0]
    min_angles:  [0, 0, 0, 0]
    max_angles:  [160, 145, 180, 180]   # было [120, ...]
    invert_angles: [false, false, false, true]
    max_speed_deg_per_sec: 120          # НОВОЕ — общий лимит для всех 4 суставов
    labels: [...]
    locked: true
```

Если позже понадобится per-joint скорость — расширим до списка
`[120, 120, 120, 90]` (CH3=клешня может быть медленнее). Сейчас YAGNI.

### 3.7 UI

`compute_node/frontend/src/components/actuators/ServoControlPanel.tsx:18`:

```typescript
const ARM_JOINTS = [
  { label: 'Основание',  min: 0, max: 160, home: 0   },  // было max: 120
  { label: 'Сустав 1',   min: 0, max: 145, home: 120 },
  { label: 'Сустав 2',   min: 0, max: 180, home: 0   },
  { label: 'Клешня',     min: 0, max: 180, home: 0   },
]
```

JS-комментарий в шапке (строка 9-14) также обновить: `CH0 [0; 160]`.

После `npm run build` Vite перегенерит `compute_node/static/assets/*.js`
и `index.html` — изменения в репо автоматические.

### 3.8 `arm/state` публикация

Текущий код:

```python
state[f'j{i+1}'] = round(self._angles[i], 1)
```

Заменяем на `self._current_angles[i]`. UI получает реальный плавно
меняющийся угол — слайдеры визуально едут, не дёргаются скачком.

## 4. Edge cases и инварианты

| Случай | Поведение |
|---|---|
| Slider дёргается (пользователь быстро тянет CH0 с 0 до 160) | Каждое движение → новый target. Интерполятор переключается на актуальный target на следующем тике. UI трottle (80мс) уже сглаживает поток до Pi. Глобально — рука едет с max_speed, всегда к последнему target. |
| Команда `joints=[160, 100, 180, 0]` во время уже идущего движения | Все 4 target обновляются разом. Интерполятор шагает каждый сустав независимо до своего target. CH3 (короткая дельта) приедет быстрее, CH0 — медленнее. Это нормально. |
| `home` команда | `_target_angles = _home_angles`. Интерполятор плавно отвезёт. Никаких отдельных «жёстких» команд для home. |
| `unlock` (первое включение) | `_target = _current = _home_angles`, серво из `start_disabled` уходит в активный режим. Один раз — мгновенный `set_angle(force=True)` для инициализации (как сейчас). Затем интерполятор продолжает с current. |
| `freeze` во время движения | Все шаги останавливаются на текущем `_current_angles`. PWM-удержание берёт это значение. Когда `unfreeze` — интерполятор возобновляет с current к (возможно изменившемуся) target. |
| `load_preset grab_hold` пришёл, пока FSM ждёт settle (1.5с), а пользователь нажал «Разморозить все» из UI | Freeze снимается, интерполятор продолжает движение к `grab_hold`. Если потом FSM шлёт freeze второй раз — нормально, заморозит на текущей позе (может быть не в `grab_hold` ровно, но близко). Не критично. |
| FSM ушёл в `RETURNING`, пользователь открыл клешню по дороге | `set_claw(open)` → `joint=4 angle=0` + `unfreeze`. Интерполятор открывает клешню плавно. CH0/CH1/CH2 остаются где были (target тот же, что и current — стоят). Если хочется автоматически home — это отдельная задача, не входит в этот scope. |
| Пользователь сохранил свой `grab_hold` с другими углами | Миграция при старте: `load_preset('grab_hold')` вернёт пользовательский → не перезаписываем. Чтобы вернуть дефолт — пользователь удаляет пресет, перезапускает Pi. |
| `max_speed_deg_per_sec` = 0 или отрицательное | Не валидируем явно — `max_step = 0`, шага нет. Рука «зависает» в текущей позе, никакие команды не работают визуально. Защита: в `__init__` clamp `max(1.0, cfg(...))`. |

## 5. Тестирование

- **Unit / smoke (Pi)** — `pi_nodes/test_arm_interpolation.py` (новый):
  - mocked `ServoDriver`, проверяем что `_interpolate_tick` приближает
    current → target ровно на `max_step` за тик;
  - load_preset → target обновляется → current не меняется до тика;
  - freeze останавливает шаги;
  - migration: при пустом `servo_presets.json` пресеты создаются.
- **Manual (на железе)**:
  - UI: тянем слайдер CH0 от 0 до 160 — серво плавно едет, не дёргается.
  - UI: нажимаем «Загрузить grab_ready» → рука едет в позу плавно за ~1.5с.
  - UI: нажимаем «Загрузить grab_hold» → клешня закрывается плавно.
  - voice: «возьми красный мяч» → робот ищет, центрируется, едет, рука
    выезжает в grab_ready, при доезде закрывается в grab_hold, freeze
    активен. Нажать UI «Open claw» → клешня открывается, рука
    размораживается.
- **Регрессии**: проверить что обычные команды (home, freeze per-joint,
  save_preset) работают как раньше; что MPS-сценарий (DRIVE_FORWARD_MPS)
  не затронут (он не использует arm/command в принципе).

## 6. Что НЕ входит в scope

- Не меняем `ServoDriver` (low-level PWM-таймеры, freeze refresh).
- Не добавляем сенсор «объект в клешне» (нет хардвера). Прокси —
  «клешня закрыта».
- Не меняем HSV-детектор и центрирование в FSM.
- Не добавляем UI-кнопку «Захват» отдельно — пресеты доступны через
  существующий `PresetSection`. Если позже потребуется one-click —
  follow-up.
- Не делаем per-joint `max_speed` — общий лимит. Если CH3 окажется
  визуально слишком медленным/быстрым относительно остальных — выделим
  потом.
- Не трогаем Samcan / Android / MPS.

## 7. Открытые вопросы

Нет. Все ключевые развилки закрыты в брейншторме 2026-05-17.
