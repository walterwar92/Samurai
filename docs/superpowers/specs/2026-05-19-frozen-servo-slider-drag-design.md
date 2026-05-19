# Drag frozen arm joint from UI slider

**Status:** draft
**Date:** 2026-05-19
**Scope:** `pi_nodes/nodes/arm_node.py`, `pi_nodes/hardware/servo_driver.py` (no-op), `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`

## Goal

Позволить пользователю двигать слайдер замороженного (frozen) arm-сустава в дашборде. После движения сустав по-прежнему удерживается под PWM на новой позиции — никакого `unfreeze` под капотом не происходит, индикатор `HOLD` не мигает.

Head-сервопривод (ch4) не затрагивается. Все mass-команды (`home`, `load_preset`, `{joints: [...]}`) при frozen продолжают игнорироваться, чтобы не сорвать захваченный мяч после FSM grab.

## Текущая семантика freeze (что меняем)

| Слой | Сейчас | После |
|---|---|---|
| `ServoDriver.set_angle(force=False)` | при `frozen=True` → no-op | без изменений |
| `ServoDriver.set_angle(force=True)` | пишет `_angle`, выставляет PWM | без изменений (используется чаще) |
| `ServoDriver._freeze_refresh` | периодически переотправляет `_angle` | без изменений |
| `arm_node._interpolate_tick` | `if frozen: continue` — сустав не движется | интерполирует current→target всегда; `set_angle(force=frozen)` |
| `arm_node._set_joint(idx, angle)` | обновляет `_target_angles[idx]` всегда | при `frozen` и `allow_frozen=False` — игнор |
| UI `ServoSlider` (arm) при `frozen` | `disabled` + ранний return в onChange | активен; шлёт `setArmJoint` |
| UI `ServoSlider` (head) при `frozen` | `disabled` | без изменений (флаг `allowFrozenDrag=false`) |

**Ключ к разграничению:** на бэке вход single-joint (`{joint, angle}`) vs mass (`home`/`load_preset`/`{joints:[...]}`). FSM grab шлёт только mass-команды до `freeze` — это даёт корректное разграничение «пользователь руками» vs «программа».

## Архитектура и data flow

```
User drag ch1 90° → 100° (frozen)
  │
  ▼
ServoSlider.handleChange (frozen больше не блокирует)
  │
  ▼
sendArm(1, 100) — throttle 80ms
  │
  ▼
api.setArmJoint(1, 100) → REST → MQTT
  │
  ▼
arm/command {"joint": 1, "angle": 100}
  │
  ▼
_cmd_cb → _set_joint(0, 100, allow_frozen=True)
  │           ├─ frozen=True, allow_frozen=True → пропускает чек
  │           └─ _target_angles[0] = 100
  ▼
_interpolate_tick @ 50Hz (без `if frozen: continue`)
  │  delta = 10°, max_step = 0.9°/tick (при 45°/с)
  │  current[0] += 0.9° per tick → достигнет 100° за ~220ms
  │  каждый tick: _servos[0].set_angle(phys, force=True)
  │
  ▼
ServoDriver.set_angle(phys, force=True) — пишет _angle, выставляет PWM
  │  (freeze_refresh подхватит новый _angle на следующей итерации)
  ▼
delta=0 → continue → _freeze_refresh держит PWM на 100°
```

## Изменения

### `pi_nodes/nodes/arm_node.py`

**`_set_joint`:**

```python
def _set_joint(self, idx: int, angle: float, allow_frozen: bool = False):
    """Set joint TARGET angle (логический) с лимитами.

    Реальный PWM шлёт _interpolate_tick @ 50Гц.
    Если сустав frozen и allow_frozen=False — target НЕ обновляется
    (mass-команды home/preset/joints-array не двигают замороженный
    сустав, чтобы случайно не сорвать захват мяча после FSM grab).
    Single-joint команды от UI слайдера передают allow_frozen=True.
    """
    if idx < 0 or idx >= self._num_joints:
        self.log_warn('Invalid joint index: %d', idx)
        return
    if not allow_frozen and self._servos[idx].frozen:
        return
    angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
    with self._state_lock:
        self._target_angles[idx] = angle
```

**`_cmd_cb` — single-joint ветка ([arm_node.py:374-380](../../pi_nodes/nodes/arm_node.py)):**

```python
if 'joint' in d and 'angle' in d:
    self._unlock_if_needed()
    idx = int(d['joint']) - 1
    angle = float(d['angle'])
    self._set_joint(idx, angle, allow_frozen=True)   # ← +allow_frozen
    self.log_info('Arm joint %d → %.1f°', idx + 1, self._target_angles[idx])
    return
```

Все прочие вызовы `_set_joint(...)` (home, load_preset, `joints:[...]`) оставить без `allow_frozen` — default False сохраняет старое поведение.

**`_interpolate_tick`:**

```python
def _interpolate_tick(self):
    """Шаг интерполяции: current → target с per-joint скоростью.

    Frozen-сустав интерполируется так же, как обычный — отличие только
    в том, что set_angle вызывается с force=True, чтобы ServoDriver не
    проигнорировал команду из-за внутреннего frozen-фильтра. После
    достижения target current==target → continue, и _freeze_refresh
    в драйвере продолжает держать PWM на новой позиции.
    """
    dt = self._TICK_DT
    with self._state_lock:
        for i in range(self._num_joints):
            delta = self._target_angles[i] - self._current_angles[i]
            if delta == 0.0:
                continue
            max_step = self._max_speeds[i] * dt
            if abs(delta) <= max_step:
                self._current_angles[i] = self._target_angles[i]
            else:
                self._current_angles[i] += math.copysign(max_step, delta)
            phys = self._to_physical(i, self._current_angles[i])
            self._servos[i].set_angle(phys, force=self._servos[i].frozen)
```

### `pi_nodes/hardware/servo_driver.py`

Без изменений. `set_angle(force=True)` уже корректно обновляет `_angle` и PWM при frozen. `_freeze_refresh` использует свежий `_angle` на следующей итерации (период `FREEZE_REFRESH_INTERVAL`), гонок нет.

### `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`

**`ServoSlider` props — новый флаг `allowFrozenDrag`:**

```tsx
interface ServoSliderProps {
  label: string
  remoteValue: number
  min?: number
  max?: number
  onCommit: (v: number) => void
  disabled?: boolean
  frozen?: boolean
  onToggleFreeze?: () => void
  allowFrozenDrag?: boolean   // ← новый, default false
}
```

**`handleChange`** — frozen не блокирует, если `allowFrozenDrag`:

```tsx
const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
  if (disabled) return
  if (frozen && !allowFrozenDrag) return
  const v = Number(e.target.value)
  setLocal(v)
  onCommit(v)
}
```

**`<input>` disabled:**

```tsx
disabled={disabled || (frozen && !allowFrozenDrag)}
```

**Visual:** все остальные стили (синий фон, HOLD-badge, opacity-40 для disabled) — без изменений. HOLD остаётся видимой во время drag.

**Use sites:**

- Head slider ([ServoControlPanel.tsx:433](../../compute_node/frontend/src/components/actuators/ServoControlPanel.tsx)) — `allowFrozenDrag` не передан (default false). Поведение для головы не меняется.
- Arm slider ([ServoControlPanel.tsx:507](../../compute_node/frontend/src/components/actuators/ServoControlPanel.tsx)) — `allowFrozenDrag={true}`.

## Тесты

### Backend (`tests/test_arm_node.py` — расширить, или создать если нет)

1. **`test_frozen_joint_accepts_single_joint_command`** — заморозить ch0, отправить `arm/command {"joint": 1, "angle": 50}`, протикать `_interpolate_tick` несколько раз → `current_angles[0]` достиг 50°, `frozen[0]` остался True.
2. **`test_frozen_joint_ignores_home_command`** — заморозить ch0 на 30°, послать `home` → `target_angles[0]` остался 30° (не сменился на home), `current_angles[0]` тоже.
3. **`test_frozen_joint_ignores_load_preset`** — то же, но через `load_preset`.
4. **`test_frozen_joint_ignores_joints_array`** — то же, но через `{"joints": [10,20,30,40]}`.
5. **`test_freeze_refresh_continues_after_user_move`** — заморозить ch0, подвинуть через single-joint, убедиться что `_servos[0].frozen` всё ещё True и `_freeze_refresh` поток не отменён.
6. **`test_unfrozen_joints_unaffected_by_mass_when_one_frozen`** — заморозить ch0, послать `home` → ch1/ch2/ch3 уехали в home, ch0 остался.

### Frontend (`compute_node/frontend/src/components/actuators/ServoControlPanel.test.tsx` — создать)

1. **frozen arm slider is interactive** — отрендерить с `arm.frozen=[true,false,false,false]`, симулировать onChange на первом слайдере, проверить что `api.setArmJoint(1, X)` вызвалась.
2. **frozen head slider stays disabled** — head с `frozen=true`, onChange не должен вызывать `setHeadAngle`.
3. **HOLD badge visible during interaction** — badge остаётся видимой при drag frozen-слайдера.

## Что не делаем (out of scope)

- Head ch4 — пользователь явно сказал «только arm».
- Изменение FSM grab sequence — single-joint от UI всегда мог вмешаться в любое состояние FSM, эта работа не меняет картину.
- Изменения протокола mass-команд — `home`/preset/joints-array продолжают молча игнорировать frozen-суставы.
- Optimistic WS updates — обходимся текущим throttled REST + 10Hz `arm/state` publish.

## Риски

- **FSM grab + ручное вмешательство:** если пользователь во время grab_hold двинет frozen-слайдер клешни (ch3) — рука разожмётся и мяч выпадет. Это и сейчас возможно после unfreeze; пользователь делает это сознательно. Не митигируем.
- **Race `_freeze_refresh` vs `_interpolate_tick`:** оба пишут `self._servo.angle`. `_interpolate_tick` обновляет `_angle` перед записью PWM, `_freeze_refresh` на следующей итерации читает свежий `_angle`. Без блокировки внутри `ServoDriver` теоретически возможна короткая запись stale angle, но это не критично: следующий тик/refresh выровняет.
- **Семантический drift:** значение «frozen» теперь зависит от типа команды (mass vs single-joint). Документируем в docstring `_set_joint`, чтобы будущий контрибьютор не упустил.
