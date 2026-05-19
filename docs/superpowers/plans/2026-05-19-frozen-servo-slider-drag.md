# Drag frozen arm joint from UI slider — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Дать пользователю двигать слайдер замороженного (frozen) arm-сустава в дашборде. Mass-команды (home/preset/joints-array) при frozen продолжают игнорироваться. Head-сервопривод (ch4) не затрагивается.

**Architecture:** Бэк (`arm_node.py`): `_interpolate_tick` интерполирует frozen-суставы с `set_angle(force=True)`. `_set_joint` получает параметр `allow_frozen` — `True` только для single-joint команд от UI. Фронт (`ServoControlPanel.tsx`): `ServoSlider` принимает `allowFrozenDrag` prop, arm-слайдеры передают `true`, head — нет.

**Tech Stack:** Python 3 + pytest (бэк), React + TypeScript + vitest + @testing-library/react (фронт).

**Spec:** [docs/superpowers/specs/2026-05-19-frozen-servo-slider-drag-design.md](../specs/2026-05-19-frozen-servo-slider-drag-design.md)

---

## Task 1: Backend — `_interpolate_tick` интерполирует frozen-суставы

Удалить `if frozen: continue` из `_interpolate_tick` и передавать `force=self._servos[i].frozen` в `set_angle`. Это позволит замороженным суставам плавно ехать к target, обходя `frozen`-фильтр в драйвере. Существующий тест `test_interpolate_tick_skips_frozen_joints` нужно переписать — он проверяет старое поведение.

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py:223-248` (`_interpolate_tick`)
- Modify: `tests/test_arm_node.py:163-174` (заменить `test_interpolate_tick_skips_frozen_joints`)

- [ ] **Step 1: Заменить старый failing test на новый**

Открыть `tests/test_arm_node.py` и заменить `test_interpolate_tick_skips_frozen_joints` (строки 163-174) на следующий тест:

```python
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
```

- [ ] **Step 2: Запустить тесты — должны FAIL**

Run: `pytest tests/test_arm_node.py::test_interpolate_tick_moves_frozen_joint_to_target tests/test_arm_node.py::test_interpolate_tick_uses_force_false_for_unfrozen -v`

Expected:
- `test_interpolate_tick_moves_frozen_joint_to_target` — FAIL (`current_angles[0]` остался 0.0, потому что `if frozen: continue` пропускает; либо `set_angle` не вызывался)
- `test_interpolate_tick_uses_force_false_for_unfrozen` — FAIL (текущий код вызывает `set_angle(phys)` без kwarg `force`, KeyError или None)

- [ ] **Step 3: Изменить `_interpolate_tick` в `pi_nodes/nodes/arm_node.py:223-248`**

Заменить тело метода `_interpolate_tick` на:

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

Отличия от старого кода (строки 234-248):
- Удалена строка `if self._servos[i].frozen: continue`
- Изменена строка `self._servos[i].set_angle(phys)` → `self._servos[i].set_angle(phys, force=self._servos[i].frozen)`

- [ ] **Step 4: Запустить все тесты `test_arm_node.py` — должны PASS**

Run: `pytest tests/test_arm_node.py -v`

Expected: все тесты PASS (включая два новых).

Если упадёт `test_interpolate_tick_steps_max_speed` или другие незамороженные — проверь, что вызов `set_angle` теперь использует kwarg `force=False`, а старый тест может ожидать positional. Если так, и старый тест проверял `assert_called_with(2.4)` — он сломается из-за нового kwarg. В этом случае подправь старый assert на `assert_called_with(2.4, force=False)`. Покажи здесь все упавшие тесты — если только новые два проходят, остальные сломались только из-за kwarg, поправь их параллельно.

- [ ] **Step 5: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm): interpolate frozen joints to target with force=True

_interpolate_tick больше не пропускает frozen-суставы — они идут к target
с set_angle(force=True), обходя frozen-фильтр в драйвере. ServoDriver._freeze_refresh
продолжает держать PWM на новой позиции. Часть фичи drag frozen servo slider."
```

---

## Task 2: Backend — `_set_joint(allow_frozen)` + single-joint ветка в `_cmd_cb`

Добавить параметр `allow_frozen=False` в `_set_joint`. При frozen и `allow_frozen=False` обновление target пропускается — mass-команды (home/preset/joints-array) не двигают frozen-суставы. Single-joint ветка в `_cmd_cb` передаёт `allow_frozen=True`.

**Files:**
- Modify: `pi_nodes/nodes/arm_node.py:209-221` (`_set_joint`)
- Modify: `pi_nodes/nodes/arm_node.py:374-380` (single-joint ветка в `_cmd_cb`)
- Modify: `tests/test_arm_node.py` (добавить тесты в конец)

- [ ] **Step 1: Добавить failing tests в `tests/test_arm_node.py`**

В конец файла `tests/test_arm_node.py` добавить:

```python
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
```

- [ ] **Step 2: Запустить новые тесты — должны FAIL**

Run: `pytest tests/test_arm_node.py -v -k "allow_frozen or moves_frozen or skips_frozen"`

Expected: все 6 новых тестов FAIL — текущий `_set_joint` не имеет параметра `allow_frozen` и обновляет target для frozen-суставов.

- [ ] **Step 3: Изменить `_set_joint` в `pi_nodes/nodes/arm_node.py:209-221`**

Заменить метод `_set_joint` на:

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
        if idx < 0 or idx >= self._num_joints:
            self.log_warn('Invalid joint index: %d', idx)
            return
        if not allow_frozen and self._servos[idx].frozen:
            return
        angle = max(self._min_angles[idx], min(self._max_angles[idx], angle))
        with self._state_lock:
            self._target_angles[idx] = angle
```

- [ ] **Step 4: Изменить single-joint ветку в `_cmd_cb` `pi_nodes/nodes/arm_node.py:374-380`**

Заменить блок:

```python
        # Single joint: {"joint": 1, "angle": 90} (1-indexed)
        if 'joint' in d and 'angle' in d:
            self._unlock_if_needed()
            idx = int(d['joint']) - 1
            angle = float(d['angle'])
            self._set_joint(idx, angle)
            self.log_info('Arm joint %d → %.1f°', idx + 1, self._target_angles[idx])
            return
```

на:

```python
        # Single joint: {"joint": 1, "angle": 90} (1-indexed)
        # allow_frozen=True: single-joint команды от UI могут двигать
        # frozen-сустав (новая позиция удерживается ServoDriver._freeze_refresh).
        if 'joint' in d and 'angle' in d:
            self._unlock_if_needed()
            idx = int(d['joint']) - 1
            angle = float(d['angle'])
            self._set_joint(idx, angle, allow_frozen=True)
            self.log_info('Arm joint %d → %.1f°', idx + 1, self._target_angles[idx])
            return
```

Все другие вызовы `_set_joint(...)` (внутри `home`, `load_preset`, `joints`-array) НЕ менять — default `allow_frozen=False` даст желаемое игнорирование frozen.

- [ ] **Step 5: Запустить весь test_arm_node.py — все тесты PASS**

Run: `pytest tests/test_arm_node.py -v`

Expected: все тесты PASS (6 новых + старые).

- [ ] **Step 6: Commit**

```bash
git add pi_nodes/nodes/arm_node.py tests/test_arm_node.py
git commit -m "feat(arm): allow_frozen param разделяет single-joint vs mass команды

_set_joint(idx, angle, allow_frozen=False) — при frozen и not allow_frozen
target пропускается. Single-joint ветка в _cmd_cb передаёт allow_frozen=True,
mass-команды (home/load_preset/joints-array) — default False.

FSM grab не затрагивается: после freeze FSM не шлёт mass-команд."
```

---

## Task 3: Frontend — `ServoSlider` принимает `allowFrozenDrag` prop

Добавить prop `allowFrozenDrag?: boolean` (default `false`). Когда `true` и `frozen` — слайдер активный, `handleChange` пропускает событие.

**Files:**
- Modify: `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx:43-146` (`ServoSlider`)
- Create: `compute_node/frontend/src/components/actuators/ServoControlPanel.test.tsx`

- [ ] **Step 1: Создать failing test**

Создать новый файл `compute_node/frontend/src/components/actuators/ServoControlPanel.test.tsx` со следующим содержимым:

```tsx
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { ServoControlPanel } from './ServoControlPanel'
import type { HeadState, ArmState } from '@/types/robot'

vi.mock('@/lib/api', () => ({
  api: {
    setArmJoint: vi.fn(),
    setHeadAngle: vi.fn(),
    armCommand: vi.fn(),
    armFreezeJoint: vi.fn(),
    armUnfreezeJoint: vi.fn(),
    headCommand: vi.fn(),
    centerHead: vi.fn(),
    homeArm: vi.fn(),
    armSavePreset: vi.fn(),
    armLoadPreset: vi.fn(),
    armDeletePreset: vi.fn(),
    armListPresets: vi.fn(),
    headSavePreset: vi.fn(),
    headLoadPreset: vi.fn(),
    headDeletePreset: vi.fn(),
    headListPresets: vi.fn(),
  },
}))

import { api } from '@/lib/api'

const head: HeadState = { angle: 90, frozen: false, locked: false }
const headFrozen: HeadState = { angle: 90, frozen: true, locked: false }
const armUnlocked: ArmState = {
  j1: 0, j2: 120, j3: 0, j4: 0,
  frozen: [false, false, false, false],
  locked: false,
}
const armFirstFrozen: ArmState = {
  j1: 50, j2: 120, j3: 0, j4: 0,
  frozen: [true, false, false, false],
  locked: false,
}

describe('ServoControlPanel — frozen slider drag', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('frozen arm slider triggers setArmJoint on change', () => {
    render(<ServoControlPanel head={head} arm={armFirstFrozen} />)
    // 5 sliders: head + 4 arm. arm joint 1 — индекс 1 в querySelectorAll.
    const sliders = document.querySelectorAll('input[type="range"]')
    expect(sliders.length).toBe(5)
    const firstArmSlider = sliders[1] as HTMLInputElement
    expect(firstArmSlider.disabled).toBe(false)

    fireEvent.change(firstArmSlider, { target: { value: '75' } })
    expect(api.setArmJoint).toHaveBeenCalledWith(1, 75)
  })

  it('frozen head slider stays disabled', () => {
    render(<ServoControlPanel head={headFrozen} arm={armUnlocked} />)
    const sliders = document.querySelectorAll('input[type="range"]')
    const headSlider = sliders[0] as HTMLInputElement
    expect(headSlider.disabled).toBe(true)

    fireEvent.change(headSlider, { target: { value: '120' } })
    expect(api.setHeadAngle).not.toHaveBeenCalled()
  })

  it('HOLD badge stays visible on frozen arm slider during interaction', () => {
    render(<ServoControlPanel head={head} arm={armFirstFrozen} />)
    // HOLD badges: для каждого frozen-сустава — отдельный <span>.
    // Текст 'HOLD' появляется в ServoSlider, когда frozen=true.
    const badges = screen.getAllByText('HOLD')
    expect(badges.length).toBeGreaterThanOrEqual(1)
  })

  it('non-frozen arm slider still triggers setArmJoint (regression)', () => {
    render(<ServoControlPanel head={head} arm={armUnlocked} />)
    const sliders = document.querySelectorAll('input[type="range"]')
    const firstArmSlider = sliders[1] as HTMLInputElement
    fireEvent.change(firstArmSlider, { target: { value: '40' } })
    expect(api.setArmJoint).toHaveBeenCalledWith(1, 40)
  })
})
```

- [ ] **Step 2: Запустить тесты — должны FAIL**

Run: `cd compute_node/frontend && npx vitest run src/components/actuators/ServoControlPanel.test.tsx`

Expected:
- `frozen arm slider triggers setArmJoint on change` — FAIL: `firstArmSlider.disabled` будет `true` (текущий код блокирует) → `api.setArmJoint` не вызывается.
- `frozen head slider stays disabled` — PASS (текущее поведение совпадает).
- `HOLD badge stays visible` — PASS (badge уже отображается для frozen).
- `non-frozen arm slider still triggers setArmJoint (regression)` — PASS.

Если все 4 PASS — что-то моки не работают или начальные условия другие. Перепроверь selectors.

- [ ] **Step 3: Изменить `ServoSlider` в `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx:43-146`**

Заменить интерфейс `ServoSliderProps` и тело компонента `ServoSlider`:

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
  allowFrozenDrag?: boolean
}

function ServoSlider({
  label,
  remoteValue,
  min = 0,
  max = 180,
  onCommit,
  disabled = false,
  frozen = false,
  onToggleFreeze,
  allowFrozenDrag = false,
}: ServoSliderProps) {
  const [local, setLocal] = useState(remoteValue)
  const dragging = useRef(false)

  useEffect(() => {
    if (!dragging.current) setLocal(remoteValue)
  }, [remoteValue])

  // frozen блокирует ввод ТОЛЬКО если allowFrozenDrag=false.
  // С allowFrozenDrag=true слайдер активен, шлёт onCommit, а сустав
  // плавно едет к новой позиции, оставаясь frozen (PWM держит).
  const inputDisabled = disabled || (frozen && !allowFrozenDrag)

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (inputDisabled) return
    const v = Number(e.target.value)
    setLocal(v)
    onCommit(v)
  }

  return (
    <div className={`space-y-1 ${disabled ? 'opacity-40' : ''}`}>
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] text-muted-foreground">{label}</span>
          {frozen && (
            <span className="text-[9px] px-1 py-0.5 rounded bg-blue-900/50 text-blue-300 font-medium">
              HOLD
            </span>
          )}
        </div>
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] font-mono font-semibold tabular-nums w-10 text-right">
            {Math.round(local)}&deg;
          </span>
          {onToggleFreeze && (
            <button
              onClick={onToggleFreeze}
              className={`text-[9px] px-1.5 py-0.5 rounded transition-colors ${
                frozen
                  ? 'bg-blue-600 text-white hover:bg-blue-500'
                  : 'bg-zinc-700 text-zinc-400 hover:bg-zinc-600 hover:text-zinc-200'
              }`}
              title={frozen ? 'Разморозить' : 'Заморозить'}
            >
              {frozen ? '❄' : '🔓'}
            </button>
          )}
        </div>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        value={Math.round(local)}
        disabled={inputDisabled}
        onPointerDown={() => { dragging.current = true }}
        onPointerUp={() => { dragging.current = false }}
        onLostPointerCapture={() => { dragging.current = false }}
        onChange={handleChange}
        className={`w-full h-2 rounded-full appearance-none cursor-pointer
                   disabled:cursor-not-allowed
                   ${frozen ? 'bg-blue-900/40' : 'bg-muted'}
                   [&::-webkit-slider-thumb]:appearance-none
                   [&::-webkit-slider-thumb]:w-4
                   [&::-webkit-slider-thumb]:h-4
                   [&::-webkit-slider-thumb]:rounded-full
                   [&::-webkit-slider-thumb]:bg-primary
                   [&::-webkit-slider-thumb]:cursor-grab
                   [&::-webkit-slider-thumb]:active:cursor-grabbing
                   [&::-webkit-slider-thumb]:shadow-md
                   [&::-webkit-slider-thumb]:border-2
                   [&::-webkit-slider-thumb]:border-background
                   [&::-moz-range-thumb]:w-4
                   [&::-moz-range-thumb]:h-4
                   [&::-moz-range-thumb]:rounded-full
                   [&::-moz-range-thumb]:bg-primary
                   [&::-moz-range-thumb]:border-2
                   [&::-moz-range-thumb]:border-background
                   [&::-moz-range-thumb]:cursor-grab
                   [&::-moz-range-thumb]:active:cursor-grabbing`}
      />
      <div className="flex justify-between text-[9px] text-muted-foreground/50">
        <span>{min}&deg;</span>
        <span>{max}&deg;</span>
      </div>
    </div>
  )
}
```

Отличия от старого:
- Добавлен prop `allowFrozenDrag?: boolean` с default `false`.
- Вычислена общая переменная `inputDisabled = disabled || (frozen && !allowFrozenDrag)` — раньше в `handleChange` была одна логика (`disabled || frozen`), в `disabled` атрибуте input другая (`disabled || frozen`). Теперь обе используют `inputDisabled`.
- `handleChange` использует `inputDisabled` вместо `disabled || frozen`.
- `<input disabled>` использует `inputDisabled`.

В этом шаге arm-слайдеры ЕЩЁ не передают `allowFrozenDrag={true}` — поведение пока то же. Frontend-тесты по-прежнему упадут на `frozen arm slider triggers setArmJoint`, потому что arm-слайдеры всё ещё блокированы при frozen. Это нормально — починим в Task 4.

- [ ] **Step 4: Запустить тесты — `frozen arm slider triggers setArmJoint` ещё FAIL**

Run: `cd compute_node/frontend && npx vitest run src/components/actuators/ServoControlPanel.test.tsx`

Expected: тот же результат что и в Step 2. Тест `frozen arm slider triggers setArmJoint` всё ещё FAIL. Остальные PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/actuators/ServoControlPanel.tsx compute_node/frontend/src/components/actuators/ServoControlPanel.test.tsx
git commit -m "feat(servo-ui): ServoSlider gets allowFrozenDrag prop

Default false — frozen блокирует слайдер как и раньше. Когда true и frozen,
input активен и handleChange шлёт onCommit. Arm-слайдеры подключат флаг
следующим коммитом."
```

---

## Task 4: Frontend — arm-слайдеры передают `allowFrozenDrag={true}`

В main `ServoControlPanel` подключить флаг только для arm-слайдеров. Head — оставить без флага (default false).

**Files:**
- Modify: `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx:507-525` (`ARM_JOINTS.map`)

- [ ] **Step 1: Добавить `allowFrozenDrag={true}` к arm-слайдеру**

Открыть `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`, найти блок `{ARM_JOINTS.map((joint, i) => (` (примерно строка 507) и заменить вызов `ServoSlider` внутри `.map` на:

```tsx
          {ARM_JOINTS.map((joint, i) => (
            <ServoSlider
              key={i}
              label={`ch${i}: ${joint.label}`}
              remoteValue={armAngles[i]}
              min={joint.min}
              max={joint.max}
              onCommit={(v) => sendArm(i + 1, v)}
              disabled={armLocked}
              frozen={armFrozen[i] ?? false}
              allowFrozenDrag
              onToggleFreeze={() => {
                if (armFrozen[i]) {
                  api.armUnfreezeJoint(i + 1)
                } else {
                  api.armFreezeJoint(i + 1)
                }
              }}
            />
          ))}
```

Единственное отличие — добавлена строка `allowFrozenDrag` (boolean-shorthand для `allowFrozenDrag={true}`).

Head-слайдер (строки 433-442, выше в файле) НЕ менять — он по-прежнему без `allowFrozenDrag`.

- [ ] **Step 2: Запустить тесты — все PASS**

Run: `cd compute_node/frontend && npx vitest run src/components/actuators/ServoControlPanel.test.tsx`

Expected: все 4 теста PASS.

- [ ] **Step 3: Проверить общий typecheck**

Run: `cd compute_node/frontend && npx tsc --noEmit`

Expected: 0 ошибок (никаких type-сюрпризов от нового prop).

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/actuators/ServoControlPanel.tsx
git commit -m "feat(servo-ui): arm sliders включают allowFrozenDrag

Замороженный arm-сустав теперь двигается через слайдер: UI шлёт setArmJoint,
arm_node применяет к target (single-joint ветка с allow_frozen=True),
_interpolate_tick ведёт current→target с set_angle(force=True). PWM остаётся
активным, индикатор HOLD не мигает. Head-слайдер не затронут."
```

---

## Task 5: Final verification — frontend build + full test suite

**Files:** —

- [ ] **Step 1: Запустить полный backend test suite**

Run: `pytest tests/test_arm_node.py tests/test_mps_node.py tests/test_mps_router.py -v` (последние два — потому что они тоже в git status M, не хочется регрессии)

Expected: все PASS.

- [ ] **Step 2: Запустить полный frontend test suite**

Run: `cd compute_node/frontend && npx vitest run`

Expected: все PASS.

- [ ] **Step 3: Frontend build для production**

Run: `cd compute_node/frontend && npm run build`

Expected: build успешный, новые `.js` chunks в `compute_node/static/assets/` (как видно в git status — обычно эти артефакты обновляются после каждого build'а).

- [ ] **Step 4: Manual UI-проверка** (если железо/симулятор доступны)

Это финальная физическая проверка. Без неё не считаем фичу done на production-железе, но в worktree можно остановиться на тестах.

1. Запустить `./samurai.sh compute` (если железо доступно — `--pi <IP>`).
2. Открыть `http://localhost:5000`.
3. Перейти на страницу Hardware → Servo Panel.
4. Нажать `Разблокировать` для arm если locked.
5. Нажать `Замор. все` — все 4 arm-слайдера получают синий фон и `HOLD` badge.
6. Подвинуть слайдер ch1 — он двигается, серво (или индикатор current angle) едет к новой позиции, `HOLD` остаётся.
7. Нажать `Домой` — frozen-суставы стоят на месте, unfrozen уезжают в home.
8. Снять freeze с ch1 (нажать ❄ рядом со слайдером), нажать `Замор. все` → опять все frozen. Слайдер ch4 (head) при frozen остаётся disabled — это ожидаемо.

Если 1–8 проходят — фича работает end-to-end.

- [ ] **Step 5: Commit обновлённых build-артефактов** (если они изменились)

```bash
git add compute_node/static/
git status   # проверить что попало в staging
git commit -m "build(frontend): rebuild after frozen-servo-drag feature"
```

(Опционально — если в проекте принято коммитить build-артефакты. По git status в начале сессии видно, что MpsPage/HardwarePage/etc. chunks уже untracked — значит, традиция «коммитить static» соблюдается.)

- [ ] **Step 6: Push в ветку**

В соответствии с user preferences (`feedback_push_after_phase.md`), после завершения фазы пушим:

```bash
git push origin <current-branch>
```

(Подставить актуальную ветку. Если фича делается прямо на `main` или `dev` — push в неё.)
