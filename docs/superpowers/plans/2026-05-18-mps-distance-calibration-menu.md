# MPS — Меню калибровки расстояния D (Implementation Plan)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** На странице `/mps` появляется меню калибровки одометрии. Пользователь может ввести `scale_fwd`/`scale_bwd`/`motor_trim` руками **или** автоматически рассчитать новый scale по измеренному D_real ÷ D_target. По пути чинится backend-пайплайн `calibration/active`, который теряет коэффициенты.

**Architecture:** (1) Backend заменяет `ControlBlock.calibration_active_profile: Optional[str]` на `calibration_coeffs: Optional[dict]`, и `_h_calibration_active` сохраняет всю структуру `{profile, scale_fwd, scale_bwd, motor_trim}`. (2) Frontend расширяет существующий `CalibrationPanel` секцией «Авто-подбор по измерению» с превью «текущий → новый» и кнопкой «Подставить в FWD/BWD», которая заполняет основной input (единая точка отправки — кнопка «Применить»). (3) `MpsPage` рендерит `CalibrationPanel` в левом aside через `useRobotState()`.

**Tech Stack:** Python 3 / pytest / FastAPI / paho-mqtt (backend), React 19 + TypeScript + Vitest + @testing-library/react (frontend).

**Spec:** [docs/superpowers/specs/2026-05-18-mps-distance-calibration-menu-design.md](../specs/2026-05-18-mps-distance-calibration-menu-design.md)

---

## Phase 1 — Backend: починка пайплайна `calibration/active`

### Task 1: Добавить новое поле `calibration_coeffs` в `_ControlBlock`

**Files:**
- Modify: `compute_node/dashboard/state.py:120-134` (определение `_ControlBlock`)

- [ ] **Step 1: Открой файл и найди dataclass `_ControlBlock`**

Найди строку 132 — там сейчас `calibration_active_profile: Optional[str] = None`.

- [ ] **Step 2: Добавь рядом новое поле (старое пока НЕ удаляй)**

Вставь сразу после строки 132 строку `calibration_coeffs: Optional[dict] = None  # {profile, scale_fwd, scale_bwd, motor_trim}`. Должно стать:

```python
    calibration_status: dict = field(default_factory=dict)
    calibration_result: dict = field(default_factory=dict)
    calibration_active_profile: Optional[str] = None
    calibration_coeffs: Optional[dict] = None  # {profile, scale_fwd, scale_bwd, motor_trim}
    calibration_profiles: list = field(default_factory=list)
```

Оставляем оба поля временно — старые consumers `routers/control.py` и `snapshot()` ещё на них опираются. Удалим в задаче 6.

- [ ] **Step 3: Убедись что dataclass импортирует/использует тот же стиль**

`Optional` уже импортирован вверху файла (используется в строке 132). Никаких новых импортов не нужно.

- [ ] **Step 4: Запусти базовую проверку синтаксиса**

Run: `python -c "from compute_node.dashboard.state import DashboardState; s = DashboardState(); print(s.control.calibration_coeffs)"`
Expected: `None`

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/state.py
git commit -m "feat(dashboard): добавить calibration_coeffs в _ControlBlock (рядом со старым calibration_active_profile)"
```

---

### Task 2: Тесты + правка `_h_calibration_active` сохранять весь dict

**Files:**
- Modify: `tests/test_dashboard_mqtt_handlers.py` (добавить тесты)
- Modify: `compute_node/dashboard/mqtt_handlers.py:493-502` (handler)

- [ ] **Step 1: Открой `tests/test_dashboard_mqtt_handlers.py` и допиши тесты в конец файла**

Добавь следующий блок (вставь перед последней строкой, если файл заканчивается каким-то test_-блоком — просто допиши в конец):

```python
# ── calibration/active ─────────────────────────────────────────────────────

def test_calibration_active_saves_full_dict(handlers):
    """_h_calibration_active должен сохранять весь dict
    {profile, scale_fwd, scale_bwd, motor_trim}, а не только имя."""
    payload = json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode()
    handlers._h_calibration_active(payload)
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_incomplete_payload(handlers):
    """Неполный payload (без scale_bwd) не должен затирать существующий state."""
    # Подготовка: положим валидный state.
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    # Атака: неполный payload.
    handlers._h_calibration_active(json.dumps({
        'profile': 'broken',
        'scale_fwd': 2.0,
    }).encode())
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    # State не должен быть затёрт.
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_non_dict_payload(handlers):
    """Голый float / string / list — игнорируем, state не затираем."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    handlers._h_calibration_active(b'42.5')
    handlers._h_calibration_active(b'"justastring"')
    handlers._h_calibration_active(b'[1,2,3]')
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }


def test_calibration_active_ignores_garbage_json(handlers):
    """Битый JSON — не падаем, state не затираем."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    handlers._h_calibration_active(b'{not json')
    with handlers._state.lock:
        coeffs = handlers._state.control.calibration_coeffs
    assert coeffs is not None and coeffs['profile'] == 'tile'
```

- [ ] **Step 2: Запусти тесты — они должны упасть**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v -k calibration_active`
Expected: FAIL — `_h_calibration_active` пока сохраняет только имя в `calibration_active_profile`, поле `calibration_coeffs` остаётся None.

- [ ] **Step 3: Поправь `_h_calibration_active` в `compute_node/dashboard/mqtt_handlers.py`**

Найди существующий метод (около строки 493):

```python
    def _h_calibration_active(self, payload: bytes):
        try:
            d = json.loads(payload)
        except Exception:
            return
        with self._state.lock:
            if isinstance(d, dict):
                self._state.control.calibration_active_profile = d.get('name')
            else:
                self._state.control.calibration_active_profile = str(d)
```

Замени тело на:

```python
    def _h_calibration_active(self, payload: bytes):
        """motor_node публикует {profile, scale_fwd, scale_bwd, motor_trim}.
        Сохраняем весь dict — фронт ждёт полную структуру.
        Неполный/некорректный payload игнорируем (не затираем хороший state).
        """
        try:
            d = json.loads(payload)
        except Exception:
            return
        if not isinstance(d, dict):
            return
        required = ('profile', 'scale_fwd', 'scale_bwd', 'motor_trim')
        if not all(k in d for k in required):
            return
        try:
            new_coeffs = {
                'profile': str(d['profile']),
                'scale_fwd': float(d['scale_fwd']),
                'scale_bwd': float(d['scale_bwd']),
                'motor_trim': float(d['motor_trim']),
            }
        except (TypeError, ValueError):
            return
        with self._state.lock:
            self._state.control.calibration_coeffs = new_coeffs
            self._state.control.calibration_active_profile = new_coeffs['profile']
```

Заметь: продолжаем поддерживать legacy-поле `calibration_active_profile` (= `profile` из dict). Уберём его в задаче 6, когда все consumers переедут.

- [ ] **Step 4: Запусти тесты — должны пройти**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v -k calibration_active`
Expected: PASS (4 теста).

- [ ] **Step 5: Запусти полный тест-файл, убедись что ничего другого не сломалось**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v`
Expected: все тесты PASS.

- [ ] **Step 6: Commit**

```bash
git add tests/test_dashboard_mqtt_handlers.py compute_node/dashboard/mqtt_handlers.py
git commit -m "fix(dashboard): _h_calibration_active сохраняет весь dict {profile, scale_fwd, scale_bwd, motor_trim}, а не только имя"
```

---

### Task 3: `legacy_snapshot()` эмитит полную структуру `calibration_coeffs`

**Files:**
- Modify: `tests/test_dashboard_mqtt_handlers.py` (добавить тест эмиссии)
- Modify: `compute_node/dashboard/state.py:528`

- [ ] **Step 1: Найди метод эмиссии state и тест для него**

В `state.py` строка 528 сейчас:
```python
'calibration_coeffs': {'name': c.calibration_active_profile} if c.calibration_active_profile else None,
```

Это в методе legacy snapshot (вокруг строки 450 — `Воспроизводит формат старого DashboardNode.get_state()`). Имя метода легко найти `grep -n "def " compute_node/dashboard/state.py | head -20`.

- [ ] **Step 2: Добавь тест в `tests/test_dashboard_mqtt_handlers.py`**

В конец файла:

```python
def test_legacy_snapshot_emits_full_calibration_coeffs(handlers):
    """После _h_calibration_active legacy snapshot должен эмитить
    весь dict {profile, scale_fwd, scale_bwd, motor_trim}, а не {name: ...}."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    # Найти метод снэпшота — может называться по-разному, попробуем известные.
    state = handlers._state
    snap = None
    for name in ('get_state', 'legacy_snapshot', 'snapshot_legacy', 'as_legacy_dict'):
        fn = getattr(state, name, None)
        if callable(fn):
            snap = fn()
            break
    assert snap is not None, "legacy snapshot method not found on DashboardState"
    assert snap['calibration_coeffs'] == {
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }
```

- [ ] **Step 3: Запусти тест — упадёт**

Run: `pytest tests/test_dashboard_mqtt_handlers.py::test_legacy_snapshot_emits_full_calibration_coeffs -v`
Expected: FAIL — либо assert на `calibration_coeffs == {name: 'tile'}`, либо `legacy snapshot method not found`. Если упало с "method not found" — выясни имя через `grep -n "calibration_coeffs" compute_node/dashboard/state.py` и поправь тест (добавь это имя в кортеж).

- [ ] **Step 4: Поправь эмиссию в `state.py`**

Замени строку 528:
```python
'calibration_coeffs': {'name': c.calibration_active_profile} if c.calibration_active_profile else None,
```

на:
```python
'calibration_coeffs': dict(c.calibration_coeffs) if c.calibration_coeffs else None,
```

- [ ] **Step 5: Запусти тест — пройдёт**

Run: `pytest tests/test_dashboard_mqtt_handlers.py::test_legacy_snapshot_emits_full_calibration_coeffs -v`
Expected: PASS.

- [ ] **Step 6: Запусти весь тест-файл — убедись что ничего не сломалось**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v`
Expected: всё PASS.

- [ ] **Step 7: Commit**

```bash
git add tests/test_dashboard_mqtt_handlers.py compute_node/dashboard/state.py
git commit -m "fix(dashboard): legacy_snapshot эмитит полный calibration_coeffs (profile + scale_fwd/bwd + motor_trim)"
```

---

### Task 4: Мигрировать consumers `calibration_active_profile` → `calibration_coeffs.profile`

**Files:**
- Modify: `compute_node/dashboard/routers/control.py:225-261` (endpoints `/profile/list`, `/coefficients`)
- Modify: `compute_node/dashboard/state.py:310, 382` (snapshot() — современный, не legacy)
- Modify: `tests/test_dashboard_mqtt_handlers.py` (тесты)

- [ ] **Step 1: Найти все оставшиеся consumers**

Run: `grep -rn calibration_active_profile compute_node/`
Expected: ровно 4 хита — `state.py:132` (определение), `state.py:310` (snapshot read), `state.py:382` (snapshot emit), `routers/control.py:234`, `routers/control.py:253`. Если получилось больше — добавь дополнительные правки в Step 4/5.

- [ ] **Step 2: Добавить тесты для роутера и snapshot()**

В `tests/test_dashboard_mqtt_handlers.py` дописать:

```python
def test_snapshot_calibration_active_uses_coeffs_profile(handlers):
    """Новый snapshot() — поле control.calibration.active должно браться
    из calibration_coeffs.profile, а не из устаревшего calibration_active_profile."""
    handlers._h_calibration_active(json.dumps({
        'profile': 'tile',
        'scale_fwd': 1.5,
        'scale_bwd': 0.9,
        'motor_trim': -10.0,
    }).encode())
    state = handlers._state
    snap = None
    for name in ('snapshot', 'snapshot_state', 'get_snapshot'):
        fn = getattr(state, name, None)
        if callable(fn):
            snap = fn()
            break
    if snap is None:
        pytest.skip("snapshot() not present — skip")
    assert snap['control']['calibration']['active'] == 'tile'


def test_calibration_profile_list_endpoint_returns_active_from_coeffs(handlers):
    """Endpoint /api/calibration/profile/list возвращает active из coeffs.profile.
    Тест через CalibrationProfileListResponse: дергаем async-функцию напрямую."""
    import asyncio
    from compute_node.dashboard.routers.control import calibration_profile_list

    class _FakeMqtt:
        def publish(self, *_a, **_kw): pass

    handlers._h_calibration_active(json.dumps({
        'profile': 'carpet',
        'scale_fwd': 1.1,
        'scale_bwd': 1.0,
        'motor_trim': 0.0,
    }).encode())
    result = asyncio.run(
        calibration_profile_list(handlers._state, _FakeMqtt())
    )
    assert result.active == 'carpet'
```

- [ ] **Step 3: Запустить тесты — упадут**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v -k "snapshot_calibration_active or profile_list_endpoint"`
Expected: FAIL — snapshot() и/или эндпоинт пока ещё читают `calibration_active_profile` (которое теперь синхронизировано, но мы хотим источник истины = `calibration_coeffs`). Один из тестов может пройти если задача 2 уже синхронизирует `calibration_active_profile`; это нормально — главное чтобы тесты были.

- [ ] **Step 4: Мигрировать `state.py:310` (snapshot read)**

Найди строку 310:
```python
            cal_active = c.calibration_active_profile
```

Замени на:
```python
            cal_active = (c.calibration_coeffs or {}).get('profile')
```

- [ ] **Step 5: Мигрировать `routers/control.py:234` и `:253`**

В `compute_node/dashboard/routers/control.py` найди два места:

Строка 234 (внутри `calibration_profile_list`):
```python
        active = state.control.calibration_active_profile
```

Замени на:
```python
        active = (state.control.calibration_coeffs or {}).get('profile')
```

Строка 253 (внутри `calibration_coefficients`):
```python
        active = state.control.calibration_active_profile
```

Замени на:
```python
        active = (state.control.calibration_coeffs or {}).get('profile')
```

(А заодно упрости функцию `calibration_coefficients` — если есть `calibration_coeffs`, возвращай его целиком вместо fallback через `calibration_status`. Но это уже опционально; не делай если код становится сложнее. Минимум — Step 5 как выше.)

- [ ] **Step 6: Запусти тесты**

Run: `pytest tests/test_dashboard_mqtt_handlers.py -v -k calibration`
Expected: все PASS.

- [ ] **Step 7: Запусти полный backend-набор**

Run: `pytest tests/ -x -q`
Expected: всё PASS. Если что-то сломалось — `grep -rn calibration_active_profile` ещё раз, добей оставшиеся consumers.

- [ ] **Step 8: Commit**

```bash
git add tests/test_dashboard_mqtt_handlers.py compute_node/dashboard/state.py compute_node/dashboard/routers/control.py
git commit -m "refactor(dashboard): consumers калибровки переходят на calibration_coeffs.profile (snapshot, /profile/list, /coefficients)"
```

---

### Task 5: Удалить устаревшее поле `calibration_active_profile`

**Files:**
- Modify: `compute_node/dashboard/state.py:132` (удалить поле)
- Modify: `compute_node/dashboard/mqtt_handlers.py:_h_calibration_active` (убрать дублирующую запись)

- [ ] **Step 1: Перепроверить что consumers мигрированы**

Run: `grep -rn calibration_active_profile compute_node/ tests/`
Expected: только строка 132 в `state.py` (определение) и обращение в `mqtt_handlers.py` (Step 3 уберём). Если хитов больше — мигрируй их сначала.

- [ ] **Step 2: Удалить поле из dataclass**

В `compute_node/dashboard/state.py:132` удали строку:
```python
    calibration_active_profile: Optional[str] = None
```

- [ ] **Step 3: Убрать дублирующую запись в `_h_calibration_active`**

В `compute_node/dashboard/mqtt_handlers.py` найди строку (внутри `_h_calibration_active`):
```python
            self._state.control.calibration_active_profile = new_coeffs['profile']
```
И удали её.

- [ ] **Step 4: Запусти полный набор тестов**

Run: `pytest tests/ -x -q`
Expected: всё PASS. Если что-то сломалось (AttributeError) — значит остался consumer, добей его через `grep`.

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/state.py compute_node/dashboard/mqtt_handlers.py
git commit -m "refactor(dashboard): удалить устаревшее поле calibration_active_profile (источник истины — calibration_coeffs)"
```

---

## Phase 2 — Frontend: helper `computeNewScale` + калькулятор в `CalibrationPanel`

### Task 6: Helper `computeNewScale` + unit-тесты

**Files:**
- Create: `compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx`
- Modify: `compute_node/frontend/src/components/controls/CalibrationPanel.tsx` (добавить export helper)

- [ ] **Step 1: Создать тест-файл с тестами на helper**

Создай `compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx`:

```tsx
import { describe, it, expect } from 'vitest'
import { computeNewScale } from './CalibrationPanel'

describe('computeNewScale', () => {
  it('масштабирует scale пропорционально измеренному', () => {
    const result = computeNewScale(1.235, 2.0, 2.18)
    expect(result).not.toBeNull()
    expect(result!).toBeCloseTo(1.3461, 4)
  })

  it('возвращает null при D_target = 0', () => {
    expect(computeNewScale(1.235, 0, 2.18)).toBeNull()
  })

  it('возвращает null при D_measured = 0', () => {
    expect(computeNewScale(1.235, 2.0, 0)).toBeNull()
  })

  it('возвращает null при отрицательном D_target', () => {
    expect(computeNewScale(1.235, -2.0, 2.18)).toBeNull()
  })

  it('возвращает null при NaN', () => {
    expect(computeNewScale(1.235, NaN, 2.18)).toBeNull()
    expect(computeNewScale(1.235, 2.0, NaN)).toBeNull()
    expect(computeNewScale(NaN, 2.0, 2.18)).toBeNull()
  })

  it('возвращает null при Infinity', () => {
    expect(computeNewScale(1.235, Infinity, 2.18)).toBeNull()
    expect(computeNewScale(1.235, 2.0, Infinity)).toBeNull()
  })

  it('точно повторяет старый scale при равных D', () => {
    expect(computeNewScale(1.235, 2.0, 2.0)).toBeCloseTo(1.235, 6)
  })
})
```

- [ ] **Step 2: Запусти тест — упадёт (модуль не экспортирует computeNewScale)**

Run: `cd compute_node/frontend && npm run test -- CalibrationPanel`
Expected: FAIL — `computeNewScale is not a function` или ошибка импорта.

- [ ] **Step 3: Добавить helper в `CalibrationPanel.tsx`**

Открой `compute_node/frontend/src/components/controls/CalibrationPanel.tsx`. В верхней части файла, **после импортов** и **перед интерфейсами**, добавь:

```ts
/**
 * Пересчёт scale-коэффициента по измеренному расстоянию.
 *
 * motor_node применяет v_target = lin × scale_fwd перед интегрированием
 * в одометрию. Если робот при D_target = 2.0 проехал реально 2.18 — текущий
 * scale недосчитывает, новый = old × (real/target).
 *
 * Возвращает null если ввод некорректен (≤ 0, NaN, Infinity).
 */
export function computeNewScale(
  oldScale: number,
  dTarget: number,
  dMeasured: number,
): number | null {
  if (
    !Number.isFinite(oldScale) ||
    !Number.isFinite(dTarget) ||
    !Number.isFinite(dMeasured) ||
    dTarget <= 0 ||
    dMeasured <= 0
  ) {
    return null
  }
  return oldScale * (dMeasured / dTarget)
}
```

- [ ] **Step 4: Запусти тесты — должны пройти**

Run: `cd compute_node/frontend && npm run test -- CalibrationPanel`
Expected: PASS (7 тестов в describe `computeNewScale`).

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/controls/CalibrationPanel.tsx compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx
git commit -m "feat(calibration): helper computeNewScale(oldScale, dTarget, dMeasured) — пересчёт коэффициента по измерению"
```

---

### Task 7: Секция «Авто-подбор по измерению» в `CalibrationPanel`

**Files:**
- Modify: `compute_node/frontend/src/components/controls/CalibrationPanel.tsx`
- Modify: `compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx`

- [ ] **Step 1: Добавить тесты UI калькулятора**

В `CalibrationPanel.test.tsx` после блока `describe('computeNewScale', ...)` добавь:

```tsx
import { render, screen, fireEvent } from '@testing-library/react'
import { vi, beforeEach } from 'vitest'
import { CalibrationPanel } from './CalibrationPanel'

// Мокаем '@/lib/api' — нам нужно проверить setCalibration call.
vi.mock('@/lib/api', () => ({
  api: {
    setCalibration: vi.fn(),
    listCalibrationProfiles: vi.fn(),
    saveCalibrationProfile: vi.fn(),
    loadCalibrationProfile: vi.fn(),
    deleteCalibrationProfile: vi.fn(),
  },
}))

import { api } from '@/lib/api'

const coeffs = {
  profile: 'default',
  scale_fwd: 1.235,
  scale_bwd: 0.988,
  motor_trim: -12.003,
}

describe('CalibrationPanel — калькулятор', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('рендерит подсказку "только в режиме Robot"', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    expect(screen.getByText(/Применяется только в режиме Robot/i)).toBeInTheDocument()
  })

  it('калькулятор показывает превью при валидном вводе', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    // Превью отображает текущий 1.235 → новый ~1.346
    expect(screen.getByTestId('calc-new-scale')).toHaveTextContent(/1\.346/)
    expect(screen.getByTestId('calc-current-scale')).toHaveTextContent(/1\.235/)
  })

  it('калькулятор подставляет в FWD при direction=fwd', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в FWD/i }))
    const fwdInput = screen.getByLabelText('FWD') as HTMLInputElement
    expect(parseFloat(fwdInput.value)).toBeCloseTo(1.3461, 3)
  })

  it('калькулятор подставляет в BWD при direction=bwd', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.click(screen.getByLabelText(/Назад/i))
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '1.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '0.9' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в BWD/i }))
    const bwdInput = screen.getByLabelText('BWD') as HTMLInputElement
    expect(parseFloat(bwdInput.value)).toBeCloseTo(0.8892, 3)
  })

  it('кнопка Подставить disabled при невалидном вводе', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    const btn = screen.getByRole('button', { name: /Подставить в FWD/i })
    expect(btn).toBeDisabled()
  })

  it('после Подставить → Применить вызывает api.setCalibration с правильными аргументами', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в FWD/i }))
    fireEvent.click(screen.getByRole('button', { name: /Применить/i }))
    expect(api.setCalibration).toHaveBeenCalledTimes(1)
    const call = (api.setCalibration as ReturnType<typeof vi.fn>).mock.calls[0]
    expect(call[0]).toBeCloseTo(1.3461, 3)        // fwd
    expect(call[1]).toBeCloseTo(0.988, 3)         // bwd (не менялся)
    expect(call[2]).toBeCloseTo(-12.003, 3)       // trim (не менялся)
  })
})
```

- [ ] **Step 2: Запусти тесты — должны упасть**

Run: `cd compute_node/frontend && npm run test -- CalibrationPanel`
Expected: FAIL — нет калькулятора, нет лейблов, нет testid'ов.

- [ ] **Step 3: Расширить `CalibrationPanel.tsx` — добавить state калькулятора**

В `CalibrationPanel.tsx`, найди существующий блок useState (около строки 30):

```ts
  const [fwd, setFwd] = useState('')
  const [bwd, setBwd] = useState('')
  const [trim, setTrim] = useState('')
  const [saveName, setSaveName] = useState('')
  const [saveDesc, setSaveDesc] = useState('')
  const [editing, setEditing] = useState(false)
  const [showSave, setShowSave] = useState(false)
```

Добавь под ним ещё три:

```ts
  // ── Калькулятор (авто-подбор по измерению) ──────────────
  const [calcDir, setCalcDir] = useState<'fwd' | 'bwd'>('fwd')
  const [dTarget, setDTarget] = useState('')
  const [dMeasured, setDMeasured] = useState('')
```

И импорт `useMemo`:

В верхней строке импорта:
```ts
import { useState, useEffect } from 'react'
```
замени на:
```ts
import { useState, useEffect, useMemo } from 'react'
```

- [ ] **Step 4: Вычислить currentScale и newScale через useMemo**

В компоненте `CalibrationPanel`, после блока useEffect-ов (около строки 50, где `useEffect(() => { api.listCalibrationProfiles() }, [])`), добавь:

```ts
  const currentScale = calcDir === 'fwd' ? coeffs?.scale_fwd : coeffs?.scale_bwd

  const newScale = useMemo(() => {
    if (currentScale == null) return null
    const t = parseFloat(dTarget)
    const m = parseFloat(dMeasured)
    return computeNewScale(currentScale, t, m)
  }, [currentScale, dTarget, dMeasured])

  function handleApplyCalculator() {
    if (newScale == null) return
    if (calcDir === 'fwd') {
      setFwd(newScale.toFixed(4))
    } else {
      setBwd(newScale.toFixed(4))
    }
    setEditing(true)
  }
```

- [ ] **Step 5: Добавить подсказку про режим Robot**

Найди блок `<CardHeader>...<CardTitle>Калибровка колёс</CardTitle></CardHeader>` (строки 89-94). Сразу **после** `</CardHeader>` (и до `<CardContent>`) ничего не вставляем. Внутри `<CardContent>` после первой строки добавь подсказку — найди:

```tsx
      <CardContent className="p-3 space-y-2.5">
        {/* Active profile indicator */}
```

Перед `{/* Active profile indicator */}` вставь:

```tsx
        <div className="text-[10px] text-zinc-500 leading-tight">
          Применяется только в режиме Robot
        </div>
```

- [ ] **Step 6: Добавить aria-label к существующим FWD/BWD/TRIM input'ам**

Тесты ищут input'ы по `getByLabelText('FWD')` и `getByLabelText('BWD')`. Найди три блока input для FWD/BWD/TRIM (строки 116-141). К каждому `<input ... />` добавь `aria-label`:

```tsx
            <input
              className={inputCls}
              value={fwd}
              onChange={e => { setFwd(e.target.value); setEditing(true) }}
              placeholder="1.235"
              aria-label="FWD"
            />
```

Аналогично для BWD и TRIM (`aria-label="BWD"`, `aria-label="TRIM %"`).

- [ ] **Step 7: Добавить секцию калькулятора**

Найди в JSX блок «Apply / Save buttons» (около строки 144-163, начинается с `{/* Apply / Save buttons */}`). **Перед ним** вставь секцию калькулятора:

```tsx
        <Separator />

        {/* Авто-подбор по измерению */}
        <div className="space-y-1.5">
          <div className="text-[10px] uppercase text-muted-foreground tracking-wider">
            Авто-подбор по измерению
          </div>

          {/* Direction toggle */}
          <div className="flex gap-3 text-xs">
            <label className="flex items-center gap-1 cursor-pointer">
              <input
                type="radio"
                name="calc-dir"
                checked={calcDir === 'fwd'}
                onChange={() => setCalcDir('fwd')}
                aria-label="Вперёд"
              />
              <span>Вперёд</span>
            </label>
            <label className="flex items-center gap-1 cursor-pointer">
              <input
                type="radio"
                name="calc-dir"
                checked={calcDir === 'bwd'}
                onChange={() => setCalcDir('bwd')}
                aria-label="Назад"
              />
              <span>Назад</span>
            </label>
          </div>

          {/* D inputs */}
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-zinc-500 w-24 shrink-0">D заданное, м</span>
            <input
              className={inputCls}
              type="number"
              step="0.01"
              min="0"
              value={dTarget}
              onChange={e => setDTarget(e.target.value)}
              placeholder="2.00"
              aria-label="D заданное, м"
            />
          </div>
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-zinc-500 w-24 shrink-0">D измеренное, м</span>
            <input
              className={inputCls}
              type="number"
              step="0.01"
              min="0"
              value={dMeasured}
              onChange={e => setDMeasured(e.target.value)}
              placeholder="2.18"
              aria-label="D измеренное, м"
            />
          </div>

          {/* Preview + button */}
          <div className="text-[10px] text-zinc-400 font-mono">
            {currentScale == null ? (
              <span className="text-zinc-600">Жду коэффициентов из robot…</span>
            ) : newScale == null ? (
              <span className="text-zinc-600">Введи D &gt; 0</span>
            ) : (
              <>
                <span>Текущий {calcDir.toUpperCase()}: </span>
                <span data-testid="calc-current-scale">{currentScale.toFixed(3)}</span>
                <span> → Новый: </span>
                <span data-testid="calc-new-scale">{newScale.toFixed(3)}</span>
                <span className={newScale >= currentScale ? 'text-emerald-400' : 'text-amber-400'}>
                  {'  (Δ '}
                  {(newScale - currentScale >= 0 ? '+' : '') + (newScale - currentScale).toFixed(3)}
                  {')'}
                </span>
              </>
            )}
          </div>
          <Button
            size="sm"
            variant="outline"
            className="text-xs h-7 w-full"
            onClick={handleApplyCalculator}
            disabled={newScale == null}
          >
            Подставить в {calcDir.toUpperCase()}
          </Button>
        </div>

        <Separator />
```

Заметь: существующий одиночный `<Separator />` перед `{/* Profile list */}` (около строки 191) **оставляем** — он логично разделяет «apply/save+dialog» и «profile list». В сумме три separator-а: перед калькулятором / после калькулятора / перед списком профилей.

- [ ] **Step 8: Запусти тесты — должны пройти**

Run: `cd compute_node/frontend && npm run test -- CalibrationPanel`
Expected: PASS (`computeNewScale` блок + 6 тестов в `CalibrationPanel — калькулятор`).

Если падают на поиске по тексту — проверь что aria-label-ы и testid-ы написаны точно как в тестах.

- [ ] **Step 9: Прогнать lint и type-check**

Run: `cd compute_node/frontend && npm run lint`
Expected: чисто (или только pre-existing warnings).

Run: `cd compute_node/frontend && npx tsc --noEmit`
Expected: 0 ошибок типов.

- [ ] **Step 10: Commit**

```bash
git add compute_node/frontend/src/components/controls/CalibrationPanel.tsx compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx
git commit -m "feat(calibration): секция авто-подбора по измерению + подсказка про Robot-режим в CalibrationPanel"
```

---

## Phase 3 — Размещение на странице `/mps`

### Task 8: Подключить `CalibrationPanel` в `MpsPage`

**Files:**
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx`

- [ ] **Step 1: Добавить импорты в `MpsPage.tsx`**

В верхнюю секцию импортов (около строки 1-29) добавь:

```ts
import { CalibrationPanel } from '@/components/controls/CalibrationPanel'
import { useRobotState } from '@/hooks/useRobotState'
```

- [ ] **Step 2: Достать robotState внутри `MpsPageInner`**

В функции `MpsPageInner()` после блока с другими хуками (после `const historyHook = useMpsHistory()`, около строки 44), добавь:

```ts
  const robotState = useRobotState()
```

- [ ] **Step 3: Отрендерить `CalibrationPanel` в aside**

Найди левый aside (около строки 232):

```tsx
          <aside className="space-y-3 lg:sticky lg:top-16 lg:self-start lg:max-h-[calc(100vh-5rem)] lg:overflow-y-auto">
            <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
            <PhysicsParams
              applied={matricesHook.applied}
              draft={matricesHook.draft}
              onPatch={(m) => void matricesHook.saveDraft(m)}
              defaults={{ tau_v: DEFAULT_TAU_V, tau_omega: DEFAULT_TAU_OMEGA }}
            />
          </aside>
```

Добавь `CalibrationPanel` после `<PhysicsParams ... />` (но **до** закрывающего `</aside>`):

```tsx
            <CalibrationPanel
              coeffs={robotState?.calibration_coeffs ?? null}
              profiles={robotState?.calibration_profiles ?? null}
            />
```

- [ ] **Step 4: Прогнать type-check**

Run: `cd compute_node/frontend && npx tsc --noEmit`
Expected: 0 ошибок.

- [ ] **Step 5: Прогнать lint**

Run: `cd compute_node/frontend && npm run lint`
Expected: чисто.

- [ ] **Step 6: Прогнать ВСЕ фронт-тесты**

Run: `cd compute_node/frontend && npm run test`
Expected: всё PASS.

- [ ] **Step 7: Commit**

```bash
git add compute_node/frontend/src/pages/MpsPage.tsx
git commit -m "feat(mps): CalibrationPanel в левом sidebar /mps — ниже PhysicsParams"
```

---

## Phase 4 — Сборка и manual smoke

### Task 9: Production-build фронта и dev-сборка

**Files:** (только сборка, без правок исходников)

- [ ] **Step 1: Сделать production build**

Run: `cd compute_node/frontend && npm run build`
Expected: 0 ошибок, новые файлы в `compute_node/static/assets/`.

- [ ] **Step 2: Проверить что попало в static/**

Run: `git status compute_node/static/`
Ожидаем новые `MpsPage-<hash>.js`, `AdminPage-<hash>.js`, `index.html` обновлён. Старые hashed-файлы из git status (которые ?? в начале) — это уже накопившиеся; не путаемся.

- [ ] **Step 3: Commit билдов (если в проекте так принято)**

Посмотри `git log --oneline -10 compute_node/static/` — если предыдущие коммиты включают `compute_node/static/`, добавляй и коммить:

```bash
git add compute_node/static/
git commit -m "build(mps): фронт-bundle с CalibrationPanel на /mps"
```

Если коммиты статики делаются отдельно или их добавляет CI — пропусти этот шаг.

---

### Task 10: Manual smoke на хосте

Этот шаг **не автоматизируется** тестами — нужно поднять стек и убедиться вживую.

- [ ] **Step 1: Поднять compute стек**

Run: `./samurai.sh compute` (или то, что обычно используешь). Дождись health-чека MQTT и FastAPI на `:5000`.

- [ ] **Step 2: Открыть `http://localhost:5000/mps`**

Ожидаем:
- В левом sidebar появилась карточка «Калибровка колёс» ниже «Параметры физики».
- Под заголовком тонкая строчка «Применяется только в режиме Robot».
- Если робот подключён к MQTT — поля FWD/BWD/TRIM показывают **реальные** числа (1.235/0.988/-12.003 либо то, что записано в активном профиле). Это проверка, что фикс пайплайна работает: до фикса поля были пустыми.

- [ ] **Step 3: Проверить ручной ввод**

Поменяй FWD на `1.300`, нажми «Применить». Через ~1 секунду в той же карточке должно отобразиться `1.300` (motor_node отправит retained `calibration/active` обратно). Верни на `1.235` и снова «Применить».

- [ ] **Step 4: Проверить калькулятор**

Введи `D заданное = 2.0`, `D измеренное = 2.18`. Жди превью: `Текущий FWD: 1.235 → Новый: 1.346 (Δ +0.111)`. Нажми «Подставить в FWD» — поле FWD заполнится `1.3461` (4 знака). Нажми «Применить». Через ~1 сек FWD в карточке должен показывать новое значение.

- [ ] **Step 5: Откатить калибровку**

Верни значение `1.235` в FWD (или загрузи профиль `default` из списка). «Применить».

- [ ] **Step 6: (Опционально) Физическая верификация на роботе**

Если есть доступ к железу:
1. Включи робота, дождись `motor_node` ready.
2. Запусти MPS-сценарий `D=2.0, source=robot`.
3. Замерь рулеткой реальную пройденную дистанцию.
4. Если ≠ 2.0 — внеси замеренное в калькулятор → «Подставить» → «Применить».
5. Запусти ещё раз — должен остановиться ближе к 2.0.
6. Если результат стабилен — «Сохранить как...» новый профиль с описанием поверхности.

⚠ Если железа сейчас нет — пометь этот шаг как **отложен на физический тест** (см. `memory/physical_tests_pending.md`).

---

## Self-Review Checklist

После завершения всех задач:

- [ ] Все pytest-тесты проходят: `pytest tests/ -x -q`.
- [ ] Все vitest-тесты проходят: `cd compute_node/frontend && npm run test`.
- [ ] Type-check чистый: `cd compute_node/frontend && npx tsc --noEmit`.
- [ ] Lint чистый: `cd compute_node/frontend && npm run lint`.
- [ ] `grep -rn calibration_active_profile compute_node/ tests/` — 0 хитов.
- [ ] На `/mps` калибровочная карточка видна и заполнена реальными числами.
- [ ] Калькулятор пересчитывает и подставляет в нужное поле (FWD/BWD).
- [ ] Кнопка «Применить» отправляет правильные коэффициенты в motor_node.
