# MPS — Меню калибровки расстояния D

> Меню калибровки одометрии на странице `/mps`: позволяет ввести
> поправочные коэффициенты вручную **или** автоматически
> рассчитать новый `scale_fwd` / `scale_bwd` по измеренному
> расхождению между заданным D и реально пройденным расстоянием.
>
> **Скоуп:** починка существующего пайплайна `calibration/active`
> на бэкенде; расширение существующего компонента
> `CalibrationPanel` секцией «Авто-подбор по измерению»;
> размещение `CalibrationPanel` на странице `/mps` в левом aside.

## 1. Контекст

### Проблема

При запуске MPS-сценария с заданным `D` (например `D = 2.0 м`)
робот в реальности останавливается на другом расстоянии (например
2.18 м). Контроллер закрывает петлю по одометрии, поэтому ошибка
вызвана рассогласованием между одометрией и реальностью — то есть
неточностью масштаба колёсной одометрии `scale_fwd` / `scale_bwd`.

Пользователю нужно меню, в котором можно:
1. Ввести коэффициенты вручную (если знаешь нужное число).
2. Автоматически пересчитать коэффициент по измерению —
   «задал 2.0, проехал 2.18» → система сама посчитает новый
   `scale`.

### Уже есть

- **`pi_nodes/nodes/motor_node.py`** — полная инфраструктура:
  - применяет `v_target = lin × scale_fwd` (для `lin ≥ 0`) или
    `× scale_bwd` (для `lin < 0`) перед интегрированием в
    одометрию (`motor_node.py:720`);
  - подписан на `calibration/set` → принимает
    `{scale_fwd, scale_bwd, motor_trim}`;
  - подписан на `calibration/profile/load|save|delete|list`;
  - публикует `calibration/active` с `{profile, scale_fwd,
    scale_bwd, motor_trim}` (retained).
- **`config.yaml:256-261`** — `wheel_calibration.scale_linear_fwd`,
  `scale_linear_bwd`, `motor_trim_pct`, `scale_angular`.
- **`calibration_profiles.yaml`** — профили по поверхностям.
- **`compute_node/frontend/src/components/controls/CalibrationPanel.tsx`** —
  компонент с ручным вводом FWD/BWD/TRIM + список профилей. Сейчас
  используется только на `AdminPage` (`pages/AdminPage.tsx:185`).
- **`compute_node/frontend/src/lib/api.ts`** — методы
  `api.setCalibration(scale_fwd, scale_bwd, motor_trim)`,
  `api.loadCalibrationProfile`, `api.saveCalibrationProfile`,
  `api.listCalibrationProfiles`.
- **`compute_node/frontend/src/types/robot.ts:247-261`** — тип уже
  объявлен как `{profile, scale_fwd, scale_bwd, motor_trim}`.

### Что сломано

Backend-пайплайн `calibration/active` теряет коэффициенты:

- `compute_node/dashboard/mqtt_handlers.py:493-502`
  `_h_calibration_active` сохраняет в state только `name`
  (`d.get('name')`), отбрасывая `scale_fwd` / `scale_bwd` /
  `motor_trim`.
- `compute_node/dashboard/state.py:528` эмитит
  `'calibration_coeffs': {'name': c.calibration_active_profile}`
  — фронт получает только имя.

В итоге существующий `CalibrationPanel` на AdminPage всегда
стартует с пустыми полями FWD/BWD/TRIM, потому что
`coeffs.scale_fwd/bwd/motor_trim` всегда `undefined`. До любого
расширения этот пайплайн нужно починить, иначе калькулятор не
сможет показать «текущий → новый».

## 2. Backend: починка пайплайна `calibration/active`

### 2.1 `compute_node/dashboard/state.py`

```python
# Было:
calibration_active_profile: Optional[str] = None

# Станет:
calibration_coeffs: Optional[dict] = None  # {profile, scale_fwd, scale_bwd, motor_trim}
```

Если есть код, который читает `calibration_active_profile` —
заменить на `calibration_coeffs['profile'] if calibration_coeffs
else None`. Старое поле удалить (узкий внутренний интерфейс,
рефакторим за один присест).

В эмиссии (`state.py:528`):
```python
'calibration_coeffs': (
    dict(c.calibration_coeffs) if c.calibration_coeffs else None
),
```

### 2.2 `compute_node/dashboard/mqtt_handlers.py`

`_h_calibration_active`:
```python
def _h_calibration_active(self, payload: bytes):
    try:
        d = json.loads(payload)
    except Exception:
        return
    if not isinstance(d, dict):
        return
    required = ('profile', 'scale_fwd', 'scale_bwd', 'motor_trim')
    if not all(k in d for k in required):
        return  # игнорируем неполные payload — не затираем хороший state
    with self._state.lock:
        self._state.control.calibration_coeffs = {
            'profile': str(d['profile']),
            'scale_fwd': float(d['scale_fwd']),
            'scale_bwd': float(d['scale_bwd']),
            'motor_trim': float(d['motor_trim']),
        }
```

### 2.3 Возможные смежные правки

- `state.py:528` (см. выше).
- Поиск других мест чтения `calibration_active_profile`
  (`grep -rn calibration_active_profile compute_node/`) — если
  найдутся, заменить.

## 3. Frontend: `CalibrationPanel` + калькулятор

### 3.1 Новая секция «Авто-подбор по измерению»

Файл: `compute_node/frontend/src/components/controls/CalibrationPanel.tsx`.
Секция добавляется ниже существующих FWD/BWD/TRIM-инпутов,
перед кнопками «Применить / Сохранить как...».

Визуально:

```
─── Авто-подбор по измерению ─────────
Направление:  [• Вперёд] [ Назад ]

D заданное, м:   [ 2.00  ]
D измеренное, м: [ 2.18  ]

Текущий FWD: 1.235 → Новый: 1.346  (Δ +0.111)
[ Подставить в FWD ]
─────────────────────────────────────
```

Лейбл кнопки динамический: `Подставить в FWD` или `Подставить в BWD`
в зависимости от выбранного направления. Превью «Текущий X → Новый Y»
тоже показывает соответствующее поле (FWD/BWD).

### 3.2 Формула

```ts
function computeNewScale(
  oldScale: number,
  dTarget: number,
  dMeasured: number,
): number | null {
  if (
    !isFinite(oldScale) || !isFinite(dTarget) || !isFinite(dMeasured)
    || dTarget <= 0 || dMeasured <= 0
  ) {
    return null
  }
  return oldScale * (dMeasured / dTarget)
}
```

**Обоснование:** `motor_node` применяет
`v_target_for_odom = lin_cmd × scale_fwd`. Если реальное
расстояние = 2.18 при `D = 2.0`, текущий `scale` недосчитывает —
новый `scale = old × (real / target)` приводит одометрию в
соответствие с реальностью. Контроллер MPS, замыкающий петлю
по этой одометрии, теперь остановится у настоящих 2.0 м.

Для `Назад` формула та же — берём `oldScale = coeffs.scale_bwd`,
обновляем `scale_bwd`.

### 3.3 Локальное состояние компонента

Дополнить существующий useState:
```ts
const [calcDir, setCalcDir] = useState<'fwd' | 'bwd'>('fwd')
const [dTarget, setDTarget] = useState('')     // строка, не number — для пустого поля
const [dMeasured, setDMeasured] = useState('')
```

### 3.4 Превью и кнопка «Подставить»

```ts
const currentScale = calcDir === 'fwd'
  ? coeffs?.scale_fwd
  : coeffs?.scale_bwd

const newScale = useMemo(() => {
  const t = parseFloat(dTarget)
  const m = parseFloat(dMeasured)
  if (currentScale == null) return null
  return computeNewScale(currentScale, t, m)
}, [currentScale, dTarget, dMeasured])

function handleApplyCalculator() {
  if (newScale == null) return
  if (calcDir === 'fwd') {
    setFwd(newScale.toFixed(4))
  } else {
    setBwd(newScale.toFixed(4))
  }
  setEditing(true)  // активируем основную кнопку «Применить»
}
```

- Превью «Текущий X → Новый Y (Δ ±Z)» рендерится только когда
  `newScale != null`. В невалидном состоянии — серая placeholder-
  строчка «Введи D > 0».
- Кнопка `Подставить` `disabled` если `newScale == null`.
- Кнопка **не отправляет** калибровку напрямую — только
  заполняет инпут FWD или BWD. Пользователь видит итоговое
  число и подтверждает обычной кнопкой «Применить». Единая
  точка отправки `api.setCalibration` упрощает state-машину.

### 3.5 Подсказка про режим Robot

Под заголовком карточки тонкая серая строчка:
```
Применяется только в режиме Robot
```
Чтобы пользователь не удивлялся, что коэффициенты не меняют
поведение симуляции (sim использует идеальную динамику).

## 4. Размещение на `/mps`

Файл: `compute_node/frontend/src/pages/MpsPage.tsx`.

### 4.1 Источник данных

```ts
import { useRobotState } from '@/hooks/useRobotState'

// внутри MpsPageInner:
const robotState = useRobotState()
```

### 4.2 JSX в левом aside

```tsx
<aside className="space-y-3 lg:sticky lg:top-16 ...">
  <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
  <PhysicsParams ... />
  <CalibrationPanel
    coeffs={robotState?.calibration_coeffs ?? null}
    profiles={robotState?.calibration_profiles ?? null}
  />
</aside>
```

`CalibrationPanel` идёт **ниже `PhysicsParams`** — концептуально
это тоже «статическая конфигурация» (физика робота), поэтому
группируется с моделью и физпараметрами.

Aside sticky → калибровка всегда видна на десктопе, не мешает
основной работе с MatrixEditor / ScenarioControls.

## 5. Поток данных (end-to-end)

```
motor_node ─publish─▶ samurai/{id}/calibration/active
                       {profile, scale_fwd, scale_bwd, motor_trim}
                            │
                            ▼
mqtt_handlers._h_calibration_active  (FIX: сохранить весь dict)
                            │
                            ▼
state.control.calibration_coeffs  (FIX: dict вместо строки)
                            │
                            ▼
state.py emit  →  WS  →  useRobotState  →  CalibrationPanel.props.coeffs
                                                       │
                                       ┌───────────────┴──────────────┐
                                       ▼                              ▼
                              ручной ввод FWD/BWD/TRIM       D_target / D_real
                                       │                              │
                                       └──── input fwd/bwd ───────────┘
                                                       │
                                                       ▼
                                       Кнопка «Применить»
                                                       │
                                                       ▼
                              api.setCalibration(fwd, bwd, trim)
                                                       │
                                                       ▼
                              POST /api/calibration/set
                                                       │
                                                       ▼
                              MQTT publish calibration/set
                                                       │
                                                       ▼
                              motor_node._cal_set_cb обновляет _scale_fwd/...
                                                       │
                                                       ▼
                              motor_node publish calibration/active (retained)
                              → круг замыкается, UI видит обновлённые числа
```

## 6. Тесты

### 6.1 Backend

| Тест | Файл | Что проверяет |
|---|---|---|
| `_h_calibration_active` сохраняет всю структуру | `tests/test_mqtt_handlers.py` | После приёма `{profile, scale_fwd, scale_bwd, motor_trim}` в `state.control.calibration_coeffs` лежит весь dict. |
| Устойчивость к битому payload | Тот же | Не-dict / неполный payload → state не меняется (старое значение сохраняется). |
| Эмиссия `calibration_coeffs` в WS-стейт | Тот же (или `test_dashboard_state.py`) | После заполнения state JSON-эмиссия содержит то же. |

### 6.2 Frontend

| Тест | Файл | Что проверяет |
|---|---|---|
| `computeNewScale` математика | `compute_node/frontend/src/components/controls/CalibrationPanel.test.tsx` | `(1.235, 2.0, 2.18) ≈ 1.3461`; `dTarget=0` → null; `dMeasured<0` → null; `NaN` → null. |
| Калькулятор → FWD | Тот же | Заполнить D_target=2.0, D_real=2.18, direction='fwd', кликнуть «Подставить в FWD» → input fwd обновлён, основная кнопка «Применить» активна. |
| Калькулятор → BWD | Тот же | direction='bwd' → обновляется input bwd, не fwd. |
| «Применить» вызывает `api.setCalibration` | Тот же | Мок `api.setCalibration` вызван с верными аргументами после комбо калькулятор→Применить. |
| Превью «Δ» рендерится | Тот же | Валидный ввод → видно `1.235 → 1.346 (Δ +0.111)`; невалидный → placeholder. |
| Подсказка «только Robot» | Тот же | В DOM есть строка `Применяется только в режиме Robot`. |

### 6.3 Не пишем тесты

- Размещение `CalibrationPanel` в aside `MpsPage` (визуальный лейаут).
- Композиция `useRobotState` в `MpsPageInner` (тривиальная).

### 6.4 Manual smoke (после имплементации)

1. Поднять compute-стек, открыть `/mps`.
2. Убедиться, что в `CalibrationPanel` слева отображаются реальные
   значения `scale_fwd` / `scale_bwd` / `motor_trim` (не пустые).
3. На железе запустить MPS-сценарий `D=2.0, source=robot`, замерить
   реальное расстояние линейкой/рулеткой.
4. Открыть калькулятор, ввести `D_target=2.0` и измеренное число,
   кликнуть «Подставить в FWD», затем «Применить».
5. Повторить сценарий — реальная дистанция должна стать
   ближе к 2.0.
6. Сохранить как новый профиль через «Сохранить как...».

## 7. Out of scope

- **Угловой scale / TRIM авто-калькулятор.** Пользователь жалуется
  только на линейную дистанцию. Угловая ошибка (yaw drift) лечится
  ручной правкой TRIM — оставляем как есть.
- **Авто-калибровка через измерение датчиками** (`calibration_node`
  уже умеет). Это отдельный механизм со своим UX — здесь не
  трогаем.
- **MPS-only коэффициент** (умножать на D перед отправкой
  сценария). Отвергнут на стадии brainstorming: физика калибровки
  — это свойство пары робот+поверхность, не свойство MPS.
  Использование общих коэффициентов motor_node корректнее.

## 8. Риски

- **Breaking change для тех, кто читает `calibration_active_profile`
  напрямую.** Поиск (`grep -rn calibration_active_profile`) перед
  правкой; если есть consumers — мигрировать.
- **Профиль с пустыми коэффициентами при первом запуске.** До
  фикса `_h_calibration_active` поля стартовали пустыми, и
  «Применить» отправлял `NaN`. После фикса state наполняется как
  только `motor_node` отправит retained `calibration/active` (это
  происходит при подключении). Edge-case: если motor_node ещё не
  стартовал — `coeffs == null`, инпуты пустые. Это уже текущее
  поведение, не регрессия.
