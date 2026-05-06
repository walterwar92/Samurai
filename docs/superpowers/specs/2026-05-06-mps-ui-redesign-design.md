# МПС — редизайн страницы /mps

> Полный визуальный и информационный редизайн страницы `/mps` в дашборде
> `compute_node/frontend`. Цель: превратить «сетку с цифрами» в учебный
> инструмент, где студент редактирует матрицы A/B/C/D **не вслепую** —
> рядом всегда видна ОДУ-модель робота, а каждая ячейка матрицы
> подписана своим физическим смыслом.
>
> Базируется на: ветка `feat/mps`, главная спека модуля
> [`2026-05-05-mps-state-space-design.md`](./2026-05-05-mps-state-space-design.md).
>
> Скоуп: **только frontend** (`compute_node/frontend/src/`). Backend
> (`pi_nodes/`, `compute_node/dashboard/`) не трогается. REST/WS контракт
> сохраняется.
>
> Ответственный: @OneAstr0 (Track B).

## 1. Цели и не-цели

### Цели

1. **Понятность.** Любая ячейка A/B/C/D должна явно говорить «коэффициент при `v` в уравнении `dθ/dt`», а сверху страницы — карта ОДУ.
2. **Скорость работы.** Меньше скроллов; формулы и физика всегда на виду через sticky-колонку.
3. **Эстетика.** Унифицированный стиль карточек, нормальная типографика (font-mono для чисел, KaTeX для формул), цветовая дисциплина для состояний.
4. **Защита от ошибок.** Подсветка отклонений от канонической линеаризации, предупреждения о неустойчивости/несогласованности, тултипы с интерпретацией.

### Не-цели

- Не меняем backend (`pi_nodes/`, `compute_node/dashboard/`).
- Не меняем REST/WS контракт (`docs/mps/api.md`).
- Не добавляем 3D-визуализацию (это пункт §10 главной спеки, отдельный PR).
- Не добавляем persistence/SQLite/CSV-экспорт (тоже §10).
- Не делаем e2e (Playwright) — только vitest.

## 2. Глобальный layout

```
┌──────────────────── Header (как сейчас) ──────────────────────────┐

┌─ Title row ──────────────────────────────────────────────────────┐
│ МПС — Модель Пространства Состояний  [DraftStatus]  [WS: idle]   │
└──────────────────────────────────────────────────────────────────┘

┌─ Sticky-left 35% (max-w 480px) ─┐ ┌─ Scrollable right 65% ──────┐
│  • OdeCard                        │ │ • MatrixEditor (табы)         │
│  • PhysicsParams                  │ │ • EigenvaluePanel              │
│  • MiniDiagnostics (mini eigen +  │ │ • ScenarioControls             │
│    canonical/stable badges)       │ │ • ResultPlots                  │
│                                    │ │ • TrajectoryView               │
│                                    │ │ • HistoryPanel                 │
└────────────────────────────────────┘ └────────────────────────────────┘
```

**Адаптив.** На `< lg` (1024 px) колонки складываются вертикально, sticky
отключается. Меньше 768 px — мобильное предупреждение «UI оптимизирован
под десктоп».

**Sticky-механика.** `position: sticky; top: 64px` (высота Header).
Высота sticky-контента не превышает `calc(100vh - 64px)`; если
содержимое выше — внутренний `overflow-y: auto`.

**Цветовая дисциплина (везде):**

| Состояние ячейки/блока | Класс |
|---|---|
| applied & canonical | нейтрально |
| dirty (несохранённый draft) | `bg-amber-500/10` |
| invalid (NaN, не число, нарушение bounds) | `border-red-500` |
| canonical deviation (значение в «нулевой по канону» ячейке) | `border-orange-400 border-dashed` |
| highlighted from cross-hover | `ring-2 ring-primary/60` |
| local hover | `bg-primary/10 ring-1 ring-primary/40` |

**Типографика.** Заголовки секций: `text-base font-semibold`. Числовые
значения: `font-mono text-sm`. Метки и подписи: `text-xs text-muted-foreground`.
Поля ввода чисел: `font-mono text-sm h-8`. Глобально поднимаем `text-xs`
до `text-sm` где это не теснит.

## 3. Канон состояния и токен-маппинг

**Каноническое состояние** (по спеке §5.1, использовано в `mps_node.py`):

```
x = [s, v, θ, ω, e_int]ᵀ ∈ ℝ⁵
u = [v_cmd, ω_cmd]ᵀ ∈ ℝ²
```

**Каноническая ОДУ-модель** (после линеаризации вокруг v₀):

```
ṡ      = v
v̇      = −(1/τ_v)·v + (1/τ_v)·u_v
θ̇      = ω
ω̇      = −(1/τ_ω)·ω + (1/τ_ω)·u_ω
ė_int  = v_target − v
```

**Каноническая структура матриц** (`lib/mps/canonical.ts: CANONICAL_PATTERN`):

| Матрица | Ячейка | Значение | Связь с физикой |
|---|---|---|---|
| A | [0][1] | 1 | ṡ ← v (фиксированная единица) |
| A | [1][1] | −1/τ_v | dynamic |
| A | [2][3] | 1 | θ̇ ← ω (фиксированная единица) |
| A | [3][3] | −1/τ_ω | dynamic |
| A | [4][1] | −1 | ė_int ← −v (фиксированная) |
| A | все остальные | 0 | — |
| B | [1][0] | 1/τ_v | dynamic |
| B | [3][1] | 1/τ_ω | dynamic |
| B | все остальные | 0 | — |

**Поправка про v₀.** В канонической форме данной спеки `v₀` не входит
в A или B (нет уравнения, которое бы его содержало в `[s, v, θ, ω, e_int]`).
Параметр фигурирует только как `v_target` в `ScenarioControls` — это
скорость, к которой едем в сценарии. Поэтому в `PhysicsParams` слайдера
v₀ нет; если в будущем бэкенд расширит линеаризацию (например, добавит
зависимость от рабочей точки в A), `CANONICAL_PATTERN` обновится и
v₀ получит двунаправленную связь — это out-of-MVP.

### Токен-маппинг (`lib/mps/tokenMap.ts`)

Каждый «крутящийся» член в формулах ОДУ имеет id, к которому привязан
DOM-узел в KaTeX-выводе и ячейка матрицы:

| Token id | В формуле | Матр. ячейка |
|---|---|---|
| `coef_s_v` | коэф `1` при `v` в `ṡ` | A[0][1] |
| `coef_v_v` | коэф `−1/τ_v` при `v` в `v̇` | A[1][1] |
| `coef_v_uv` | коэф `1/τ_v` при `u_v` в `v̇` | B[1][0] |
| `coef_theta_omega` | коэф `1` при `ω` в `θ̇` | A[2][3] |
| `coef_omega_omega` | коэф `−1/τ_ω` при `ω` в `ω̇` | A[3][3] |
| `coef_omega_uomega` | коэф `1/τ_ω` при `u_ω` в `ω̇` | B[3][1] |
| `coef_eint_v` | коэф `−1` при `v` в `ė_int` | A[4][1] |
| `coef_eint_vt` | `v_target` в `ė_int` | (нет в матрицах — ref-параметр из ScenarioControls) |

## 4. Cross-highlight Context

```typescript
// components/mps/HighlightContext.tsx
interface MpsHighlightState {
  hoveredEquation: 0 | 1 | 2 | 3 | 4 | null         // строка ОДУ
  hoveredCell: { matrix: 'A'|'B'|'C'|'D'; row: number; col: number } | null
  hoveredVector: { name: 'Q'|'R'|'u_min'|'u_max'; index: number } | null
}
interface MpsHighlightActions {
  setEquation: (i: number | null) => void
  setCell: (cell: MpsHighlightState['hoveredCell']) => void
  setVector: (v: MpsHighlightState['hoveredVector']) => void
}
```

**Производные подсветки** (вычисляются в потребителях):

- `hoveredEquation === i` → подсветить строку i в A, B, и `Q[i]`
- `hoveredCell === {matrix:'A', row:i, col:j}` →
  - подсветить уравнение i в OdeCard
  - подсветить токен из `tokenMap` если ячейка каноническая
  - подсветить столбец j в качестве «откуда берётся вклад»
- `hoveredVector === {name:'Q', index:i}` → подсветить уравнение i + строку i в A, B

**Throttle.** Обновления Context идут через `requestAnimationFrame` (≈16 мс),
чтобы не флудить рендером. Сбрасывается на `mouseleave` контейнера-родителя.

## 5. OdeCard — карта ОДУ

Sticky-блок в левой колонке. Read-only, рендерит 5 уравнений через
KaTeX с token-spans для cross-highlight.

```
┌─ ОДУ-модель робота (непрерывная) ────────────────────────────┐
│                                                                 │
│  ╭───────────────────────────────────────────────────────╮  │
│  │ ṡ      = v                                              │  │  row 0
│  │ v̇      = −(1/τ_v)·v + (1/τ_v)·u_v                       │  │  row 1
│  │ θ̇      = ω                                              │  │  row 2
│  │ ω̇      = −(1/τ_ω)·ω + (1/τ_ω)·u_ω                       │  │  row 3
│  │ ė_int  = v_target − v                                   │  │  row 4
│  ╰───────────────────────────────────────────────────────╯  │
│                                                                 │
│  Состояние:  x = [s, v, θ, ω, e_int]ᵀ                          │
│  Управление: u = [v_cmd, ω_cmd]ᵀ                               │
│                                                                 │
│  ┌─ Дискретизация ZOH (Ts = 50 мс) ─┐                          │
│  │ x[k+1] = A·x[k] + B·u[k]          │                         │
│  │ y[k]   = C·x[k] + D·u[k]          │                         │
│  └────────────────────────────────────┘                         │
│                                                                 │
│  [✓ Показывать численные значения]                             │
└────────────────────────────────────────────────────────────────┘
```

**Toggle «Показывать численные значения».** Если включён — справа от
символьного коэффициента в скобках численное значение из текущего
draft/applied: `−(1/τ_v) [−6.67]·v + (1/τ_v) [6.67]·u_v`.

**Hover на строке уравнения** → `setEquation(i)` в Context.
**Hover на токене коэффициента** → если ячейка каноническая,
`setCell(tokenMap.find(token))`.

**Деканонизация (бейдж).** Если в матрице есть значения, выходящие за
канонический паттерн (например `A[0][2] = 0.5`), над уравнением 0
появляется маленький `Badge variant="warning"`: `+ нестандартные члены (1)`.
Клик по бейджу — popup со списком отклонений.

**API:**
```tsx
interface OdeCardProps {
  matrices: MpsMatrices | null   // current applied или draft (что отображать)
  showNumeric?: boolean
}
```

## 6. PhysicsParams — двунаправленные слайдеры

Sticky-блок под OdeCard. Управляет τ_v и τ_ω (двунаправленно с матрицами).
**v₀ не входит в `MpsMatrices` в канонической форме данной спеки**
(в `[s, v, θ, ω, e_int]` нет уравнения с `v₀`-членом), поэтому слайдер
v₀ в этом блоке не нужен — параметр фигурирует только как
`v_target` в `ScenarioControls`.

```
┌─ Физические параметры ───────────────────────────────────────┐
│                                                                │
│  τ_v   [───────●──────────] 0.150 c   [↻]                    │
│        быстро ←──────→ медленно                                │
│        A[1][1] = −6.67    B[1][0] = 6.67                      │
│                                                                │
│  τ_ω   [─────●────────────] 0.100 c   [↻]                    │
│        A[3][3] = −10.0    B[3][1] = 10.0                      │
│                                                                │
│  ┌───────────────────────────────────────────┐                │
│  │ ✓ Каноническая форма                       │                │
│  │ Все «крутящиеся» ячейки A, B соответствуют │                │
│  │ τ_v=0.150, τ_ω=0.100.                       │                │
│  └───────────────────────────────────────────┘                │
│                                                                │
│  [Восстановить каноническую форму]                            │
└────────────────────────────────────────────────────────────────┘
```

### Алгоритм (`lib/mps/canonical.ts`)

```typescript
type CanonicalStatus = 'canonical' | 'incoherent' | 'non_canonical'

interface PhysicsExtraction {
  tau_v: number | null         // null если не определяется однозначно
  tau_omega: number | null
  status: CanonicalStatus
  deviations: Array<{
    matrix: 'A' | 'B'
    row: number
    col: number
    expected: number
    actual: number
  }>
}

export function detectPhysics(m: MpsMatrices): PhysicsExtraction {
  // 1. Проверить ВСЕ ячейки A, B вне CANONICAL_PATTERN на ноль (с tol).
  //    Любая ненулевая → push в deviations, status = 'non_canonical'.
  // 2. Проверить «фиксированные» (A[0][1]=1, A[2][3]=1, A[4][1]=−1) на
  //    точное соответствие. Любое отклонение → deviations.
  // 3. Из A[1][1] и B[1][0] восстановить tau_v двумя путями:
  //    tau_v_from_A = -1 / A[1][1]
  //    tau_v_from_B = 1 / B[1][0]
  //    Если |tau_v_from_A - tau_v_from_B| / tau_v_from_A > 1e-3 →
  //      tau_v = null, status = 'incoherent'.
  //    Иначе tau_v = (tau_v_from_A + tau_v_from_B) / 2.
  // 4. Аналогично tau_omega из A[3][3] и B[3][1].
  // 5. Если deviations пустые и tau_v/omega определены → 'canonical'.
}

export function buildCanonical(
  tau_v: number,
  tau_omega: number,
): { A: number[][]; B: number[][] } {
  // 5x5 нулевая + проставить ячейки CANONICAL_PATTERN
}
```

**Tolerance.** Значения сравниваются по relative tolerance `1e-3`
(`Math.abs(actual − expected) / Math.max(1e-9, Math.abs(expected)) < 1e-3`).
Это покрывает округления типа `−6.6666666` vs `−6.67`.

### Поведение

1. **Слайдер → A,B.** Двигаешь слайдер τ_v → вызывается `buildCanonical(...)`,
   результат через `onPatch(next)` уезжает в `useMpsMatrices.saveDraft`.
   Только «крутящиеся» ячейки A[1][1], B[1][0] меняются (фиксированные
   единицы и нули — оставляются как есть, чтобы не сбрасывать deviations
   студента).
2. **Ручная правка A → слайдер.** Когда в Context приходит обновление
   `applied`/`draft`, `PhysicsParams` запускает `detectPhysics`. При
   `canonical` слайдер едет на новое значение. При `incoherent` слайдер
   показывает «N/A» и серый бейдж: `τ_v неоднозначно: A→0.25, B→0.15`.
   При `non_canonical` все слайдеры дисейблятся, primary-бейдж: `Восстановить`.
3. **Кнопка `[↻]` рядом со слайдером.** Сброс **только этого** параметра
   к дефолту из `config.yaml` (получаем через `applied` initial state).
4. **Кнопка «Восстановить каноническую форму».** Зануляет ВСЕ ячейки A,
   B, потом проставляет канонический паттерн с текущими значениями
   τ_v/τ_ω слайдеров. Сильно destructive — confirm-диалог.

**API:**
```tsx
interface PhysicsParamsProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onPatch: (next: MpsMatrices) => void  // тот же хук что MatrixEditor.onChange
  defaults: { tau_v: number; tau_omega: number }
}
```

**Диапазоны слайдеров:**
- `τ_v`: `[0.05, 0.50]` c, step `0.005`
- `τ_ω`: `[0.05, 0.50]` c, step `0.005`

## 7. MatrixEditor (рефактор)

Карточка с табами. Удаляет inline `MatrixGrid` (заменяется компонентом
из `components/mps/MatrixGrid.tsx`). Поглощает функционал `TuningSliders.tsx`.

### Tabs

```
[• Динамика A·B ] [ Веса Q·R·N ] [ Выход C·D ]      Apply  Validate
```

#### Таб 1: «Динамика A·B»

Две сетки:

**A — 5×5 с подписями:**

```
              s        v        θ        ω      e_int
        ┌──────────────────────────────────────────────┐
   ṡ   │  0   ¹   1   ¹    0        0        0         │
   v̇   │  0       -6.67¹  0        0        0         │
   θ̇   │  0       0       0       1   ¹    0         │
   ω̇   │  0       0       0      -10  ¹    0         │
  ė_int │  0      -1   ¹   0        0        0         │
        └──────────────────────────────────────────────┘
   ¹ — индикатор: каноническая ячейка
```

**B — 5×2 с подписями:**

```
              u_v       u_ω
        ┌──────────────────┐
   ṡ   │  0       0       │
   v̇   │  6.67¹  0       │
   θ̇   │  0       0       │
   ω̇   │  0       10  ¹  │
  ė_int │  0       0       │
        └──────────────────┘
```

**Подписи строк/столбцов** sticky внутри таблицы (если widht не помещается,
columns sticky-top тоже).

**Маркер «¹» на канонической ячейке** — мелкий superscript-индекс.
Объяснение через legend под матрицей.

**Tooltip** (Radix Tooltip, delay 400 мс):

```
Каноническая ячейка:
  A[1][1] = ∂v̇/∂v = −1/τ_v
  Текущее значение: −6.67
  Связано с физ. параметром τ_v = 0.150 c
  Меняется слайдером τ_v или прямой правкой.

Не-каноническая (нулевая по канону):
  A[0][2] = ∂ṡ/∂θ
  В канонической линеаризации = 0.
  Ваше значение: 0.5 — добавит +0.5·θ к ṡ.
  ⚠ Это отклонение от стандартной модели Козлова §3.
```

#### Таб 2: «Веса Q·R·N»

Поглощает функционал `TuningSliders.tsx` (этот компонент удаляется).

```
┌─ Q — веса ошибки состояния ─────────────────────────────┐
│  Q[s]  состояние s     [────●─────] 10.0                │
│  Q[v]  состояние v     [──●───────] 5.0                 │
│  Q[θ]  состояние θ     [─●────────] 1.0                 │
│  Q[ω]  состояние ω     [─●────────] 1.0                 │
│  Q[eᵢ] состояние e_int [──●───────] 5.0                 │
│  ⓘ Большее Qᵢ = MPC сильнее штрафует отклонение i-го    │
│    состояния от reference.                               │
└──────────────────────────────────────────────────────────┘

┌─ R — веса управления ───────────────────────────────────┐
│  R[u_v]  v_cmd     [──●──────] 1.0                      │
│  R[u_ω]  ω_cmd     [──●──────] 1.0                      │
│  ⓘ Большие R ⇒ экономия управления, плавнее езда.       │
└──────────────────────────────────────────────────────────┘

┌─ N — горизонт прогноза ─────────────────────────────────┐
│  N steps           [──●──────] 20    (1.0 c при Ts=50мс)│
│  ⓘ Больше N — точнее план, дороже вычисления.           │
└──────────────────────────────────────────────────────────┘

┌─ Ограничения управления ────────────────────────────────┐
│  u_min = [v_min, ω_min]   [-0.30] [-1.50]               │
│  u_max = [v_max, ω_max]   [ 0.30] [ 1.50]               │
│  ⓘ MPC clip-ит u в эти рамки на каждом шаге.            │
└──────────────────────────────────────────────────────────┘
```

**Слайдеры Q, R, N — debounced re-sim** убирается с этого этапа (была в
`TuningSliders`). Превью прогона делается через явную кнопку
«Симулировать с этими весами» — клик запускает sim с draft, результат
в `ResultPlots`. Это снимает нагрузку «при каждом движении ползунка
запускается sim» (которая сейчас в `TuningSliders.tsx:92-112`).

**Cross-highlight:** hover на `Q[v]` подсвечивает уравнение `v̇` в
OdeCard и строку 1 в матрицах A, B.

#### Таб 3: «Выход C·D»

```
┌─ C — матрица выхода (5×5) ──────────────────────────────┐
│  [сетка как у A, default I_5]                            │
│  ⓘ y = C·x + D·u — для UI-визуализации, в управлении не │
│    используется (см. инвариант 3).                       │
│  [Сбросить к I_5]                                        │
└──────────────────────────────────────────────────────────┘

┌─ D — прямая связь (5×2) ────────────────────────────────┐
│  [сетка как у B, default 0]                              │
│  [Сбросить к 0]                                          │
└──────────────────────────────────────────────────────────┘
```

### Bottom action bar (общий)

```
[Apply (5 изменений)] [Validate]    [Reset draft] [Reset all]
Status: канонически: ✓ · λ stable: ✓ · 5 ячеек изменены
```

- **Apply** primary, активна если `dirty && !invalid`. Текст: `Apply (N изменений)` / `Apply` / `Применяем…`.
- **Validate** secondary, всегда активна.
- **Reset draft** outline, возврат в applied.
- **Reset all** ghost destructive, возврат в дефолт. Через `mpsApi.resetMatrices()` если есть, иначе клиентский reset через хардкод.

**Mini-диагностика** в одной строке: count изменений · canonical/incoherent/non-canonical · последний λ-результат.

### API

```tsx
interface MatrixEditorProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onChange: (m: MpsMatrices) => void
  onApply: () => void
  onValidate: () => void
  onReset: () => void           // в applied
  onResetAll: () => void        // в default
  saving?: boolean
  validationStatus?: 'unknown' | 'stable' | 'unstable'
}
```

## 8. EigenvaluePanel (расширен)

```
┌─ Анализ устойчивости ────────────────────────────────────────┐
│  ┌─ Unit-circle SVG ─┐  ┌─ Численные ────────────────────┐ │
│  │   ⊕ ⊕              │  │ λ(A):                            │ │
│  │  ⊕    ⊕            │  │   0.951 + 0.000i  ◯ stable      │ │
│  │ ★      ⊕           │  │   ...                             │ │
│  └────────────────────┘  └───────────────────────────────────┘ │
│                                                                 │
│  ┌─ Диагностика ────────────────────────────────────────────┐ │
│  │ ✓ Объект (A) устойчив                                     │ │
│  │ ✓ Замкнутая система (A−B·K) устойчива                     │ │
│  │ ✓ Управляемость: rank([B AB A²B A³B A⁴B]) = 5             │ │
│  │ ⚠ Один полюс на границе |λ|=1 — численная чувствительность │ │
│  └────────────────────────────────────────────────────────────┘ │
│  Последняя проверка: 14:52:03                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Изменения vs текущей версии:**
- **Численная таблица всегда видна** (сейчас под `<details>`).
- **Диагностика-чеклист** — три пункта явно видны:
  1. Open-loop stable (`isPlantStable`)
  2. Closed-loop stable (`isClosedLoopStable`)
  3. Controllable (новое — нужна доп. логика на frontend для рассчёта rank, или поле в `MpsValidateResult` от backend; на старт — отображаем «Не проверено» если поля нет)
- **Warnings** из `MpsValidateResult.warnings` рендерятся как `⚠`-строки внутри чеклиста.
- **Timestamp** последней валидации.

`isControllable` — если backend не предоставляет, пока показываем
«Управляемость: ⓘ не вычислено в текущей версии бэкенда». Не блокируем.

## 9. ScenarioControls (компактнее)

```
┌─ Сценарий «проехать D м вперёд» ────────────────────────────┐
│                                                                │
│  D = [2.0  ] м    v_target = [0.15] м/с    [○ Sim ● Robot]   │
│                                                                │
│  [▶ Run on Robot]   [⏹ Abort]                                │
│                                                                │
│  ⓘ Sim: ~100 мс. Robot: ~D/v_target c.                        │
│                                                                │
│  [─── progress bar при running ───]                           │
└────────────────────────────────────────────────────────────────┘
```

`Save params to config` **выкидываем** из MVP (нет endpoint на бэке;
если потом появится — добавим отдельным PR).

**Progress bar** при running: рассчитывается из `live.points` (если есть
последняя точка) или времени с момента запуска для sim-режима.

## 10. ResultPlots (раздельные табы + reference)

```
┌─ Графики прогона ────────────────────────────────────────────┐
│  Tabs: [s(t)] [v(t)] [θ,ω(t)] [u(t)] [y(t)] [Все вместе]    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ [Recharts с reference-линиями: D пунктиром, v_target] │ │
│  │ [overlay-runs из Compare, разные цвета + opacity 0.6]  │ │
│  │ [при live mode — анимированная точка current]          │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌─ Метрики прогона ──────────────────────────────────────┐ │
│  │ overshoot:     0.12 м    settling:    1.84 c           │ │
│  │ ss_error:      0.018 м   peak v:      0.16 м/с         │ │
│  │ control E:     0.42      peak ω:      0.05 рад/с       │ │
│  └────────────────────────────────────────────────────────┘ │
│  Run: a3f7c2 · 2026-05-06 14:52 · status: reached            │
└────────────────────────────────────────────────────────────────┘
```

**Изменения vs текущего:**
- **Раздельные табы для каждой переменной.** Сейчас `x(t)` рисует все 5 на одном (каша).
  Новые табы:
  - `s(t)` — главное; reference D горизонтальной пунктирной + settling time vertical
  - `v(t)` — reference `v_target` пунктиром
  - `θ,ω(t)` — две линии вместе
  - `u(t)` — управление с u_min/u_max штрих-пунктирными
  - `y(t)` — выход
  - `Все вместе` — старое поведение для сравнения
- **Reference-линии** на `s(t)` рисуются всегда если есть completed run (есть `request.distance`). При live-mode — пунктирная D-линия рисуется сразу при старте.
- **Метрики из `MpsScenarioResult.metrics`** — отдельная сетка под графиком, 6 чисел в 2 столбца. Сейчас не отображается.
- **Run header** под графиком: id (короткий 6-symbol), timestamp, status.

## 11. TrajectoryView (рестайл)

Поведение не меняется (2D top-down SVG). Только визуально:
- Заголовок «Траектория (top-down)»
- Scale bar внизу-слева (1 м marker)
- Легенда внутри SVG: `Plan (пунктир)`, `Actual (сплошная)`, `Live position` (точка)
- Унифицированный padding/border с другими карточками

## 12. HistoryPanel (улучшения)

```
┌─ История прогонов (12)  · Compare: 2/3 ─────────────────────┐
│  [Status: All ▾]   [Очистить выбор]                          │
│                                                                │
│  ┌─ Scrollable list ─────────────────────────────────────┐  │
│  │ │ ☑ a3f7c2  14:52  D=2.0 v=0.15  reached  ss=0.018 [▶] │  │
│  │ │ ☑ b1e9a4  14:48  D=2.0 v=0.20  reached  ss=0.041 [▶] │  │
│  │ │ ☐ c2d8f1  14:45  D=3.0 v=0.15  timeout  ss=0.510 [▶] │  │
│  │ │ ...                                                    │  │
│  └─────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

**Изменения:**
- **Status filter** dropdown (`All` / `reached` / `timeout` / `aborted` / `error`).
- **Подпись `Compare: 2/3`** в заголовке.
- **Цветовая полоса слева** у каждой строки (3 px, цвет по status).
- **Кнопка `[▶]`** заменяет текстовое `Replay`.

## 13. Errors panel (Sticky alerts)

Сейчас рендерится только если есть ошибки, сидит в самом низу. Переносим
наверх правой колонки:

```
┌─ ⚠ matrices: shape mismatch на B[1][2]      [×] dismiss ─────┐
└────────────────────────────────────────────────────────────────┘
┌─ ⚠ run: timeout — пройдено 1.2 м из 2.0 м   [×] dismiss ────┐
└────────────────────────────────────────────────────────────────┘
```

Сверху правой колонки (под header), накопляются стеком, dismiss-able.

## 14. Файлы

### Новые

```
compute_node/frontend/src/
├── components/mps/
│   ├── OdeCard.tsx                  ⭐ sticky левая, KaTeX-формулы
│   ├── PhysicsParams.tsx            ⭐ двунаправленные слайдеры
│   ├── MatrixGrid.tsx               ⭐ переиспользуемая сетка
│   ├── MatrixCellTooltip.tsx        ⭐ Radix Tooltip с интерпретацией
│   ├── HighlightContext.tsx         ⭐ Context для cross-highlight
│   ├── KatexFormula.tsx             ⭐ обёртка над react-katex с token-spans
│   └── MatrixTabs.tsx               ⭐ табы внутри MatrixEditor
├── lib/mps/
│   ├── canonical.ts                 ⭐ detectPhysics, buildCanonical
│   └── tokenMap.ts                  ⭐ маппинг ячейка ↔ token-id
└── hooks/
    └── useMpsHighlight.ts           ⭐ sugar над HighlightContext
```

### Перерабатываемые

```
├── pages/MpsPage.tsx                  🔄 sticky-grid layout
├── components/mps/MatrixEditor.tsx    🔄 табы + использование MatrixGrid
├── components/mps/ScenarioControls.tsx 🔄 horizontal layout
├── components/mps/ResultPlots.tsx     🔄 раздельные табы + reference + метрики
├── components/mps/EigenvaluePanel.tsx 🔄 inline numerics + чеклист
├── components/mps/HistoryPanel.tsx    🔄 status filter + цветовая полоса
├── components/mps/TrajectoryView.tsx  🔄 рестайл (legend, scale bar)
├── components/mps/DraftStatus.tsx     🔄 расширить под canonical/incoherent
└── components/mps/ValidationBadge.tsx 🔄 минор (логика уезжает в EigenvaluePanel)
```

### Удаляемые

```
├── components/mps/TuningSliders.tsx   🗑 функционал → MatrixEditor таб «Q·R·N»
```

## 15. Зависимости

```json
"dependencies": {
  "katex": "^0.16.x",
  "react-katex": "^3.x"
}
```

В `main.tsx`:
```tsx
import 'katex/dist/katex.min.css'
```

## 16. Тесты

### Новые (vitest)

```
src/__tests__/mps/
├── canonical.test.ts
│   ├ detectPhysics на чистом каноне → {tau_v, tau_omega, v_zero, status:'canonical'}
│   ├ detectPhysics при отклонении в нулевой ячейке → status:'non_canonical', deviations[]
│   ├ detectPhysics при несогласованности A[1][1]/B[1][0] → tau_v=null, status:'incoherent'
│   ├ buildCanonical → правильная структура матриц
│   └ tolerance: округлённые −6.6666666 vs −6.67 → canonical
├── tokenMap.test.ts
│   └ маппинг A[i][j] ↔ token-id, обратная функция
├── OdeCard.test.tsx
│   ├ рендер 5 уравнений
│   ├ hover на уравнении → setEquation вызывается
│   ├ showNumeric=true → численные коэффициенты в формуле
│   └ деканонизация → бейдж «нестандартные члены»
├── PhysicsParams.test.tsx
│   ├ слайдер τ_v изменяется → onPatch с новыми A[1][1], B[1][0]
│   ├ ручная правка A → слайдер едет (через detectPhysics)
│   ├ incoherent state → слайдер показывает N/A, disabled
│   └ non_canonical → все слайдеры disabled, primary-кнопка «Восстановить»
└── MatrixGrid.test.tsx
    ├ invalid cell → border-red-500
    ├ dirty cell → bg-amber-500/10
    ├ canonical deviation → border-orange-400 dashed
    ├ hover → setCell в Context
    └ highlighted from context → ring-2
```

### Расширяемые

```
└── MpsPage.integration.test.tsx (или новый)
    ├ hover в OdeCard на уравнении 1 → подсветка в MatrixGrid строка 1
    ├ hover в MatrixGrid на A[1][1] → подсветка токена в OdeCard
    ├ полный цикл: ползунок τ_v → A draft → Apply → applied
    └ smoke: все секции рендерятся без ошибок
```

## 17. Бэкенд

**НИЧЕГО НЕ МЕНЯЕМ.** Все правки во frontend. REST/WS контракт сохраняется.

**Возможный (опциональный) endpoint:**
- `POST /api/v1/mps/matrices/reset-to-default` — для кнопки «Reset all».
- Если нет — клиентский reset через хардкод дефолтного `MpsMatrices` на
  фронте или через `?source=default` параметр в GET (если backend
  поддерживает).

Договорённость с @razdryzg-dev на этапе implementation: если endpoint
тривиален, добавляем; если сложнее — делаем клиентский fallback.

## 18. Риски и митигации

| Риск | Импакт | Митигация |
|---|---|---|
| `react-katex` SSR/HMR проблемы | средний | Обёртка `KatexFormula` — `useEffect` + `dangerouslySetInnerHTML` с очисткой |
| Cross-highlight performance (re-render всех ячеек на mouseEvent) | средний | Throttle `requestAnimationFrame`, `React.memo` на `MatrixCell` |
| `detectPhysics` ложные срабатывания на округлениях | низкий | Relative tolerance `1e-3` вместо абсолютного |
| KaTeX bundle size +280KB | низкий | Внутренний инструмент, не критично для UX |
| Навигация по табам сбрасывает hover-state | низкий | Context на уровне MpsPage, не MatrixEditor |
| Старые регистрации hover-handler-ов накапливаются | низкий | Очистка в return useEffect, AbortController на event listeners |

## 19. Объём работ

| Этап | Файлов | Прибл. строк | Слож. |
|---|---|---|---|
| 1. Каркас: канонические утилиты + Context + KatexFormula | 4 | ~400 | низкая |
| 2. OdeCard с подсветкой | 1 | ~250 | средняя |
| 3. PhysicsParams (двунаправленные) | 1 | ~250 | средняя |
| 4. MatrixGrid + MatrixTabs + рефактор MatrixEditor | 3 | ~500 | средняя |
| 5. EigenvaluePanel + ScenarioControls + ResultPlots + History рестайл | 4 | ~450 | низкая |
| 6. MpsPage layout + sticky | 1 | ~200 | низкая |
| 7. Тесты | 6 | ~700 | средняя |
| **Итого** | **~20 файлов** | **~2750 строк** | |

Имплементация: 4-6 коммитов в `feat/mps`.

## 20. Acceptance criteria

- [ ] OdeCard в sticky-левой колонке всегда видна при работе с матрицами справа на десктопе.
- [ ] Hover на уравнении в OdeCard → подсвечивается соответствующая строка в матрицах A, B.
- [ ] Hover на ячейке матрицы A или B → подсвечивается уравнение в OdeCard и токен коэффициента (если каноническая).
- [ ] PhysicsParams — двигание слайдера τ_v изменяет A[1][1] и B[1][0] в draft (двигание τ_ω аналогично для A[3][3], B[3][1]).
- [ ] Ручная правка A[1][1] → ползунок τ_v едет на новое значение (если consistent).
- [ ] Несогласованность A[1][1]/B[1][0] → ползунок показывает N/A.
- [ ] Запись в «не-каноническую» ячейку → ячейка получает оранжевую пунктирную рамку, OdeCard показывает бейдж «нестандартные члены».
- [ ] Кнопка «Восстановить каноническую форму» сбрасывает все нестандартные клетки в 0 и проставляет канонический паттерн.
- [ ] EigenvaluePanel показывает численные значения inline без раскрытия `<details>`.
- [ ] ResultPlots имеет раздельные табы для s/v/θω/u/y/Все вместе.
- [ ] Reference-линия `D` пунктиром на графике `s(t)`.
- [ ] Метрики прогона видны под графиком, 6 значений в 2 столбца.
- [ ] HistoryPanel — фильтр по status, цветовая полоса, кнопка ▶ для replay.
- [ ] TuningSliders.tsx удалён, функционал работает в табе «Q·R·N» MatrixEditor.
- [ ] Все vitest-тесты зелёные.
- [ ] `npm run build` зелёный, `npm run lint` без новых warnings.
- [ ] На `< lg` (1024 px) колонки складываются вертикально, sticky отключается.
- [ ] Page renders без console.error.

## 21. Open questions (решить на implementation)

1. `isControllable` — добавить в `MpsValidateResult` бэкендом или вычислять на фронте? **Решение:** на старт показываем «не вычислено», договариваемся с @razdryzg-dev параллельно (отдельный PR).
2. `Reset all` endpoint — есть ли уже? **Решение:** проверить на этапе implementation; если нет — клиентский хардкод defaults.
3. `react-katex` vs `@matejmazur/react-katex` — какая стабильнее? **Решение:** проверить npm trends + последний релиз; если оба ок — `react-katex` (короче имя).
4. Делать ли confirm-диалог на «Восстановить каноническую форму»? **Решение:** да, через shadcn Dialog — destructive действие.
