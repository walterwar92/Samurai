# Samurai Dashboard — Redesign Prompt

> **Аудитория:** Claude (Code), задача — переработать визуальный слой React-фронтенда `compute_node/frontend/` без изменения функциональности.
> **Ветка:** `feat/redesign` (создана из `dev`).
> **Стиль:** «Тёмный Claude» — тёплый графит + коралловый акцент, в духе Claude.ai dark theme, адаптированный под телеметрию робота-оператора.
> **Принцип:** «Тихая основа, живые акценты». Поверхности молчат, статусы и интерактив — выразительные.

---

## 0. Контекст проекта (минимум, чтобы войти)

`compute_node/frontend/` — React 19 + TypeScript + Vite 7 + Tailwind 3 + shadcn/ui (на @radix-ui).
Сборка: `npm run build` → `compute_node/static/`. FastAPI отдаёт статику (`/dashboard`, `/admin`, `/3d`, `/hardware`, `/samcan`).

**Маршруты (App.tsx):**
- `/dashboard` → `DashboardPage` — главная панель оператора
- `/admin` → `AdminPage` — расширенное управление, FSM/servo/calibration
- `/3d` → `Visualization3DPage` — 3D-сцена Three.js / R3F
- `/hardware` → `HardwarePage` — редактор пресетов железа
- `/samcan` → `SamcanPage` — управление вторым роботом (Arduino)

**Провайдеры:** `RobotProvider` (выбор активного робота samurai/samcan, localStorage), `SocketProvider` (Socket.IO к backend :5000, push-state).

**Hooks (НЕ трогать):** `useRobotState`, `useSamcanState`, `useJoystick`, `useMapCanvas`, `useZoneDrawing`, `useSensorHistory`.

**API клиент (НЕ трогать):** `src/lib/api.ts` — ~60 REST-обёрток.

Полная карта фронта — в `MEMORY.md` → `frontend.md` (читать, если нужно понять расположение конкретного компонента).

---

## 1. Цели и не-цели

### 1.1 Goals
1. Переработать визуальный слой — палитра, типографика, поверхности, layout, иконография — в стилистике «Тёмный Claude».
2. Сохранить **весь** функционал. Все 5 страниц, все ~60 компонентов, все REST-вызовы, весь Socket.IO state. Ни один существующий пользовательский сценарий не должен сломаться.
3. Ввести две темы (dark по умолчанию + light) с тоглом в sidebar. Темы — параллельные палитры одного дизайна.
4. Добавить sidebar (вместо горизонтального navbar) — складная, с памятью состояния.
5. Унифицировать визуальный язык: одинаковые отступы, радиусы, тени, easing на всех страницах.
6. Поднять «считываемость» оператором: HUD на камере, FSM-timeline, технический джойстик с readout.

### 1.2 Non-goals (HARD блок — НЕ менять)

**Бэкенд и инфраструктура:**
- ❌ `compute_node/*.py` — backend, FastAPI endpoints, Socket.IO emitters
- ❌ `vite.config.ts` proxy-маршруты (`/api/*`, `/video_feed`, `/map.png`, `/socket.io/*`)
- ❌ Маршруты React Router (`/dashboard`, `/admin`, `/3d`, `/hardware`, `/samcan`)

**Логика фронта:**
- ❌ `src/hooks/*` — ни одного хука внутри не трогать (только импорты в компонентах)
- ❌ `src/providers/*` — `RobotProvider`, `SocketProvider` — внутренности неизменны
- ❌ `src/lib/api.ts` — REST-обёртки, методы, endpoints
- ❌ `src/types/*` — TypeScript-типы (`RobotState`, `Detection`, etc.)
- ❌ `src/lib/constants.ts` — `FSM_COLORS`, `COLOUR_CSS`, `QUICK_COMMANDS`, `ALL_STATES`, `COLOUR_RU`, `ACTION_RU` — **значения и ключи остаются прежними**. Не править. Компоненты, которым нужны новые цвета FSM/мячей, **не используют эти константы для стилей**, а получают цвет через CSS-классы (см. §3.3, §3.4)

**Контракт компонентов:**
- ❌ Сигнатуры props всех существующих компонентов — родители должны продолжать работать
- ❌ Названия экспортируемых компонентов и хуков
- ❌ Поведение при тех же данных (что показывает компонент, в каких случаях скрывается)

**Зависимости:**
- ❌ `package.json` — НЕ добавлять рантайм-библиотеки (никаких `framer-motion`, `react-spring`, `headlessui` и т.п.)
- ✅ Можно добавить шрифты через `<link rel="preconnect">` + `<link rel="stylesheet">` в `index.html` (Google Fonts: Inter, JetBrains Mono)
- ✅ Можно добавить `tailwind-merge`, `class-variance-authority` версии — они уже есть

### 1.3 Свобода действий

- ✅ `src/index.css`, `tailwind.config.ts`, `components.json` — переписать
- ✅ Внутренности всех компонентов: JSX, классы, разметка, локальный state, добавление подкомпонентов внутри файла
- ✅ `src/components/ui/*` — shadcn primitives, переписать стили
- ✅ Реорганизовать layout `pages/*Page.tsx` — раскладка, группировка, новые wrapper-компоненты
- ✅ Создавать новые **визуальные** компоненты (Sidebar, ThemeToggle, FsmTimeline, HudOverlay, ScanlineEffect, KatanaIcon, JoystickIcon, ClawIcon)
- ✅ Менять `index.html` — title, fonts, meta-теги, favicon
- ✅ Заменить `samurai_256.png` логотип на новую monoline-версию (как inline SVG-компонент)

---

## 2. Концепция и тон

**Одной фразой:** Claude.ai в тёмной теме встретил пульт оператора робота. Бумажный покой Anthropic + кинематографичный HUD телеметрии.

**Эмоция:** не «cyberpunk киберпанк», не «материал-дизайн», не «glassmorphism». Спокойный, читаемый, технический. Как чистая кабина управления, где ничего не отвлекает, но видно всё что нужно.

**Что значит «живые акценты»:**
- Цветным сигналом считается **только** активное состояние и текущая детекция — coral акцент.
- Всё остальное — приглушённая ink-палитра. Не «outline = бесцветный, primary = синий», а «outline = тише, primary = ярче».
- Анимации **выразительны там, где они говорят оператору о изменении** (FSM перешёл, детекция появилась, путь записался) — и **тихи там, где не должны отвлекать** (hover, focus).

---

## 3. Дизайн-токены

> Хекс-значения — для понимания. В CSS-переменных используется формат `H S% L%` без `hsl()`-обёртки (как требует shadcn), потому что primitives в `src/components/ui/*` пишут `hsl(var(--background))`.

### 3.1 Поверхности (Dark theme — default)

| Токен | Hex | HSL | Назначение |
|-------|-----|-----|-----------|
| `--background` | `#1F1E1D` | `30 4% 12%` | Корневой фон страницы (тёплый графит) |
| `--surface-1` | `#262624` | `40 4% 15%` | Карточки, панели, sidebar |
| `--surface-2` | `#2D2B28` | `30 6% 17%` | Вложенные элементы (input, badge внутри карточки), popover |
| `--surface-3` | `#34322F` | `30 5% 19%` | Hover/elevated, dropdown items |
| `--border-subtle` | `#2F2D2A` | `30 5% 17%` | 1px бордеры карточек (по умолчанию) |
| `--border-strong` | `#3A3733` | `30 6% 21%` | Hover/focus бордеры |
| `--foreground` | `#E8E5DD` | `40 19% 89%` | Основной текст (тёплый off-white) |
| `--foreground-muted` | `#A8A39A` | `35 9% 63%` | Вторичный текст, лейблы |
| `--foreground-faint` | `#6F6A60` | `35 7% 40%` | Placeholder, disabled, мета-инфо |

### 3.2 Акцент и семантика

| Токен | Hex | Назначение |
|-------|-----|-----------|
| `--accent` | `#CC785C` | Anthropic coral — primary action, active state, focused detection |
| `--accent-hover` | `#D5876C` | Hover у accent кнопок/линков |
| `--accent-active` | `#B86A50` | Pressed/active |
| `--accent-foreground` | `#1F1E1D` | Текст поверх accent (читается на coral) |
| `--ring` | `rgba(204, 120, 92, 0.4)` | Focus-ring (через `outline: 2px solid var(--ring)` или `box-shadow`) |
| `--success` | `#7DA88A` | Muted sage — успех, completed, healthy |
| `--warning` | `#C9A26B` | Muted ochre — внимание, warning |
| `--danger` | `#C97B7B` | Muted rose — destructive, errors, E-stop активный |
| `--info` | `#7DA1C9` | Muted slate blue — информационные подсказки |

### 3.3 FSM-палитра (7 состояний)

Сейчас `FsmBadge.tsx` берёт цвет из `FSM_COLORS` в `lib/constants.ts`. **Constants блокированы** (§1.2), поэтому новые цвета задаём через **CSS-классы**, а компоненты вместо `style={{ color: FSM_COLORS[state] }}` используют lookup-объект `FSM_CLASS_MAP` **внутри файла компонента** (или в новом `src/components/fsm/fsm-styles.ts` — это новый файл, не constants).

Новая палитра (muted ink):

| State | New Hex | CSS class (Tailwind) | Семантика |
|-------|---------|---------------------|-----------|
| `IDLE`        | `#8A8278` | `text-fsm-idle`        | Muted bone — покой |
| `SEARCHING`   | `#7DA1C9` | `text-fsm-searching`   | Muted slate — сканирование |
| `TARGETING`   | `#CC785C` | `text-accent`          | **Coral — единственный «активный фокус» в палитре** |
| `APPROACHING` | `#C9A26B` | `text-fsm-approaching` | Muted ochre |
| `GRABBING`    | `#B894C9` | `text-fsm-grabbing`    | Muted lavender |
| `CALLING`     | `#7DA88A` | `text-fsm-calling`     | Muted sage |
| `RETURNING`   | `#9A9089` | `text-fsm-returning`   | Muted khaki |

В `tailwind.config.ts` добавить семейство `fsm`:
```ts
colors: {
  fsm: {
    idle:        '#8A8278',
    searching:   '#7DA1C9',
    approaching: '#C9A26B',
    grabbing:    '#B894C9',
    calling:     '#7DA88A',
    returning:   '#9A9089',
    // targeting → используем существующий accent
  },
}
```

В компоненте — lookup:
```ts
// src/components/fsm/fsm-styles.ts (НОВЫЙ файл, рядом с FsmBadge)
import type { FsmState } from '@/types/robot';

export const FSM_TEXT_CLASS: Record<FsmState, string> = {
  IDLE:        'text-fsm-idle',
  SEARCHING:   'text-fsm-searching',
  TARGETING:   'text-accent',
  APPROACHING: 'text-fsm-approaching',
  GRABBING:    'text-fsm-grabbing',
  CALLING:     'text-fsm-calling',
  RETURNING:   'text-fsm-returning',
};

export const FSM_BG_CLASS: Record<FsmState, string> = {
  IDLE:        'bg-fsm-idle',
  // ... аналогично
};
```

**Правило:** только TARGETING получает coral, остальные — приглушённые. Это работает с FSM-timeline (§6.7): активное состояние горит coral, остальные тихие.

### 3.4 Цвета мячей (7 цветов, десатурированные)

`COLOUR_CSS` в `lib/constants.ts` — **блокирован**. Новые цвета задаём через Tailwind-классы и lookup-объект внутри компонентов потребителей (`BallsTable`, `DetectionBanner`, `DetectionTable`).

Десатурируем на ~25% (от исходных pure colors), чтобы не резать глаз на тёмном:

| Цвет | New Hex | Tailwind class |
|------|---------|----------------|
| red    | `#B85C5C` | `text-ball-red` / `bg-ball-red` |
| blue   | `#5C7DC9` | `text-ball-blue` |
| green  | `#6BA86B` | `text-ball-green` |
| yellow | `#C9B05C` | `text-ball-yellow` |
| orange | `#C9885C` | `text-ball-orange` |
| white  | `#D8D2C8` | `text-ball-white` |
| black  | `#2A2826` | `text-ball-black` |

В `tailwind.config.ts`:
```ts
colors: {
  ball: {
    red: '#B85C5C', blue: '#5C7DC9', green: '#6BA86B',
    yellow: '#C9B05C', orange: '#C9885C',
    white: '#D8D2C8', black: '#2A2826',
  },
}
```

В компонентах потребителях — новый lookup (например, `src/components/detection/ball-styles.ts`):
```ts
export const BALL_BG_CLASS: Record<string, string> = {
  red: 'bg-ball-red', blue: 'bg-ball-blue', green: 'bg-ball-green',
  yellow: 'bg-ball-yellow', orange: 'bg-ball-orange',
  white: 'bg-ball-white', black: 'bg-ball-black',
};
```

`COLOUR_CSS` из constants остаётся, но не используется новыми компонентами — код, который читает `COLOUR_CSS` для рендера, переписать на новый lookup.

Для `black` и `white` мячей на dark/light фоне — добавить тонкий 1px `border-strong` обводку у бейджей/чипов цвета, чтобы был виден контраст с background.

### 3.5 Light theme (параллельная палитра)

| Токен | Hex |
|-------|-----|
| `--background` | `#F5F2EA` (warm cream) |
| `--surface-1` | `#FAF7EF` |
| `--surface-2` | `#F0EDE3` |
| `--surface-3` | `#E8E4D8` |
| `--border-subtle` | `#E5E1D6` |
| `--border-strong` | `#D8D2C5` |
| `--foreground` | `#2A2826` |
| `--foreground-muted` | `#6F6A60` |
| `--foreground-faint` | `#A8A39A` |
| `--accent` | `#B85F45` (чуть темнее coral для контраста на светлом) |
| `--accent-hover` | `#A85439` |
| `--accent-active` | `#9C4D33` |
| `--accent-foreground` | `#FAF7EF` |

FSM/мячи в light — те же hue, но opacity/saturation подкручены чтобы давать читаемый контраст на кремовом фоне (Claude должен подобрать пары, при необходимости — увеличить L на 10-15% от dark-варианта).

### 3.6 Типографика

**Шрифты (через Google Fonts в `index.html`):**
```html
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
```

**Стек в `index.css`:**
```css
body { font-family: 'Inter', 'Segoe UI', 'Ubuntu', sans-serif; }
.font-mono, code, pre { font-family: 'JetBrains Mono', ui-monospace, 'Cascadia Mono', Consolas, monospace; }
```

**Где Mono обязательно:**
- `CommandInput` (терминальный режим, см. §6.8)
- Header → sim time (`12:34:56`)
- FSM-timeline state labels (`IDLE / SRCH / TGT / APR / GRB / CAL / RTN` — сокращения uppercase)
- Все числовые показания: скорости, координаты, углы, расстояние, температура, заряд, IP
- Лог событий (`EventLog`, `SamcanEventLog`) — таймстампы и значения
- Detection table — координаты bbox, confidence
- Сетки таблиц (Tables) — числовые ячейки

**Шкала (Tailwind extensions):**
```ts
// в tailwind.config.ts → theme.extend.fontSize
'display': ['1.5rem',  { lineHeight: '1.75rem',  letterSpacing: '-0.01em', fontWeight: '600' }],
'h1':      ['1.125rem',{ lineHeight: '1.5rem',   letterSpacing: '-0.005em',fontWeight: '600' }],
'h2':      ['1rem',    { lineHeight: '1.375rem', letterSpacing: '-0.003em',fontWeight: '500' }],
'body':    ['0.875rem',{ lineHeight: '1.25rem' }],
'small':   ['0.75rem', { lineHeight: '1rem' }],
'micro':   ['0.625rem',{ lineHeight: '0.875rem', letterSpacing: '0.06em',  fontWeight: '500' }], // FSM badge labels
```

**Числовые значения везде:** `font-variant-numeric: tabular-nums;` — Claude добавляет это в `body` глобально или класс `.tabular`. Это **обязательно** для всех показаний — иначе цифры будут «прыгать» в реальном времени (1Hz обновление телеметрии).

### 3.7 Радиусы

```ts
// tailwind.config.ts → theme.extend.borderRadius
'sm':     '6px',
'DEFAULT':'8px',
'md':     '8px',
'lg':     '12px',
'xl':     '16px',
'pill':   '9999px',
```

Карточки — `rounded-lg` (12px). Кнопки и инпуты — `rounded-md` (8px). Pills и badges — `rounded-pill`.

### 3.8 Тени — НЕТ

В дарк-теме теней почти нет. Иерархия — через яркость поверхностей (`surface-1 / 2 / 3`) и тонкие 1px бордеры. Исключение — `dialog`, `popover`, `tooltip`: лёгкая тень `shadow: 0 8px 24px -8px rgba(0,0,0,0.5)`.

В light-теме можно добавить тонкую `shadow-sm` карточкам — кремовая поверхность того требует.

### 3.9 Easing и продолжительности

```ts
// tailwind.config.ts → theme.extend.transitionTimingFunction
'standard': 'cubic-bezier(0.4, 0, 0.2, 1)',
'spring':   'cubic-bezier(0.34, 1.56, 0.64, 1)', // overshoot — для joystick release, scale-in
'swift':    'cubic-bezier(0.4, 0, 1, 1)',         // exits

// theme.extend.transitionDuration
'fast':     '120ms',
'standard': '200ms',
'slow':     '320ms',
```

**Правила:**
- Цвет/прозрачность — `transition-colors duration-standard ease-standard`
- Transform (scale, translate) при появлении — `ease-spring`
- Ничего не анимируется дольше 320ms кроме keyframe-pulse'ов.

---

## 4. `index.css` и `tailwind.config.ts` — конкретика

### 4.1 `src/index.css` — целевой каркас

```css
@tailwind base;
@tailwind components;
@tailwind utilities;

@layer base {
  :root {
    /* Dark by default */
    --background: 30 4% 12%;
    --surface-1: 40 4% 15%;
    --surface-2: 30 6% 17%;
    --surface-3: 30 5% 19%;
    --border-subtle: 30 5% 17%;
    --border-strong: 30 6% 21%;
    --foreground: 40 19% 89%;
    --foreground-muted: 35 9% 63%;
    --foreground-faint: 35 7% 40%;

    --accent: 16 53% 58%;
    --accent-hover: 16 56% 63%;
    --accent-active: 16 41% 52%;
    --accent-foreground: 30 4% 12%;

    --success: 138 16% 57%;
    --warning: 36 45% 60%;
    --danger:  0 41% 64%;
    --info:    211 39% 64%;

    /* shadcn-compatible aliases (НЕ удалять — primitives зависят) */
    --card: var(--surface-1);
    --card-foreground: var(--foreground);
    --popover: var(--surface-2);
    --popover-foreground: var(--foreground);
    --primary: var(--accent);
    --primary-foreground: var(--accent-foreground);
    --secondary: var(--surface-2);
    --secondary-foreground: var(--foreground);
    --muted: var(--surface-2);
    --muted-foreground: var(--foreground-muted);
    --destructive: var(--danger);
    --destructive-foreground: var(--foreground);
    --border: var(--border-subtle);
    --input: var(--border-subtle);
    --ring: var(--accent);
    --radius: 0.5rem;
  }

  .light {
    --background: 42 32% 94%;
    --surface-1: 42 38% 96%;
    --surface-2: 42 24% 92%;
    --surface-3: 40 22% 88%;
    --border-subtle: 40 20% 87%;
    --border-strong: 38 17% 81%;
    --foreground: 30 6% 15%;
    --foreground-muted: 35 7% 40%;
    --foreground-faint: 35 9% 63%;

    --accent: 12 46% 50%;
    --accent-hover: 12 50% 44%;
    --accent-active: 12 51% 40%;
    --accent-foreground: 42 38% 96%;
    /* семантика — те же hue с подкрученной L под светлый фон */
  }
}

@layer base {
  * { @apply border-border; }
  html, body { @apply bg-background text-foreground; }
  body {
    font-family: 'Inter', 'Segoe UI', 'Ubuntu', sans-serif;
    font-variant-numeric: tabular-nums;
    min-height: 100vh;
    overflow-x: hidden;
  }
  code, pre, .font-mono {
    font-family: 'JetBrains Mono', ui-monospace, 'Cascadia Mono', Consolas, monospace;
  }
}

/* Scrollbar — тонкие, в палитре */
::-webkit-scrollbar { width: 6px; height: 6px; }
::-webkit-scrollbar-track { background: transparent; }
::-webkit-scrollbar-thumb {
  background: hsl(var(--border-strong));
  border-radius: 3px;
}
::-webkit-scrollbar-thumb:hover { background: hsl(var(--foreground-faint)); }

/* Joystick area */
.joystick-area { touch-action: none; }

/* Keyframes (см. §7) */
@keyframes pulse-soft {
  0%, 100% { opacity: 1; }
  50%      { opacity: 0.55; }
}
@keyframes scanline {
  0%   { transform: translateY(-100%); opacity: 0; }
  10%  { opacity: 0.08; }
  90%  { opacity: 0.08; }
  100% { transform: translateY(100%); opacity: 0; }
}
@keyframes fade-in-up {
  from { opacity: 0; transform: translateY(4px); }
  to   { opacity: 1; transform: translateY(0); }
}
```

### 4.2 `tailwind.config.ts` — extensions

```ts
export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  darkMode: ['class', '.dark'], // тогглим class на <html>; default — dark, .light — переключатель
  theme: {
    extend: {
      colors: {
        background: 'hsl(var(--background))',
        foreground: 'hsl(var(--foreground))',
        'foreground-muted': 'hsl(var(--foreground-muted))',
        'foreground-faint': 'hsl(var(--foreground-faint))',
        surface: {
          1: 'hsl(var(--surface-1))',
          2: 'hsl(var(--surface-2))',
          3: 'hsl(var(--surface-3))',
        },
        border: {
          DEFAULT: 'hsl(var(--border-subtle))',
          subtle:  'hsl(var(--border-subtle))',
          strong:  'hsl(var(--border-strong))',
        },
        accent: {
          DEFAULT: 'hsl(var(--accent))',
          hover:   'hsl(var(--accent-hover))',
          active:  'hsl(var(--accent-active))',
          foreground: 'hsl(var(--accent-foreground))',
        },
        success: 'hsl(var(--success))',
        warning: 'hsl(var(--warning))',
        danger:  'hsl(var(--danger))',
        info:    'hsl(var(--info))',
        // shadcn aliases для совместимости
        card: 'hsl(var(--card))',
        popover: 'hsl(var(--popover))',
        primary: { DEFAULT: 'hsl(var(--primary))', foreground: 'hsl(var(--primary-foreground))' },
        secondary: { DEFAULT: 'hsl(var(--secondary))', foreground: 'hsl(var(--secondary-foreground))' },
        muted: { DEFAULT: 'hsl(var(--muted))', foreground: 'hsl(var(--muted-foreground))' },
        destructive: { DEFAULT: 'hsl(var(--destructive))', foreground: 'hsl(var(--destructive-foreground))' },
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
      },
      fontSize: { /* см. §3.6 */ },
      borderRadius: { /* см. §3.7 */ },
      transitionTimingFunction: {
        standard: 'cubic-bezier(0.4, 0, 0.2, 1)',
        spring:   'cubic-bezier(0.34, 1.56, 0.64, 1)',
        swift:    'cubic-bezier(0.4, 0, 1, 1)',
      },
      transitionDuration: { fast: '120ms', standard: '200ms', slow: '320ms' },
      animation: {
        'pulse-soft': 'pulse-soft 1.6s ease-in-out infinite',
        'scanline':   'scanline 4s linear infinite',
        'fade-in-up': 'fade-in-up 200ms cubic-bezier(0.34, 1.56, 0.64, 1)',
      },
    },
  },
  plugins: [],
} satisfies Config;
```

---

## 5. Layout & Navigation

### 5.1 Sidebar (новый компонент)

Создать `src/components/layout/Sidebar.tsx`. Заменяет текущий `Header.tsx` как основной навигатор. Header остаётся, но превращается в **контекстную панель** конкретной страницы (см. §5.2).

**Структура:**
```
┌─────────────────┐
│ [icon] Samurai  │  ← KatanaIcon (coral) + wordmark, кликабельно → /dashboard
├─────────────────┤
│ Robot Selector  │  ← RobotProvider's selector, в стиле dropdown surface-2
├─────────────────┤
│ ⌘ Dashboard     │  ← lucide LayoutDashboard
│ ⚙ Admin         │  ← lucide Sliders
│ ⌖ 3D View       │  ← lucide Box
│ ⎯ Hardware      │  ← lucide Cpu
│ ⊟ Samcan        │  ← lucide Bot
├─────────────────┤
│ ☼ Theme toggle  │  ← lucide Sun/Moon
│ ● Connection    │  ← Socket.IO статус (зелёный=connected, danger=disconnected)
│ ⏷ Collapse      │  ← lucide PanelLeftClose / PanelLeftOpen
└─────────────────┘
```

**Поведение:**
- Ширина: развёрнут `220px`, свёрнут `56px`. Состояние в `localStorage['samurai.sidebarCollapsed']`.
- Активный маршрут: фон `surface-2`, текст и иконка `accent`, тонкая 2px coral-полоска слева (border-left).
- Hover неактивного: фон `surface-2/40`, иконка `foreground`.
- Анимация collapse: `transition-[width] duration-standard ease-standard`. Лейблы скрываются `opacity-0 pointer-events-none` через 80ms раньше (чтобы не «телепались» при сужении).
- В свёрнутом — Tooltip справа от каждой иконки на hover (radix Tooltip, delay 300ms).
- На экране < 1024px sidebar по умолчанию свёрнут (можно вручную развернуть, но при следующем reflow снова свернётся).

### 5.2 Header (контекстная панель)

`Header.tsx` урезается до правой панели сверху страницы (внутри main-area, после sidebar). Содержит:
- Слева: заголовок страницы (`Dashboard`, `Admin`, `3D View`, `Hardware`, `Samcan`) — `text-display`
- Справа: контекст-специфичные действия:
  - **Везде:** кнопка `Debug` (открывает `DebugModal`), sim time (моноспейсом, секундная точность)
  - **DashboardPage:** `Reset sim` (если симулятор)
  - **HardwarePage:** статус сохранения пресета (`saved / dirty / saving…`)
  - **SamcanPage:** статус подключения Arduino + COM-порт

Высота: `48px`. Фон `surface-1`. Bottom-border 1px `border-subtle`.

### 5.3 ThemeToggle

Новый компонент `src/components/layout/ThemeToggle.tsx`. Тоглит класс `light` на `<html>` (по умолчанию dark = нет класса). Сохраняет в `localStorage['samurai.theme']` (`'dark' | 'light'`).

При первой загрузке — читает localStorage, иначе `prefers-color-scheme`.

UI: иконки `Sun` (light активна) / `Moon` (dark активна) — переход cross-fade 200ms.

### 5.4 Адаптивность

- `≥ 1440px`: 3 колонки, sidebar развёрнут
- `1024-1439px`: 3 колонки, sidebar свёрнут по умолчанию
- `768-1023px`: 2 колонки (третья переезжает вниз), sidebar свёрнут
- `< 768px`: 1 колонка, sidebar превращается в bottom tab bar (5 иконок). Это edge-case — дашборд робота на телефоне используется редко, но не должен разваливаться.

`max-w-[1920px]` на DashboardPage сохраняем.

---

## 6. Компоненты — детали с примерами

### 6.1 Card / Surface primitives

`src/components/ui/card.tsx` — обновить:

```tsx
<div className={cn(
  "rounded-lg border border-subtle bg-surface-1",
  "transition-colors duration-standard",
  className
)}>
  {children}
</div>
```

Никаких теней. Hover у интерактивных карточек: `hover:border-strong`. Pressed: `active:bg-surface-2`.

CardHeader — `px-4 py-3 border-b border-subtle`. CardContent — `p-4`. CardFooter — `px-4 py-3 border-t border-subtle`.

### 6.2 Buttons

`src/components/ui/button.tsx` — варианты:

| Variant | Стиль |
|---------|-------|
| `default` | `bg-accent text-accent-foreground hover:bg-accent-hover active:bg-accent-active` |
| `secondary` | `bg-surface-2 text-foreground hover:bg-surface-3 border border-subtle` |
| `ghost` | `bg-transparent text-foreground-muted hover:bg-surface-2 hover:text-foreground` |
| `outline` | `border border-strong bg-transparent hover:bg-surface-2` |
| `destructive` | `bg-danger text-foreground hover:opacity-90` |
| `link` | `text-accent underline-offset-4 hover:underline` |

Размеры: `sm` (32px), `default` (36px), `lg` (44px), `icon` (квадрат 36px).

Focus-ring: `focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-accent/40 focus-visible:ring-offset-2 focus-visible:ring-offset-background`.

Hover scale-on-press: `active:scale-[0.98] transition-transform duration-fast ease-spring` — лёгкий «нажим». Только для `default` и `destructive` (главные действия), остальные без scale.

### 6.3 Inputs

`src/components/ui/input.tsx`:

```tsx
className={cn(
  "h-9 w-full rounded-md border border-subtle bg-surface-2 px-3 py-1",
  "text-body text-foreground placeholder:text-foreground-faint",
  "focus-visible:outline-none focus-visible:border-accent focus-visible:ring-2 focus-visible:ring-accent/30",
  "disabled:cursor-not-allowed disabled:opacity-50",
  "transition-colors duration-fast",
  className
)}
```

### 6.4 Badges

`src/components/ui/badge.tsx`:
- `default`: `bg-surface-2 text-foreground-muted border border-subtle px-2 py-0.5 rounded-pill text-micro uppercase tracking-wider`
- `accent`: `bg-accent/10 text-accent border border-accent/20`
- `success / warning / danger / info`: `bg-{color}/10 text-{color} border border-{color}/20`
- `dot` (вариант с цветной точкой): `<span className="h-1.5 w-1.5 rounded-full bg-current mr-1.5" /> {label}`

### 6.5 Tables

`src/components/ui/table.tsx`:
- Заголовок: `bg-surface-2 text-foreground-muted text-micro uppercase tracking-wider`
- Строки: `border-b border-subtle hover:bg-surface-2/40 transition-colors duration-fast`
- Числовые ячейки: `font-mono tabular-nums text-right`
- Текстовые: `text-body`

Никаких чередующихся фонов (zebra). Только тонкие разделители строк.

### 6.6 Tooltips, Dialogs, Scroll-area

- **Tooltip:** `bg-surface-3 text-foreground border border-subtle rounded-md px-2 py-1 text-small shadow-lg`. Delay 300ms. `data-[state=delayed-open]:animate-fade-in-up`.
- **Dialog:** overlay `bg-background/80 backdrop-blur-sm`, content `bg-surface-1 border border-subtle rounded-lg shadow-2xl max-w-lg`.
- **ScrollArea (radix):** thumb — `bg-border-strong rounded-full`, ширина 4px, hover 6px.

### 6.7 FsmBadge + FsmTimeline

**`FsmBadge.tsx`** — переписать:

```tsx
<div className="flex items-center gap-2 rounded-pill bg-surface-2 border border-subtle px-3 py-1">
  <span
    className="h-2 w-2 rounded-full animate-pulse-soft"
    style={{ backgroundColor: FSM_COLORS[state] }}
  />
  <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
    {state}
  </span>
  {targetColour && (
    <span className="text-micro text-foreground-faint">
      → {COLOUR_RU[targetColour]}
    </span>
  )}
</div>
```

Анимация `pulse-soft` — мягкий pulse точки (1.6s loop). Для `IDLE` — без pulse (статичная opacity 1).

**Новый компонент `FsmTimeline.tsx`** — 7 сегментов в линию:

```tsx
const STATES: FsmState[] = ['IDLE','SEARCHING','TARGETING','APPROACHING','GRABBING','CALLING','RETURNING'];
const SHORT = { IDLE: 'IDL', SEARCHING: 'SRCH', TARGETING: 'TGT', APPROACHING: 'APR', GRABBING: 'GRB', CALLING: 'CAL', RETURNING: 'RTN' };

export function FsmTimeline({ current }: { current: FsmState }) {
  const idx = STATES.indexOf(current);
  return (
    <div className="flex items-center gap-1">
      {STATES.map((s, i) => {
        const isActive = i === idx;
        const isPast = i < idx;
        return (
          <div key={s} className="flex flex-col items-center gap-1">
            <div className={cn(
              "h-1 w-8 rounded-full transition-colors duration-standard",
              isActive && "bg-accent",
              isPast && "bg-foreground-muted",
              !isActive && !isPast && "bg-surface-3"
            )} />
            <span className={cn(
              "font-mono text-micro tracking-wider",
              isActive && "text-accent",
              !isActive && "text-foreground-faint"
            )}>{SHORT[s]}</span>
          </div>
        );
      })}
    </div>
  );
}
```

Размещение: рядом с `FsmBadge` в DashboardPage и AdminPage. Можно вместо длинного баджа.

### 6.8 CommandInput — терминальный режим

`src/components/controls/CommandInput.tsx` — переписать:

```tsx
<div className="group flex items-center gap-2 rounded-md border border-subtle bg-surface-1 px-3 py-2 transition-colors focus-within:border-accent">
  <span className="font-mono text-body text-accent select-none">&gt;</span>
  <input
    type="text"
    placeholder="найди красный мяч"
    className={cn(
      "flex-1 bg-transparent border-none outline-none",
      "font-mono text-body text-foreground placeholder:text-foreground-faint",
      "focus:ring-0"
    )}
    onKeyDown={(e) => { if (e.key === 'Enter') onSend(value); }}
  />
  <kbd className="font-mono text-micro text-foreground-faint border border-subtle rounded px-1.5 py-0.5">↵</kbd>
</div>
```

При фокусе курсор-индикатор `>` мигает (CSS `@keyframes blink` 1s steps(2) infinite). При отправке — короткое scale-pulse контейнера через `animate-fade-in-up` reverse.

### 6.9 CameraFeed — HUD overlay + scanline

`src/components/camera/CameraFeed.tsx` — обернуть в overlay-контейнер:

```tsx
<div className="relative aspect-video overflow-hidden rounded-lg border border-subtle bg-surface-2">
  <img src="/video_feed" alt="camera" className="h-full w-full object-cover" />

  {/* Scanline (CSS keyframes, opacity 8%) */}
  <div className="pointer-events-none absolute inset-0 overflow-hidden">
    <div className="absolute inset-x-0 h-px bg-accent/20 animate-scanline" />
  </div>

  {/* HUD top-left: FSM badge */}
  <div className="absolute top-2 left-2">
    <FsmBadge state={state} compact />
  </div>

  {/* HUD top-right: timestamp */}
  <div className="absolute top-2 right-2 font-mono text-micro text-foreground-muted bg-background/60 backdrop-blur-sm rounded px-2 py-1">
    {formatTime(now)}
  </div>

  {/* HUD bottom-right: connection status */}
  <div className="absolute bottom-2 right-2 flex items-center gap-1.5 bg-background/60 backdrop-blur-sm rounded-full px-2 py-1">
    <span className={cn("h-1.5 w-1.5 rounded-full", connected ? "bg-success animate-pulse-soft" : "bg-danger")} />
    <span className="font-mono text-micro text-foreground-muted">{connected ? 'LIVE' : 'OFFLINE'}</span>
  </div>

  {/* Detection bboxes overlay (если backend отдаёт detections с координатами в нормализованных 0..1) */}
  {detections.map(d => (
    <div
      key={d.id}
      className="absolute border border-accent/60 bg-accent/5 transition-all duration-fast"
      style={{ left: `${d.x*100}%`, top: `${d.y*100}%`, width: `${d.w*100}%`, height: `${d.h*100}%` }}
    >
      <span className="absolute -top-5 left-0 font-mono text-micro text-accent bg-background/80 px-1.5 rounded">
        {d.colour} · {(d.conf*100).toFixed(0)}%
      </span>
    </div>
  ))}
</div>
```

**Важно:** если backend сейчас не отдаёт normalized bbox координат — сохранить визуальный bbox-блок как опциональный (рендерить только если `detections[i].x` существует). Не ломать существующий поток данных.

### 6.10 MapCanvas — палитра + toggle'ы

`src/hooks/useMapCanvas.ts` — **НЕ трогать логику**. Менять только цветовые константы. Если они захардкожены внутри хука — поднять в проп `theme: MapTheme` или CSS-переменные через `getComputedStyle(document.documentElement).getPropertyValue('--accent')`.

**Вариант А (предпочтительный):** добавить в `MapDrawData` опциональное поле `theme`, читаемое родителем из CSS-переменных, передавать в хук. Это **расширение интерфейса**, не изменение существующей сигнатуры — обратно-совместимо.

**Цвета (HSL → передавать как rgba для canvas):**
- map PNG — `mix-blend-mode: multiply` через CSS на canvas, либо tint через ImageData (если хук сам тинит — захардкодить tint = `surface-2` 80% непрозрачности)
- robot — coral круг (radius 8) + coral стрелка (length 14)
- planned path — `mint #7DD3A8` пунктир (dash [6, 4])
- forbidden zones — `rose #C97B7B` пунктир + fill `#C97B7B` 8%
- scanpoints — `--foreground-muted` 30%

**MapToolbar.tsx** — добавить toggle-кнопки:
- `[#] Сетка` — рисует grid поверх карты, шаг = 0.5м (преобразовать в пиксели по `MapInfo.resolution`), цвет `--surface-3`, opacity 40%
- `[≋] Покрытие` — heatmap покрытия (там где робот уже был). Берём данные из `pathRecorder` или одометрии, render как coral fade (alpha по плотности).

### 6.11 Joystick — технический + readout вектора

`src/hooks/useJoystick.ts` — НЕ трогать.

`src/components/joystick/JoystickControl.tsx` (или `controls/JoystickControl.tsx` — у тебя дубль, см. frontend.md, **выбрать один и оставить**, удалить дубль не надо если есть импорты в обоих местах — оставить оба, но привести к одному визуалу):

```tsx
<div className="flex flex-col items-center gap-3">
  <div
    ref={containerRef}
    onPointerDown={startJoy} onPointerMove={moveJoy} onPointerUp={endJoy}
    className="joystick-area relative h-48 w-48 rounded-full border border-subtle bg-surface-2"
  >
    {/* Кросс-метки (SVG inline) */}
    <svg viewBox="0 0 192 192" className="absolute inset-0 pointer-events-none">
      <line x1="96" y1="20" x2="96" y2="172" stroke="hsl(var(--surface-3))" strokeWidth="1" />
      <line x1="20" y1="96" x2="172" y2="96" stroke="hsl(var(--surface-3))" strokeWidth="1" />
      <circle cx="96" cy="96" r="60" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="1" strokeDasharray="3 4" />
    </svg>

    {/* Heading vector — линия от центра к knob */}
    {state.active && (
      <svg viewBox="0 0 192 192" className="absolute inset-0 pointer-events-none">
        <line
          x1="96" y1="96"
          x2={96 + state.dx} y2={96 + state.dy}
          stroke="hsl(var(--accent))" strokeWidth="2" strokeLinecap="round"
        />
      </svg>
    )}

    {/* Knob */}
    <div
      className={cn(
        "absolute h-11 w-11 rounded-full bg-surface-3 border-2 border-accent",
        "transition-transform duration-standard",
        state.active ? "ease-standard" : "ease-spring" // на release — overshoot обратно к центру
      )}
      style={{
        left: `calc(50% - 22px + ${state.dx}px)`,
        top:  `calc(50% - 22px + ${state.dy}px)`,
      }}
    />
  </div>

  {/* Readout */}
  <div className="font-mono text-small text-foreground-muted tabular-nums">
    <span className={cn(state.linear !== 0 && "text-accent")}>{state.linear.toFixed(2)}</span>
    <span className="mx-1.5 text-foreground-faint">m/s</span>
    <span className="mx-2 text-foreground-faint">·</span>
    <span className={cn(state.angular !== 0 && "text-accent")}>{state.angular.toFixed(2)}</span>
    <span className="ml-1.5 text-foreground-faint">rad/s</span>
  </div>
</div>
```

**Spring-feel на release** — через `transition-timing-function` переключаемый между `ease-standard` (drag) и `ease-spring` (release). Никаких JS animation libs.

### 6.12 Charts — Recharts restyle

`src/components/charts/SensorCharts.tsx`. Recharts позволяет настроить стили через props:

```tsx
<LineChart data={data}>
  <CartesianGrid stroke="hsl(var(--surface-3))" strokeDasharray="0" vertical={false} />
  <XAxis dataKey="t" tick={{ fill: 'hsl(var(--foreground-faint))', fontSize: 10, fontFamily: 'JetBrains Mono' }} stroke="hsl(var(--surface-3))" />
  <YAxis tick={{ fill: 'hsl(var(--foreground-faint))', fontSize: 10, fontFamily: 'JetBrains Mono' }} stroke="hsl(var(--surface-3))" />
  <Tooltip
    contentStyle={{ background: 'hsl(var(--surface-3))', border: '1px solid hsl(var(--border-subtle))', borderRadius: '8px', fontFamily: 'JetBrains Mono', fontSize: '12px' }}
    labelStyle={{ color: 'hsl(var(--foreground-muted))' }}
    itemStyle={{ color: 'hsl(var(--foreground))' }}
  />
  <Legend wrapperStyle={{ fontSize: '11px', color: 'hsl(var(--foreground-muted))' }} />
  <Line type="monotone" dataKey="battery" stroke="hsl(var(--accent))" strokeWidth={1.5} dot={false} />
  <Line type="monotone" dataKey="temp"    stroke="hsl(var(--warning))" strokeWidth={1.5} dot={false} />
  <Line type="monotone" dataKey="speed"   stroke="hsl(var(--info))"    strokeWidth={1.5} dot={false} />
</LineChart>
```

**Правила:**
- Grid — только горизонтальный, `stroke-width: 1px`, цвет `surface-3`.
- Линии — `1.5px`, без точек на каждом сэмпле (`dot={false}`).
- Только активная точка под курсором — `activeDot={{ r: 3, fill: 'hsl(var(--accent))' }}`.
- Палитра серий: accent (главная) + warning + info + success — максимум 4 серии в одном чарте.

### 6.13 3D scene — Visualization3DPage

`src/pages/Visualization3DPage.tsx` + компоненты `src/components/3d/*`.

**Сцена:**
- `<color attach="background" args={['#1F1E1D']} />` (или из CSS-vars через JS)
- Сетка (`<gridHelper>`): args `[100, 200]`, colors `'#2D2B28'` x2
- Робот (RobotModel) — material color `#CC785C` (accent), metalness 0.1, roughness 0.6
- Оси (`<axesHelper>`) — colors заменить на: X=`#C97B7B` (danger/red rose), Y=`#7DA88A` (success/green sage), Z=`#7DA1C9` (info/blue slate)
- PathTrail — `LineBasicMaterial` color `#CC785C`, opacity 0.8
- PlannedPathTrail — color `#7DD3A8` (mint), dashed
- SlamMap3D облако — `#A8A39A` (foreground-muted), size 0.02

**Освещение** (заменить):
- Убрать 3 directional, оставить 1 directional слабый (intensity 0.3) для теней
- Добавить 2 rim light: один справа сзади (`color #CC785C`, intensity 0.4), один слева спереди (`color #E8E5DD`, intensity 0.3) — «студийная» подача
- Ambient: `#1F1E1D` intensity 0.6
- HemisphereLight: skyColor `#34322F`, groundColor `#1F1E1D`, intensity 0.4

**Fog:**
```tsx
<fog attach="fog" args={['#1F1E1D', 8, 30]} />
```
Глубина при больших расстояниях.

**InfoPanel.tsx** — overlay внутри Canvas, переводим на новые токены: `bg-surface-1/80 backdrop-blur-md border border-subtle rounded-lg p-4 font-mono text-small`.

### 6.14 Branding — KatanaIcon

Создать `src/components/icons/KatanaIcon.tsx` — inline SVG в стиле lucide (1.5px stroke, 24px viewBox):

```tsx
export function KatanaIcon({ className, ...props }: SVGProps<SVGSVGElement>) {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" className={className} {...props}>
      {/* Стилизованная катана: рукоять + цуба + клинок */}
      <path d="M3 21 L8 16" />          {/* рукоять (короткая диагональ) */}
      <path d="M7 17 L9 15 M9 19 L11 17" /> {/* цуба */}
      <path d="M9 15 L21 3" />           {/* клинок */}
      <path d="M19 5 L21 3 L21 5" />     {/* остриё (треугольник) */}
    </svg>
  );
}
```

(Это набросок — Claude может уточнить пропорции, главное — monoline, 24px viewBox, currentColor stroke. Альтернативно — стилизованный самурайский шлем `kabuto`. Решение за Claude в рамках стиля.)

Используется в Sidebar (header), favicon (заменить `samurai.ico`/`samurai_256.png` рендером SVG → PNG в `public/`, оставить fallback на старый PNG если не получится).

Для **Samcan** — отдельная иконка `BotIcon` (или lucide `Bot`), без своего акцента (Q16=A — единый стиль).

### 6.15 Иконки — Lucide + кастомные SVG

**Lucide остаётся** для всего стандартного:
- Навигация: `LayoutDashboard, Sliders, Box, Cpu, Bot, PanelLeftClose, PanelLeftOpen, Sun, Moon`
- Действия: `Play, Pause, Square, RotateCcw, Trash2, Plus, Minus, X, Check`
- Сенсоры/связь: `Wifi, WifiOff, Battery, Thermometer, Signal, AlertTriangle, ShieldAlert`

**Кастомные SVG-иконки** (создать в `src/components/icons/`):
- `KatanaIcon.tsx` — брендинг
- `JoystickIcon.tsx` — для контролов
- `ClawIcon.tsx` (gripper) — для actuators (open/close states)
- `FsmStateIcon.tsx` — 7 миниатюр для FSM-состояний (опционально, если нужен символ кроме pill)

Все — 1.5px stroke, 24px viewBox, `currentColor`.

Везде — единая толщина `strokeWidth=1.5`, размеры через Tailwind `h-4 w-4` (16px) для inline / `h-5 w-5` (20px) для кнопок / `h-6 w-6` (24px) для sidebar.

---

## 7. Анимации и motion (CSS only)

### 7.1 Принципы
- **Нет JS animation libs.** Всё через CSS transitions, keyframes, Tailwind utilities.
- **Spring-feel** — через `cubic-bezier(0.34, 1.56, 0.64, 1)` (overshoot easing) на transform. Дёшево и достаточно.
- **Pulse** — для активных индикаторов: FSM-точка, LIVE-индикатор камеры, recording-точка path recorder.
- **Fade-in-up** — для появления новых элементов: тосты, новая запись в EventLog, popover.
- **Scanline** — только на CameraFeed, opacity 8%, period 4s. Один на весь экран.

### 7.2 Что **должно** анимироваться
- Hover у кнопок: `transition-colors duration-fast` + scale `active:scale-[0.98]` на primary/destructive
- Focus у inputs: `transition-colors duration-fast` (border + ring)
- FSM badge точка: `animate-pulse-soft` (если state ≠ IDLE)
- FSM timeline сегмент: `transition-colors duration-standard` при смене активного
- Sidebar collapse: `transition-[width] duration-standard`
- Theme toggle: cross-fade иконок Sun/Moon, `transition-opacity duration-standard`
- Joystick knob release: `transition-transform duration-standard ease-spring`
- Camera scanline: `animate-scanline` (4s linear infinite)
- Camera LIVE dot: `animate-pulse-soft` если connected
- Detection bbox fade-in: `animate-fade-in-up` на mount
- EventLog новая запись: `animate-fade-in-up` (только последняя, остальные статичны)

### 7.3 Что **НЕ должно** анимироваться
- Загрузка карты, сенсорные графики (никаких «прогрессивных растущих линий»)
- Числа в readout (никаких «count-up» эффектов — числа меняются 5-10 раз/сек, анимация мешает)
- Текст в EventLog (только сам контейнер fade-in)
- Hover у пассивных элементов (карточек, лейблов)

### 7.4 `prefers-reduced-motion`

Глобально в `index.css`:
```css
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

---

## 8. Page-by-page — что меняется

### 8.1 DashboardPage (`src/pages/DashboardPage.tsx`)

**Layout:** 3 колонки `grid grid-cols-12 gap-4 p-6 max-w-[1920px] mx-auto`:
- Колонка 1 (cols 1-4): CameraFeed (с HUD), CommandInput, QuickCommandButtons, DetectionBanner, BallsTable
- Колонка 2 (cols 5-8): MapCanvas + MapToolbar, SensorCharts, EventLog
- Колонка 3 (cols 9-12): SamuraiStatusBanner (компактнее), FsmBadge + FsmTimeline (новое), SensorPanel, ActuatorToggles, GamepadController, LedPanel, DetectionTogglePanel, PathRecorderPanel

**Изменения:**
- `SamuraiStatusBanner` урезается — battery/connection переходят в SensorPanel и Sidebar (connection там), баннер показывает только активный robot_id и FSM state бейджем
- Добавить `FsmTimeline` сразу под `FsmBadge`
- `EventLog` — fade-in новые записи, моноспейс таймстампы, `bg-surface-1` без видимой scrollbar (только при hover)

### 8.2 AdminPage (`src/pages/AdminPage.tsx`)

Сохранить inline-хелперы (`PathBtn`, `SpeedBtns`) — переписать только классы.

Добавить FsmTimeline в верхнюю секцию. ServoControlPanel — слайдеры в новом стиле (см. §6.x — нужно добавить `Slider` в `ui/`, на radix Slider).

EmergencyStop — крупная кнопка `bg-danger text-foreground hover:bg-danger/90`, центральная позиция, scale-pulse при наведении (`hover:scale-[1.02] active:scale-[0.98]`).

### 8.3 Visualization3DPage (`src/pages/Visualization3DPage.tsx`)

3D-сцена — см. §6.13. InfoPanel переезжает в правый верхний угол (overlay), стили карточки.

Тогглы (EKF, clear) — в нижнем-правом углу, ghost-buttons на полупрозрачном фоне.

Header страницы — заголовок «3D View» + sim time + кнопка `Reset position`.

### 8.4 HardwarePage (`src/pages/HardwarePage.tsx`)

Layout остаётся 2-колоночным (PlatformSelector + блоки слева, PresetPanel справа).

`HardwareBlockDiagram` — drag-n-drop сохраняется логически, визуально блоки в стиле `surface-2` карточек с тонкими бордерами, при hover — `border-strong`. При drag — `border-accent` + `cursor-grabbing`.

Сохраняем custom event `'hw-preset-save'` без изменений.

### 8.5 SamcanPage (`src/pages/SamcanPage.tsx`)

Полностью идентичный стиль — те же токены, тот же sidebar, та же Header. Coral акцент сохраняется (никакого sub-accent).

`HeadingCompass`, `DistanceRadar`, `MotorBars`, `TelemetryHistory`, `ArmVisualizer` — кастомные SVG-визуализации, перекрасить под палитру:
- Compass: круг `surface-2`, метки `foreground-faint`, стрелка курса `accent`, фоновое кольцо градусной шкалы `surface-3`
- Radar: дуга `info` (slate blue), активное препятствие `warning` или `danger`, anim-flash `animate-pulse-soft`
- MotorBars: вертикальные полоски `surface-2` фон, заполнение `accent` (вперёд) / `info` (назад)
- ArmVisualizer: stick-фигура линии `foreground-muted`, joints `accent` точки

Inline `Kbd` (как в SamcanPage hint) — переиспользовать стиль из CommandInput (`<kbd>` класс).

`SamcanEventLog` — стиль идентичен `EventLog` Dashboard.

Клавиатурные хинты — `kbd`-стиль (рамка `border-subtle`, `bg-surface-2`, моноспейс).

---

## 9. Light theme — нюансы

Параллельная палитра (см. §3.5). Что отличается визуально:
- Тени: добавить `shadow-sm` карточкам (на кремовом фоне иначе сливаются)
- Border-subtle становится чуть видимее (на кремовом 1px бежевая линия читается)
- Coral акцент темнеет до `#B85F45` (для контраста на светлом)
- Scanline в CameraFeed — opacity 12% (на светлом нужно чуть ярче)
- Фон map PNG — без mix-blend-multiply (карта обычно светлая, не нужно тинить)

**Проверка:** Claude должен на каждой странице переключить тему и убедиться что всё читается. Особое внимание — графики (Recharts), карта, 3D-сцена (фон сцены остаётся тёмным даже в light-теме, или светлым в `#F5F2EA` — на усмотрение, но один из вариантов выбрать и зафиксировать).

**Рекомендация:** 3D-сцена остаётся тёмной всегда (студийный stand). В light-теме — она просто «врезана» в светлый интерфейс как чёрная панель.

---

## 10. Deliverables — что Claude должен закоммитить

**Новые файлы:**
- `src/components/layout/Sidebar.tsx`
- `src/components/layout/ThemeToggle.tsx`
- `src/components/fsm/FsmTimeline.tsx`
- `src/components/icons/KatanaIcon.tsx`
- `src/components/icons/JoystickIcon.tsx` (если нужен — иначе lucide)
- `src/components/icons/ClawIcon.tsx`
- `src/components/camera/HudOverlay.tsx` (или встроено в CameraFeed.tsx)
- `src/components/camera/Scanline.tsx` (CSS only компонент)

**Сильно меняются:**
- `src/index.css` (полная замена палитры + keyframes)
- `tailwind.config.ts` (extend.colors, fontSize, animation, etc.)
- `index.html` (fonts link, title, theme color meta)
- `src/App.tsx` (вместо Header — Sidebar + Layout wrapper)
- `src/components/layout/Header.tsx` (превращается в page-header)
- `src/components/ui/*` (все primitives — новые классы)
- `src/components/camera/CameraFeed.tsx` (HUD)
- `src/components/fsm/FsmBadge.tsx` (новый дизайн)
- `src/components/controls/CommandInput.tsx` (терминальный стиль)
- `src/components/joystick/JoystickControl.tsx` + `src/components/controls/JoystickControl.tsx` (если оба используются — оба к одному стилю)
- `src/components/charts/SensorCharts.tsx` (Recharts restyle)
- `src/components/map/MapToolbar.tsx` (новые toggle'ы)
- `src/components/3d/*` (палитра, освещение, fog)
- Все 5 `src/pages/*Page.tsx` (новый layout с Sidebar)
- `src/components/samcan/*` (адаптация SVG-визуализаций под палитру)

**Новые вспомогательные файлы стилей (не constants):**
- `src/components/fsm/fsm-styles.ts` — `FSM_TEXT_CLASS`, `FSM_BG_CLASS` маппинги (см. §3.3)
- `src/components/detection/ball-styles.ts` — `BALL_BG_CLASS`, `BALL_TEXT_CLASS` маппинги (см. §3.4)

**НЕ должны измениться:**
- Все `src/hooks/*`
- Все `src/providers/*`
- `src/lib/api.ts`
- `src/lib/constants.ts` (значения и ключи остаются)
- Все `src/types/*`
- `vite.config.ts`
- `compute_node/*.py`

**Один коммит или несколько — на усмотрение Claude** (Q24=D). Если несколько — логично разбить так:
1. `redesign(tokens): css vars, tailwind config, fonts`
2. `redesign(layout): sidebar, theme toggle, header restructure`
3. `redesign(primitives): shadcn ui/* update`
4. `redesign(widgets): camera HUD, fsm timeline, command input, joystick, charts`
5. `redesign(pages): apply layout to all 5 pages`
6. `redesign(3d): scene palette and lighting`

Каждый коммит должен оставлять `npm run build` зелёным.

---

## 11. Acceptance — verification

### 11.1 Build & lint (обязательно)
```bash
cd compute_node/frontend
npm run build       # должен пройти без ошибок и без новых warnings
npx tsc --noEmit    # должен пройти без ошибок типов
npm run lint        # ESLint без ошибок (если есть npm-script)
```

### 11.2 Routes & components (обязательно)
- Все 5 маршрутов открываются без console-ошибок и без React errors
- Все `~60` компонентов рендерятся (не должно быть пустых мест где раньше что-то было)
- Все REST-вызовы проходят (Network tab — 200/204 для всех методов из `lib/api.ts`)
- Socket.IO `state_update` обновляет UI (FSM, sensors, detections — всё реагирует)

### 11.3 Screenshots (Q25=D — обязательно)

Через preview tools или ручные скриншоты собрать в `docs/redesign/screenshots/`:
- `dashboard-dark.png`, `dashboard-light.png`
- `admin-dark.png`, `admin-light.png`
- `3d-dark.png`, `3d-light.png`
- `hardware-dark.png`, `hardware-light.png`
- `samcan-dark.png`, `samcan-light.png`
- `sidebar-expanded.png`, `sidebar-collapsed.png`
- `fsm-states.png` — все 7 FSM-состояний на одном скриншоте (например, в timeline)
- `ball-colors.png` — все 7 цветов мячей (badges)

### 11.4 Light/dark проверка (обязательно)
На каждой странице переключить тему — оба варианта читаемые, контрастные, без «провалов» текста.

### 11.5 Dev-checklist (Q25=D — обязательно)

Claude должен пройти этот список и отчитаться по каждому пункту в финальном комментарии:

**Tokens & базовое:**
- [ ] Шрифты Inter и JetBrains Mono подключены через Google Fonts
- [ ] Все CSS-переменные определены в `:root` и `.light`
- [ ] `body` имеет `font-variant-numeric: tabular-nums`
- [ ] Tailwind extend цвета доступны как `bg-accent`, `text-foreground-muted`, `border-subtle`
- [ ] `cubic-bezier(0.34, 1.56, 0.64, 1)` доступен как `ease-spring`

**Layout & navigation:**
- [ ] Sidebar отображается на всех 5 страницах
- [ ] Активный маршрут подсвечен coral-полоской слева
- [ ] Sidebar складывается до 56px и разворачивается обратно
- [ ] Состояние sidebar сохраняется в `localStorage`
- [ ] ThemeToggle переключает `light` класс на `<html>`
- [ ] Тема сохраняется в `localStorage`
- [ ] Header стал page-header с заголовком страницы и контекстными действиями

**FSM:**
- [ ] FsmBadge — pill с пульсирующей точкой (кроме IDLE)
- [ ] FsmTimeline — 7 сегментов, активный coral, пройденные muted, будущие тихие
- [ ] FSM-цвета — приглушённая палитра, TARGETING = coral

**Camera:**
- [ ] CameraFeed имеет HUD-overlay (FSM, timestamp, LIVE-dot)
- [ ] Scanline-эффект работает, opacity ≤12%
- [ ] Detection bbox рендерятся (если backend отдаёт)

**Map:**
- [ ] Robot — coral круг + стрелка
- [ ] Path — mint пунктир
- [ ] Forbidden zones — rose пунктир
- [ ] MapToolbar имеет toggle сетки и toggle покрытия

**Joystick:**
- [ ] Технический визуал (кросс + кольца внутри круга)
- [ ] Heading vector рисуется при активном касании
- [ ] Numeric readout `m/s · rad/s` под джойстиком, моноспейс, tabular-nums
- [ ] Spring-feel при release (cubic-bezier overshoot)

**CommandInput:**
- [ ] Префикс `>` слева, моноспейс
- [ ] kbd-индикатор `↵` справа

**Charts:**
- [ ] Recharts перерисованы (без рамок, тонкие линии, моноспейс осей)
- [ ] Tooltip в стиле карточки

**3D:**
- [ ] Robot model — coral
- [ ] Сцена тёмная даже в light-теме (или зафиксировать решение)
- [ ] Rim-lights вместо 3 directionals
- [ ] Fog активен

**Branding:**
- [ ] KatanaIcon (или новая иконка) в Sidebar
- [ ] Favicon обновлён (или fallback к старому)
- [ ] Wordmark «Samurai» в новой типографике

**Анимации:**
- [ ] `prefers-reduced-motion` уважается (анимации выключаются)
- [ ] Scanline не блокирует pointer-события на камере
- [ ] Pulse только на активных индикаторах

**Контракт:**
- [ ] Ни один файл из `src/hooks/` не изменён
- [ ] Ни один файл из `src/providers/` не изменён
- [ ] `src/lib/api.ts` не изменён
- [ ] `src/types/*` не изменены
- [ ] `vite.config.ts` не изменён
- [ ] `package.json` не получил новых runtime-зависимостей (кроме шрифтов в `index.html`)
- [ ] Все 5 маршрутов работают
- [ ] Все REST-вызовы из `lib/api.ts` отрабатывают (проверка Network tab)
- [ ] Socket.IO state_update обновляет UI

---

## 12. Non-goals — повтор для дисциплины

Чтобы не было соблазна «улучшить попутно»:

- ❌ Не рефакторить хуки и провайдеры
- ❌ Не переименовывать компоненты, props, экспортируемые типы
- ❌ Не менять REST endpoints или Socket.IO события
- ❌ Не добавлять новые runtime-зависимости (особенно — никаких animation libs)
- ❌ Не убирать существующие компоненты (даже если кажется что не нужны)
- ❌ Не менять ширину `max-w-[1920px]` — оператор работает на больших экранах
- ❌ Не делать «pure rewrite» — менять минимум JSX чтобы достичь нового вида
- ❌ Не трогать `compute_node/*.py` — backend остаётся как есть
- ❌ Не изменять структуру маршрутов (`/dashboard`, `/admin`, etc.)

Если по ходу обнаружится что-то требующее изменений за этим списком — **остановиться, спросить пользователя**.

---

## Appendix A — карта ключевых файлов

| Зона | Файлы |
|------|-------|
| Tokens | `src/index.css`, `tailwind.config.ts`, `index.html` |
| Layout | `src/App.tsx`, `src/components/layout/{Header,Sidebar,ThemeToggle,RobotSelector,SamuraiStatusBanner}.tsx` |
| UI primitives | `src/components/ui/{button,card,input,badge,table,tooltip,dialog,scroll-area,separator,progress,toggle,slot}.tsx` |
| Camera | `src/components/camera/CameraFeed.tsx` (+ новые HudOverlay, Scanline) |
| FSM | `src/components/fsm/{FsmBadge,FsmStatePanel,FsmTransitionButtons}.tsx` (+ новый FsmTimeline) |
| Map | `src/components/map/{MapCanvas,MapToolbar,MapManagerPanel}.tsx` |
| Charts | `src/components/charts/SensorCharts.tsx` |
| Joystick | `src/components/joystick/JoystickControl.tsx`, `src/components/controls/JoystickControl.tsx` (если оба) |
| 3D | `src/components/3d/*.tsx` |
| Pages | `src/pages/{Dashboard,Admin,Visualization3D,Hardware,Samcan}Page.tsx` |
| Samcan widgets | `src/components/samcan/*.tsx` |
| Constants | `src/lib/constants.ts` (только значения цветов) |
| Icons | `src/components/icons/*.tsx` (новая папка) |

---

## Appendix B — порядок работы (рекомендация)

Это **рекомендация**, не жёсткое требование (Q24=D — Claude сам решает).

1. **Tokens & базовое** — `index.css` + `tailwind.config.ts` + `index.html`. Проверить что build зелёный, страница рендерится (хоть и «слегка кривой» из-за того что компоненты ещё в старом стиле).
2. **Layout & navigation** — Sidebar + ThemeToggle + Header restructure + App.tsx wrapper. Проверить что все 5 маршрутов открываются.
3. **UI primitives** — `components/ui/*`. После этого все базовые элементы выглядят в новом стиле, страницы автоматически «подхватывают» новый вид.
4. **Branding** — KatanaIcon + favicon.
5. **Widgets первого слоя** — FsmBadge + FsmTimeline + CommandInput + Sidebar довести до конца.
6. **Camera** — HUD + scanline.
7. **Map** — палитра + toggle'ы.
8. **Joystick** — технический + readout.
9. **Charts** — Recharts restyle.
10. **3D scene** — палитра + освещение.
11. **Page-by-page review** — пройтись по каждой из 5 страниц, отполировать раскладку.
12. **Light theme проверка** — все 5 страниц в светлой теме, скриншоты.
13. **Dev-checklist** — пройтись по списку из §11.5, отчитаться.

После каждого шага — `npm run build` зелёный, `git commit`.

---

## Appendix C — что делать в неоднозначных ситуациях

- **Если палитра не «звучит» на конкретной странице** — Claude может предложить корректировку токена (например, `--surface-2` чуть светлее), но **не менять без согласования**.
- **Если в существующем коде обнаружен баг (не связанный с дизайном)** — оставить TODO-комментарий, не чинить (мы редизайн делаем, не bugfix).
- **Если нужно расширить пропсы компонента для нового визуала** — допустимо **только если это обратно-совместимо** (новый проп опционален с дефолтом). Любое breaking change — стоп, спросить.
- **Если `useMapCanvas` нужно расширить для toggle'ов сетки/heatmap** — добавить опциональные поля в `MapDrawData`, не менять сигнатуру `useMapCanvas(canvasRef, data)`.
- **Если рамки `package.json` мешают** (нужен новый цвет/иконка/анимация) — почти всегда есть CSS-only решение. Если нет — **спросить**.

---

**Конец промта.** Длина ~700 строк. Ветка `feat/redesign`. Удачи.
