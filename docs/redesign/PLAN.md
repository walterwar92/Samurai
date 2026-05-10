# UI Redesign — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Перенести дизайн из `docs/redesign/preview/` в боевой фронт `compute_node/frontend/` согласно спеке `docs/redesign/PROMPT.md`, сохранив весь функционал.

**Architecture:** Поэтапный визуальный порт. Сначала фундамент (токены/шрифты), потом раскладка (sidebar + page-header), затем primitives, центральные виджеты (FSM, камера, карта, джойстик, графики, 3D), наконец сборка пяти страниц и финальная проверка обеих тем + dev-checklist.

**Tech Stack:** React 19 + TypeScript + Vite 7 + Tailwind 3 + shadcn/ui (radix primitives) + Recharts + three.js/R3F + Zustand (state) + Socket.IO (transport). Тесты: Vitest + @testing-library/react + happy-dom (уже настроены).

**Branch:** `feat/redesign` (текущая, от `dev`).

---

## Реальность vs PROMPT.md (поправки на берегу)

При проверке боевого фронта обнаружилось:
- **Маршрутов 6, не 5** — есть `/mps` (учебный модуль курсовой Козлова). Включаем в sidebar, обновляем `MpsPage` визуально (токены), но содержимое (KaTeX-формулы, ODE-карточки) трогаем минимально — это reading-friendly страница со своей логикой.
- **State не через `SocketProvider`**, а через **Zustand `useRobotStore`** (`src/stores/robotStore.ts` + `src/stores/selectors.ts`). PROMPT.md упоминает `providers/SocketProvider` — на деле его нет. Дополняем block-list: **`src/stores/*` тоже не трогаем**.
- **Header.tsx не используется глобально в App.tsx** — каждая страница может рендерить его сама (или не рендерить). Решаем: убираем старый Header, делаем Sidebar + PageHeader через layout-обёртку.
- **DashboardPage — eager**, остальные страницы lazy через `React.lazy`. Сохраняем.

---

## Test strategy

Vitest + RTL уже стоят (есть `*.test.tsx` для `mps/*` компонентов). Стратегия по типу таска:

- **Pure helpers** (FSM lookup, ball lookup, theme storage) → TDD: тест вперёд, потом реализация.
- **Visual components** → unit-тест на «не упало + ключевые data-attributes/role/aria», без снапшотов (снапшоты бесполезны при визуальном редизайне). Визуальная проверка через `npm run dev`.
- **Каждый таск завершается:** `npm run build` (зелёный) + `npx tsc --noEmit` (нет ошибок типов) + `npm run lint` (без новых ошибок). Это **обязательный gate** перед коммитом.
- **После каждой фазы:** ручная проверка `npm run dev` → пройти все 6 маршрутов → нет console-ошибок.

Working dir для всех команд: `compute_node/frontend/`.

---

## File map (where things land)

| Зона | Файлы |
|------|-------|
| **Tokens** | `src/index.css` (rewrite), `tailwind.config.ts` (rewrite), `index.html` (fonts) |
| **Icons** | `src/components/icons/{KatanaIcon,index}.tsx` (новые) |
| **Style helpers** | `src/components/fsm/fsm-styles.ts`, `src/components/detection/ball-styles.ts` (новые) |
| **Layout** | `src/components/layout/{Sidebar,ThemeToggle,PageHeader}.tsx` (новые), `Header.tsx` (deprecate/repurpose), `App.tsx` (rewrite) |
| **Primitives** | `src/components/ui/{button,card,input,badge,tooltip,kbd}.tsx` |
| **FSM** | `src/components/fsm/{FsmBadge,FsmTimeline}.tsx` |
| **Centerpieces** | `src/components/camera/CameraFeed.tsx`, `src/components/controls/CommandInput.tsx`, `src/components/joystick/JoystickControl.tsx`, `src/components/map/{MapCanvas,MapToolbar}.tsx` |
| **Charts/3D** | `src/components/charts/SensorCharts.tsx`, `src/components/3d/*.tsx`, `src/pages/Visualization3DPage.tsx` |
| **Pages** | `src/pages/{Dashboard,Admin,Hardware,Samcan,Visualization3D,Mps}Page.tsx` |
| **Samcan widgets** | `src/components/samcan/{HeadingCompass,DistanceRadar,MotorBars,ArmVisualizer}.tsx` |

**Reference (не править, только читать):** `docs/redesign/preview/{components,dashboard,other-pages,app}.jsx` + `docs/redesign/PROMPT.md`.

---

## Phase 0 — Pre-flight

### Task 0: Запустить baseline build

Убедиться что текущий `feat/redesign` собирается до начала работ.

**Files:** none

- [ ] **Step 1: Установить зависимости (если ещё не)**

```bash
cd compute_node/frontend && npm install
```

Expected: установка без ошибок (могут быть warnings — игнорируем).

- [ ] **Step 2: Запустить build**

```bash
npm run build
```

Expected: PASS, артефакты в `dist/` или `compute_node/static/` (см. `vite.config.ts`).

- [ ] **Step 3: Запустить тесты**

```bash
npm test
```

Expected: PASS — существующие mps-тесты проходят (8 файлов).

- [ ] **Step 4: Зафиксировать baseline в памяти**

Никаких коммитов — это только проверка стартовой точки. Если что-то сломано, чинить ДО начала редизайна и спросить пользователя.

---

## Phase 1 — Foundation tokens

### Task 1: Подключить шрифты Inter + JetBrains Mono

**Files:**
- Modify: `compute_node/frontend/index.html`

- [ ] **Step 1: Прочитать текущий `index.html`**

```bash
# Use Read tool on compute_node/frontend/index.html
```

- [ ] **Step 2: Добавить preconnect и link на Google Fonts перед `</head>`**

```html
<link rel="preconnect" href="https://fonts.googleapis.com" />
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin />
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet" />
<meta name="theme-color" content="#1F1E1D" />
```

- [ ] **Step 3: Изменить `<title>` (если он сейчас generic)**

```html
<title>Samurai · Dashboard</title>
```

- [ ] **Step 4: Build check**

```bash
npm run build
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/index.html
git commit -m "redesign(tokens): подключить Inter + JetBrains Mono"
```

---

### Task 2: Перезаписать `src/index.css` с новой палитрой

**Files:**
- Modify: `compute_node/frontend/src/index.css`

- [ ] **Step 1: Прочитать текущий `index.css`**

Запомнить кастомные правила вне `:root`/`.dark` (custom scrollbar, joystick-area).

- [ ] **Step 2: Полностью заменить содержимое**

Контент берём из `docs/redesign/preview/index.html:100-156` (CSS-блок) + `docs/redesign/PROMPT.md §4.1`. Финальный файл:

```css
@tailwind base;
@tailwind components;
@tailwind utilities;

@layer base {
  :root {
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

    /* shadcn aliases */
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
  }
}

@layer base {
  * { @apply border-border; }
  html, body {
    @apply bg-background text-foreground;
    font-family: 'Inter', 'Segoe UI', 'Ubuntu', sans-serif;
    font-variant-numeric: tabular-nums;
    min-height: 100vh;
    overflow-x: hidden;
  }
  code, pre, .font-mono {
    font-family: 'JetBrains Mono', ui-monospace, 'Cascadia Mono', Consolas, monospace;
  }
}

::-webkit-scrollbar { width: 6px; height: 6px; }
::-webkit-scrollbar-track { background: transparent; }
::-webkit-scrollbar-thumb { background: hsl(var(--border-strong)); border-radius: 3px; }
::-webkit-scrollbar-thumb:hover { background: hsl(var(--foreground-faint)); }

.joystick-area { touch-action: none; }

.scanline-bg::before {
  content: '';
  position: absolute;
  inset: 0;
  pointer-events: none;
  background: repeating-linear-gradient(180deg, transparent 0 3px, rgba(255,255,255,.018) 3px 4px);
}

@keyframes pulse-soft {
  0%, 100% { opacity: 1; }
  50%      { opacity: 0.55; }
}
@keyframes scanline {
  0%   { transform: translateY(-100%); opacity: 0; }
  10%  { opacity: 0.16; }
  90%  { opacity: 0.16; }
  100% { transform: translateY(2400%); opacity: 0; }
}
@keyframes fade-in-up {
  from { opacity: 0; transform: translateY(4px); }
  to   { opacity: 1; transform: translateY(0); }
}
@keyframes blink {
  0%, 100% { opacity: 1; }
  50%      { opacity: 0; }
}

@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

- [ ] **Step 3: Build check**

```bash
npm run build
```

Expected: PASS. Страницы будут выглядеть «странно» — компоненты ещё используют старые токены (`bg-card`, `border-border`), но shadcn-aliases в CSS поддерживают совместимость.

- [ ] **Step 4: Запустить dev и убедиться что не упало**

```bash
npm run dev
```

Открыть `http://localhost:5173` (или какой Vite присваивает) → `/dashboard`. Контент рендерится. Цвета пока «между старым и новым» — ОК.

Остановить dev-server (Ctrl+C).

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/index.css
git commit -m "redesign(tokens): новая палитра в index.css (warm graphite + coral)"
```

---

### Task 3: Перезаписать `tailwind.config.ts` с расширенной темой

**Files:**
- Modify: `compute_node/frontend/tailwind.config.ts`

- [ ] **Step 1: Прочитать текущий `tailwind.config.ts`**

Запомнить структуру и что-то специфичное (плагины, contentpath).

- [ ] **Step 2: Заменить содержимое**

```ts
import type { Config } from 'tailwindcss';

export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  darkMode: ['class', '.light'],
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
        fsm: {
          idle:        '#8A8278',
          searching:   '#7DA1C9',
          approaching: '#C9A26B',
          grabbing:    '#B894C9',
          calling:     '#7DA88A',
          returning:   '#9A9089',
        },
        ball: {
          red:    '#B85C5C',
          blue:   '#5C7DC9',
          green:  '#6BA86B',
          yellow: '#C9B05C',
          orange: '#C9885C',
          white:  '#D8D2C8',
          black:  '#2A2826',
        },
        // shadcn aliases (для существующих компонентов)
        card: { DEFAULT: 'hsl(var(--card))', foreground: 'hsl(var(--card-foreground))' },
        popover: { DEFAULT: 'hsl(var(--popover))', foreground: 'hsl(var(--popover-foreground))' },
        primary: { DEFAULT: 'hsl(var(--primary))', foreground: 'hsl(var(--primary-foreground))' },
        secondary: { DEFAULT: 'hsl(var(--secondary))', foreground: 'hsl(var(--secondary-foreground))' },
        muted: { DEFAULT: 'hsl(var(--muted))', foreground: 'hsl(var(--muted-foreground))' },
        destructive: { DEFAULT: 'hsl(var(--destructive))', foreground: 'hsl(var(--destructive-foreground))' },
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
      },
      fontFamily: {
        sans: ['Inter', 'Segoe UI', 'Ubuntu', 'sans-serif'],
        mono: ['"JetBrains Mono"', 'ui-monospace', 'Consolas', 'monospace'],
      },
      fontSize: {
        display:['1.5rem',  { lineHeight:'1.75rem',  letterSpacing:'-0.01em',  fontWeight:'600' }],
        h1:     ['1.125rem',{ lineHeight:'1.5rem',   letterSpacing:'-0.005em', fontWeight:'600' }],
        h2:     ['1rem',    { lineHeight:'1.375rem', letterSpacing:'-0.003em', fontWeight:'500' }],
        body:   ['0.875rem',{ lineHeight:'1.25rem' }],
        small:  ['0.75rem', { lineHeight:'1rem' }],
        micro:  ['0.625rem',{ lineHeight:'0.875rem', letterSpacing:'0.06em',   fontWeight:'500' }],
      },
      borderRadius: {
        sm: '6px',
        DEFAULT: '8px',
        md: '8px',
        lg: '12px',
        xl: '16px',
        pill: '9999px',
      },
      transitionTimingFunction: {
        standard: 'cubic-bezier(0.4, 0, 0.2, 1)',
        spring:   'cubic-bezier(0.34, 1.56, 0.64, 1)',
        swift:    'cubic-bezier(0.4, 0, 1, 1)',
      },
      transitionDuration: {
        fast: '120ms',
        standard: '200ms',
        slow: '320ms',
      },
      animation: {
        'pulse-soft': 'pulse-soft 1.6s ease-in-out infinite',
        'scanline':   'scanline 4s linear infinite',
        'fade-in-up': 'fade-in-up 200ms cubic-bezier(0.34, 1.56, 0.64, 1)',
        'blink':      'blink 1s steps(2) infinite',
      },
    },
  },
  plugins: [],
} satisfies Config;
```

- [ ] **Step 3: Build check**

```bash
npm run build
```

Expected: PASS. Если ошибки про неизвестные классы (`bg-fsm-idle` etc.) — значит компоненты ещё не используют новые классы, это нормально.

- [ ] **Step 4: Typecheck**

```bash
npx tsc --noEmit
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/tailwind.config.ts
git commit -m "redesign(tokens): tailwind config — surface/accent/fsm/ball семейства, fontSize, easing, animations"
```

---

## Phase 2 — Style helpers + icons

### Task 4: Создать style-helpers (fsm-styles + ball-styles) с тестами

**Files:**
- Create: `compute_node/frontend/src/components/fsm/fsm-styles.ts`
- Create: `compute_node/frontend/src/components/fsm/fsm-styles.test.ts`
- Create: `compute_node/frontend/src/components/detection/ball-styles.ts`
- Create: `compute_node/frontend/src/components/detection/ball-styles.test.ts`

- [ ] **Step 1: Написать тест для FSM-стилей**

`src/components/fsm/fsm-styles.test.ts`:
```ts
import { describe, it, expect } from 'vitest';
import { FSM_TEXT_CLASS, FSM_BG_CLASS, FSM_DOT_HEX, FSM_SHORT } from './fsm-styles';

describe('fsm-styles', () => {
  it('TARGETING uses accent token, not own color', () => {
    expect(FSM_TEXT_CLASS.TARGETING).toBe('text-accent');
    expect(FSM_BG_CLASS.TARGETING).toBe('bg-accent');
  });

  it('IDLE uses fsm-idle muted token', () => {
    expect(FSM_TEXT_CLASS.IDLE).toBe('text-fsm-idle');
    expect(FSM_BG_CLASS.IDLE).toBe('bg-fsm-idle');
  });

  it('FSM_DOT_HEX has hex for all 7 states', () => {
    const states = ['IDLE','SEARCHING','TARGETING','APPROACHING','GRABBING','CALLING','RETURNING'] as const;
    for (const s of states) {
      expect(FSM_DOT_HEX[s]).toMatch(/^#[0-9A-F]{6}$/i);
    }
  });

  it('FSM_SHORT shortens names to 3-4 chars uppercase', () => {
    expect(FSM_SHORT.IDLE).toBe('IDL');
    expect(FSM_SHORT.SEARCHING).toBe('SRCH');
    expect(FSM_SHORT.RETURNING).toBe('RTN');
  });
});
```

- [ ] **Step 2: Запустить тест → fail**

```bash
npm test -- fsm-styles
```

Expected: FAIL — `Cannot find module './fsm-styles'`.

- [ ] **Step 3: Реализовать `fsm-styles.ts`**

```ts
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
  SEARCHING:   'bg-fsm-searching',
  TARGETING:   'bg-accent',
  APPROACHING: 'bg-fsm-approaching',
  GRABBING:    'bg-fsm-grabbing',
  CALLING:     'bg-fsm-calling',
  RETURNING:   'bg-fsm-returning',
};

// Hex for places where Tailwind class won't work (inline SVG, canvas, three.js).
export const FSM_DOT_HEX: Record<FsmState, string> = {
  IDLE:        '#8A8278',
  SEARCHING:   '#7DA1C9',
  TARGETING:   '#CC785C',
  APPROACHING: '#C9A26B',
  GRABBING:    '#B894C9',
  CALLING:     '#7DA88A',
  RETURNING:   '#9A9089',
};

export const FSM_SHORT: Record<FsmState, string> = {
  IDLE:        'IDL',
  SEARCHING:   'SRCH',
  TARGETING:   'TGT',
  APPROACHING: 'APR',
  GRABBING:    'GRB',
  CALLING:     'CAL',
  RETURNING:   'RTN',
};

export const FSM_ORDER: ReadonlyArray<FsmState> = [
  'IDLE','SEARCHING','TARGETING','APPROACHING','GRABBING','CALLING','RETURNING',
];
```

- [ ] **Step 4: Запустить тест → pass**

```bash
npm test -- fsm-styles
```

Expected: PASS (4 tests).

- [ ] **Step 5: Аналогично для ball-styles — тест**

`src/components/detection/ball-styles.test.ts`:
```ts
import { describe, it, expect } from 'vitest';
import { BALL_BG_CLASS, BALL_TEXT_CLASS, BALL_HEX, BALL_NAMES } from './ball-styles';

describe('ball-styles', () => {
  it('has 7 colors', () => {
    expect(BALL_NAMES).toHaveLength(7);
  });

  it('hex pairs are valid', () => {
    for (const c of BALL_NAMES) {
      expect(BALL_HEX[c]).toMatch(/^#[0-9A-F]{6}$/i);
    }
  });

  it('class maps cover all colors', () => {
    for (const c of BALL_NAMES) {
      expect(BALL_BG_CLASS[c]).toBe(`bg-ball-${c}`);
      expect(BALL_TEXT_CLASS[c]).toBe(`text-ball-${c}`);
    }
  });
});
```

- [ ] **Step 6: Тест fail → реализация → pass**

`src/components/detection/ball-styles.ts`:
```ts
export const BALL_NAMES = ['red','blue','green','yellow','orange','white','black'] as const;
export type BallColour = typeof BALL_NAMES[number];

export const BALL_HEX: Record<BallColour, string> = {
  red:    '#B85C5C',
  blue:   '#5C7DC9',
  green:  '#6BA86B',
  yellow: '#C9B05C',
  orange: '#C9885C',
  white:  '#D8D2C8',
  black:  '#2A2826',
};

export const BALL_BG_CLASS: Record<BallColour, string> = {
  red:    'bg-ball-red',
  blue:   'bg-ball-blue',
  green:  'bg-ball-green',
  yellow: 'bg-ball-yellow',
  orange: 'bg-ball-orange',
  white:  'bg-ball-white',
  black:  'bg-ball-black',
};

export const BALL_TEXT_CLASS: Record<BallColour, string> = {
  red:    'text-ball-red',
  blue:   'text-ball-blue',
  green:  'text-ball-green',
  yellow: 'text-ball-yellow',
  orange: 'text-ball-orange',
  white:  'text-ball-white',
  black:  'text-ball-black',
};
```

```bash
npm test -- ball-styles
```

Expected: PASS.

- [ ] **Step 7: Build + commit**

```bash
npm run build && npx tsc --noEmit
git add compute_node/frontend/src/components/fsm/ compute_node/frontend/src/components/detection/ball-styles.*
git commit -m "redesign(helpers): fsm-styles и ball-styles с unit-тестами"
```

---

### Task 5: Создать KatanaIcon и набор кастомных SVG

**Files:**
- Create: `compute_node/frontend/src/components/icons/KatanaIcon.tsx`
- Create: `compute_node/frontend/src/components/icons/index.ts`
- Create: `compute_node/frontend/src/components/icons/icons.test.tsx`

- [ ] **Step 1: Написать тест на рендер иконки**

```tsx
import { render } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import { KatanaIcon } from './KatanaIcon';

describe('KatanaIcon', () => {
  it('renders SVG with currentColor stroke', () => {
    const { container } = render(<KatanaIcon className="text-accent h-5 w-5" />);
    const svg = container.querySelector('svg');
    expect(svg).not.toBeNull();
    expect(svg?.getAttribute('stroke')).toBe('currentColor');
    expect(svg?.getAttribute('viewBox')).toBe('0 0 24 24');
  });
});
```

- [ ] **Step 2: Реализовать KatanaIcon**

Контент берётся из `docs/redesign/preview/components.jsx:12-20` (KatanaIcon). Адаптация под TS:

```tsx
import type { SVGProps } from 'react';

export function KatanaIcon({ className, ...props }: SVGProps<SVGSVGElement>) {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.5}
      strokeLinecap="round"
      strokeLinejoin="round"
      className={className}
      {...props}
    >
      <path d="M3.5 20.5 L8 16" />
      <path d="M6.5 17.5 L8.5 19.5" />
      <path d="M9 15 L20.5 3.5" />
      <path d="M18.5 3.5 L20.5 3.5 L20.5 5.5" />
      <circle cx="7.5" cy="18.5" r=".4" fill="currentColor" />
    </svg>
  );
}
```

- [ ] **Step 3: Создать `icons/index.ts` для удобных импортов**

```ts
export { KatanaIcon } from './KatanaIcon';
// re-export часто используемые lucide-react для единообразия
export {
  LayoutDashboard as DashIcon,
  Sliders as SlidersIcon,
  Box as BoxIcon,
  Cpu as CpuIcon,
  Bot as BotIcon,
  Sun as SunIcon,
  Moon as MoonIcon,
  PanelLeftClose,
  PanelLeftOpen,
  Play as PlayIcon,
  Pause as PauseIcon,
  Square as StopIcon,
  RotateCcw as RotateIcon,
  AlertTriangle as AlertIcon,
  Wifi as WifiIcon,
  Battery as BatteryIcon,
  Thermometer as ThermoIcon,
  Terminal as TerminalIcon,
  Grid3x3 as GridIcon,
  Layers as LayersIcon,
  Target as TargetIcon,
  Zap as ZapIcon,
  Plus as PlusIcon,
  Minus as MinusIcon,
  Check as CheckIcon,
  X as XIcon,
  Circle as RecordIcon,
  type LucideProps,
} from 'lucide-react';
```

- [ ] **Step 4: Test + build**

```bash
npm test -- icons
npm run build && npx tsc --noEmit
```

Expected: тест PASS, build PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/icons/
git commit -m "redesign(icons): KatanaIcon + barrel-экспорт lucide-иконок"
```

---

## Phase 3 — Layout & navigation

### Task 6: Создать `Sidebar` компонент

**Files:**
- Create: `compute_node/frontend/src/components/layout/Sidebar.tsx`
- Create: `compute_node/frontend/src/components/layout/Sidebar.test.tsx`

Reference (читать перед реализацией): `docs/redesign/preview/components.jsx:176-242`.

- [ ] **Step 1: Тест на навигацию и collapse**

```tsx
import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import { MemoryRouter } from 'react-router-dom';
import { Sidebar } from './Sidebar';

function renderWithRouter(initial = '/dashboard') {
  return render(
    <MemoryRouter initialEntries={[initial]}>
      <Sidebar />
    </MemoryRouter>
  );
}

describe('Sidebar', () => {
  it('renders all 6 routes', () => {
    renderWithRouter();
    for (const label of ['Dashboard', 'Admin', '3D View', 'Hardware', 'Samcan', 'MPS']) {
      expect(screen.getByText(label)).toBeInTheDocument();
    }
  });

  it('collapse button toggles width', () => {
    const { container } = renderWithRouter();
    const aside = container.querySelector('aside')!;
    expect(aside.className).toContain('w-[220px]');
    fireEvent.click(screen.getByLabelText(/collapse/i));
    expect(aside.className).toContain('w-[56px]');
  });

  it('persists collapsed state in localStorage', () => {
    renderWithRouter();
    fireEvent.click(screen.getByLabelText(/collapse/i));
    expect(localStorage.getItem('samurai.sidebarCollapsed')).toBe('1');
  });
});
```

- [ ] **Step 2: Реализовать Sidebar**

```tsx
import { useEffect, useState } from 'react';
import { Link, useLocation } from 'react-router-dom';
import { cn } from '@/lib/utils';
import {
  KatanaIcon, DashIcon, SlidersIcon, BoxIcon, CpuIcon, BotIcon,
  SunIcon, MoonIcon, PanelLeftClose, PanelLeftOpen,
} from '@/components/icons';
import { useTheme } from './useTheme';
import { useRobotStore } from '@/stores/robotStore';

const NAV = [
  { to: '/dashboard', label: 'Dashboard', Icon: DashIcon },
  { to: '/admin',     label: 'Admin',     Icon: SlidersIcon },
  { to: '/3d',        label: '3D View',   Icon: BoxIcon },
  { to: '/hardware',  label: 'Hardware',  Icon: CpuIcon },
  { to: '/samcan',    label: 'Samcan',    Icon: BotIcon },
  { to: '/mps',       label: 'MPS',       Icon: SlidersIcon },
] as const;

const STORAGE_KEY = 'samurai.sidebarCollapsed';

export function Sidebar() {
  const location = useLocation();
  const [collapsed, setCollapsed] = useState(() => localStorage.getItem(STORAGE_KEY) === '1');
  const { theme, toggle: toggleTheme } = useTheme();
  const connected = useRobotStore(s => s.connected);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, collapsed ? '1' : '0');
  }, [collapsed]);

  return (
    <aside
      className={cn(
        'shrink-0 border-r border-subtle bg-surface-1',
        'transition-[width] duration-standard ease-standard',
        'flex flex-col h-screen sticky top-0 z-30',
        collapsed ? 'w-[56px]' : 'w-[220px]'
      )}
    >
      {/* Brand */}
      <Link to="/dashboard" className="flex items-center gap-2.5 h-12 px-3 border-b border-subtle hover:bg-surface-2/40 transition-colors">
        <KatanaIcon className="h-5 w-5 text-accent shrink-0" />
        {!collapsed && <span className="font-semibold tracking-tight">Samurai</span>}
      </Link>

      {/* Connection status (заменяем RobotSelector — он остаётся отдельным компонентом, см. Task 7) */}
      <div className={cn('flex items-center gap-2 h-8 px-3 border-b border-subtle', collapsed && 'justify-center')}>
        <span className={cn('h-1.5 w-1.5 rounded-full', connected ? 'bg-success animate-pulse-soft' : 'bg-danger')} />
        {!collapsed && (
          <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
            {connected ? 'connected' : 'offline'}
          </span>
        )}
      </div>

      {/* Nav */}
      <nav className="flex-1 px-2 py-2 space-y-0.5">
        {NAV.map(({ to, label, Icon: I }) => {
          const isActive = location.pathname.startsWith(to);
          return (
            <Link
              key={to}
              to={to}
              className={cn(
                'relative w-full flex items-center gap-2.5 h-9 rounded-md transition-colors',
                collapsed ? 'justify-center px-0' : 'px-2.5',
                isActive
                  ? 'bg-surface-2 text-accent'
                  : 'text-foreground-muted hover:bg-surface-2/60 hover:text-foreground'
              )}
              title={collapsed ? label : undefined}
            >
              {isActive && <span className="absolute left-0 top-1.5 bottom-1.5 w-[2px] rounded-full bg-accent" />}
              <I className="h-[18px] w-[18px] shrink-0" />
              {!collapsed && <span className="text-body">{label}</span>}
            </Link>
          );
        })}
      </nav>

      {/* Footer */}
      <div className="border-t border-subtle px-2 py-2 space-y-1">
        <button
          onClick={toggleTheme}
          className={cn(
            'w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted',
            'hover:bg-surface-2/60 hover:text-foreground transition-colors',
            collapsed ? 'justify-center px-0' : 'px-2.5'
          )}
          title={collapsed ? 'Toggle theme' : undefined}
        >
          {theme === 'dark' ? <MoonIcon className="h-[18px] w-[18px]" /> : <SunIcon className="h-[18px] w-[18px]" />}
          {!collapsed && <span className="text-body capitalize">{theme}</span>}
        </button>
        <button
          onClick={() => setCollapsed(c => !c)}
          aria-label="Collapse sidebar"
          className={cn(
            'w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted',
            'hover:bg-surface-2/60 hover:text-foreground transition-colors',
            collapsed ? 'justify-center px-0' : 'px-2.5'
          )}
        >
          {collapsed
            ? <PanelLeftOpen className="h-[18px] w-[18px]" />
            : <PanelLeftClose className="h-[18px] w-[18px]" />}
          {!collapsed && <span className="text-body">Collapse</span>}
        </button>
      </div>
    </aside>
  );
}
```

- [ ] **Step 3: Build (тесты пока не пройдут, нет useTheme)**

```bash
npm run build
```

Если `useTheme` не существует — это ожидаемо, реализуем в следующем таске. Билд должен дать конкретную ошибку про missing module — это ОК как промежуточное состояние, **не коммитим пока зелёным не станет**.

- [ ] **Step 4: Перейти к Task 7, потом вернуться**

Откладываем коммит этого таска до конца Task 7. (Альтернатива — вкомпилировать `useTheme` сразу в Sidebar.tsx как локальный, но это нарушает SoC.)

---

### Task 7: Создать `useTheme` хук + ThemeToggle (опционально как UI)

**Files:**
- Create: `compute_node/frontend/src/components/layout/useTheme.ts`
- Create: `compute_node/frontend/src/components/layout/useTheme.test.ts`

- [ ] **Step 1: Тест**

```ts
import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useTheme } from './useTheme';

describe('useTheme', () => {
  beforeEach(() => {
    localStorage.clear();
    document.documentElement.classList.remove('light');
  });
  afterEach(() => {
    localStorage.clear();
    document.documentElement.classList.remove('light');
  });

  it('starts with dark when no preference', () => {
    const { result } = renderHook(() => useTheme());
    expect(result.current.theme).toBe('dark');
    expect(document.documentElement.classList.contains('light')).toBe(false);
  });

  it('reads from localStorage', () => {
    localStorage.setItem('samurai.theme', 'light');
    const { result } = renderHook(() => useTheme());
    expect(result.current.theme).toBe('light');
    expect(document.documentElement.classList.contains('light')).toBe(true);
  });

  it('toggle switches and persists', () => {
    const { result } = renderHook(() => useTheme());
    act(() => result.current.toggle());
    expect(result.current.theme).toBe('light');
    expect(localStorage.getItem('samurai.theme')).toBe('light');
    expect(document.documentElement.classList.contains('light')).toBe(true);
  });
});
```

- [ ] **Step 2: Тест fail → реализация**

```ts
import { useEffect, useState, useCallback } from 'react';

type Theme = 'dark' | 'light';
const STORAGE_KEY = 'samurai.theme';

export function useTheme() {
  const [theme, setTheme] = useState<Theme>(() => {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored === 'light' || stored === 'dark') return stored;
    if (typeof window !== 'undefined' && window.matchMedia('(prefers-color-scheme: light)').matches) {
      return 'light';
    }
    return 'dark';
  });

  useEffect(() => {
    document.documentElement.classList.toggle('light', theme === 'light');
    localStorage.setItem(STORAGE_KEY, theme);
  }, [theme]);

  const toggle = useCallback(() => {
    setTheme(t => (t === 'dark' ? 'light' : 'dark'));
  }, []);

  return { theme, toggle, setTheme };
}
```

- [ ] **Step 3: Тест pass + Sidebar тесты тоже проходят**

```bash
npm test -- useTheme
npm test -- Sidebar
```

Expected: оба PASS.

- [ ] **Step 4: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

Expected: PASS.

- [ ] **Step 5: Commit Sidebar + useTheme вместе**

```bash
git add compute_node/frontend/src/components/layout/Sidebar.* compute_node/frontend/src/components/layout/useTheme.*
git commit -m "redesign(layout): Sidebar + useTheme hook (collapsible, theme persistence)"
```

---

### Task 8: Создать `PageHeader` и переписать App.tsx (Sidebar + Outlet)

**Files:**
- Create: `compute_node/frontend/src/components/layout/PageHeader.tsx`
- Modify: `compute_node/frontend/src/App.tsx`
- Modify: `compute_node/frontend/src/components/layout/Header.tsx` (deprecate)

Reference: `docs/redesign/preview/components.jsx:244-255` (PageHeader) + `docs/redesign/preview/app.jsx`.

- [ ] **Step 1: Реализовать PageHeader**

```tsx
import type { ReactNode } from 'react';
import { TerminalIcon } from '@/components/icons';
import { Button } from '@/components/ui/button';

interface PageHeaderProps {
  title: string;
  simTime?: string;
  right?: ReactNode;
  onDebug?: () => void;
}

export function PageHeader({ title, simTime, right, onDebug }: PageHeaderProps) {
  return (
    <header className="sticky top-0 z-20 h-12 border-b border-subtle bg-surface-1/95 backdrop-blur-sm">
      <div className="h-full px-6 flex items-center justify-between">
        <h1 className="text-display">{title}</h1>
        <div className="flex items-center gap-3">
          {simTime && (
            <span className="font-mono text-small text-foreground-muted tabular-nums">
              {simTime}
            </span>
          )}
          {right}
          {onDebug && (
            <Button variant="secondary" size="sm" onClick={onDebug}>
              <TerminalIcon className="h-3.5 w-3.5" />Debug
            </Button>
          )}
        </div>
      </div>
    </header>
  );
}
```

- [ ] **Step 2: Создать `Layout` обёртку**

`src/components/layout/Layout.tsx`:
```tsx
import { Outlet } from 'react-router-dom';
import { Sidebar } from './Sidebar';

export function Layout() {
  return (
    <div className="flex min-h-screen bg-background text-foreground">
      <Sidebar />
      <div className="flex-1 min-w-0 flex flex-col">
        <Outlet />
      </div>
    </div>
  );
}
```

- [ ] **Step 3: Переписать App.tsx**

```tsx
import { useEffect, lazy, Suspense } from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { RobotProvider } from '@/providers/RobotProvider';
import { useRobotStore } from '@/stores/robotStore';
import { Layout } from '@/components/layout/Layout';
import { DashboardPage } from '@/pages/DashboardPage';

const AdminPage = lazy(() => import('@/pages/AdminPage').then(m => ({ default: m.AdminPage })));
const Visualization3DPage = lazy(() =>
  import('@/pages/Visualization3DPage').then(m => ({ default: m.Visualization3DPage })));
const HardwarePage = lazy(() => import('@/pages/HardwarePage').then(m => ({ default: m.HardwarePage })));
const SamcanPage = lazy(() => import('@/pages/SamcanPage').then(m => ({ default: m.SamcanPage })));
const MpsPage = lazy(() => import('@/pages/MpsPage').then(m => ({ default: m.MpsPage })));

function PageLoadingFallback() {
  return (
    <div className="flex items-center justify-center min-h-[60vh] text-foreground-muted text-body">
      Загрузка…
    </div>
  );
}

export default function App() {
  const connect = useRobotStore(s => s.connect);
  const disconnect = useRobotStore(s => s.disconnect);

  useEffect(() => {
    connect();
    return () => disconnect();
  }, [connect, disconnect]);

  return (
    <RobotProvider>
      <BrowserRouter>
        <Routes>
          <Route element={<Layout />}>
            <Route path="/" element={<Navigate to="/dashboard" replace />} />
            <Route path="/dashboard" element={<DashboardPage />} />
            <Route path="/admin"     element={<Suspense fallback={<PageLoadingFallback />}><AdminPage /></Suspense>} />
            <Route path="/3d"        element={<Suspense fallback={<PageLoadingFallback />}><Visualization3DPage /></Suspense>} />
            <Route path="/hardware"  element={<Suspense fallback={<PageLoadingFallback />}><HardwarePage /></Suspense>} />
            <Route path="/samcan"    element={<Suspense fallback={<PageLoadingFallback />}><SamcanPage /></Suspense>} />
            <Route path="/mps"       element={<Suspense fallback={<PageLoadingFallback />}><MpsPage /></Suspense>} />
          </Route>
        </Routes>
      </BrowserRouter>
    </RobotProvider>
  );
}
```

- [ ] **Step 4: Если в страницах сейчас рендерится `Header.tsx` — убрать**

```bash
# поиск использований
grep -rn "from '@/components/layout/Header'" compute_node/frontend/src/
```

Если найдены — удалить эти импорты и `<Header />` из page-компонентов. Если в Header была какая-то логика (sim-time, RobotSelector) — она перейдёт в PageHeader через пропсы. Подробнее на стадии Pages (Phase 8).

Если страницы НЕ использовали Header — пропускаем шаг.

- [ ] **Step 5: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
```

Expected: PASS.

```bash
npm run dev
```

Открыть `http://localhost:5173` (или другой порт vite). Проверить:
- Sidebar слева, виден на всех 6 маршрутах
- Кликабельная навигация работает
- Collapse-кнопка работает
- Theme toggle переключает (`<html>` получает класс `.light`)
- Соединение со state — индикатор зелёный, если бэкенд запущен; красный иначе

Если страницы выглядят «поломанными» (пустые места или дублированный header) — это ожидаемо, Phase 8 их починит. Главное: маршруты живые, console чистый, sidebar работает.

Остановить dev-server.

- [ ] **Step 6: Commit**

```bash
git add compute_node/frontend/src/components/layout/{PageHeader,Layout}.tsx compute_node/frontend/src/App.tsx
git commit -m "redesign(layout): Sidebar + Layout (Outlet) + PageHeader, App.tsx через layout-обёртку"
```

---

## Phase 4 — UI primitives (shadcn)

### Task 9: Обновить `Button`

**Files:**
- Modify: `compute_node/frontend/src/components/ui/button.tsx`

Reference: `docs/redesign/preview/components.jsx:68-90`.

- [ ] **Step 1: Прочитать текущий `button.tsx`**

Запомнить cva-паттерн (если используется), props-сигнатуру.

- [ ] **Step 2: Заменить варианты на новые**

Содержимое (адаптация под существующий cva-паттерн или прямая замена):

```tsx
import { Slot } from '@radix-ui/react-slot';
import { cva, type VariantProps } from 'class-variance-authority';
import { forwardRef, type ButtonHTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

const buttonVariants = cva(
  cn(
    'inline-flex items-center justify-center gap-1.5 rounded-md font-medium',
    'transition-all duration-fast ease-standard',
    'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-accent/40 focus-visible:ring-offset-2 focus-visible:ring-offset-background',
    'disabled:opacity-50 disabled:pointer-events-none',
  ),
  {
    variants: {
      variant: {
        default:    'bg-accent text-accent-foreground hover:bg-accent-hover active:bg-accent-active active:scale-[.98]',
        secondary:  'bg-surface-2 text-foreground hover:bg-surface-3 border border-subtle',
        ghost:      'bg-transparent text-foreground-muted hover:bg-surface-2 hover:text-foreground',
        outline:    'border border-strong bg-transparent hover:bg-surface-2',
        destructive:'bg-danger/90 text-foreground hover:bg-danger active:scale-[.98]',
        link:       'text-accent underline-offset-4 hover:underline px-0',
      },
      size: {
        sm:      'h-8 px-2.5 text-small',
        default: 'h-9 px-3 text-body',
        lg:      'h-11 px-4 text-body',
        icon:    'h-9 w-9',
      },
    },
    defaultVariants: { variant: 'default', size: 'default' },
  }
);

export interface ButtonProps
  extends ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean;
}

export const Button = forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : 'button';
    return <Comp className={cn(buttonVariants({ variant, size, className }))} ref={ref} {...props} />;
  }
);
Button.displayName = 'Button';

export { buttonVariants };
```

- [ ] **Step 3: Build + typecheck**

Если есть существующие тесты на Button — могут падать на снапшотах (новый класс-набор). Обновить если есть, или удалить устаревшие снапшоты.

```bash
npm test -- button
npm run build && npx tsc --noEmit
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/ui/button.tsx
git commit -m "redesign(primitives): Button — новые варианты (default/secondary/ghost/outline/destructive/link)"
```

---

### Task 10: Обновить `Card` и его подкомпоненты

**Files:**
- Modify: `compute_node/frontend/src/components/ui/card.tsx`

Reference: `docs/redesign/preview/components.jsx:49-66`.

- [ ] **Step 1: Заменить содержимое**

```tsx
import { forwardRef, type HTMLAttributes, type ReactNode } from 'react';
import { cn } from '@/lib/utils';

export const Card = forwardRef<HTMLDivElement, HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'rounded-lg border border-subtle bg-surface-1',
        'transition-colors duration-standard',
        className
      )}
      {...props}
    />
  )
);
Card.displayName = 'Card';

interface CardHeaderProps extends HTMLAttributes<HTMLDivElement> {
  right?: ReactNode;
}

export const CardHeader = forwardRef<HTMLDivElement, CardHeaderProps>(
  ({ className, children, right, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'flex items-center justify-between gap-2 px-4 py-3 border-b border-subtle',
        className
      )}
      {...props}
    >
      <div className="flex items-center gap-2 min-w-0">{children}</div>
      {right ? <div className="flex items-center gap-1.5">{right}</div> : null}
    </div>
  )
);
CardHeader.displayName = 'CardHeader';

export const CardTitle = forwardRef<HTMLHeadingElement, HTMLAttributes<HTMLHeadingElement>>(
  ({ className, ...props }, ref) => (
    <h2 ref={ref} className={cn('text-h2 text-foreground truncate', className)} {...props} />
  )
);
CardTitle.displayName = 'CardTitle';

export const CardSubtitle = forwardRef<HTMLSpanElement, HTMLAttributes<HTMLSpanElement>>(
  ({ className, ...props }, ref) => (
    <span
      ref={ref}
      className={cn(
        'font-mono text-micro uppercase tracking-wider text-foreground-faint',
        className
      )}
      {...props}
    />
  )
);
CardSubtitle.displayName = 'CardSubtitle';

export const CardContent = forwardRef<HTMLDivElement, HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div ref={ref} className={cn('p-4', className)} {...props} />
  )
);
CardContent.displayName = 'CardContent';

// Footer + Description оставляем для обратной совместимости
export const CardFooter = forwardRef<HTMLDivElement, HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div ref={ref} className={cn('px-4 py-3 border-t border-subtle flex items-center', className)} {...props} />
  )
);
CardFooter.displayName = 'CardFooter';

export const CardDescription = forwardRef<HTMLParagraphElement, HTMLAttributes<HTMLParagraphElement>>(
  ({ className, ...props }, ref) => (
    <p ref={ref} className={cn('text-small text-foreground-muted', className)} {...props} />
  )
);
CardDescription.displayName = 'CardDescription';
```

- [ ] **Step 2: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

Если ошибки про `CardHeader.right` — это ожидаемое **расширение API**: добавили опциональный `right`. Существующие потребители продолжат работать без него.

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/ui/card.tsx
git commit -m "redesign(primitives): Card — flat surface-1 с border-subtle, опциональный right в CardHeader"
```

---

### Task 11: Обновить `Input`, добавить `Badge` и `Kbd`

**Files:**
- Modify: `compute_node/frontend/src/components/ui/input.tsx`
- Modify: `compute_node/frontend/src/components/ui/badge.tsx`
- Create: `compute_node/frontend/src/components/ui/kbd.tsx`

- [ ] **Step 1: Input — заменить классы**

```tsx
import { forwardRef, type InputHTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

export const Input = forwardRef<HTMLInputElement, InputHTMLAttributes<HTMLInputElement>>(
  ({ className, type = 'text', ...props }, ref) => (
    <input
      ref={ref}
      type={type}
      className={cn(
        'h-9 w-full rounded-md border border-subtle bg-surface-2 px-3 py-1',
        'text-body text-foreground placeholder:text-foreground-faint',
        'focus-visible:outline-none focus-visible:border-accent focus-visible:ring-2 focus-visible:ring-accent/30',
        'disabled:cursor-not-allowed disabled:opacity-50',
        'transition-colors duration-fast',
        className
      )}
      {...props}
    />
  )
);
Input.displayName = 'Input';
```

- [ ] **Step 2: Badge — переписать с tone-вариантами**

```tsx
import { forwardRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

type Tone = 'default' | 'accent' | 'success' | 'warning' | 'danger' | 'info';

interface BadgeProps extends HTMLAttributes<HTMLSpanElement> {
  tone?: Tone;
  dot?: boolean;
}

const TONE_CLASS: Record<Tone, string> = {
  default: 'bg-surface-2 text-foreground-muted border-subtle',
  accent:  'bg-accent/10 text-accent border-accent/20',
  success: 'bg-success/10 text-success border-success/25',
  warning: 'bg-warning/10 text-warning border-warning/25',
  danger:  'bg-danger/10 text-danger border-danger/25',
  info:    'bg-info/10 text-info border-info/25',
};

export const Badge = forwardRef<HTMLSpanElement, BadgeProps>(
  ({ tone = 'default', dot = false, className, children, ...props }, ref) => (
    <span
      ref={ref}
      className={cn(
        'inline-flex items-center gap-1.5 rounded-pill border px-2 py-0.5',
        'font-mono text-micro uppercase tracking-wider',
        TONE_CLASS[tone],
        className
      )}
      {...props}
    >
      {dot && <span className="h-1.5 w-1.5 rounded-full bg-current" />}
      {children}
    </span>
  )
);
Badge.displayName = 'Badge';
```

- [ ] **Step 3: Kbd — новый**

```tsx
import { forwardRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

export const Kbd = forwardRef<HTMLElement, HTMLAttributes<HTMLElement>>(
  ({ className, children, ...props }, ref) => (
    <kbd
      ref={ref}
      className={cn(
        'inline-flex items-center justify-center min-w-[1.5rem] h-5 px-1.5 rounded',
        'border border-subtle bg-surface-2 font-mono text-micro text-foreground-muted',
        className
      )}
      {...props}
    >
      {children}
    </kbd>
  )
);
Kbd.displayName = 'Kbd';
```

- [ ] **Step 4: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

Expected: PASS. Возможны ошибки в местах потребления Badge — если где-то использовался `<Badge variant="...">` (старая сигнатура) вместо `tone`. Найти и заменить:

```bash
grep -rn '<Badge' compute_node/frontend/src/ | grep -v 'tone='
```

Перейти на `tone=` если попадётся — это малое изменение.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/ui/{input,badge,kbd}.tsx
git commit -m "redesign(primitives): Input/Badge/Kbd под новую палитру"
```

---

### Task 12: Tooltip — лёгкая стилевая правка

**Files:**
- Modify: `compute_node/frontend/src/components/ui/tooltip.tsx`

- [ ] **Step 1: Прочитать текущий**

Скорее всего обёртка над `@radix-ui/react-tooltip`. Найти `TooltipContent` и заменить классы.

- [ ] **Step 2: Заменить классы у `TooltipContent`**

```tsx
// в TooltipContent — className пропу:
className={cn(
  'z-50 overflow-hidden rounded-md border border-subtle bg-surface-3 px-2 py-1',
  'text-small text-foreground shadow-lg',
  'data-[state=delayed-open]:animate-fade-in-up',
  className
)}
```

- [ ] **Step 3: Build + commit**

```bash
npm run build && npx tsc --noEmit
git add compute_node/frontend/src/components/ui/tooltip.tsx
git commit -m "redesign(primitives): Tooltip — surface-3 + border-subtle + animate-fade-in-up"
```

---

## Phase 5 — FSM components

### Task 13: Переписать `FsmBadge`

**Files:**
- Modify: `compute_node/frontend/src/components/fsm/FsmBadge.tsx`

Reference: `docs/redesign/preview/components.jsx:135-145`.

- [ ] **Step 1: Прочитать текущий FsmBadge.tsx, запомнить пропсы**

- [ ] **Step 2: Переписать (сохранить пропсы, изменить только JSX)**

```tsx
import type { FsmState } from '@/types/robot';
import { FSM_DOT_HEX } from './fsm-styles';
import { cn } from '@/lib/utils';

export interface FsmBadgeProps {
  state: FsmState;
  target?: string;
  compact?: boolean;
  className?: string;
}

export function FsmBadge({ state, target, compact, className }: FsmBadgeProps) {
  return (
    <div
      className={cn(
        'inline-flex items-center gap-2 rounded-pill border border-subtle bg-surface-2',
        compact ? 'px-2 py-1' : 'px-3 py-1.5',
        className
      )}
    >
      <span
        className={cn('h-2 w-2 rounded-full shrink-0', state !== 'IDLE' && 'animate-pulse-soft')}
        style={{ backgroundColor: FSM_DOT_HEX[state] }}
      />
      <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
        {state}
      </span>
      {target && (
        <span className="font-mono text-micro text-foreground-faint">→ {target}</span>
      )}
    </div>
  );
}
```

**Внимание:** если текущий `FsmBadge` принимал другие пропсы (например, прямой пропс `colour` вместо `target`) — сохранить совместимость, добавить новые опциональные, не убирая старые. Старые потребители продолжат компилироваться.

- [ ] **Step 3: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/fsm/FsmBadge.tsx
git commit -m "redesign(fsm): FsmBadge — pill с пульсирующей точкой"
```

---

### Task 14: Создать `FsmTimeline`

**Files:**
- Create: `compute_node/frontend/src/components/fsm/FsmTimeline.tsx`
- Create: `compute_node/frontend/src/components/fsm/FsmTimeline.test.tsx`

Reference: `docs/redesign/preview/components.jsx:147-165`.

- [ ] **Step 1: Тест**

```tsx
import { render, screen } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import { FsmTimeline } from './FsmTimeline';

describe('FsmTimeline', () => {
  it('renders all 7 states as short labels', () => {
    render(<FsmTimeline current="IDLE" />);
    for (const lbl of ['IDL','SRCH','TGT','APR','GRB','CAL','RTN']) {
      expect(screen.getByText(lbl)).toBeInTheDocument();
    }
  });

  it('marks current state with text-accent', () => {
    render(<FsmTimeline current="TARGETING" />);
    const tgt = screen.getByText('TGT');
    expect(tgt.className).toContain('text-accent');
  });
});
```

- [ ] **Step 2: Реализация**

```tsx
import type { FsmState } from '@/types/robot';
import { FSM_ORDER, FSM_SHORT } from './fsm-styles';
import { cn } from '@/lib/utils';

export function FsmTimeline({ current, className }: { current: FsmState; className?: string }) {
  const idx = FSM_ORDER.indexOf(current);
  return (
    <div className={cn('flex items-end gap-1.5', className)}>
      {FSM_ORDER.map((s, i) => {
        const isActive = i === idx;
        const isPast = i < idx;
        return (
          <div key={s} className="flex flex-col items-center gap-1.5 flex-1 min-w-0">
            <div
              className={cn(
                'h-1 w-full rounded-full transition-colors duration-standard',
                isActive ? 'bg-accent' : isPast ? 'bg-foreground-muted/60' : 'bg-surface-3'
              )}
            />
            <span
              className={cn(
                'font-mono text-micro tracking-wider',
                isActive ? 'text-accent' : 'text-foreground-faint'
              )}
            >
              {FSM_SHORT[s]}
            </span>
          </div>
        );
      })}
    </div>
  );
}
```

- [ ] **Step 3: Тест pass + build**

```bash
npm test -- FsmTimeline
npm run build && npx tsc --noEmit
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/fsm/FsmTimeline.*
git commit -m "redesign(fsm): FsmTimeline — 7 сегментов с активным/пройденным/будущим"
```

---

## Phase 6 — Centerpieces

### Task 15: Переписать `CommandInput` (терминал)

**Files:**
- Modify: `compute_node/frontend/src/components/controls/CommandInput.tsx`

Reference: `docs/redesign/preview/dashboard.jsx:75-90`.

- [ ] **Step 1: Прочитать текущий, сохранить пропсы**

- [ ] **Step 2: Заменить рендер**

```tsx
import { useState } from 'react';
import { cn } from '@/lib/utils';
import { Kbd } from '@/components/ui/kbd';

export interface CommandInputProps {
  onSend: (text: string) => void;
  placeholder?: string;
  disabled?: boolean;
}

export function CommandInput({ onSend, placeholder = 'найди красный мяч', disabled }: CommandInputProps) {
  const [value, setValue] = useState('');
  const [focused, setFocused] = useState(false);

  return (
    <div
      className={cn(
        'flex items-center gap-2 rounded-md border bg-surface-1 px-3 py-2 transition-colors',
        focused ? 'border-accent' : 'border-subtle',
        disabled && 'opacity-50 pointer-events-none'
      )}
    >
      <span className={cn('font-mono text-body text-accent select-none', focused && 'animate-blink')}>
        &gt;
      </span>
      <input
        value={value}
        onChange={e => setValue(e.target.value)}
        onFocus={() => setFocused(true)}
        onBlur={() => setFocused(false)}
        onKeyDown={e => {
          if (e.key === 'Enter' && value.trim()) {
            onSend(value.trim());
            setValue('');
          }
        }}
        placeholder={placeholder}
        disabled={disabled}
        className="flex-1 bg-transparent border-none outline-none font-mono text-body text-foreground placeholder:text-foreground-faint focus:ring-0"
      />
      <Kbd>↵</Kbd>
    </div>
  );
}
```

- [ ] **Step 3: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/controls/CommandInput.tsx
git commit -m "redesign(centerpieces): CommandInput — терминал-стайл с blink-курсором"
```

---

### Task 16: Переписать `CameraFeed` с HUD-overlay + scanline

**Files:**
- Modify: `compute_node/frontend/src/components/camera/CameraFeed.tsx`

Reference: `docs/redesign/preview/dashboard.jsx:5-72`.

**Важно:** боевой CameraFeed использует `<img src="/video_feed">`. Превью использует декоративный фон. Сохраняем `<img>` для реального стрима, но добавляем HUD поверх.

- [ ] **Step 1: Прочитать текущий**

- [ ] **Step 2: Переписать**

```tsx
import { useRobotStore } from '@/stores/robotStore';
import { FsmBadge } from '@/components/fsm/FsmBadge';
import { TargetIcon } from '@/components/icons';
import { cn } from '@/lib/utils';

export interface CameraFeedProps {
  className?: string;
  src?: string;
  showHud?: boolean;
}

export function CameraFeed({ className, src = '/video_feed', showHud = true }: CameraFeedProps) {
  const fsm = useRobotStore(s => s.status?.state ?? 'IDLE');
  const target = useRobotStore(s => s.status?.target_colour);
  const connected = useRobotStore(s => s.connected);
  // simTime если есть в стейте — иначе пропускаем
  const simTime = useRobotStore(s => (s as { simTime?: string }).simTime);

  return (
    <div
      className={cn(
        'relative aspect-video overflow-hidden rounded-lg border border-subtle bg-surface-2 scanline-bg',
        className
      )}
    >
      <img src={src} alt="camera" className="h-full w-full object-cover" />

      {showHud && (
        <>
          {/* Scanline */}
          <div className="pointer-events-none absolute inset-0 overflow-hidden">
            <div className="absolute left-0 right-0 h-px bg-accent/40 animate-scanline" />
          </div>

          {/* HUD top-left FSM */}
          <div className="absolute top-2 left-2">
            <FsmBadge state={fsm} target={target} compact />
          </div>

          {/* HUD top-right time */}
          {simTime && (
            <div className="absolute top-2 right-2 font-mono text-micro text-foreground-muted bg-background/70 backdrop-blur-sm rounded px-2 py-1 border border-subtle">
              {simTime}
            </div>
          )}

          {/* HUD bottom-left meta */}
          <div className="absolute bottom-2 left-2 flex items-center gap-1.5 bg-background/70 backdrop-blur-sm rounded-full border border-subtle px-2 py-1">
            <TargetIcon className="h-3 w-3 text-foreground-muted" />
            <span className="font-mono text-micro text-foreground-muted">CAM 0</span>
          </div>

          {/* HUD bottom-right LIVE/OFFLINE */}
          <div className="absolute bottom-2 right-2 flex items-center gap-1.5 bg-background/70 backdrop-blur-sm rounded-full border border-subtle px-2 py-1">
            <span className={cn('h-1.5 w-1.5 rounded-full', connected ? 'bg-success animate-pulse-soft' : 'bg-danger')} />
            <span className="font-mono text-micro text-foreground-muted">
              {connected ? 'LIVE' : 'OFFLINE'}
            </span>
          </div>
        </>
      )}
    </div>
  );
}
```

**Note:** Detection bbox-overlay из preview опускаем — backend не отдаёт normalized координаты в текущем формате. Можно добавить позже отдельной фичей.

- [ ] **Step 3: Build + typecheck + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/dashboard`. CameraFeed должен показать `<img>` с HUD-bordering, scanline-эффектом, FSM-бaджем сверху-слева, LIVE-индикатором снизу-справа.

Если бэкенд не запущен — `<img>` сломается (broken image), но сами HUD-overlay должны быть видны. Это ожидаемо без бэкенда.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/camera/CameraFeed.tsx
git commit -m "redesign(centerpieces): CameraFeed — HUD-overlay с FSM, LIVE-индикатор и scanline"
```

---

### Task 17: Переписать `JoystickControl` (технический + readout)

**Files:**
- Modify: `compute_node/frontend/src/components/joystick/JoystickControl.tsx`
- Modify: `compute_node/frontend/src/components/controls/JoystickControl.tsx` (если существует — привести к идентичному визуалу через переэкспорт или копию, не ломая существующих импортов)

Reference: `docs/redesign/preview/dashboard.jsx:279-332`.

**Важно:** `useJoystick` — НЕ ТРОГАТЬ (заблокировано в PROMPT.md). Меняем только JSX.

- [ ] **Step 1: Прочитать текущие файлы (оба, если есть)**

```bash
grep -rn 'useJoystick' compute_node/frontend/src/
```

Найти, как именно подключается хук, какие из него возвращаются значения. Если `state.dx, state.dy, state.linear, state.angular, state.active` — отлично, prefer-их. Если другие имена — адаптировать ниже.

- [ ] **Step 2: Переписать визуал в `joystick/JoystickControl.tsx`**

```tsx
import { useJoystick } from '@/hooks/useJoystick';
import { cn } from '@/lib/utils';

export interface JoystickControlProps {
  maxLinear?: number;
  maxAngular?: number;
  className?: string;
}

export function JoystickControl({ maxLinear = 0.3, maxAngular = 2.0, className }: JoystickControlProps) {
  const { containerRef, state, startJoy, moveJoy, endJoy } = useJoystick(maxLinear, maxAngular);

  // Если useJoystick возвращает другие имена — заменить здесь:
  const dx = state.dx ?? 0;
  const dy = state.dy ?? 0;
  const linear = state.linear ?? 0;
  const angular = state.angular ?? 0;
  const active = state.active ?? false;

  return (
    <div className={cn('flex flex-col items-center gap-3 select-none', className)}>
      <div
        ref={containerRef}
        onPointerDown={startJoy}
        onPointerMove={moveJoy}
        onPointerUp={endJoy}
        onPointerCancel={endJoy}
        className="joystick-area relative h-44 w-44 rounded-full border border-subtle bg-surface-2 cursor-grab active:cursor-grabbing"
      >
        <svg viewBox="0 0 176 176" className="absolute inset-0 pointer-events-none">
          <line x1="88" y1="20" x2="88" y2="156" stroke="hsl(var(--surface-3))" strokeWidth="1" />
          <line x1="20" y1="88" x2="156" y2="88" stroke="hsl(var(--surface-3))" strokeWidth="1" />
          <circle cx="88" cy="88" r="56" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="1" strokeDasharray="3 4" />
          <circle cx="88" cy="88" r="28" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="1" strokeDasharray="2 3" />
          {active && (
            <line
              x1="88" y1="88"
              x2={88 + dx} y2={88 + dy}
              stroke="hsl(var(--accent))" strokeWidth="2" strokeLinecap="round"
            />
          )}
        </svg>
        <div
          className={cn(
            'absolute h-11 w-11 rounded-full bg-surface-3 border-2 border-accent',
            'transition-transform',
            active ? 'duration-fast ease-standard' : 'duration-standard ease-spring'
          )}
          style={{
            left: `calc(50% - 22px + ${dx}px)`,
            top:  `calc(50% - 22px + ${dy}px)`,
          }}
        />
      </div>
      <div className="font-mono text-small text-foreground-muted tabular-nums flex items-center gap-1">
        <span className={cn(linear !== 0 && 'text-accent')}>{linear.toFixed(2)}</span>
        <span className="text-foreground-faint">m/s</span>
        <span className="mx-2 text-foreground-faint">·</span>
        <span className={cn(angular !== 0 && 'text-accent')}>{angular.toFixed(2)}</span>
        <span className="text-foreground-faint">rad/s</span>
      </div>
    </div>
  );
}
```

- [ ] **Step 3: Если в `controls/JoystickControl.tsx` дубль — сделать переэкспорт**

Чтобы не плодить копии:
```tsx
// controls/JoystickControl.tsx
export { JoystickControl } from '@/components/joystick/JoystickControl';
```

(Проверить что нет конфликта именованных экспортов.)

- [ ] **Step 4: Build + typecheck + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/dashboard` или `/admin`, найти джойстик. Проверить что drag работает, vector рисуется при активном касании, readout показывает скорости моноспейсом.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/joystick/JoystickControl.tsx compute_node/frontend/src/components/controls/JoystickControl.tsx
git commit -m "redesign(centerpieces): JoystickControl — крест/кольца, heading-vector, readout m/s·rad/s"
```

---

### Task 18: Перерисовать `MapCanvas` под новую палитру + `MapToolbar` toggle'ы

**Files:**
- Modify: `compute_node/frontend/src/hooks/useMapCanvas.ts` — **ВНИМАНИЕ:** хуки заблокированы. Только если внутри **захардкожены цвета** — расширяем интерфейс `MapDrawData` опциональным `theme` (back-compat).
- Modify: `compute_node/frontend/src/components/map/MapCanvas.tsx`
- Modify: `compute_node/frontend/src/components/map/MapToolbar.tsx`

Reference: `docs/redesign/preview/dashboard.jsx:160-236` (рисует SVG-набросок) + `docs/redesign/PROMPT.md §6.10`.

- [ ] **Step 1: Прочитать `useMapCanvas.ts`**

Если цвета захардкожены — есть два пути:
- (a) Минимально расширить `MapDrawData` опциональным полем `theme: { robot, path, planned, forbidden, scan, mapTint }` со значениями по умолчанию (не ломая старых потребителей).
- (b) Считать цвета из CSS-переменных через `getComputedStyle(document.documentElement).getPropertyValue('--accent')` (это утилитарный код в потребителе или в самом хуке).

Выбираем **(b)**: написать хелпер `useThemeColors()` в `MapCanvas.tsx`, передавать в данные. Хук `useMapCanvas` не меняется.

- [ ] **Step 2: В `MapCanvas.tsx` подмешать тему через хелпер**

```tsx
import { useEffect, useState } from 'react';

function useCssThemeColors() {
  const [colors, setColors] = useState({
    robot: 'hsl(16 53% 58%)',
    path:  'hsl(16 53% 58%)',
    planned: '#7DD3A8',
    forbidden: '#C97B7B',
    scan: 'hsl(35 9% 63%)',
  });
  useEffect(() => {
    const cs = getComputedStyle(document.documentElement);
    setColors({
      robot: `hsl(${cs.getPropertyValue('--accent').trim()})`,
      path:  `hsl(${cs.getPropertyValue('--accent').trim()})`,
      planned: '#7DD3A8',
      forbidden: '#C97B7B',
      scan: `hsl(${cs.getPropertyValue('--foreground-muted').trim()})`,
    });
  }, []);
  return colors;
}
```

Использовать в `MapCanvas` и пробросить в `useMapCanvas` через расширенный data-объект.

- [ ] **Step 3: `MapToolbar` — добавить toggle'ы Grid/Coverage/Zones**

Текущая логика рисования в `useMapCanvas` уже умеет zones. `Grid` и `Coverage` — новые. Если хук пока не поддерживает — добавляем стейт **в потребителе** (`MapCanvas`), рисуем поверх через дополнительный canvas или SVG-overlay (не трогая хук).

```tsx
// MapToolbar — props расширяем опционально
import { Button } from '@/components/ui/button';
import { GridIcon, LayersIcon, AlertIcon, PlusIcon, MinusIcon } from '@/components/icons';

export interface MapToolbarProps {
  showGrid?: boolean;
  onToggleGrid?: () => void;
  showCoverage?: boolean;
  onToggleCoverage?: () => void;
  showZones?: boolean;
  onToggleZones?: () => void;
  zoom?: number;
  onZoomIn?: () => void;
  onZoomOut?: () => void;
  // ... legacy props (Draw zone, Delete, Clear) сохраняются как есть
}

export function MapToolbar(props: MapToolbarProps) {
  const { showGrid = false, onToggleGrid, showCoverage = false, onToggleCoverage,
          showZones = true, onToggleZones, zoom = 100, onZoomIn, onZoomOut } = props;
  return (
    <div className="flex flex-wrap items-center gap-1.5">
      <Button size="sm" variant={showGrid ? 'default' : 'secondary'} onClick={onToggleGrid}>
        <GridIcon className="h-3.5 w-3.5" />Grid
      </Button>
      <Button size="sm" variant={showCoverage ? 'default' : 'secondary'} onClick={onToggleCoverage}>
        <LayersIcon className="h-3.5 w-3.5" />Coverage
      </Button>
      <Button size="sm" variant={showZones ? 'default' : 'secondary'} onClick={onToggleZones}>
        <AlertIcon className="h-3.5 w-3.5" />Zones
      </Button>
      <span className="ml-auto inline-flex items-center gap-1.5">
        <Button size="sm" variant="ghost" onClick={onZoomIn}><PlusIcon className="h-3.5 w-3.5" /></Button>
        <span className="font-mono text-micro text-foreground-faint tabular-nums">{zoom}%</span>
        <Button size="sm" variant="ghost" onClick={onZoomOut}><MinusIcon className="h-3.5 w-3.5" /></Button>
      </span>
    </div>
  );
}
```

**Важно:** старые legacy-пропсы (`mode`, `setMode`, `clearAll`, etc.) сохраняем — добавляем новые как опциональные. Существующие места употребления продолжают работать.

- [ ] **Step 4: Build + typecheck**

```bash
npm run build && npx tsc --noEmit
```

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/map/{MapCanvas,MapToolbar}.tsx
git commit -m "redesign(centerpieces): MapCanvas палитра через CSS-vars, MapToolbar — Grid/Coverage/Zones"
```

---

## Phase 7 — Charts, 3D, sensors

### Task 19: Перерисовать `SensorCharts` (Recharts restyle)

**Files:**
- Modify: `compute_node/frontend/src/components/charts/SensorCharts.tsx`

Reference: `docs/redesign/preview/dashboard.jsx:246-276` (SparkLine SVG — для вдохновения, реальный SensorCharts использует Recharts) + `docs/redesign/PROMPT.md §6.12`.

- [ ] **Step 1: Прочитать текущий SensorCharts**

Запомнить какие серии (battery, temp, range, accel, speed?), какие компоненты Recharts.

- [ ] **Step 2: Заменить классы и пропсы Recharts**

Шаблон преобразования (применить ко всем `<LineChart>`):

```tsx
<LineChart data={data}>
  <CartesianGrid stroke="hsl(var(--surface-3))" strokeDasharray="0" vertical={false} />
  <XAxis
    dataKey="t"
    tick={{ fill: 'hsl(var(--foreground-faint))', fontSize: 10, fontFamily: 'JetBrains Mono' }}
    stroke="hsl(var(--surface-3))"
  />
  <YAxis
    tick={{ fill: 'hsl(var(--foreground-faint))', fontSize: 10, fontFamily: 'JetBrains Mono' }}
    stroke="hsl(var(--surface-3))"
  />
  <Tooltip
    contentStyle={{
      background: 'hsl(var(--surface-3))',
      border: '1px solid hsl(var(--border-subtle))',
      borderRadius: '8px',
      fontFamily: 'JetBrains Mono',
      fontSize: '12px',
    }}
    labelStyle={{ color: 'hsl(var(--foreground-muted))' }}
    itemStyle={{ color: 'hsl(var(--foreground))' }}
  />
  <Legend wrapperStyle={{ fontSize: '11px', color: 'hsl(var(--foreground-muted))' }} />
  <Line
    type="monotone"
    dataKey="battery"
    stroke="hsl(var(--accent))"
    strokeWidth={1.5}
    dot={false}
    activeDot={{ r: 3, fill: 'hsl(var(--accent))' }}
  />
  {/* остальные серии — stroke = warning/info/success */}
</LineChart>
```

- [ ] **Step 3: Build + typecheck + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/dashboard`, проверить что графики выглядят согласно палитре, без рамок, моноспейс на осях, accent-линия видна.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/charts/SensorCharts.tsx
git commit -m "redesign(charts): SensorCharts — Recharts в палитре (без рамок, mono оси, surface-3 grid)"
```

---

### Task 20: 3D-сцена — палитра, освещение, fog

**Files:**
- Modify: `compute_node/frontend/src/pages/Visualization3DPage.tsx`
- Modify (если есть): `compute_node/frontend/src/components/3d/RobotModel.tsx`, `PathTrail.tsx`, `PlannedPathTrail.tsx`, `ImuVectors.tsx`, `SlamMap3D.tsx`, `CoverageHeatmap.tsx`, `InfoPanel.tsx`

Reference: `docs/redesign/PROMPT.md §6.13`.

- [ ] **Step 1: В `Visualization3DPage.tsx` обновить сцену**

Заменить `<color>`, `<gridHelper>`, освещение, добавить fog:

```tsx
<Canvas shadows camera={{ position: [4, 4, 4], fov: 50 }}>
  <color attach="background" args={['#1F1E1D']} />
  <fog attach="fog" args={['#1F1E1D', 8, 30]} />

  <ambientLight color="#1F1E1D" intensity={0.6} />
  <hemisphereLight color="#34322F" groundColor="#1F1E1D" intensity={0.4} />
  <directionalLight position={[5, 8, 5]} intensity={0.3} castShadow />
  {/* Rim lights */}
  <directionalLight position={[-3, 4, -5]} intensity={0.4} color="#CC785C" />
  <directionalLight position={[5, 3, 3]} intensity={0.3} color="#E8E5DD" />

  <gridHelper args={[100, 200, '#2D2B28', '#2D2B28']} />
  <axesHelper args={[1.5]} />
  {/* Прим.: цвета осей в three.js через axesHelper не настраиваются — для кастома понадобится свой `<group>` с тремя `<line>`. Если сейчас стандартный axesHelper подходит — оставляем. Иначе — заменяем на:

  <group>
    <line>...X coral...</line>
    <line>...Y sage...</line>
    <line>...Z slate...</line>
  </group>

  */}

  {/* RobotModel + PathTrail + ... */}
</Canvas>
```

- [ ] **Step 2: В `RobotModel.tsx` (или где материал) сменить color**

```tsx
<meshStandardMaterial color="#CC785C" metalness={0.1} roughness={0.6} />
```

- [ ] **Step 3: `InfoPanel.tsx` — переписать классы**

Заменить классы на `bg-surface-1/80 backdrop-blur-md border border-subtle rounded-lg p-4 font-mono text-small`.

- [ ] **Step 4: Build + typecheck + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/3d`. Сцена должна быть тёплая, графитовая, робот coral, легкий rim-light, fog на дальних объектах.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/pages/Visualization3DPage.tsx compute_node/frontend/src/components/3d/
git commit -m "redesign(3d): тёмный фон, coral-робот, rim-lights вместо 3 directionals, fog"
```

---

### Task 21: Sensor / actuator / event-log виджеты — визуальный апдейт

**Files:**
- Modify: `compute_node/frontend/src/components/sensors/SensorPanel.tsx`
- Modify: `compute_node/frontend/src/components/actuators/ActuatorToggles.tsx`
- Modify: `compute_node/frontend/src/components/log/EventLog.tsx`
- Modify (если ещё используются): `BatteryIndicator`, `TemperatureIndicator`, `RangeBar`, `WatchdogPanel`

Reference: `docs/redesign/preview/dashboard.jsx:335-403`.

- [ ] **Step 1: SensorPanel — переписать строки**

```tsx
import { BatteryIcon, ThermoIcon, WifiIcon, ZapIcon, TargetIcon } from '@/components/icons';
import { useRobotStore } from '@/stores/robotStore';

function SensorRow({ Icon, label, value, unit, tone = 'foreground' }: {
  Icon: React.ComponentType<{ className?: string }>;
  label: string;
  value: string;
  unit: string;
  tone?: 'foreground' | 'warning' | 'success' | 'danger' | 'foreground-muted';
}) {
  return (
    <div className="flex items-center justify-between py-1.5">
      <span className="inline-flex items-center gap-2 text-foreground-muted">
        <Icon className="h-3.5 w-3.5" />
        <span className="text-body">{label}</span>
      </span>
      <span className="font-mono text-small tabular-nums">
        <span className={`text-${tone}`}>{value}</span>
        <span className="text-foreground-faint ml-1">{unit}</span>
      </span>
    </div>
  );
}

export function SensorPanel() {
  const battery = useRobotStore(s => s.sensors?.battery_pct ?? 0);
  const temp    = useRobotStore(s => s.sensors?.cpu_temp ?? 0);
  const range   = useRobotStore(s => s.sensors?.range_cm ?? 0);
  // ... другие селекторы по факту наличия в store

  return (
    <div className="divide-y divide-subtle">
      <SensorRow Icon={BatteryIcon} label="Battery" value={battery.toFixed(1)} unit="%"  tone={battery < 20 ? 'danger' : 'foreground'} />
      <SensorRow Icon={ThermoIcon}  label="Temp"    value={temp.toFixed(1)}    unit="°C" tone={temp > 65 ? 'warning' : 'foreground'} />
      <SensorRow Icon={TargetIcon}  label="Range"   value={range.toFixed(0)}   unit="cm" tone="foreground" />
    </div>
  );
}
```

- [ ] **Step 2: ActuatorToggles — переписать как toggle-grid с новым стилем**

Использовать тот же `Toggle` из preview как локальный компонент или вынести в `ui/toggle.tsx`. Если он уже существует в `ui/toggle.tsx` (radix toggle) — стилизуем.

- [ ] **Step 3: EventLog — переписать в моноспейс с цветными бейджами**

```tsx
import { Badge } from '@/components/ui/badge';
import { useRobotStore } from '@/stores/robotStore';
import { cn } from '@/lib/utils';

export function EventLog() {
  const log = useRobotStore(s => s.voiceLog ?? []);
  return (
    <ul className="font-mono text-small divide-y divide-subtle">
      {log.slice(0, 50).map((e, i) => (
        <li
          key={`${e.time}-${i}`}
          className={cn('grid grid-cols-[auto_1fr] items-center gap-3 py-2 px-1', i === 0 && 'animate-fade-in-up')}
        >
          <span className="text-foreground-faint tabular-nums">{e.time}</span>
          <span className="text-foreground-muted truncate">{e.text}</span>
        </li>
      ))}
    </ul>
  );
}
```

(Адаптация по форме записи `LogEntry { text, time }` из `types/robot.ts`.)

- [ ] **Step 4: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
```

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/{sensors,actuators,log}/
git commit -m "redesign(widgets): SensorPanel/ActuatorToggles/EventLog — палитра + моноспейс значений"
```

---

## Phase 8 — Pages assembly

### Task 22: `DashboardPage` — раскладка под новую структуру

**Files:**
- Modify: `compute_node/frontend/src/pages/DashboardPage.tsx`

Reference: `docs/redesign/preview/dashboard.jsx:460-565`.

- [ ] **Step 1: Прочитать текущий DashboardPage**

Запомнить какие виджеты используются, в какой раскладке.

- [ ] **Step 2: Обернуть в новый layout с PageHeader + 3-колоночный grid**

Структура (псевдокод):
```tsx
import { PageHeader } from '@/components/layout/PageHeader';
import { Card, CardHeader, CardTitle, CardSubtitle, CardContent } from '@/components/ui/card';
// ... остальные импорты

export function DashboardPage() {
  return (
    <>
      <PageHeader title="Dashboard" simTime={simTime} right={<Button size="sm" variant="ghost"><RotateIcon className="h-3.5 w-3.5"/>Reset sim</Button>} />
      <div className="p-6 max-w-[1920px] mx-auto">
        <div className="grid grid-cols-12 gap-4">
          {/* Col 1 */}
          <div className="col-span-12 xl:col-span-4 space-y-4">
            <Card><CardHeader><CardTitle>Camera</CardTitle></CardHeader><CardContent className="p-3"><CameraFeed /></CardContent></Card>
            <Card><CardContent className="space-y-3"><CommandInput onSend={...} /><QuickCommandButtons /><DetectionBanner /></CardContent></Card>
            <Card><CardHeader><CardTitle>Detected balls</CardTitle></CardHeader><CardContent className="p-0"><BallsTable /></CardContent></Card>
          </div>
          {/* Col 2 */}
          <div className="col-span-12 xl:col-span-5 space-y-4">
            <Card><CardHeader right={<MapToolbar {...} />}><CardTitle>Map</CardTitle></CardHeader><CardContent className="p-3"><MapCanvas /></CardContent></Card>
            <Card><CardHeader><CardTitle>Telemetry</CardTitle></CardHeader><CardContent><SensorCharts /></CardContent></Card>
            <Card><CardHeader><CardTitle>Event log</CardTitle></CardHeader><CardContent className="p-2"><EventLog /></CardContent></Card>
          </div>
          {/* Col 3 */}
          <div className="col-span-12 xl:col-span-3 space-y-4">
            <Card><CardContent><FsmBadge state={fsm} target={target} />{/* status banner inline */}</CardContent></Card>
            <Card><CardHeader><CardTitle>State machine</CardTitle></CardHeader><CardContent className="space-y-4"><FsmTimeline current={fsm} /></CardContent></Card>
            <Card><CardHeader><CardTitle>Sensors</CardTitle></CardHeader><CardContent><SensorPanel /></CardContent></Card>
            <Card><CardHeader><CardTitle>Actuators</CardTitle></CardHeader><CardContent><ActuatorToggles /></CardContent></Card>
            <Card><CardHeader><CardTitle>Joystick</CardTitle></CardHeader><CardContent><JoystickControl /></CardContent></Card>
            <Card><CardHeader><CardTitle>Detect colors</CardTitle></CardHeader><CardContent><DetectionTogglePanel /></CardContent></Card>
            <Card><CardHeader><CardTitle>Path recorder</CardTitle></CardHeader><CardContent><PathRecorderPanel /></CardContent></Card>
          </div>
        </div>
      </div>
    </>
  );
}
```

**Важно:** все виджеты, которые есть в текущей DashboardPage — **должны остаться**. Добавить недостающие из preview (DetectionBanner, FsmTimeline). Удалить ничего нельзя без явного согласия пользователя.

- [ ] **Step 3: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/dashboard`. Проверить:
- 3 колонки (на широком экране)
- Все виджеты на месте
- Кликабельность виджетов работает (отправка команд, кнопки)
- Console чистый

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/pages/DashboardPage.tsx
git commit -m "redesign(pages): DashboardPage — 3-колоночный grid с FsmTimeline и новыми Card-обёртками"
```

---

### Task 23: `AdminPage` — со ServoSlider и FsmManualGrid

**Files:**
- Modify: `compute_node/frontend/src/pages/AdminPage.tsx`
- Create (если такого ещё нет): `compute_node/frontend/src/components/controls/ServoSlider.tsx`
- Create: `compute_node/frontend/src/components/fsm/FsmManualGrid.tsx`

Reference: `docs/redesign/preview/other-pages.jsx:7-162`.

- [ ] **Step 1: ServoSlider — реализовать**

```tsx
import { useState } from 'react';
import { cn } from '@/lib/utils';

export interface ServoSliderProps {
  label: string;
  value: number;
  min?: number;
  max?: number;
  onChange?: (v: number) => void;
  className?: string;
}

export function ServoSlider({ label, value, min = 0, max = 180, onChange, className }: ServoSliderProps) {
  const [v, setV] = useState(value);
  const pct = ((v - min) / (max - min)) * 100;

  function update(next: number) {
    setV(next);
    onChange?.(next);
  }

  return (
    <div className={cn('space-y-1.5', className)}>
      <div className="flex items-center justify-between">
        <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">{label}</span>
        <span className="font-mono text-small tabular-nums">{v}°</span>
      </div>
      <div className="relative h-1.5 rounded-full bg-surface-3">
        <div className="absolute left-0 top-0 h-full rounded-full bg-accent" style={{ width: `${pct}%` }} />
        <input
          type="range" min={min} max={max} value={v}
          onChange={e => update(+e.target.value)}
          className="absolute inset-0 w-full opacity-0 cursor-pointer"
        />
        <div
          className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 h-3 w-3 rounded-full bg-accent border-2 border-background pointer-events-none"
          style={{ left: `${pct}%` }}
        />
      </div>
      <div className="flex items-center justify-between font-mono text-micro text-foreground-faint">
        <span>{min}°</span><span>{max}°</span>
      </div>
    </div>
  );
}
```

- [ ] **Step 2: FsmManualGrid — реализовать**

```tsx
import type { FsmState } from '@/types/robot';
import { FSM_ORDER, FSM_SHORT } from './fsm-styles';
import { cn } from '@/lib/utils';

export interface FsmManualGridProps {
  current: FsmState;
  onTransition?: (state: FsmState) => void;
}

export function FsmManualGrid({ current, onTransition }: FsmManualGridProps) {
  return (
    <div className="grid grid-cols-4 gap-1.5">
      {FSM_ORDER.map(s => (
        <button
          key={s}
          onClick={() => onTransition?.(s)}
          className={cn(
            'h-9 rounded-md border font-mono text-micro uppercase tracking-wider transition-colors',
            s === current
              ? 'border-accent bg-accent/10 text-accent'
              : 'border-subtle bg-surface-2 text-foreground-muted hover:bg-surface-3 hover:text-foreground'
          )}
        >
          {FSM_SHORT[s]}
        </button>
      ))}
    </div>
  );
}
```

- [ ] **Step 3: Переписать AdminPage**

Сборка по образцу preview `other-pages.jsx:61-162`. Сохранить все существующие виджеты + добавить FsmManualGrid и ServoSlider там, где раньше были другие контролы. Подключить к реальным API через `lib/api.ts` (`forceTransition`, `setArmJoint`, etc.).

- [ ] **Step 4: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/admin`. Все секции отображаются, FSM-кнопки кликаются, слайдеры скользят, EmergencyStop виден.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/pages/AdminPage.tsx compute_node/frontend/src/components/controls/ServoSlider.tsx compute_node/frontend/src/components/fsm/FsmManualGrid.tsx
git commit -m "redesign(pages): AdminPage — FsmManualGrid, ServoSlider, новая раскладка с PathBtn/SpeedBtns"
```

---

### Task 24: `HardwarePage` — HwBlock + PresetPanel

**Files:**
- Modify: `compute_node/frontend/src/pages/HardwarePage.tsx`
- Modify: `compute_node/frontend/src/components/hardware/*` (HardwareBlockDiagram, MotorBlock, ServoBlock, SensorBlock, LedBlock, PresetPanel, PlatformSelector)

Reference: `docs/redesign/preview/other-pages.jsx:298-426`.

- [ ] **Step 1: Прочитать текущие компоненты в `components/hardware/`**

Сохранить существующую логику drag-n-drop, custom event `'hw-preset-save'`. Меняем только классы и обёртки в Card/CardHeader.

- [ ] **Step 2: Применить новые токены к каждому**

Каждый компонент → обернуть в новый `<Card>`, подменить классы на `surface-2 / border-subtle / hover:border-strong`. Активные состояния пресета — `border-accent/40 bg-accent/8 text-foreground`.

- [ ] **Step 3: HardwarePage layout**

3-колоночная раскладка как в preview (PlatformSelector / BlockDiagram / Preset+I/O map). Сохранить custom event при сохранении.

- [ ] **Step 4: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/hardware`. Платформа-селектор работает, блоки рендерятся, сохранение пресета триггерит `hw-preset-save`.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/pages/HardwarePage.tsx compute_node/frontend/src/components/hardware/
git commit -m "redesign(pages): HardwarePage — Card-обёртки, новые состояния пресета"
```

---

### Task 25: `SamcanPage` — SVG-визуализации (Compass, Radar, Motors, Arm)

**Files:**
- Modify: `compute_node/frontend/src/pages/SamcanPage.tsx`
- Modify: `compute_node/frontend/src/components/samcan/{HeadingCompass,DistanceRadar,MotorBars,ArmVisualizer}.tsx`
- Modify: `compute_node/frontend/src/components/samcan/{TelemetryHistory,VirtualJoystick,PresetManager,SamcanEventLog,ConnectionDiagnostics,ReverseFinder}.tsx`

Reference: `docs/redesign/preview/other-pages.jsx:431-640`.

- [ ] **Step 1: HeadingCompass — переписать SVG под новые токены**

Использовать `hsl(var(--surface-2))`, `hsl(var(--accent))`, `hsl(var(--foreground-faint))` вместо хардкоднутых hex. Логика углов — без изменений.

- [ ] **Step 2: DistanceRadar — аналогично**

Цвет дуг → `hsl(var(--surface-3))`, sweep → `hsl(var(--info))`, obstacle → `hsl(var(--warning))` с `animate-pulse-soft`.

- [ ] **Step 3: MotorBars + ArmVisualizer — palette swap**

Полоски моторов: `bg-surface-2` фон, `bg-accent` для +PWM, `bg-info` для −PWM. Arm: линии `foreground-muted`, joints `accent`-точки.

- [ ] **Step 4: TelemetryHistory + остальные компоненты — обёртки в новые Card**

Сохранить useSamcan() и поллинг — не трогаем. Только классы и Card.

- [ ] **Step 5: SamcanPage layout**

3-колоночный (1600px max), как в preview. Все существующие компоненты остаются + новые карточки `Quick`, `Heading`, `Motors`.

- [ ] **Step 6: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/samcan`. Если есть подключённый Arduino — данные обновляются. Иначе — все виджеты статичные, но рендерятся.

- [ ] **Step 7: Commit**

```bash
git add compute_node/frontend/src/pages/SamcanPage.tsx compute_node/frontend/src/components/samcan/
git commit -m "redesign(pages): SamcanPage — Compass/Radar/MotorBars/Arm в новой палитре"
```

---

### Task 26: `Visualization3DPage` — overlay-контролы поверх Canvas

**Files:**
- Modify: `compute_node/frontend/src/pages/Visualization3DPage.tsx` (если ещё не финализирован после Task 20)

Reference: `docs/redesign/preview/other-pages.jsx:167-296`.

- [ ] **Step 1: Добавить PageHeader и overlay-контролы**

```tsx
<>
  <PageHeader title="3D View" simTime={simTime} right={<><Button size="sm" variant="secondary">Reset cam</Button><Button size="sm" variant="secondary">Snapshot</Button></>} />
  <div className="p-6 max-w-[1920px] mx-auto">
    <div className="grid grid-cols-12 gap-4">
      <div className="col-span-12 xl:col-span-9">
        <Card>
          <CardHeader><CardTitle>3D Scene</CardTitle></CardHeader>
          <CardContent className="p-0">
            <div className="relative aspect-[16/9]">
              <Canvas>{/* сцена из Task 20 */}</Canvas>
              {/* InfoPanel в углу + Toggle EKF/SLAM/Path в правом-верхнем */}
              {/* Кнопки Clear/Reset в правом-нижнем */}
              {/* hint про управление в левом-нижнем */}
            </div>
          </CardContent>
        </Card>
      </div>
      <div className="col-span-12 xl:col-span-3 space-y-4">
        <Card><CardHeader><CardTitle>Layers</CardTitle></CardHeader><CardContent>{/* Toggle по слоям */}</CardContent></Card>
        <Card><CardHeader><CardTitle>Camera</CardTitle></CardHeader><CardContent>{/* Top/Side/Front/ISO + Follow robot */}</CardContent></Card>
      </div>
    </div>
  </div>
</>
```

- [ ] **Step 2: Build + dev preview**

```bash
npm run build && npx tsc --noEmit
npm run dev
```

Открыть `/3d`. Сцена рендерится, overlay-контролы кликаются.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/pages/Visualization3DPage.tsx
git commit -m "redesign(pages): Visualization3DPage — overlay-контролы (Layers, Camera presets, Reset)"
```

---

### Task 27: `MpsPage` — лёгкое обновление (только токены, без структурных правок)

**Files:**
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx`
- Modify: `compute_node/frontend/src/components/mps/*.tsx` (по необходимости)

- [ ] **Step 1: Обернуть в PageHeader**

```tsx
<>
  <PageHeader title="МПС" />
  <div className="p-6 max-w-[1920px] mx-auto">
    {/* существующее содержимое — оставляем как есть */}
  </div>
</>
```

- [ ] **Step 2: Если в MpsPage есть `<Card>` старого вида — toks автоматически переключатся через CSS-vars**

Не правим внутренности, оставляем reading-friendly стиль.

- [ ] **Step 3: Build + dev preview**

Открыть `/mps`. Должно выглядеть согласно теме (тёплый фон, тот же контент).

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/pages/MpsPage.tsx
git commit -m "redesign(pages): MpsPage — обёртка PageHeader, контент без изменений"
```

---

## Phase 9 — Polish & verification

### Task 28: Light theme — проверка на всех 6 страницах

**Files:**
- (правок по необходимости в `index.css` light-блоке если что-то не читается)

- [ ] **Step 1: Запустить dev**

```bash
npm run dev
```

- [ ] **Step 2: Через ThemeToggle переключиться на light**

- [ ] **Step 3: Пройти все 6 маршрутов в light, заметить проблемы**

Что проверять:
- Текст читается на кремовом фоне
- Coral акцент даёт контраст
- Графики (Recharts) видны
- Карта читается (без mix-blend-multiply на light, иначе всё бледно)
- 3D-сцена остаётся тёмной (это by design)
- Sidebar контрастен

- [ ] **Step 4: Если что-то не так — поправить light-блок в `index.css`**

Например, добавить `shadow-sm` карточкам в light:
```css
.light .surface-1, .light [class*="bg-surface-1"] {
  box-shadow: 0 1px 2px 0 rgba(0,0,0,0.04);
}
```

(Или через Tailwind utilities в местах потребления.)

- [ ] **Step 5: Build + commit (если были правки)**

```bash
npm run build && npx tsc --noEmit
git add compute_node/frontend/src/index.css
git commit -m "redesign(polish): light-тема — корректировки контраста по результатам ручной проверки"
```

Если правок не понадобилось — пропустить коммит.

---

### Task 29: Скриншоты для `docs/redesign/screenshots/`

**Files:**
- Create: `docs/redesign/screenshots/*.png` (множество)

- [ ] **Step 1: Создать папку**

```bash
mkdir -p docs/redesign/screenshots
```

- [ ] **Step 2: Запустить dev**

```bash
npm run dev
```

(Запустить и backend если нужны live-данные — `./samurai.sh sim` или `./samurai.sh compute`.)

- [ ] **Step 3: Снять скриншоты вручную**

Для каждого: Win+Shift+S → выделить окно → сохранить в `docs/redesign/screenshots/`:

- `dashboard-dark.png`, `dashboard-light.png`
- `admin-dark.png`, `admin-light.png`
- `3d-dark.png`, `3d-light.png`
- `hardware-dark.png`, `hardware-light.png`
- `samcan-dark.png`, `samcan-light.png`
- `mps-dark.png`, `mps-light.png`
- `sidebar-expanded.png`, `sidebar-collapsed.png`

- [ ] **Step 4: Commit**

```bash
git add docs/redesign/screenshots/
git commit -m "docs(redesign): скриншоты после редизайна (dark + light, все маршруты)"
```

---

### Task 30: Dev-checklist run-through (PROMPT.md §11.5)

**Files:**
- Create: `docs/redesign/CHECKLIST.md` — заполненный отчёт

- [ ] **Step 1: Скопировать чек-лист из PROMPT.md §11.5 в `CHECKLIST.md`**

Каждый пункт проверить и пометить `[x]` если выполнен, `[~]` если частично, `[ ]` если не выполнен (с комментарием почему).

- [ ] **Step 2: Особое внимание контракту**

Запустить:
```bash
git diff main -- compute_node/frontend/src/hooks/
git diff main -- compute_node/frontend/src/providers/
git diff main -- compute_node/frontend/src/lib/api.ts
git diff main -- compute_node/frontend/src/lib/constants.ts
git diff main -- compute_node/frontend/src/types/
git diff main -- compute_node/frontend/src/stores/
git diff main -- compute_node/frontend/vite.config.ts
git diff main -- compute_node/
```

Везде, где НЕ должно быть изменений — должно быть пусто. Если есть — анализировать (если только refactor типизации — допустимо; если изменена логика — откатить).

- [ ] **Step 3: package.json — проверить, что новых runtime-deps нет**

```bash
git diff main -- compute_node/frontend/package.json
```

Допустимо: ничего не изменилось, либо изменились только devDependencies (что тоже лучше избегать). Никаких новых deps в `dependencies`.

- [ ] **Step 4: Final build + tests + lint**

```bash
cd compute_node/frontend
npm run build
npx tsc --noEmit
npm run lint
npm test
```

Все четыре — PASS, без новых warnings.

- [ ] **Step 5: Commit checklist**

```bash
git add docs/redesign/CHECKLIST.md
git commit -m "docs(redesign): заполненный dev-checklist"
```

---

### Task 31: Push + готовность к ревью

- [ ] **Step 1: Push всей ветки**

```bash
git push origin feat/redesign
```

- [ ] **Step 2: Открыть PR**

```bash
gh pr create --base dev --title "Redesign UI: тёмный Claude" --body "$(cat <<'EOF'
## Summary
- Перерисован визуальный слой `compute_node/frontend/` согласно `docs/redesign/PROMPT.md`
- Тёплый графит + coral акцент, обе темы (dark default + light toggle)
- Новый Sidebar layout (collapsible, persisted)
- HUD на CameraFeed, FsmTimeline, terminal CommandInput, технический Joystick
- Recharts restyle, 3D-сцена с rim-lights и fog
- Все ~60 компонентов сохранены, hooks/providers/api/types/stores не тронуты

## Test plan
- [ ] `npm run build` зелёный
- [ ] `npm test` все тесты PASS (новые: fsm-styles, ball-styles, useTheme, Sidebar, FsmTimeline)
- [ ] `npx tsc --noEmit` без ошибок
- [ ] `npm run lint` без новых warning'ов
- [ ] Все 6 маршрутов открываются в dark + light без console-ошибок
- [ ] Скриншоты обеих тем в `docs/redesign/screenshots/`
- [ ] Dev-checklist из PROMPT.md §11.5 заполнен в `docs/redesign/CHECKLIST.md`
EOF
)"
```

- [ ] **Step 3: Готово**

PR открыт, проверен, ждёт ревью пользователя.

---

## Self-review

После прогона плана с фрешим взглядом:

**Spec coverage** — каждый раздел PROMPT.md покрыт:
- §3 Tokens → Tasks 1, 2, 3, 4
- §4 CSS/Tailwind → Tasks 2, 3
- §5 Layout → Tasks 6, 7, 8
- §6 Components → Tasks 9-21 (по компонентам)
- §7 Animations → Tasks 2 (keyframes), множество мест
- §8 Pages → Tasks 22-27
- §9 Light theme → Task 28
- §10 Deliverables → Task 31 (PR)
- §11 Acceptance → Tasks 28-30 (light/screenshots/checklist)

**Placeholder scan** — нет TBD/TODO в шагах. В нескольких местах есть условные «если такое уже есть — оставить» — это не placeholder, а явное условие.

**Type consistency** — `FSM_TEXT_CLASS`, `FSM_BG_CLASS`, `FSM_DOT_HEX`, `FSM_SHORT`, `FSM_ORDER`, `BALL_HEX`, `BALL_BG_CLASS`, `BALL_TEXT_CLASS`, `BALL_NAMES` — определены в Task 4, используются единообразно во всех последующих тасках. `FsmBadge` props (`state, target, compact, className`) — заданы в Task 13, используются с теми же именами в Tasks 16, 22.

**Гранулярность** — большинство тасков 30-90 минут чистой работы. Большие (Task 22 DashboardPage, Task 25 SamcanPage) — до 2-3 часов из-за объёма кода, но поделить их сложнее без потери коherence.

---

**Plan complete.** Сохранён в `docs/redesign/PLAN.md`.
