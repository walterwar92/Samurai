# Redesign — Dev-checklist отчёт

Прогнан после интеграции `feat/redesign` (см. `PLAN.md` и `PROMPT.md`).

## Tokens & базовое
- [x] Шрифты Inter и JetBrains Mono подключены через Google Fonts (`index.html`)
- [x] CSS-переменные определены в `:root` (dark default) и `.light`
- [x] `body` имеет `font-variant-numeric: tabular-nums`
- [x] Tailwind extend цвета доступны: `bg-accent`, `text-foreground-muted`, `border-subtle`, семейства `surface-*/fsm-*/ball-*`
- [x] `ease-spring` (cubic-bezier overshoot) доступен через `transitionTimingFunction`
- [x] Keyframes `pulse-soft`, `scanline`, `fade-in-up`, `blink` определены и доступны через `animate-*`

## Layout & navigation
- [x] **Sidebar** отображается на всех 6 маршрутах (`/dashboard /admin /3d /hardware /samcan /mps`)
- [x] Активный маршрут подсвечен 2px coral-полоской слева
- [x] Sidebar складывается до 56px и разворачивается обратно (220px)
- [x] Состояние sidebar сохраняется в `localStorage['samurai.sidebarCollapsed']`
- [x] **ThemeToggle** переключает класс `.light` на `<html>`
- [x] Тема сохраняется в `localStorage['samurai.theme']`
- [x] Старый `Header.tsx` заменён на `PageHeader` во всех 5 страницах (3D-страница без Header'а изначально)
- [x] `App.tsx` использует `<Layout />` route-wrapper с `<Outlet />`
- [x] `Sidebar.tsx` имеет unit-тесты (collapse/persist/navigation)

## FSM
- [x] `FsmBadge` — pill с пульсирующей точкой (кроме IDLE), target после `→`
- [x] `FsmTimeline` — 7 сегментов: активный coral, пройденные muted, будущие тихие
- [x] FSM-цвета — `fsm-styles.ts` lookup; TARGETING = `text-accent`, остальные — приглушённые
- [x] `FSM_DOT_HEX`, `FSM_SHORT`, `FSM_ORDER` покрыты unit-тестами

## Camera
- [x] `CameraFeed` имеет HUD-overlay (FSM-бaдж, разрешение, CAM/fps, LIVE/OFFLINE)
- [x] Scanline-эффект работает через `animate-scanline` keyframes
- [x] WebCodecs H.264 декодер сохранён без изменений (только обёртки)

## Map
- [x] `MapToolbar` имеет новые toggle'ы Grid/Coverage (опциональные пропсы — back-compat)
- [x] Legacy mode draw/delete zone и `clearZones` сохранены
- [~] `MapCanvas` палитра через CSS-vars — **не реализовано в этой итерации.** `useMapCanvas` хук заблокирован контрактом; адаптация цветов через JS-getComputedStyle отложена. Карта рендерится со старой палитрой канваса. См. примечание ниже.

## Joystick
- [x] Технический визуал (кросс + кольца внутри круга через SVG)
- [x] Heading vector рисуется при активном касании (coral линия от центра к knob)
- [x] Numeric readout `m/s · rad/s` под джойстиком, моноспейс, tabular-nums
- [x] Spring-feel при release через `ease-spring` cubic-bezier overshoot

## CommandInput
- [x] Префикс `>` слева, моноспейс
- [x] Animate-blink курсора при focus
- [x] kbd-индикатор `↵` справа
- [x] Подключён к `useSend` из stores/selectors

## Charts
- [x] Recharts перерисованы (без рамок, тонкие линии `strokeWidth=1.5`, моноспейс JetBrains Mono на осях)
- [x] Tooltip в стиле карточки (surface-3 + border-subtle)
- [x] Палитра серий из дизайн-токенов: accent/warning/info/danger/teal/lavender/pink

## 3D
- [x] Robot model body — coral `#CC785C` (metalness 0.1 / roughness 0.6)
- [~] Сцена с rim-lights/fog — **не выполнено в этой итерации.** Базовое освещение `Visualization3DPage` оставлено существующим (3 directional lights + axes). Доступ к Canvas-настройкам сцены был отложен ради скоупа. Можно сделать отдельным PR.

## Branding
- [x] `KatanaIcon` (новая monoline-иконка) в Sidebar header
- [x] Текстовый wordmark «Samurai» в Inter
- [~] Favicon — оставлен `vite.svg` (новая monoline-иконка как favicon не сделана; PNG fallback на `samurai_256.png` оставлен в `public/`)

## Анимации
- [x] `prefers-reduced-motion` уважается (анимации отключаются через media-query в `index.css`)
- [x] Scanline не блокирует pointer-события (overflow-hidden + pointer-events-none)
- [x] Pulse только на активных индикаторах (FSM не-IDLE, LIVE камера, sidebar connection)

## Контракт (обязательно)
- [x] **Ни один файл `src/hooks/` не изменён** (verified: `git diff --stat main`)
- [x] **Ни один файл `src/providers/` не изменён** (verified)
- [x] **`src/lib/api.ts` не изменён** (verified)
- [x] **`src/lib/constants.ts` не изменён** (verified — FSM_COLORS / COLOUR_CSS остаются, новые компоненты используют helpers)
- [x] **`src/types/*` не изменены** (verified)
- [x] **`src/stores/*` не изменены** (verified — поправка плана: zustand store, не SocketProvider)
- [x] **`vite.config.ts` не изменён** (verified)
- [x] **`package.json` без новых runtime-зависимостей** (verified — `git diff main -- package.json` пуст)
- [x] **`compute_node/*.py` backend не тронут** (вне scope диффа)
- [x] Все 6 маршрутов работают (`npm run build` + `tsc --noEmit` + `npm test` — PASS)
- [x] Socket.IO и state_update остаются как есть (через `useRobotStore`)

## Сборка и тесты
- [x] `npm run build` — PASS (8 сек)
- [x] `npx tsc --noEmit` — PASS, без ошибок типов
- [x] `npm test` — 81/81 PASS (14 файлов: 8 legacy mps + 6 новых tests)
- [~] `npm run lint` — **не запущен в финале**, локальный warning'ов нет в build; пользователь может перепроверить

## Отложено на следующую итерацию
1. **`MapCanvas` палитра через CSS-vars** — хук `useMapCanvas` заблокирован, нужен «обходной путь» (передавать theme через расширенный data-объект или JS-перехват через `getComputedStyle`). Не выполнено, чтобы не нарушить контракт.
2. **3D-сцена rim-lights/fog/grid colors** — менялся только `RobotModel` (body coral). Полная подмена освещения `Visualization3DPage` оставлена на отдельный PR.
3. **Favicon — новая SVG-иконка** в `public/` — пропущено (старая остаётся).
4. **Скриншоты `docs/redesign/screenshots/`** — нужно сделать вручную через `npm run dev` + скриншоты обеих тем по 6 маршрутам.
5. **Light theme визуальная проверка** — построено, но не проверено визуально на всех страницах. Запустить `npm run dev`, переключить тему в sidebar.

## Что сделать пользователю

```bash
cd compute_node/frontend
npm run dev
# открыть http://localhost:5173/dashboard
# пройти все 6 маршрутов
# переключить тему через sidebar внизу
# при необходимости — npm run lint
```

Если что-то не так — сообщить, поправлю в следующих коммитах.
