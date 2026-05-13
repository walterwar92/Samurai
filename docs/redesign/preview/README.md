# Redesign Preview

Самодостаточный React-прототип нового дизайна, выданный на этапе генерации.
**Это референс**, не production-код. Реальный порт идёт в `compute_node/frontend/`
по спеке [`../PROMPT.md`](../PROMPT.md).

## Что внутри

| Файл | Что |
|------|-----|
| `index.html` | HTML-обёртка: Tailwind через CDN, React 18 + Babel standalone (через unpkg), `<script type="text/babel">` подгружает 4 .jsx |
| `components.jsx` | `cn`, ~25 иконок (KatanaIcon, DashIcon, …), shadcn-ish primitives (Card, Button, Badge, Kbd, Tooltip), FsmBadge, FsmTimeline, Sidebar, PageHeader |
| `dashboard.jsx` | DashboardPage и его виджеты: CameraFeed (HUD + scanline + bbox), CommandInput (терминал), QuickCommandButtons, DetectionBanner, BallsTable, MapCanvas (SVG), MapToolbar, SensorCharts (sparklines), Joystick, SensorPanel, Toggle, ActuatorToggles, EventLog, StatusBanner, DetectionTogglePanel, PathRecorderPanel |
| `other-pages.jsx` | AdminPage, Visualization3DPage, HardwarePage, SamcanPage + их сабкомпоненты (ServoSlider, FsmManualGrid, HwBlock, HeadingCompass, DistanceRadar, MotorBars, ArmVisualizer) |
| `app.jsx` | Root `<App>` со стейтом темы / sidebar / роута / FSM, тикающим simTime, `ReactDOM.createRoot` |

Все межфайловые связи — через `window.<Name>` (Object.assign в конце каждого .jsx).
Это работает только в этом standalone-прототипе. В реальном фронте всё импортируется ESM.

## Как посмотреть

Из-за `<script src="...jsx">` нужен HTTP-сервер (file:// CORS не пустит):

```bash
# Из этой папки
python -m http.server 8000
# затем открыть http://localhost:8000
```

Или любой другой статик-сервер (`npx serve`, VS Code Live Server, etc.).

## Чем отличается от боевого фронта

| Аспект | Preview | Compute_node/frontend |
|--------|---------|----------------------|
| Сборка | Tailwind CDN + Babel standalone | Vite + tsc |
| Язык | JSX | TSX |
| Tailwind config | inline в `<script>` | `tailwind.config.ts` |
| Интер-файловые связи | `window.<Name>` | ESM imports |
| Состояние робота | заглушки (детекции/мячи захардкожены) | `useRobotState` через Socket.IO |
| Камера | placeholder с диагональными полосами | `<img src="/video_feed">` |
| Карта | SVG-набросок | `<canvas>` + `useMapCanvas` + реальный `/map.png` |
| 3D-сцена | SVG-набросок | three.js + R3F |
| Маршрутизация | один компонент `<App>` со стейт-роутером | React Router (`/dashboard`, `/admin`, `/3d`, `/hardware`, `/samcan`) |
| FSM | таймер 6с переключает циклически | реальное состояние из бэкенда |

## Как использовать как референс при порте

1. Открыть превью в браузере (см. выше).
2. Открыть боевой фронт (`./samurai.sh compute` → `:5000/dashboard`).
3. Сравнить попиксельно.
4. Брать токены/классы/SVG из preview и применять в `compute_node/frontend/src/...` согласно [`../PROMPT.md`](../PROMPT.md).

**Что брать напрямую (skopipasta):**
- `index.html` `<style>` блок → `src/index.css`
- `tailwind.config` объект → `tailwind.config.ts`
- SVG-иконки из `components.jsx` → `src/components/icons/*.tsx`
- SVG-визуализации (HeadingCompass, DistanceRadar, ArmVisualizer, MotorBars) → `src/components/samcan/*.tsx`
- JSX-разметка виджетов → внутрь существующих `compute_node/frontend/src/components/*` с заменой заглушек на реальные хуки

**Что НЕ брать:**
- `window.<Name>` экспорты — в TS используем ESM imports
- Заглушки данных (BALLS, EVENTS, detections) — в реальном фронте они приходят из `useRobotState`
- `app.jsx`-овский стейт-роутер — у нас React Router
- Захардкоженный `connected = true` — в реале из `useSocket().connected`

## Источник

Сгенерировано по спеке [`../PROMPT.md`](../PROMPT.md). Сохранено как референс
для последующего порта в боевой фронт.
