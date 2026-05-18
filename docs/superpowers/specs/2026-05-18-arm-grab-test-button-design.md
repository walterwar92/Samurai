# Кнопка «Тест захвата» в UI

> Кнопка в Arm-секции `ServoControlPanel`, которая прогоняет
> arm-секвенцию `grab_ready → grab_hold → freeze` без движения робота
> и без детектора. Цель — проверить позы и плавность на железе
> отдельно от FSM-ханта.
>
> **Базируется на:** ветка после реализации
> `docs/superpowers/specs/2026-05-17-arm-grab-sequence-design.md`
> (коммиты `ccabda5..a651ffc`).
> **Скоуп:** только
> `compute_node/frontend/src/components/actuators/ServoControlPanel.tsx`
> + ребилд `compute_node/static/`. Без изменений в бэкенде, Pi-нодах,
> схемах MQTT, конфиге.

## 1. Контекст

После [2026-05-17-arm-grab-sequence](2026-05-17-arm-grab-sequence-design.md)
у нас есть автономный FSM-хант (voice «возьми красный мяч»), который
делает полный цикл `SEARCHING → ... → GRABBING → RETURNING`. Чтобы
проверять две новые позы (`grab_ready`, `grab_hold`) и плавность
интерполятора на физическом железе без катания робота и без мяча —
нужна UI-кнопка, которая прогоняет только arm-часть сценария.

Пресеты `grab_ready=[160, 100, 180, 0]` и `grab_hold=[10, 30, 180, 180]`
уже мигрируются `arm_node` на старте Pi. Endpoint'ы
`POST /api/v1/actuators/arm` принимают `preset`, `freeze`, `joints[]`
и т.д. — никаких новых маршрутов не нужно.

## 2. Поведение

Кнопка «Тест захвата» в Arm-секции `ServoControlPanel`. Onclick — async:

```
1. armCommand('unfreeze')      — снять прошлую заморозку (idempotent)
2. armLoadPreset('grab_ready') — pre-grab поза
3. sleep(GRAB_TEST_SETTLE_MS)  — ждём интерполятор (1500мс)
4. armLoadPreset('grab_hold')  — поза захвата
5. sleep(GRAB_TEST_SETTLE_MS)  — ждём интерполятор (1500мс)
6. armCommand('freeze')        — удерживаем
```

Итого ~3.0с от клика до freeze. По окончании рука остаётся в
`grab_hold` под `arm/command freeze`. Чтобы выйти — пользователь
жмёт UI «Open claw» (auto-unfreeze из Task 6 старой спеки).

### 2.1 UI-состояние

Локальный state в компоненте:

```typescript
type TestPhase = null | 'unfreezing' | 'grab_ready' | 'grab_hold' | 'freezing'
const [testPhase, setTestPhase] = useState<TestPhase>(null)
```

Лейбл кнопки по фазе:
| `testPhase` | Лейбл | `disabled` |
|---|---|---|
| `null` (idle) | «Тест захвата» | `armLocked` |
| `'unfreezing'` | «Тест: разморозка…» | `true` |
| `'grab_ready'` | «Тест: pre-grab…» | `true` |
| `'grab_hold'` | «Тест: захват…» | `true` |
| `'freezing'` | «Тест: freeze…» | `true` |

После шага 6 — `setTestPhase(null)` → кнопка снова доступна, но
существующий `StatusBadge` показывает `FROZEN` (рука действительно
заморожена через arm/state).

### 2.2 Место в UI

В разделе Arm рядом с существующими кнопками «Домой» и «Замор. все».
Variant `outline`, размер `sm` (как соседи). Стиль не привлекает
внимание — это диагностический инструмент, не основной flow.

## 3. Тайминги

```typescript
// Соответствует spec 2026-05-17-arm-grab-sequence §3.4:
// settle = 150°/max_speed + 0.25c. При max_speed_deg_per_sec=120 (config)
// это 1.25с + 0.25с jitter = 1.5с. Если конфиг изменится — этот
// литерал тоже надо обновить (нет автоматической синхронизации
// фронта с Pi-конфигом).
const GRAB_TEST_SETTLE_MS = 1500
```

## 4. Edge cases

| Случай | Поведение |
|---|---|
| `armLocked` (рука не разблокирована) | Кнопка disabled. Tooltip/hint не нужен — рядом видна кнопка «Разблокировать». |
| Пользователь нажал кнопку ещё раз во время run | Кнопка `disabled` → клик игнорируется браузером. |
| Пользователь перезагрузил страницу мид-секвенцию | Frontend state теряется, текущий MQTT-командный поток уже доставлен; рука доедет до последней отправленной цели и не пойдёт дальше. Безопасно: останется в промежуточной позе. Восстановление — клик «Домой» или повторный «Тест захвата». |
| API-ошибка на шаге N (HTTP 4xx/5xx) | Промис reject, setTestPhase в `catch` сбрасывается в null. Рука остаётся в последней успешной позе. Логируем `console.error`, alert/toast не добавляем (минимум шума). |
| Очень медленный network → 1500мс между POST'ами не успевает | Запасной jitter 0.25с заложен. На локальной сети это не происходит. |
| Двойной клик до того как state обновился (React batching) | `disabled` ставится до первого `await`. Второй клик придёт в already-disabled button — браузер не дёрнет onclick. |

## 5. Что НЕ делаем

- Не добавляем backend endpoint (`/api/v1/actuators/arm/test-grab`). Frontend оркестрация трёх существующих вызовов проще и не требует Pi/compute изменений.
- Не делаем cancel-кнопку. Если что — рядом «Разм. все» / «Домой» / «Open claw».
- Не пишем vitest. Кнопка вызывает existing endpoints, у которых есть pytest-покрытие (Task 6). Frontend-тестов в проекте нет; добавлять vitest setup ради одной кнопки — disproportionate.
- Не делаем polling `arm/state` для точного «доехал». Фиксированные 1500мс с запасом по jitter — простое и достаточное решение.

## 6. Тестирование

Только manual smoke на железе (плюс существующая pytest-регрессия не должна сломаться, но мы трогаем только TSX):

1. UI разблокировать руку → нажать «Тест захвата»
2. Наблюдать: лейбл проходит фазы за ~3с, рука едет grab_ready → grab_hold плавно
3. После завершения — StatusBadge показывает FROZEN, попытка двигать слайдер CH0 ничего не делает
4. Кликнуть Open claw → CH3 → 0, FROZEN → ACTIVE, рука снова управляема

## 7. Открытые вопросы

Нет.
