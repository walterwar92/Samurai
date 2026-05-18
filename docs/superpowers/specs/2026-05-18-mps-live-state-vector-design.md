# MPS — Live State Vector в реальном времени

> Постоянный блок на странице `/mps`, показывающий текущий вектор
> состояния `x ∈ ℝ⁵` и управление `u ∈ ℝ²` робота в реальном времени
> (10 Hz) — независимо от того, запущен сценарий или нет.
>
> **Скоуп:** новый MQTT-топик `samurai/{robot_id}/mps/live_state`,
> новый WS-эндпоинт `/ws/mps/live_state`, новый хук
> `useMpsLiveState` и компонент `LiveStateVector` в левом sidebar
> над `OdeCard`.

## 1. Контекст

Сейчас на `/mps` вектор состояния виден только в двух местах:
- `ResultPlots`/`TrajectoryView` — реплей телеметрии завершённого
  прогона (после `finished`).
- `useMpsLiveTelemetry` (`/ws/mps/telemetry`) — live во время
  активного прогона, привязан к `run_id`.

Вне прогона страница ничего не показывает про текущее состояние
робота. Для курсовой удобно видеть `x = [s, v, θ, ω, e_int]` и
`u = [v_cmd, ω_cmd]` всегда — например, чтобы убедиться, что
одометрия и IMU работают перед запуском сценария, или чтобы
наблюдать дрейф между прогонами.

### Уже есть на бэкенде

- `MpsNode._x_meas: np.ndarray(5)` — постоянно поддерживается из
  подписок на `odom` и `imu` (`pi_nodes/nodes/mps_node.py:210`).
- `MpsNode._x_meas_ts` — timestamp последнего odom-обновления.
- `MpsNode._active_run` (`_RunState | None`) — текущий прогон.
- `MQTTHandlers._broadcast_mps` — паттерн MQTT→WS bridge для
  существующего `/ws/mps/telemetry`
  (`compute_node/dashboard/mqtt_handlers.py:648`).

То есть данные уже есть; нужно только добавить публикацию и
доставку до фронта.

## 2. Контракты данных

### 2.1 MQTT

**Topic:** `samurai/{robot_id}/mps/live_state`
**QoS:** 0, **retained:** false, **rate:** ~10 Hz (период 0.1 c).

**Payload** (JSON):
```json
{
  "ts": 1747574400.123,
  "x": [0.0, 0.12, -0.05, 0.0, 0.0],
  "u": [0.12, 0.0],
  "scenario_active": false,
  "run_id": null,
  "schema_version": "1.0"
}
```

Семантика полей:

| Поле | Тип | Описание |
|---|---|---|
| `ts` | float | Pi-clock unix-секунды (для расчёта `stale` на фронте). |
| `x` | float[5] | `[s, v, θ, ω, e_int]`. См. ниже. |
| `u` | float[2] | `[v_cmd, ω_cmd]` — последнее опубликованное `cmd_vel`. |
| `scenario_active` | bool | `true` если `_active_run is not None`. |
| `run_id` | string \| null | UUID прогона, если активен. |
| `schema_version` | string | `"1.0"`. |

Компоненты `x`:
- `s` — продольное перемещение в локальном фрейме сценария (м).
  Без сценария — относительно последнего `_init_local_frame()` или
  `0.0` если фрейма ещё не было.
- `v` — продольная скорость (м/с) из odom. Всегда актуально.
- `θ` — курс (рад, диапазон −π…π) из IMU yaw. Всегда актуально.
- `ω` — угловая скорость (рад/с) из IMU. Всегда актуально.
- `e_int` — интеграл ошибки курса. Без активного сценария
  принудительно `0.0` (контроллер не интегрирует).

Компоненты `u`:
- Без активного сценария — `[0.0, 0.0]` (mps_node не публикует
  `cmd_vel` вне `DRIVE_FORWARD_MPS`).
- Во время сценария — последнее `linear_x`, `angular_z`, которое
  ушло в `cmd_vel`. Хранится в новом поле `self._last_u`.

### 2.2 WebSocket

**Endpoint:** `/ws/mps/live_state`
**Handshake:** отсутствует (в отличие от `/ws/mps/telemetry`).
Клиент просто подключается; сервер сразу шлёт последний известный
фрейм (если есть) — чтобы UI не висел в «—» секунду на свежем
открытии страницы.

**Frame:**
```ts
{
  type: 'live_state',
  point: MpsLiveStatePoint
}
```

При потере соединения клиент сам переподключается.

### 2.3 TypeScript

Новое в `compute_node/frontend/src/types/mps.ts`:
```ts
export const MPS_LIVE_STATE_SCHEMA = '1.0' as const

export interface MpsLiveStatePoint {
  ts: number
  x: number[]              // length 5
  u: number[]              // length 2
  scenario_active: boolean
  run_id: string | null
  schema_version: string
}

export interface MpsLiveStateWsFrame {
  type: 'live_state'
  point: MpsLiveStatePoint
}
```

## 3. Backend

### 3.1 `pi_nodes/nodes/mps_node.py` (~25 строк)

1. Константа на уровне модуля:
   ```python
   LIVE_STATE_RATE_HZ = 10.0
   ```
2. В `MpsNode.__init__` после инициализации `_x_meas`:
   ```python
   self._last_u = np.zeros(2)
   ```
3. После существующего `self.create_timer(1.0, self._publish_status)`:
   ```python
   self.create_timer(1.0 / LIVE_STATE_RATE_HZ, self._publish_live_state)
   ```
4. В `_publish_cmd_and_telemetry` в самом начале (перед `self.publish('cmd_vel', …)`):
   ```python
   self._last_u = u.copy()
   ```
   (Метод принимает `u: np.ndarray` напрямую — это компоненты, которые
   сейчас же уходят в `cmd_vel.linear_x` / `cmd_vel.angular_z`.)
5. Новый метод:
   ```python
   def _publish_live_state(self) -> None:
       x = self._x_meas.copy()
       run = self._active_run
       if run is None:
           x[_EINT] = 0.0
           u = [0.0, 0.0]
           scenario_active = False
           run_id = None
       else:
           u = self._last_u.tolist()
           scenario_active = True
           run_id = run.run_id
       self.publish('mps/live_state', {
           'ts': time.time(),
           'x': x.tolist(),
           'u': u,
           'scenario_active': scenario_active,
           'run_id': run_id,
           'schema_version': '1.0',
       }, qos=0)
   ```

Метод вызывается всегда, включая стадию до первого odom-сообщения
— в этом случае `_x_meas` все ещё нули, что и публикуется. На
фронте отдельный stale-таймер (см. §4.2) определит свежесть.

### 3.2 `compute_node/dashboard/mqtt_handlers.py` (~30 строк)

Следуем тому же single-slot паттерну, что уже используется для
`_mps_ws_broadcaster` (`/ws/mps/telemetry`) — а не fan-out list.
Это согласовано с реальной структурой файла (один сервер dashboard
= один WS broker для топика).

1. Добавить `'mps/live_state'` в `_SUBSCRIBE_TOPICS` (список подписок).
2. В `MQTTHandlers.__init__`:
   ```python
   self._last_live_state: Optional[dict] = None
   ```
3. Class-level (рядом с `_mps_ws_broadcaster`):
   ```python
   _mps_live_state_broadcaster: Optional[Callable] = None
   ```
4. Setter (рядом с `set_mps_ws_broadcaster`):
   ```python
   def set_mps_live_state_broadcaster(self, broadcaster: Optional[Callable]) -> None:
       self._mps_live_state_broadcaster = broadcaster
   ```
5. Handler `_h_mps_live_state(payload: bytes)` — парсит JSON сам, валидирует, пишет в `_last_live_state` под `self._state.lock`, вызывает `_mps_live_state_broadcaster` если он зарегистрирован (try/except вокруг вызова).
6. Зарегистрировать в маппинге `_dispatch` (топик → handler):
   ```python
   'mps/live_state': MQTTHandlers._h_mps_live_state,
   ```

### 3.3 `compute_node/dashboard/routers/mps.py` (~40 строк)

Новый WS:
```python
_live_state_clients: set[WebSocket] = set()

@ws_router.websocket('/ws/mps/live_state')
async def mps_live_state_ws(websocket: WebSocket):
    await websocket.accept()
    _live_state_clients.add(websocket)

    handlers = get_mqtt_handlers()
    last = handlers._last_live_state           # под lock
    if last is not None:
        await websocket.send_json({'type': 'live_state', 'point': last})

    try:
        while True:
            # держим соединение; receive_text вернёт при close
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _live_state_clients.discard(websocket)
```

Broadcaster (sync-функция, регистрируется на старте приложения):
```python
def _broadcast_live_state(frame: dict) -> None:
    loop = asyncio.get_event_loop()
    for ws in list(_live_state_clients):
        loop.create_task(_safe_send(ws, frame))
```

При старте FastAPI: `handlers.register_live_state_broadcaster(_broadcast_live_state)`.

### 3.4 `config.yaml`

Без изменений. 10 Hz — фиксированная константа; нет смысла делать
её настраиваемой пользователем.

## 4. Frontend

### 4.1 Хук `useMpsLiveState` — `compute_node/frontend/src/hooks/useMpsLiveState.ts`

```ts
interface UseMpsLiveStateResult {
  point: MpsLiveStatePoint | null
  connected: boolean
  stale: boolean
  ageMs: number | null
}

export function useMpsLiveState(): UseMpsLiveStateResult
```

- WebSocket к `/ws/mps/live_state` всегда; auto-reconnect с
  экспоненциальным backoff: 1s → 2s → 4s → 8s → 10s (cap).
- `point` обновляется на каждый frame.
- Отдельный `setInterval(500ms)` пересчитывает `ageMs = Date.now() - lastReceivedAtMs`
  и `stale = ageMs > 2000`. Это гарантирует, что бейдж перейдёт в
  stale даже если поток молча оборвался без `onclose`.
- При `WS.onclose` → `connected=false`, `point` сохраняется (чтобы
  UI продолжал показывать последние значения).

### 4.2 Компонент `LiveStateVector` — `compute_node/frontend/src/components/mps/LiveStateVector.tsx`

Один компонент без props. Использует `useMpsLiveState` внутри.

Структура (Tailwind, стиль согласован с `OdeCard`/`PhysicsParams`):
```
┌────────────────────────────────────────────┐
│ Вектор состояния          [● live | stale] │
├────────────────────────────────────────────┤
│ x:                                         │
│   s      = +0.234   м                      │
│   v      = +0.118   м/с                    │
│   θ      = −2.7°    (−0.0471 рад)          │
│   ω      = +12.3°/с (+0.215 рад/с)         │
│   e_int  = +0.0000                         │
├────────────────────────────────────────────┤
│ u:                                         │
│   v_cmd  = +0.120   м/с                    │
│   ω_cmd  =  0.0°/с  (0.000 рад/с)          │
├────────────────────────────────────────────┤
│ scenario: idle · обновлено 5s назад        │
└────────────────────────────────────────────┘
```

Детали:
- Все числа — `font-mono tabular-nums` + явный знак.
- Точность: `s, v, e_int, u[0]` — 3 знака после запятой; углы — 1
  знак (градусы), 4 знака (радианы).
- Бейдж:
  - `● live` (зелёный) — `connected && !stale`.
  - `stale · Ns` (жёлтый) — `connected && stale`.
  - `disconnected` (серый) — `!connected`.
- Нижняя строка:
  - `scenario_active=true`: `сценарий: <run_id[:8]> · обновлено Ns назад`.
  - иначе: `idle · обновлено Ns назад`.
- Если `point === null` (ни одного фрейма ещё не было):
  все числовые поля = `—`, бейдж = `disconnected` или `stale` по
  обстановке.

### 4.3 Размещение в `MpsPage.tsx`

В `MpsPageInner`, левый `<aside>`:
```tsx
<aside className="...">
  <LiveStateVector />     {/* ← новое, первый ребёнок */}
  <OdeCard ... />
  <PhysicsParams ... />
</aside>
```

`MpsPageInner` не передаёт пропсы — компонент самодостаточен.

## 5. Тесты

### 5.1 Pi — `tests/test_mps_node_live_state.py`

- `test_publish_live_state_idle` — без активного `_active_run`:
  `publish('mps/live_state', …)` вызван, `scenario_active=False`,
  `run_id=None`, `x[4]==0.0` (e_int обнулён), `u==[0,0]`.
- `test_publish_live_state_active` — с подсунутым `_active_run`:
  `scenario_active=True`, `run_id` совпадает, `u==_last_u.tolist()`.
- `test_publish_live_state_schema_version` — `schema_version=='1.0'`,
  `ts` — float > 0.
- `test_last_u_updated_in_publish_cmd_and_telemetry` —
  `_last_u` обновляется при каждом тике сценария.

### 5.2 Dashboard — `compute_node/dashboard/tests/test_mqtt_handlers_live_state.py`

- `test_h_mps_live_state_valid` — корректный payload →
  `_last_live_state` обновлён, broadcaster вызван 1 раз.
- `test_h_mps_live_state_invalid_x_len` — `x` длины 4 → handler
  молча игнорирует, broadcaster не вызван.
- `test_h_mps_live_state_invalid_u_len` — `u` длины 1 → игнор.
- `test_h_mps_live_state_missing_keys` — `KeyError` поглощается.

### 5.3 Dashboard WS — `compute_node/dashboard/tests/test_ws_live_state.py`

- `test_ws_replays_last_on_connect` — кладём в `mqtt_handlers._last_live_state`,
  открываем WS, первое сообщение — этот фрейм.
- `test_ws_broadcasts_new_frames` — после connect инжектим новый
  фрейм через broadcaster, клиент получает.
- `test_ws_disconnect_cleanup` — клиент закрылся, повторный
  broadcast не падает; клиент удалён из `_live_state_clients`.

### 5.4 Frontend vitest

- `useMpsLiveState.test.ts` — mock WebSocket:
  - receive frame → `point` обновлён, `connected=true`.
  - >2s без фрейма → `stale=true`, `ageMs` растёт.
  - onclose → `connected=false`, `point` сохраняется.
- `LiveStateVector.test.tsx` — render с моком хука:
  - `point=null`: «—» во всех полях, бейдж `disconnected`.
  - live-фрейм: правильное форматирование чисел, °, рад, м/с.
  - stale: бейдж `stale · 5s`, числа последние известные.
  - `scenario_active=true`: префикс `сценарий: <run_id[:8]>`.

### 5.5 Manual sanity

`./samurai.sh sim` + dashboard:
- Открыть `/mps`.
- Блок появляется в sidebar над `OdeCard`.
- WS подключается, числа `v, θ, ω` обновляются.
- Остановить sim → через 2с — бейдж `stale`, числа замороженные.
- Снова запустить sim → бейдж возвращается в `live` без перезагрузки страницы.

## 6. Скоуп / не-скоуп

В скоупе:
- Публикация `mps/live_state` из mps_node @ 10 Hz.
- MQTT→WS bridge в dashboard.
- Один новый UI-блок в левом sidebar `/mps`.

Не в скоупе:
- Изменения других страниц дашборда.
- Спарклайны/исторические графики (могут быть добавлены позже).
- Новые ключи в `config.yaml`.
- Изменение существующих контрактов (`/ws/mps/telemetry`,
  `mps/telemetry`, схема `MpsTelemetryPoint`).
- Тайминги/частота — фиксированы (10 Hz).

## 7. Риски

- **Двойная публикация во время сценария.** Во время прогона
  mps_node одновременно публикует `mps/telemetry` (per-tick) и
  `mps/live_state` (10 Hz). Это нормально — это разные каналы с
  разной семантикой. Нагрузка ничтожна.
- **e_int=0 в idle.** Пользователь может удивиться, увидев 0.0000
  для `e_int` вне сценария. Подпись «idle» внизу карточки даёт
  контекст; в будущем можно добавить tooltip с пояснением.
- **`s` относительно фрейма.** При смене позы робота между
  сценариями `s` может «прыгнуть» обратно к нулю при следующем
  `_init_local_frame()`. Это семантически верно для state vector,
  но требует мысленной модели от пользователя — снимается той же
  «idle/сценарий» подписью.

## 8. План работы (превью для writing-plans)

1. Pi: `_publish_live_state` + тесты — изолированно (через мок MQTT).
2. Dashboard: handler + broadcaster + WS endpoint + тесты.
3. Frontend: типы → хук → компонент → размещение → тесты.
4. Manual sanity на sim.
5. Commit + (если ветка `feat/mps`) push.

Детальный пошаговый план — отдельный документ через `writing-plans`.
