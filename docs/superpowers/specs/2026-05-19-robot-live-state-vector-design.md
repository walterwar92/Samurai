# Robot — Live State Vector в реальном времени

> Постоянная панель на `DashboardPage`, показывающая текущий вектор
> состояния основного робота (pose + velocity + полный IMU) в
> реальном времени @ 10 Hz, по той же архитектуре что и
> `mps/live_state` для МПС-нода
> ([2026-05-18-mps-live-state-vector-design.md](2026-05-18-mps-live-state-vector-design.md)).
>
> **Скоуп:** новый WS-эндпоинт `/ws/robot/live_state` с server-side
> aggregator из `DashboardState`, новый хук `useRobotLiveState`,
> новый компонент `RobotStateVector`, размещение на `DashboardPage`.
>
> **Pi не трогаем** — все данные уже агрегируются в
> `compute_node/dashboard/state.py` из существующих MQTT-подписок.

## 1. Контекст

Пользователь сообщил два связанных симптома:

1. «Координаты улетают очень далеко» — дрифт одометрии. Робот не
   имеет энкодеров; одометрия по умолчанию — dead-reckoning из
   `cmd_vel × scale` (`config.yaml:24` `source: wheel`).
2. SLAM не работает: фрейм `map` не существует
   (`tf error: Invalid frame ID "map"`). Отдельная задача — этот
   спек её не покрывает.

Из вопроса «можем ли отслеживать координаты как в МПС со всеми
углами поворотами» вытекает потребность в **едином месте,
показывающем полный вектор состояния робота в реальном времени** —
аналогично тому, как `LiveStateVector` показывает
`x = [s, v, θ, ω, e_int]` для МПС-сценариев.

### Что уже есть

| Что | Где | Используется |
|---|---|---|
| `DashboardState.snapshot()` | `compute_node/dashboard/state.py:239` | агрегирует все слайсы (`robot`, `sensors`, …) |
| MQTT `_h_odom`, `_h_imu` | `compute_node/dashboard/mqtt_handlers.py` | заполняют `state.robot.pose/velocity` и `state.sensors.imu` |
| SocketIO `state_update` @ 10 Hz | `compute_node/dashboard/app.py:568-608` | пушит весь state на фронт (используется как «глобальный» канал) |
| `useMpsLiveState` + `LiveStateVector` | `compute_node/frontend/src/hooks/useMpsLiveState.ts`, `components/mps/LiveStateVector.tsx` | образец-паттерн для зеркалирования |
| `RobotImuState`, `RobotPoseState` | `compute_node/frontend/src/types/robot.ts:143-174` | TypeScript-типы текущего состояния |
| `OdometryComparePanel`, `SensorPanel` | `compute_node/frontend/src/components/...` | существующие виджеты — частично перекрываются, но остаются |

То есть данные уже доступны как на бэкенде (через `state.snapshot()`),
так и в TS-типах. Не хватает только:

- выделенного WS-канала для «full state» (текущий `state_update`
  пушит всё, привязан к глобальному store, без stale-таймера);
- единого компактного UI-блока с полным вектором.

### Почему отдельный WS, а не SocketIO state_update

`state_update` пушит весь снимок (~50 полей) — большой и
«пузырится» по global store; компонент, читающий только pose+imu,
рендерится при изменении detection, voice_log и т.д. Отдельный WS:

- Контракт фиксированный, версионируется (`schema_version`).
- Stale-таймер на клиенте (`ageMs > 2000`) — критично для
  «дрифтит/не дрифтит» диагностики.
- Изолирован от global store — re-render только когда меняется
  именно state vector.
- Тот же паттерн что `mps/live_state` — единая ментальная модель.

## 2. Контракты данных

### 2.1 WebSocket

**Endpoint:** `/ws/robot/live_state`
**Handshake:** отсутствует. Клиент подключается; сервер сразу шлёт
последний известный фрейм (если есть) или собирает свежий из
текущего state. Клиент при разрыве сам переподключается.

**Frame:**
```ts
{
  type: 'live_state',
  point: RobotLiveStatePoint
}
```

### 2.2 Payload `RobotLiveStatePoint`

```jsonc
{
  "ts": 1747574400.123,            // Pi-clock unix-сек: state.robot.mqtt_odom_ts || time.time()
  "pose": {
    "x": 0.234,                    // м, world frame
    "y": 0.118,
    "yaw_rad": -0.047,
    "yaw_deg": -2.7
  },
  "vel": {
    "linear": 0.118,               // м/с — VelocityDetail.linear_x
    "angular": 0.215               // рад/с — VelocityDetail.angular_z
  },
  "imu": {
    "ypr_deg": [-2.7, 0.5, 0.0],         // [yaw, pitch, roll] в °; EKF-результат если has_ekf,
                                         //   иначе raw-fallback из акселя (yaw=0 без магнитометра).
                                         //   В state хранится ОДНО значение под `i.yaw/pitch/roll`.
    "gyro":  [0.01, 0.02, 0.21],         // рад/с по [x, y, z]
    "accel": [0.05, 0.02, 9.81],         // м/с² по [x, y, z]
    "ekf_bias_deg": [0.06, -0.11, 0.17], // °/с по [x, y, z]; null если has_ekf=false.
                                         //   Pi публикует bias в рад/с — конвертируем в aggregator.
    "has_ekf": true
  },
  "stationary": false,             // ZUPT-флаг (state.robot.stationary)
  "schema_version": "1.0"
}
```

Семантика и источник полей:

| Поле | Тип | Источник в DashboardState |
|---|---|---|
| `ts` | float | `state.robot.mqtt_odom_ts` (last odom MQTT) или `time.time()` если нет |
| `pose.x/y` | float | `state.robot.pose.x/y` |
| `pose.yaw_rad` | float | `state.robot.pose.yaw` |
| `pose.yaw_deg` | float | `math.degrees(pose.yaw)` |
| `vel.linear` | float | `state.robot.velocity_estimated.linear_x` |
| `vel.angular` | float | `state.robot.velocity_estimated.angular_z` |
| `imu.ypr_deg` | [3] | `[imu.yaw, imu.pitch, imu.roll]` — уже в °, без конверсии (Pi публикует EKF-Euler в градусах через `get_euler_deg()`) |
| `imu.gyro` | [3] | `[imu.gyro.x, imu.gyro.y, imu.gyro.z]` (Vec3 → list), рад/с |
| `imu.accel` | [3] | `[imu.accel.x, imu.accel.y, imu.accel.z]` (Vec3 → list), м/с² |
| `imu.ekf_bias_deg` | [3]\|null | `[math.degrees(b) for b in state.sensors.imu_ekf_bias]` если `has_ekf`, иначе `None` (Pi публикует bias в рад/с — конвертируем для UI) |
| `imu.has_ekf` | bool | `state.sensors.imu.ekf is not None` |
| `stationary` | bool | `state.robot.stationary` |
| `schema_version` | str | константа `'1.0'` |

### 2.3 TypeScript

Добавить в `compute_node/frontend/src/types/robot.ts`:

```ts
export const ROBOT_LIVE_STATE_SCHEMA = '1.0' as const

export interface RobotLiveStatePoint {
  ts: number
  pose: { x: number; y: number; yaw_rad: number; yaw_deg: number }
  vel: { linear: number; angular: number }
  imu: {
    ypr_deg: [number, number, number]                  // [yaw, pitch, roll] °, активный (EKF или raw-fallback)
    gyro: [number, number, number]                     // [x, y, z] рад/с
    accel: [number, number, number]                    // [x, y, z] м/с²
    ekf_bias_deg: [number, number, number] | null      // [x, y, z] °/с; null если has_ekf=false
    has_ekf: boolean
  }
  stationary: boolean
  schema_version: string
}

export interface RobotLiveStateWsFrame {
  type: 'live_state'
  point: RobotLiveStatePoint
}
```

## 3. Backend

### 3.1 Pydantic-схема — `compute_node/dashboard/schemas/robot_live_state.py` (~35 строк)

Новый файл, единственный источник правды для контракта:

```python
from typing import Literal
from pydantic import BaseModel

class RobotLiveStatePose(BaseModel):
    x: float
    y: float
    yaw_rad: float
    yaw_deg: float

class RobotLiveStateVel(BaseModel):
    linear: float
    angular: float

class RobotLiveStateImu(BaseModel):
    ypr_deg: list[float]                  # length 3, [yaw, pitch, roll] в °
    gyro: list[float]                     # length 3, rad/s
    accel: list[float]                    # length 3, m/s²
    ekf_bias_deg: list[float] | None      # length 3, °/s (None если has_ekf=False)
    has_ekf: bool

class RobotLiveStatePoint(BaseModel):
    ts: float
    pose: RobotLiveStatePose
    vel: RobotLiveStateVel
    imu: RobotLiveStateImu
    stationary: bool
    schema_version: Literal['1.0'] = '1.0'
```

### 3.2 Aggregator в `routers/robot.py` (~50 строк)

Чистая функция `state → RobotLiveStatePoint`; lock берётся внутри.
Юнит-тестируется без всего FastAPI окружения.

```python
import math
import time
from ._deps import StateDep                          # sibling _deps.py
from ..schemas.robot_live_state import (
    RobotLiveStatePoint, RobotLiveStatePose,
    RobotLiveStateVel, RobotLiveStateImu,
)

def build_live_state_point(state) -> RobotLiveStatePoint:
    with state.lock:
        p = state.robot.pose
        ve = state.robot.velocity_estimated
        imu = state.sensors.imu
        bias_rad = list(state.sensors.imu_ekf_bias)
        ts = state.robot.mqtt_odom_ts or time.time()
        stationary = state.robot.stationary
        has_ekf = imu.ekf is not None
        # imu.yaw/pitch/roll уже в °; gyro/accel — Vec3 → list
        ypr_deg = [imu.yaw, imu.pitch, imu.roll]
        gyro = [imu.gyro.x, imu.gyro.y, imu.gyro.z]
        accel = [imu.accel.x, imu.accel.y, imu.accel.z]
    return RobotLiveStatePoint(
        ts=ts,
        pose=RobotLiveStatePose(
            x=p.x, y=p.y, yaw_rad=p.yaw, yaw_deg=math.degrees(p.yaw),
        ),
        vel=RobotLiveStateVel(linear=ve.linear_x, angular=ve.angular_z),
        imu=RobotLiveStateImu(
            ypr_deg=ypr_deg,
            gyro=gyro, accel=accel,
            ekf_bias_deg=[math.degrees(b) for b in bias_rad] if has_ekf else None,
            has_ekf=has_ekf,
        ),
        stationary=stationary,
    )
```

### 3.3 WS endpoint — `routers/robot.py` (~50 строк)

```python
from fastapi import WebSocket, WebSocketDisconnect
from fastapi import APIRouter

ws_router = APIRouter()
_live_state_clients: set[WebSocket] = set()
_last_live_state: dict | None = None     # готовый JSON-dict для replay

@ws_router.websocket('/ws/robot/live_state')
async def robot_live_state_ws(websocket: WebSocket, state: StateDep):
    await websocket.accept()
    _live_state_clients.add(websocket)
    try:
        if _last_live_state is not None:
            await websocket.send_json({'type': 'live_state', 'point': _last_live_state})
        else:
            point = build_live_state_point(state).model_dump()
            await websocket.send_json({'type': 'live_state', 'point': point})
        while True:
            await websocket.receive_text()      # держим соединение
    except WebSocketDisconnect:
        pass
    finally:
        _live_state_clients.discard(websocket)

async def _broadcast_robot_live_state(point: dict) -> None:
    """Вызывается из aggregator-loop @ 10 Hz."""
    global _last_live_state
    _last_live_state = point
    frame = {'type': 'live_state', 'point': point}
    for ws in list(_live_state_clients):
        try:
            await ws.send_json(frame)
        except Exception:
            _live_state_clients.discard(ws)
```

### 3.4 Aggregator loop — `app.py` (~25 строк)

Регистрируется в `create_app` рядом с существующим `_push_loop`:

```python
from .routers.robot import (
    build_live_state_point,
    _broadcast_robot_live_state,
    _live_state_clients,
)

async def _robot_live_state_loop():
    """10 Hz aggregator → /ws/robot/live_state.
    Отдельный loop от _push_loop потому что:
     - всегда тикает (нет dirty-skip);
     - изоляция: если SocketIO-broadcast тормозит, WS-канал не страдает.
    """
    while True:
        await asyncio.sleep(0.1)              # 10 Hz
        if not _live_state_clients:
            continue                          # никто не слушает — пропускаем
        try:
            point = build_live_state_point(state).model_dump()
        except Exception as exc:
            log.exception('robot live_state aggregator failed: %s', exc)
            continue
        await _broadcast_robot_live_state(point)

@app.on_event('startup')
async def _start_robot_live_state_loop():
    asyncio.create_task(_robot_live_state_loop())
```

### 3.5 Регистрация `ws_router` в фабрике

В `create_app` (после блока с существующим `router`):

```python
from .routers.robot import ws_router as robot_ws_router
app.include_router(robot_ws_router)            # без префикса, путь полный
```

### 3.6 Производительность

- 10 Hz × ~250 байт JSON = ~2.5 KB/s на одного клиента.
- Lock держится <100 мкс на snapshot (копии примитивов).
- При 0 клиентов loop пропускает `build_live_state_point` (early continue).
- Конкурентный с `_push_loop` за state.lock — нагрузка эквивалентна
  второй копии _push_loop @ 10 Hz; безопасно по latency.

### 3.7 Без изменений

- `config.yaml` — нет новых ключей; 10 Hz фиксировано.
- `pi_nodes/*` — Pi не трогаем.
- `mqtt_handlers.py` — данные уже там через `_h_odom`/`_h_imu`.
- `state.py` — `snapshot()` не нужен, читаем поля напрямую.

## 4. Frontend

### 4.1 Hook `useRobotLiveState` — `compute_node/frontend/src/hooks/useRobotLiveState.ts` (~110 строк)

Структурно идентичен `useMpsLiveState`. Различия:
- URL: `/ws/robot/live_state`.
- Тип: `RobotLiveStatePoint` / `RobotLiveStateWsFrame`.

Возврат:
```ts
export interface UseRobotLiveStateResult {
  point: RobotLiveStatePoint | null
  connected: boolean
  stale: boolean             // ageMs > 2000
  ageMs: number | null
}
```

Поведение:
- WebSocket подключается всегда при mount.
- Auto-reconnect backoff `[1s, 2s, 4s, 8s, 10s]` (cap на 10s).
- `setInterval(500ms)` пересчитывает `ageMs`.
- `setPoint` на каждый valid frame; при unmount — close + clearTimeout.

Шаблон уже работает в проде для MPS — копируем 1:1 с заменой URL и типов.

### 4.2 Компонент `RobotStateVector` — `compute_node/frontend/src/components/sensors/RobotStateVector.tsx` (~150 строк)

Самодостаточный, без props. Внутри использует `useRobotLiveState()`.

**Структура (одна Card, 4 секции):**

```
┌────────────────────────────────────────────────┐
│ Состояние робота           [● live | stale 3s] │
├────────────────────────────────────────────────┤
│ Поза (world)                                   │
│   x  = +0.234   м                              │
│   y  = +0.118   м                              │
│   θ  = −2.7°    (−0.0471 рад)                  │
├────────────────────────────────────────────────┤
│ Скорости                                       │
│   v  = +0.118   м/с                            │
│   ω  = +12.3°/с (+0.215 рад/с)                 │
├────────────────────────────────────────────────┤
│ IMU — ориентация (EKF активен)                 │
│   Y = −2.7°    P = +0.5°    R = +0.0°          │
├────────────────────────────────────────────────┤
│ IMU — сырые                                    │
│   gyro  [+0.010, +0.020, +0.210] рад/с         │
│   accel [+0.050, +0.020, +9.810] м/с²          │
│   bias  [+0.06,  −0.11,  +0.17 ] °/с           │
├────────────────────────────────────────────────┤
│ ZUPT: moving · обновлено 0.1 s назад           │
└────────────────────────────────────────────────┘
```

**Правила форматирования:**
- Все числа — `font-mono tabular-nums`, явный знак `+`/`−`.
- Точности:
  - `x, y, gyro, accel, bias, v, linear` — 3 знака.
  - Углы в градусах — 1 знак.
  - Углы в радианах — 4 знака.
  - `ω` показывается одновременно в °/с (1 знак) и рад/с (3 знака).
- Бейдж:
  - `● live` (зелёный) — `connected && !stale`.
  - `stale · Ns` (жёлтый) — `connected && stale`.
  - `disconnected` (серый) — `!connected`.
- Stale-порог — 2000 ms (как в MPS).
- Заголовок секции «IMU — ориентация» меняется: «(EKF активен)» если
  `has_ekf=true`, иначе «(raw fallback, без EKF — yaw ≈ 0)».
- Строка «bias» внутри секции «сырые»: «—» если
  `ekf_bias_deg === null` (т.е. EKF выключен).
- ZUPT: `STATIONARY` (зелёный) / `moving` (нейтральный).
- При `point === null`: все значения «—», бейдж `disconnected`.

### 4.3 Размещение — `DashboardPage.tsx`

В той же колонке что и `OdometryComparePanel`, **над ним**:

```tsx
import { RobotStateVector } from '@/components/sensors/RobotStateVector'
…
<RobotStateVector />
<OdometryComparePanel sources={…} pose={…} />
```

`RobotStateVector` не получает пропсы — самодостаточен.

### 4.4 Без изменений

- `SensorPanel.tsx` — оставляем как есть (краткий YPR + ультразвук +
  ближайшая детекция). Дубль 3-х чисел YPR приемлемый: разный
  контекст использования.
- `OdometryComparePanel.tsx` — fusion-mode switcher + diff между
  источниками остаются на своём месте.
- `useRobotStore` / `selectors.ts` — не трогаем; новый компонент
  изолирован от global store.

## 5. Тесты

### 5.1 Pytest — `compute_node/dashboard/tests/test_robot_live_state.py` (~120 строк)

1. **`test_build_live_state_point_basic`** — мокаем `DashboardState`
   с `state.robot.pose=RobotPose(x=1.0, y=2.0, yaw=π/2)`,
   `state.robot.velocity_estimated=VelocityDetail(linear_x=0.1, angular_z=0.05)`,
   `state.sensors.imu=ImuData(yaw=10, pitch=5, roll=0, gyro=Vec3(0.01,0.02,0.21),
   accel=Vec3(0.05,0.02,9.81), ekf=ImuYpr(yaw=10,pitch=5,roll=0))`;
   вызываем `build_live_state_point`; проверяем все поля,
   `yaw_deg ≈ 90.0`, `ypr_deg == [10,5,0]`, `has_ekf == True`.
2. **`test_build_live_state_point_no_ekf`** — `imu.ekf=None`;
   ожидаем `has_ekf=False`, `ekf_bias_deg=None`, `ypr_deg` всё равно
   возвращается (raw fallback из state).
3. **`test_build_live_state_point_bias_conversion`** —
   `imu_ekf_bias=[0.001, -0.002, 0.003]` рад/с; в payload должно
   быть `[0.057, -0.115, 0.172]` °/с (с tolerance 1e-3).
4. **`test_schema_version`** — `point.schema_version == '1.0'`.
5. **`test_ws_replays_on_connect`** — через `TestClient.websocket_connect`
   подключаемся; первый frame — `live_state` с полным набором полей.
6. **`test_ws_broadcast_to_multiple_clients`** — два клиента
   подключены; имитируем broadcast → оба получают тот же frame.
7. **`test_ws_disconnect_cleanup`** — клиент закрыл WS,
   `_live_state_clients` не содержит его, повторный broadcast не падает.
8. **`test_aggregator_loop_skips_when_no_clients`** — патчим
   `_live_state_clients = set()`; запускаем 3 итерации;
   `build_live_state_point` НЕ вызывается (sentinel-mock).

### 5.2 Vitest — `useRobotLiveState.test.tsx` (~80 строк)

Mock `WebSocket`:
- **`test_receive_frame_updates_point`** — JSON-frame пришёл →
  `point` обновлён, `connected=true`.
- **`test_stale_after_2s`** — нет frame > 2000 мс →
  `stale=true`, `ageMs` растёт.
- **`test_reconnect_on_close`** — `ws.onclose` срабатывает →
  `connected=false`; через 1000 мс новый `WebSocket()` создан.
- **`test_invalid_frame_ignored`** — `type !== 'live_state'` →
  `point` не меняется.
- **`test_unmount_closes_ws`** — при unmount `ws.close()` вызван,
  setInterval очищен.

### 5.3 Vitest — `RobotStateVector.test.tsx` (~100 строк)

Mock `useRobotLiveState`:
- **`test_renders_placeholder_when_null`** — `point=null`: все числа
  «—», бейдж `disconnected`.
- **`test_renders_live_frame`** — frame с известными значениями;
  проверяем строки `+0.234 м`, `−2.7°`, `+12.3°/с` и т.д.
- **`test_stale_badge`** — `stale=true, ageMs=5300`: бейдж
  `stale · 5s` жёлтый.
- **`test_orientation_header_changes_on_ekf`** —
  при `has_ekf=true` заголовок содержит «EKF активен»; при `false` —
  «raw fallback».
- **`test_shows_bias_dash_when_null`** — `ekf_bias_deg=null`:
  строка bias = «—» вместо чисел.
- **`test_stationary_label`** — `stationary=true`: метка
  `STATIONARY` (зелёный).

### 5.4 Manual sanity

`./samurai.sh sim` + dashboard:
- Открыть `/` (DashboardPage).
- Карточка «Состояние робота» видна над `OdometryComparePanel`.
- WS подключается, числа `pose.x, pose.y, vel.linear, ypr_deg`
  обновляются.
- Остановить sim → через 2с бейдж становится `stale`, числа
  замороженные.
- Снова запустить sim → бейдж возвращается в `live` без перезагрузки.
- `has_ekf=true` (по дефолту EKF включён) — заголовок секции
  «ориентация» показывает «(EKF активен)», строка bias заполнена.
  Временно отключить EKF на Pi → заголовок «(raw fallback)»,
  строка bias = «—».

## 6. Скоуп / не-скоуп

**В скоупе:**
- Pydantic-схема `RobotLiveStatePoint` в новом файле
  `compute_node/dashboard/schemas/robot_live_state.py`.
- `build_live_state_point()` aggregator в `routers/robot.py`.
- WS `/ws/robot/live_state` + `_live_state_clients` + `_last_live_state`.
- `_robot_live_state_loop` background task в `app.py`.
- TS-типы `RobotLiveStatePoint` + `RobotLiveStateWsFrame`.
- Hook `useRobotLiveState`.
- Компонент `RobotStateVector`.
- Размещение на `DashboardPage`.
- Pytest + vitest тесты по §5.

**Не в скоупе:**
- Изменения `SensorPanel` и `OdometryComparePanel`.
- Спарклайны / time-series графики.
- Pi-side изменения (`mps_node.py`, `motor_node.py`, `imu_node.py`).
- Новые ключи в `config.yaml`.
- Изменение существующих контрактов (`/ws/mps/telemetry`,
  `/ws/mps/live_state`, SocketIO `state_update`).
- Исправление дрейфа одометрии (это отдельная задача — переключение
  fusion mode на ekf или комплементарный).
- Починка SLAM (`map` frame не существует — отдельная задача).
- Frequencies: 10 Hz зафиксирована, не настраивается.

## 7. Риски

- **Lock contention.** Aggregator берёт `state.lock` @ 10 Hz; уже
  есть `_push_loop` с такой же частотой. Удвоение нагрузки, lock
  держится <100 мкс — безопасно.
- **Дубль данных с SocketIO `state_update`.** `pose, velocity, imu`
  пушатся также в `state_update`; это нормально — разные каналы с
  разной семантикой (глобальный store vs. изолированный канал).
  Нагрузка ничтожна: +2.5 KB/s на клиента.
- **Schema drift.** Если потом добавим поля (например `body_s`),
  bump `schema_version → '1.1'`; на 1.0 хук принимает любой
  `type === 'live_state'` без строгой проверки.
- **Reconnect storm.** Backoff capped на 10s, не больше 1 попытки в
  10 секунд при долго недоступном backend.
- **`mqtt_odom_ts = 0` при холодном старте.** Fallback на
  `time.time()` исключает «1970-01-01» в UI.

## 8. План работы (превью для writing-plans)

1. Pydantic-схема `RobotLiveStatePoint` + юнит-тест.
2. `build_live_state_point()` + юнит-тесты (4 кейса из §5.1).
3. WS-endpoint + broadcaster + replay + тесты (4 кейса из §5.1).
4. Aggregator loop в `app.py` + регистрация router'а.
5. TS-типы.
6. Hook `useRobotLiveState` + vitest.
7. Компонент `RobotStateVector` + vitest.
8. Размещение на `DashboardPage`.
9. Manual sanity на sim.
10. Commit + push в `dev`.

Детальный пошаговый план — отдельный документ через `writing-plans`.
