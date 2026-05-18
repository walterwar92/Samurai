# Кнопка «Выключить всё» в Header

> Красный крестик в правом верхнем углу дашборда, который
> одной кнопкой завершает работу всего проекта: робота (Pi) и
> дашборда (ПК).
>
> **Скоуп:** Header.tsx + Dialog подтверждения + новый endpoint
> `POST /api/system/shutdown` + новая Pi-нода `system_node.py`.

## 1. Контекст

Сейчас единственный способ остановить проект — пойти в терминал и
запустить `./samurai.sh stop` на обоих машинах. Это неудобно,
особенно когда дашборд открыт во весь экран. Пользователь хочет
кнопку прямо в UI, чтобы одним кликом завершить работу
полностью — и на роботе, и на ПК.

Текущее устройство:
- На ПК `samurai.sh compute` запускает Docker-контейнер
  `samurai_compute`. Внутри — FastAPI с дашбордом на `:5000`.
  Сам bash-launcher держит lock-файл и в `trap cleanup_all`
  останавливает Docker.
- На Pi `samurai.sh robot` (или `samurai-robot.service`) запускает
  `pi_nodes/robot_launcher.py`, который форкает 22 ноды и держит
  MQTT-соединение через `mqtt_node.py`.

Существующие endpoint'ы `/api/robot/stop` и `/api/emergency_stop`
останавливают только моторы — не процессы.

## 2. Поведение

### 2.1 UI

В правой части `Header.tsx` добавляется красная кнопка с иконкой
`Power` (lucide-react) — справа от индикатора «Подключено». Кнопка
видна на всех страницах (`/dashboard`, `/admin`, `/3d`,
`/hardware`, `/mps`), кроме режима Samcan (там USB-связь, проект
не выключаем этим способом).

Onclick → открывается shadcn `Dialog`:
- Заголовок: **«Выключить всё?»**
- Текст: «Робот (Pi) и дашборд (ПК) будут остановлены. Это нельзя
  отменить.»
- Кнопки: «Отмена» / «Выключить» (destructive).

После клика на «Выключить»:
1. Кнопка переходит в состояние `shutting_down`, лейбл «Выключаю…»
2. POST `/api/system/shutdown`
3. На любой ответ (200 или сетевая ошибка из-за того, что бэк уже
   умер) — UI показывает полноэкранный overlay:
   **«Выключение… можно закрыть вкладку.»**

### 2.2 UI-состояния кнопки

```typescript
type ShutdownPhase = 'idle' | 'confirming' | 'shutting_down' | 'done'
```

| `phase` | UI |
|---|---|
| `idle` | Кнопка `<Power />` красная, кликабельна |
| `confirming` | Dialog открыт |
| `shutting_down` | Dialog с лейблом «Выключаю…», кнопки disabled |
| `done` | Full-screen overlay, остальной UI скрыт |

### 2.3 Доступность

- Кнопка появляется только когда `!isSamcan` (в Samcan-режиме UI
  работает через USB-bridge, отдельный pipeline, его «крестик» не
  выключает)
- Disable при отсутствии `connected` — нет смысла, бэк уже не
  ответит, но визуально показываем что кнопка серая (опционально,
  YAGNI: оставляем активной, сетевая ошибка → всё равно overlay)

## 3. Backend (compute)

### 3.1 Endpoint

Файл `compute_node/dashboard/routers/system.py`:

```python
import asyncio
import os
import signal

shutdown_router = APIRouter()


@shutdown_router.post('', response_model=CommandAck, tags=['system'])
async def system_shutdown(
    background_tasks: BackgroundTasks,
    mqtt: MQTTDep,
) -> CommandAck:
    """Stop robot (MQTT) + compute (SIGTERM self).

    Pi-side: `system_node.py` ловит samurai/{robot_id}/system/shutdown
    и убивает robot_launcher.

    Compute-side: через 500мс шлём SIGTERM себе. uvicorn делает
    graceful shutdown, контейнер `samurai_compute` останавливается,
    bash-launcher `samurai.sh compute` отлавливает выход docker и
    выполняет cleanup_all.
    """
    mqtt.publish('system/shutdown', {'source': 'dashboard'}, qos=1)

    async def _shutdown_self() -> None:
        await asyncio.sleep(0.5)
        os.kill(os.getpid(), signal.SIGTERM)

    background_tasks.add_task(_shutdown_self)
    return CommandAck()
```

Регистрация в `compute_node/dashboard/app.py`:
```python
app.include_router(system.shutdown_router, prefix='/api/system/shutdown')
```

### 3.2 Аутентификация

Без auth — соответствует существующим `/api/robot/stop` и
`/api/emergency_stop`. Дашборд только в LAN.

## 4. Pi-side

### 4.1 Новая нода `pi_nodes/system_node.py`

```python
"""SystemNode — обрабатывает global system commands от dashboard.

Подписан на samurai/{robot_id}/system/shutdown — при получении
шлёт SIGTERM родительскому процессу (robot_launcher), который
каскадно останавливает все pi_nodes через signal-handlers.
"""
import os
import signal
import threading
import time

from .mqtt_node import MqttNode


class SystemNode(MqttNode):
    def __init__(self):
        super().__init__('system_node')
        self.subscribe('system/shutdown', self._on_shutdown)
        self.log.info('SystemNode ready')

    def _on_shutdown(self, payload):
        source = payload.get('source', 'unknown')
        self.log.warning('Shutdown requested from %s — killing launcher in 200ms', source)
        threading.Timer(0.2, self._kill_launcher).start()

    def _kill_launcher(self):
        parent_pid = os.getppid()
        self.log.warning('SIGTERM → PID %d (launcher)', parent_pid)
        os.kill(parent_pid, signal.SIGTERM)
```

### 4.2 Регистрация в `robot_launcher.py`

Добавить `SystemNode` в список нод, которые launcher форкает на
старте, рядом с остальными.

## 5. Frontend — точечные изменения

### 5.1 `lib/api.ts`

Добавить:
```typescript
shutdownAll: () => postJson('/api/system/shutdown', {}),
```

### 5.2 `components/layout/Header.tsx`

Добавить:
- `import { Power } from 'lucide-react'`
- `import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter } from '@/components/ui/dialog'`
- Локальный state: `const [phase, setPhase] = useState<ShutdownPhase>('idle')`
- Хендлер `onConfirm`: try `api.shutdownAll()` (игнорируем ошибки) → `setPhase('done')`
- Кнопка-крестик (только если `!isSamcan`)
- Conditional render Dialog (при `phase === 'confirming' || phase === 'shutting_down'`)
- Conditional render overlay (при `phase === 'done'`) — fixed inset-0, z-100, чёрно-полупрозрачный фон, центрированный текст

## 6. Что НЕ делаем (YAGNI)

- ❌ Аутентификация — её и так нет на `/api/robot/stop` и `/api/emergency_stop`
- ❌ Перезапуск через web — отдельная задача
- ❌ Раздельная остановка (только Pi / только compute) — всегда «всё»
- ❌ Сохранение state перед выключением (последняя поза руки, лог пути и т.д.)
- ❌ Авто-выключение `detector` или `agent`, если они запущены отдельно
  через `./samurai.sh detector` — пользователь сам уберёт. Документируем.
- ❌ В Samcan-режиме кнопка не показывается — Samcan по USB, нет MQTT-цикла

## 7. Тестирование

### 7.1 Unit-тесты

`compute_node/dashboard/routers/tests/test_system.py`:
- Мокаем `mqtt` + `os.kill` — проверяем что endpoint публикует в
  правильный topic и планирует SIGTERM
- Проверяем что отвечает 200 OK сразу (до `kill`)

### 7.2 Smoke на железе (вручную)

1. Запустить `./samurai.sh compute --pi 192.168.x.x` на ноуте
2. Запустить `./samurai.sh robot` на Pi
3. Открыть дашборд `http://localhost:5000`
4. Кликнуть крестик → подтвердить
5. Ожидаемо:
   - На Pi: `journalctl -u samurai-robot` → запись «Shutdown requested from dashboard»; через ~200мс процессы умирают
   - На ноуте: терминал с `samurai compute` → cleanup_all, Docker контейнер stopped, lock released
   - UI: overlay «Выключение… можно закрыть вкладку»

### 7.3 Edge cases

- **MQTT broker down** → `mqtt.publish` упадёт молча (внутри paho).
  Pi не выключится. Compute выключится. UI: overlay показан, но
  Pi продолжит работать. Mitigation: пользователь увидит на физ.
  роботе светодиоды → дёрнет питание.
- **Pi уже не подключен к MQTT** (например, перезагружался) →
  то же что выше. OK.
- **Дублирующий клик** → кнопка disabled в `shutting_down`. OK.

## 8. Риски

- **uvicorn внутри Docker запущен не как PID 1.** Если в Dockerfile
  есть init или wrapper, `os.kill(getpid(), SIGTERM)` убьёт только
  uvicorn, контейнер не завершится. Проверить при тесте; если так
  — заменить на `os.kill(1, SIGTERM)`.
- **`getppid()` на Pi не = launcher.** Если `system_node` запущен
  не как fork от `robot_launcher.py`, а как отдельный процесс,
  parent окажется shell/systemd. Проверить в `robot_launcher.py`
  — все ноды форкаются через `multiprocessing.Process` или подобное;
  если так — `getppid()` = launcher PID. Иначе хранить launcher PID
  в env-переменной `SAMURAI_LAUNCHER_PID` при старте.

## 9. Файлы для правки

| Файл | Что |
|---|---|
| `compute_node/frontend/src/components/layout/Header.tsx` | Кнопка + Dialog + overlay |
| `compute_node/frontend/src/lib/api.ts` | `shutdownAll()` |
| `compute_node/dashboard/routers/system.py` | `shutdown_router` |
| `compute_node/dashboard/app.py` | `include_router(shutdown_router)` |
| `pi_nodes/system_node.py` | новая нода (NEW) |
| `pi_nodes/robot_launcher.py` | регистрация `SystemNode` |
| `compute_node/dashboard/routers/tests/test_system.py` | unit-тесты |
