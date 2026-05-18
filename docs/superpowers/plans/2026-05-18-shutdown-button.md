# Кнопка «Выключить всё» — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Красный крестик в Header дашборда, который одной кнопкой завершает работу проекта на роботе (Pi) и на ПК.

**Architecture:** Frontend POST `/api/v1/system/shutdown` → бэк публикует MQTT `system/shutdown` и шлёт SIGTERM сам себе (через 500мс). Pi-нода `SystemNode` ловит MQTT и шлёт SIGTERM родителю (`robot_launcher`).

**Tech Stack:** FastAPI + paho-mqtt (бэк), React 19 + shadcn/ui Dialog + lucide-react (фронт), `MqttNode` base class (Pi).

**Spec:** [`docs/superpowers/specs/2026-05-18-shutdown-button-design.md`](../specs/2026-05-18-shutdown-button-design.md)

---

## File Structure

| Файл | Что |
|---|---|
| `pi_nodes/nodes/system_node.py` | NEW — подписка на shutdown, kill launcher |
| `pi_nodes/robot_launcher.py` | MODIFY — регистрация `system` в NODE_REGISTRY + DEFAULT_NODES |
| `compute_node/dashboard/routers/system.py` | MODIFY — `shutdown_router` |
| `compute_node/dashboard/app.py` | MODIFY — include_router |
| `tests/test_system_shutdown_router.py` | NEW — тесты endpoint |
| `tests/test_system_node_pi.py` | NEW — тест Pi-ноды |
| `compute_node/frontend/src/lib/api.ts` | MODIFY — `shutdownAll()` |
| `compute_node/frontend/src/components/layout/Header.tsx` | MODIFY — кнопка + Dialog + overlay |
| `compute_node/static/` | REBUILD — `npm run build` |

---

## Task 1: Backend endpoint — failing test

**Files:**
- Create: `tests/test_system_shutdown_router.py`

- [ ] **Step 1.1: Создать тест-файл с двумя тестами**

```python
"""Тесты для POST /api/v1/system/shutdown.

Endpoint должен:
1. Опубликовать MQTT samurai/{robot_id}/system/shutdown
2. Запланировать SIGTERM самому себе через BackgroundTask
3. Вернуть 200 OK сразу (до kill)
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

fastapi = pytest.importorskip('fastapi')

from fastapi.testclient import TestClient  # noqa: E402

from compute_node.dashboard.app import create_app  # noqa: E402
from compute_node.dashboard.state import DashboardState  # noqa: E402


@pytest.fixture
def fake_mqtt():
    m = MagicMock()
    m.connected = True
    m.publish.return_value = True
    return m


@pytest.fixture
def client(fake_mqtt):
    state = DashboardState()
    app = create_app(state, mqtt=fake_mqtt, ros2=None, enable_socketio=False)
    return TestClient(app)


def test_shutdown_publishes_mqtt(client, fake_mqtt):
    """POST /api/v1/system/shutdown публикует system/shutdown в MQTT."""
    with patch('compute_node.dashboard.routers.system.os.kill'):
        r = client.post('/api/v1/system/shutdown')

    assert r.status_code == 200
    pub_calls = fake_mqtt.publish.call_args_list
    topics = [c.args[0] for c in pub_calls]
    assert any(t.endswith('system/shutdown') for t in topics), (
        f'Ожидался publish в system/shutdown, было: {topics}'
    )


def test_shutdown_schedules_self_kill(client, fake_mqtt):
    """BackgroundTask делает os.kill(getpid(), SIGTERM)."""
    import signal as _signal

    with patch('compute_node.dashboard.routers.system.os.kill') as mock_kill:
        r = client.post('/api/v1/system/shutdown')
        # TestClient ждёт BackgroundTasks → к этому моменту _shutdown_self уже отработал
        assert r.status_code == 200

    assert mock_kill.called, 'os.kill должен быть вызван BackgroundTask-ом'
    args = mock_kill.call_args.args
    assert args[0] == os.getpid()
    assert args[1] == _signal.SIGTERM
```

- [ ] **Step 1.2: Запустить, убедиться что падает**

```bash
pytest tests/test_system_shutdown_router.py -v
```

Ожидаемо: оба теста FAIL — endpoint ещё не существует, `client.post('/api/v1/system/shutdown')` вернёт 404.

- [ ] **Step 1.3: Коммит теста**

```bash
git add tests/test_system_shutdown_router.py
git commit -m "test(dashboard): добавить тесты для /api/v1/system/shutdown (red)"
```

---

## Task 2: Backend endpoint — implementation

**Files:**
- Modify: `compute_node/dashboard/routers/system.py`
- Modify: `compute_node/dashboard/app.py`

- [ ] **Step 2.1: Добавить shutdown_router в system.py**

В конец `compute_node/dashboard/routers/system.py` (после `hardware_router`) добавить:

```python
# ── System shutdown ────────────────────────────────────────────────────
import asyncio
import os
import signal

from fastapi import BackgroundTasks

shutdown_router = APIRouter()


@shutdown_router.post('', response_model=CommandAck, tags=['system'])
async def system_shutdown(
    background_tasks: BackgroundTasks,
    mqtt: MQTTDep,
) -> CommandAck:
    """Полное выключение робота и дашборда.

    Pi-side: SystemNode ловит samurai/{robot_id}/system/shutdown
    и шлёт SIGTERM родительскому процессу (robot_launcher).

    Compute-side: через 500мс шлём SIGTERM самому себе. uvicorn
    делает graceful shutdown, Docker контейнер samurai_compute
    останавливается; bash-launcher `samurai.sh compute` отлавливает
    выход docker и выполняет cleanup_all + release_lock.
    """
    mqtt.publish('system/shutdown', {'source': 'dashboard'}, qos=1)

    async def _shutdown_self() -> None:
        await asyncio.sleep(0.5)
        os.kill(os.getpid(), signal.SIGTERM)

    background_tasks.add_task(_shutdown_self)
    log.warning('System shutdown requested from dashboard')
    return CommandAck()
```

**Важно:** импорты `asyncio`, `os`, `signal`, `BackgroundTasks` уже могут быть выше — если есть, не дублируй (`os` уже импортирован в начале файла).

- [ ] **Step 2.2: Зарегистрировать router в app.py**

Файл `compute_node/dashboard/app.py`, после строки `app.include_router(system.hardware_router, prefix='/api/v1/hardware')` (около строки 481) добавить:

```python
    app.include_router(system.shutdown_router, prefix='/api/v1/system/shutdown')
```

- [ ] **Step 2.3: Запустить тесты, убедиться что зелёные**

```bash
pytest tests/test_system_shutdown_router.py -v
```

Ожидаемо: оба теста PASS.

- [ ] **Step 2.4: Запустить весь test_system_* чтобы не сломать другое**

```bash
pytest tests/test_system_shutdown_router.py tests/test_actuators_router.py -v
```

Ожидаемо: всё зелёное.

- [ ] **Step 2.5: Коммит**

```bash
git add compute_node/dashboard/routers/system.py compute_node/dashboard/app.py
git commit -m "feat(dashboard): POST /api/v1/system/shutdown — MQTT + self-SIGTERM"
```

---

## Task 3: Pi system_node — failing test

**Files:**
- Create: `tests/test_system_node_pi.py`

- [ ] **Step 3.1: Создать тест Pi-ноды**

```python
"""Тесты для pi_nodes/nodes/system_node.py.

SystemNode подписывается на samurai/{robot_id}/system/shutdown и
по получении шлёт SIGTERM родителю (robot_launcher).
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def test_system_node_subscribes_to_shutdown_topic():
    """SystemNode в __init__ подписывается на system/shutdown."""
    from pi_nodes.mqtt_node import MqttNode

    with patch('pi_nodes.mqtt_node.mqtt.Client'), \
         patch.object(MqttNode, 'subscribe') as mock_sub:
        from pi_nodes.nodes.system_node import SystemNode
        SystemNode()
        topics = [c.args[0] for c in mock_sub.call_args_list]
        assert 'system/shutdown' in topics, (
            f'SystemNode должен subscribe на system/shutdown, было: {topics}'
        )


def test_system_node_kills_parent_on_shutdown():
    """При получении shutdown — SystemNode шлёт SIGTERM родителю."""
    import signal as _signal

    with patch('pi_nodes.mqtt_node.mqtt.Client'):
        from pi_nodes.nodes.system_node import SystemNode
        node = SystemNode()

        with patch('pi_nodes.nodes.system_node.os.kill') as mock_kill, \
             patch('pi_nodes.nodes.system_node.os.getppid', return_value=12345):
            # Вызвать handler напрямую — без 200мс таймера
            node._kill_launcher()

        assert mock_kill.call_args.args == (12345, _signal.SIGTERM)
```

- [ ] **Step 3.2: Запустить, убедиться что падает**

```bash
pytest tests/test_system_node_pi.py -v
```

Ожидаемо: оба теста FAIL — `pi_nodes.nodes.system_node` не существует (ImportError).

- [ ] **Step 3.3: Коммит**

```bash
git add tests/test_system_node_pi.py
git commit -m "test(pi): добавить тесты для SystemNode (red)"
```

---

## Task 4: Pi system_node — implementation

**Files:**
- Create: `pi_nodes/nodes/system_node.py`
- Modify: `pi_nodes/robot_launcher.py`

- [ ] **Step 4.1: Создать system_node.py**

```python
#!/usr/bin/env python3
"""
system_node — Handle global system commands from dashboard.

Subscribed:
    samurai/{robot_id}/system/shutdown  — kill robot_launcher (всё умрёт каскадно)
"""

import os
import signal
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode


class SystemNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('system_node', **kwargs)
        self.subscribe('system/shutdown', self._on_shutdown)
        self.log.info('SystemNode ready: подписан на system/shutdown')

    def _on_shutdown(self, payload):
        source = (payload or {}).get('source', 'unknown')
        self.log.warning(
            'Shutdown requested from %s — SIGTERM в robot_launcher через 200мс',
            source,
        )
        # Задержка чтобы MQTT ack ушёл до того как процесс умрёт
        threading.Timer(0.2, self._kill_launcher).start()

    def _kill_launcher(self):
        parent_pid = os.getppid()
        self.log.warning('SIGTERM → PID %d (launcher)', parent_pid)
        os.kill(parent_pid, signal.SIGTERM)


if __name__ == '__main__':
    SystemNode().run()
```

- [ ] **Step 4.2: Зарегистрировать `system` в robot_launcher.py**

В `pi_nodes/robot_launcher.py` в `NODE_REGISTRY` (около строки 75, после `'mps'`) добавить:

```python
    'system':       'pi_nodes.nodes.system_node.SystemNode',
```

И в `DEFAULT_NODES` (около строки 87) добавить `'system'` в конец списка:

```python
DEFAULT_NODES = [
    'motor', 'imu', 'camera', 'ultrasonic',
    'battery', 'temperature',
    'head', 'arm', 'led',
    'fsm', 'watchdog',
    'slam_map',
    'precision_drive',
    'mps',
    'system',                                  # MQTT-shutdown handler
]
```

- [ ] **Step 4.3: Запустить тесты Pi-ноды**

```bash
pytest tests/test_system_node_pi.py -v
```

Ожидаемо: PASS.

- [ ] **Step 4.4: Прогнать smoke-набор pi-тестов чтобы не сломать соседей**

```bash
pytest tests/test_imu_node.py tests/test_arm_node.py tests/test_mps_node.py tests/test_system_node_pi.py -v
```

Ожидаемо: всё зелёное.

- [ ] **Step 4.5: Коммит**

```bash
git add pi_nodes/nodes/system_node.py pi_nodes/robot_launcher.py
git commit -m "feat(pi): SystemNode — обработка system/shutdown через MQTT"
```

---

## Task 5: Frontend — API client

**Files:**
- Modify: `compute_node/frontend/src/lib/api.ts`

- [ ] **Step 5.1: Прочитать api.ts чтобы найти место для вставки**

```bash
grep -n "emergencyStop\|sendVelocity\|resetPosition" compute_node/frontend/src/lib/api.ts
```

Найти секцию **Robot** в `export const api = { ... }`.

- [ ] **Step 5.2: Добавить `shutdownAll`**

В `compute_node/frontend/src/lib/api.ts`, в объекте `api`, после `resetPosition` (или в любом месте секции **System**, если она есть) добавить:

```typescript
  shutdownAll: () => post('/api/v1/system/shutdown'),
```

Если хелпер `post` отсутствует (внимательно проверить, что есть: `post`, `postJson`, `get`, `del`), использовать соответствующий:
- если endpoint без body — `post(url)`
- если с body — `postJson(url, {})`

Endpoint без body, поэтому `post('/api/v1/system/shutdown')`.

- [ ] **Step 5.3: Коммит**

```bash
git add compute_node/frontend/src/lib/api.ts
git commit -m "feat(frontend): api.shutdownAll() — POST /api/v1/system/shutdown"
```

---

## Task 6: Frontend — Header кнопка + Dialog + overlay

**Files:**
- Modify: `compute_node/frontend/src/components/layout/Header.tsx`

- [ ] **Step 6.1: Проверить что shadcn Dialog уже подключён**

```bash
ls compute_node/frontend/src/components/ui/dialog.tsx
```

Должен существовать (упомянут в `frontend.md` memory, есть в `package.json` через `@radix-ui/react-dialog`).

- [ ] **Step 6.2: Заменить Header.tsx целиком**

Старый файл — простая обёртка. Новый добавляет state и Dialog. Перезаписать `compute_node/frontend/src/components/layout/Header.tsx`:

```typescript
import { useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import { Power } from 'lucide-react'

import { cn } from '@/lib/utils'
import { useConnected } from '@/stores/selectors'
import { useRobot } from '@/providers/RobotProvider'
import { api } from '@/lib/api'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog'
import { RobotSelector } from '@/components/layout/RobotSelector'

type ShutdownPhase = 'idle' | 'confirming' | 'shutting_down' | 'done'

interface HeaderProps {
  isAdmin?: boolean
  simTime?: number
  onDebugOpen?: () => void
}

export function Header({ isAdmin, simTime, onDebugOpen }: HeaderProps) {
  const connected = useConnected()
  const { activeRobot } = useRobot()
  const { pathname } = useLocation()
  const [phase, setPhase] = useState<ShutdownPhase>('idle')

  const navLink = (to: string, label: string) => {
    const active = pathname === to || (to === '/dashboard' && pathname === '/')
    return (
      <Link to={to}>
        <Button
          variant="ghost"
          size="sm"
          className={cn('text-xs', active && 'bg-accent/15 text-accent')}
        >
          {label}
        </Button>
      </Link>
    )
  }

  const isSamcan = activeRobot === 'samcan'

  const handleConfirmShutdown = async () => {
    setPhase('shutting_down')
    try {
      await api.shutdownAll()
    } catch {
      // Бэк скорее всего уже умер до того как ответить — это OK
    }
    setPhase('done')
  }

  return (
    <>
      <header className="flex items-center justify-between px-5 py-3 bg-card border-b border-border sticky top-0 z-50">
        <div className="flex items-center gap-4">
          <h1 className="text-lg font-bold tracking-widest text-primary">
            {isSamcan ? 'SAMCAN' : 'SAMURAI'}
          </h1>
          {isAdmin && (
            <Badge variant="destructive" className="text-[10px] tracking-wider">
              ADMIN
            </Badge>
          )}
          <RobotSelector />
          {!isSamcan && (
            <nav className="flex items-center gap-2 ml-2">
              {navLink('/dashboard', 'Панель')}
              {navLink('/admin', 'Админ')}
              {navLink('/3d', '3D Карта')}
              {navLink('/hardware', 'Оборудование')}
              {navLink('/mps', 'МПС')}
            </nav>
          )}
        </div>

        <div className="flex items-center gap-4">
          {simTime !== undefined && (
            <span className="text-xs text-muted-foreground tabular-nums">
              {simTime.toFixed(1)}s
            </span>
          )}
          {onDebugOpen && (
            <Button variant="outline" size="sm" className="text-xs" onClick={onDebugOpen}>
              Все данные
            </Button>
          )}
          <div className="flex items-center gap-2 text-xs text-muted-foreground">
            <div
              className={cn(
                'w-2 h-2 rounded-full transition-colors',
                isSamcan ? 'bg-samurai-red' : connected ? 'bg-samurai-green' : 'bg-samurai-red'
              )}
            />
            {isSamcan ? 'Нет связи (USB)' : connected ? 'Подключено' : 'Отключено'}
          </div>
          {!isSamcan && (
            <Button
              variant="destructive"
              size="sm"
              className="text-xs h-8 w-8 p-0"
              title="Выключить робота и дашборд"
              onClick={() => setPhase('confirming')}
            >
              <Power className="h-4 w-4" />
            </Button>
          )}
        </div>
      </header>

      <Dialog
        open={phase === 'confirming' || phase === 'shutting_down'}
        onOpenChange={(open) => {
          if (!open && phase === 'confirming') setPhase('idle')
        }}
      >
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Выключить всё?</DialogTitle>
            <DialogDescription>
              Робот (Pi) и дашборд (ПК) будут остановлены. Это нельзя отменить.
            </DialogDescription>
          </DialogHeader>
          <DialogFooter>
            <Button
              variant="outline"
              onClick={() => setPhase('idle')}
              disabled={phase === 'shutting_down'}
            >
              Отмена
            </Button>
            <Button
              variant="destructive"
              onClick={handleConfirmShutdown}
              disabled={phase === 'shutting_down'}
            >
              {phase === 'shutting_down' ? 'Выключаю…' : 'Выключить'}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {phase === 'done' && (
        <div className="fixed inset-0 z-[100] bg-background/95 backdrop-blur-sm flex items-center justify-center">
          <div className="text-center space-y-3">
            <Power className="h-12 w-12 mx-auto text-destructive" />
            <h2 className="text-2xl font-bold tracking-wide">Выключение…</h2>
            <p className="text-muted-foreground">Можно закрыть вкладку.</p>
          </div>
        </div>
      )}
    </>
  )
}
```

- [ ] **Step 6.3: Type-check**

```bash
cd compute_node/frontend && npx tsc --noEmit
```

Ожидаемо: 0 errors.

- [ ] **Step 6.4: Lint**

```bash
cd compute_node/frontend && npm run lint 2>&1 | tail -20
```

Если есть warnings — посмотреть и поправить только в новом коде.

- [ ] **Step 6.5: Коммит**

```bash
git add compute_node/frontend/src/components/layout/Header.tsx
git commit -m "feat(ui): кнопка «Выключить всё» в Header — Dialog + overlay"
```

---

## Task 7: Frontend — rebuild

**Files:**
- Modify: `compute_node/static/` (build output)

- [ ] **Step 7.1: Билд**

```bash
cd compute_node/frontend && npm run build
```

Ожидаемо: успех, hash файлы обновляются в `compute_node/static/assets/`.

- [ ] **Step 7.2: Проверка размера и наличия Power иконки**

```bash
grep -l "Выключить всё" compute_node/static/assets/*.js | head -3
```

Должен быть хотя бы один матч (lazy chunks могут содержать строку).

- [ ] **Step 7.3: Коммит ребилда**

```bash
git add compute_node/static/
git commit -m "build(frontend): rebuild — кнопка «Выключить всё»"
```

---

## Task 8: Финальная сверка

- [ ] **Step 8.1: Прогнать всё что трогали**

```bash
pytest tests/test_system_shutdown_router.py tests/test_system_node_pi.py tests/test_actuators_router.py tests/test_imu_node.py -v
```

Ожидаемо: всё зелёное.

- [ ] **Step 8.2: Проверить, что в Header.tsx нет упоминаний Samcan-страницы (кнопка не должна рендериться там) — это уже в `!isSamcan` гарде, но визуально пробежать по diff**

```bash
git diff main -- compute_node/frontend/src/components/layout/Header.tsx | head -100
```

- [ ] **Step 8.3: Краткий git log**

```bash
git log --oneline main..HEAD
```

Должно быть ~6 коммитов: test→impl backend, test→impl pi, api, ui, rebuild.

---

## Что НЕ покрывается тестами (manual smoke на железе)

После merge — пользователь запускает:
1. `./samurai.sh compute --pi <IP>` на ноуте
2. `./samurai.sh robot` на Pi
3. Открывает `http://localhost:5000`
4. Жмёт крестик → подтверждает
5. Ожидаемо: Pi-логи показывают «Shutdown requested from dashboard», `systemctl status samurai-robot` (если через unit) → inactive; на ноуте Docker контейнер stopped, lock-файл удалён.

Если на проверке выясняется, что uvicorn в Docker запущен не как PID 1 — fallback: заменить `os.kill(os.getpid(), SIGTERM)` на `os.kill(1, SIGTERM)` в Task 2 patch. Зафиксировать в отдельном коммите.
