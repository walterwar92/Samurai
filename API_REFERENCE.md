# Samurai — API Reference

> Документ описывает три варианта API:
> 1. **MQTT API** — прямое управление роботом через MQTT (Pi / Android)
> 2. **REST API (реальный робот)** — `dashboard_node.py`, порт **5000**
> 3. **REST API (симулятор)** — `simulator.py`, порт **5000**
>
> REST API одинаковый для реального робота и симулятора. MQTT API работает только с реальным роботом.

---

## 1. MQTT API (прямое управление)

Pi запускает mosquitto broker на порту 1883. Все топики имеют префикс `samurai/{robot_id}/` (по умолчанию `robot1`).

### Запуск

```bash
# Pi
./start_robot_mqtt.sh

# Ноутбук (Docker + ROS2 + MQTT bridge)
./start_laptop_robot.sh
./start_laptop_robot.sh --pi 192.168.1.50      # указать IP вручную
./start_laptop_robot.sh --hotspot               # мобильный хотспот
```

### MQTT-топики: Сенсоры (Pi → подписчики)

| Топик | Payload | Частота | QoS |
|-------|---------|---------|-----|
| `samurai/robot1/odom` | `{"x": 0.5, "y": 1.2, "theta": 0.78, "vx": 0.1, "vz": 0.0, "ts": 1234567890.1}` | 20 Hz | 0 |
| `samurai/robot1/imu` | `{"ax": 0.01, "ay": 0.02, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.1, "ts": ...}` | 50 Hz | 0 |
| `samurai/robot1/range` | `{"range": 0.45, "ts": ...}` | 20 Hz | 0 |
| `samurai/robot1/camera` | JPEG bytes (binary) | 15 fps | 0 |
| `samurai/robot1/battery` | `{"voltage": 7.8, "percent": 82.0}` | 1 Hz | 1, retain |
| `samurai/robot1/temperature` | `42.5` (float) | 2 Hz | 0 |
| `samurai/robot1/claw/state` | `90.0` (float, angle) | 10 Hz | 0 |
| `samurai/robot1/watchdog` | `{"odom": true, "camera": true, "imu": true, ...}` | 1 Hz | 0 |
| `samurai/robot1/status` | `{"state": "IDLE", "target_colour": "", "target_action": "", "range_m": -1}` | 1 Hz | 1, retain |

### MQTT-топики: Команды (подписчики → Pi)

| Топик | Payload | QoS | Описание |
|-------|---------|-----|----------|
| `samurai/robot1/cmd_vel` | `{"linear_x": 0.1, "angular_z": 0.0}` | 1 | Скорость моторов |
| `samurai/robot1/speed_profile` | `"slow"` / `"normal"` / `"fast"` | 1 | Профиль скорости |
| `samurai/robot1/claw/command` | `"open"` / `"close"` / `"angle:90"` | 1 | Клешня |
| `samurai/robot1/laser/command` | `true` / `false` | 1 | Лазер (auto-off 10с) |
| `samurai/robot1/voice_command` | `"найди красный мяч"` | 1 | Голосовая команда FSM |
| `samurai/robot1/goal_pose` | `{"x": 1.0, "y": 2.0, "theta": 0.0}` | 1 | Навигационная цель |
| `samurai/robot1/ball_detection` | `{"colour": "red", "x": 280, "y": 200, "w": 40, "h": 40, "conf": 0.87}` | 0 | Детекция мяча (от YOLO) |
| `samurai/robot1/detections` | JSON array | 0 | Все детекции (heartbeat) |
| `samurai/robot1/gesture/command` | `"stop"` / `"forward"` / `"grab"` / `"follow"` | 1 | Жест |
| `samurai/robot1/call_robot` | `{"colour": "red", "action": "grab"}` | 1 | Вызов другого робота |
| `samurai/robot1/patrol/command` | `"start"` / `"stop"` | 1 | Патрулирование |
| `samurai/robot1/follow_me/command` | `"start"` / `"stop"` | 1 | Следование за человеком |
| `samurai/robot1/path_recorder/command` | `"record"` / `"stop"` / `"replay"` | 1 | Запись/воспроизведение маршрута |

### Примеры MQTT

```bash
# Мониторинг всех топиков
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/#' -v

# Управление моторами
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/cmd_vel' \
  -m '{"linear_x": 0.15, "angular_z": 0.0}'

# Голосовая команда
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/voice_command' \
  -m 'найди красный мяч'

# Остановка
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/cmd_vel' \
  -m '{"linear_x": 0.0, "angular_z": 0.0}'

# Клешня
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/claw/command' -m 'open'
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/claw/command' -m 'close'

# Лазер
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/laser/command' -m 'true'
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/laser/command' -m 'false'

# Патруль
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/patrol/command' -m 'start'

# Следование
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/follow_me/command' -m 'start'

# Навигация к точке
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/goal_pose' \
  -m '{"x": 1.5, "y": 2.0, "theta": 0.0}'
```

```python
# Python (paho-mqtt)
import paho.mqtt.client as mqtt
import json

client = mqtt.Client()
client.connect("raspberrypi.local", 1883)

# Отправить команду
client.publish("samurai/robot1/voice_command", "найди красный мяч")

# Управление моторами
client.publish("samurai/robot1/cmd_vel",
               json.dumps({"linear_x": 0.1, "angular_z": 0.0}))

# Подписка на статус
def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    print(f"State: {data['state']}, Range: {data.get('range_m', '?')}")

client.on_message = on_message
client.subscribe("samurai/robot1/status")
client.loop_forever()
```

### MQTT Bridge (ноутбук)

`mqtt_bridge_compute` на ноутбуке транслирует MQTT-топики Pi в ROS2 и обратно:

| MQTT (Pi) | ROS2 (ноутбук) | Направление |
|-----------|-----------------|-------------|
| `samurai/robot1/odom` | `/odom` (Odometry) + TF odom→base_link | Pi → ROS2 |
| `samurai/robot1/imu` | `/imu/data` (Imu) | Pi → ROS2 |
| `samurai/robot1/camera` | `/camera/image_raw/compressed` (CompressedImage) | Pi → ROS2 |
| `samurai/robot1/range` | `/range` (Range) | Pi → ROS2 |
| `samurai/robot1/battery` | `/battery` + `/battery_percent` (Float32) | Pi → ROS2 |
| `samurai/robot1/temperature` | `/cpu_temperature` (Float32) | Pi → ROS2 |
| `samurai/robot1/status` | `/robot_status` (String) | Pi → ROS2 |
| `samurai/robot1/goal_pose` | `/goal_pose` (PoseStamped) | Pi → ROS2 |
| `/ball_detection` (String) | `samurai/robot1/ball_detection` | ROS2 → Pi |
| `/yolo/detections` (String) | `samurai/robot1/detections` | ROS2 → Pi |
| `/gesture/command` (String) | `samurai/robot1/gesture/command` | ROS2 → Pi |

---

## 2. REST API — Реальный робот

### Запуск

```bash
# Рекомендуется
./start_laptop_robot.sh

# Или внутри Docker вручную
ros2 launch robot_pkg compute_bringup.launch.py mqtt_broker:=192.168.1.50
```

### Таблица эндпоинтов

| Метод | Путь | Описание |
|-------|------|----------|
| GET | `/api/status` | Полный снапшот робота |
| GET | `/api/robot/pose` | Позиция и угол (x, y, yaw) |
| GET | `/api/robot/velocity` | Скорость (одометрия + команда) |
| GET | `/api/sensors` | Все сенсоры |
| GET | `/api/sensors/ultrasonic` | Только дальномер |
| GET | `/api/sensors/imu` | Гироскоп, акселерометр, углы |
| GET | `/api/detection` | Все объекты от YOLO |
| GET | `/api/detection/closest` | Ближайший объект (`?color=red`) |
| GET | `/api/fsm` | Текущее состояние FSM |
| GET | `/api/actuators` | Состояние клешни и лазера |
| GET | `/api/battery` | Заряд батареи |
| GET | `/api/speed_profile` | Текущий профиль скорости |
| GET | `/api/camera/frame` | Кадр с камеры (JPEG) |
| GET | `/api/camera/frame.json` | Кадр с камеры (base64 JSON) |
| GET | `/api/map/image` | SLAM-карта (PNG) |
| GET | `/api/map/info` | Метаданные карты |
| GET | `/api/map/list` | Список сохранённых карт |
| GET | `/api/path_recorder/list` | Список записанных маршрутов |
| GET | `/api/log` | Лог команд и событий |
| POST | `/api/fsm/command` | Голосовая/текстовая команда FSM |
| POST | `/api/actuators/claw` | Управление клешнёй |
| POST | `/api/actuators/laser` | Управление лазером |
| POST | `/api/speed_profile` | Смена профиля скорости |
| POST | `/api/patrol/command` | Старт/стоп патруля |
| POST | `/api/patrol/waypoints` | Задать точки патруля |
| POST | `/api/follow_me` | Старт/стоп следования |
| POST | `/api/path_recorder/command` | Запись/воспроизведение маршрута |
| POST | `/api/map/save` | Сохранить SLAM-карту |
| POST | `/api/map/load` | Загрузить SLAM-карту |

### Примеры

```python
import requests

BASE = "http://192.168.1.42:5000"  # IP ноутбука

# Полный снапшот
state = requests.get(f"{BASE}/api/status").json()
print(state["pose"])
print(state["speed_profile"])

# FSM состояние
fsm = requests.get(f"{BASE}/api/fsm").json()
print(fsm["state"])  # IDLE, SEARCHING, PATROLLING, FOLLOWING, PATH_REPLAY...

# Команда
requests.post(f"{BASE}/api/fsm/command", json={"command": "найди красный мяч"})

# Клешня
requests.post(f"{BASE}/api/actuators/claw", json={"state": "open"})

# Лазер
requests.post(f"{BASE}/api/actuators/laser", json={"state": "on"})

# Патруль
requests.post(f"{BASE}/api/patrol/waypoints",
              json={"waypoints": [[0.5, 0.5], [1.0, 0.0], [0.0, 0.0]]})
requests.post(f"{BASE}/api/patrol/command", json={"command": "start"})

# Следование за человеком
requests.post(f"{BASE}/api/follow_me", json={"command": "start"})

# Запись маршрута
requests.post(f"{BASE}/api/path_recorder/command",
              json={"command": "start", "name": "route_1"})

# Сохранить карту
requests.post(f"{BASE}/api/map/save", json={"name": "arena_map"})

# Кадр камеры
import base64
resp = requests.get(f"{BASE}/api/camera/frame.json").json()
img_bytes = base64.b64decode(resp["jpeg_b64"])
```

### WebSocket — реальный робот

**Endpoint:** `ws://<ip>:5000/ws/state`
Пушит полный JSON-снапшот **4 раза в секунду** (250 мс).

```python
import asyncio
import json
import websockets

async def watch():
    async with websockets.connect("ws://192.168.1.42:5000/ws/state") as ws:
        async for msg in ws:
            data = json.loads(msg)
            print(data["pose"])
            print(data["status"])
            print(data["actuators"])
            print(data["battery"])

asyncio.run(watch())
```

```javascript
const ws = new WebSocket("ws://192.168.1.42:5000/ws/state");
ws.onmessage = (e) => {
    const data = JSON.parse(e.data);
    console.log(data.pose);
    console.log(data.status.state);
};
```

---

## 3. REST API — Симулятор

### Запуск

```bash
./start_laptop_sim.sh
# или
python compute_node/simulator.py
```

**Base URL:** `http://localhost:5000`

### Response Format

```json
{"ok": true, "data": { ... }}
{"ok": false, "error": "Human-readable error message"}
```

### Эндпоинты симулятора

| # | Метод | Путь | Описание |
|---|-------|------|----------|
| 1 | GET | `/api/status` | Полный снапшот (робот + арена + FSM + датчики) |
| 2 | GET | `/api/robot/pose` | Позиция (x, y, theta, theta_deg) |
| 3 | GET | `/api/robot/velocity` | Скорость + лимиты |
| 4 | POST | `/api/robot/velocity` | Установить скорость (`linear`, `angular`) |
| 5 | POST | `/api/robot/stop` | Экстренная остановка |
| 6 | POST | `/api/robot/reset` | Сброс симуляции |
| 7 | GET | `/api/sensors` | Все сенсоры |
| 8 | GET | `/api/sensors/ultrasonic` | Ультразвук (range_m) |
| 9 | GET | `/api/sensors/imu` | IMU (yaw, pitch, roll, gyro, accel) |
| 10 | GET | `/api/detection` | Все видимые детекции |
| 11 | GET | `/api/detection/closest` | Ближайший мяч (`?colour=red`) |
| 12 | GET | `/api/actuators` | Состояние клешни и лазера |
| 13 | POST | `/api/actuators/claw` | Открыть/закрыть клешню (`{open: true}`) |
| 14 | POST | `/api/actuators/laser` | Лазер вкл/выкл (`{on: true}`) |
| 15 | GET | `/api/fsm` | Состояние FSM |
| 16 | GET | `/api/fsm/states` | Список всех состояний |
| 17 | POST | `/api/fsm/command` | Голосовая команда (`{text: "..."}`) |
| 18 | POST | `/api/fsm/transition` | Принудительный переход (`{state: "IDLE"}`) |
| 19 | GET | `/api/arena` | Информация об арене |
| 20 | GET | `/api/arena/balls` | Все мячи с позициями |
| 21 | GET | `/api/map/image` | Карта (PNG) |
| 22 | GET | `/api/map/info` | Метаданные карты |
| 23 | GET | `/api/camera/frame` | Кадр камеры (JPEG) |
| 24 | GET | `/api/camera/frame.json` | Кадр камеры (base64) |
| 25 | GET | `/api/log` | Лог событий (`?limit=10`) |
| 26 | GET | `/video_feed` | MJPEG стрим |

### Пример: автономный цикл

```python
import requests
import time

BASE = 'http://localhost:5000'

def get(path):
    return requests.get(f'{BASE}{path}').json()['data']

def post(path, data=None):
    return requests.post(f'{BASE}{path}', json=data or {}).json()['data']

colours = ['red', 'blue', 'green', 'yellow', 'orange']

for colour in colours:
    post('/api/fsm/command', {'text': f'найди {colour} мяч'})
    while True:
        state = get('/api/fsm')['state']
        if state == 'IDLE':
            break
        time.sleep(0.5)
    arena = get('/api/arena')
    print(f"Balls remaining: {arena['balls_remaining']}")

print("All balls collected!")
```

### WebSocket (Socket.IO) — симулятор

```javascript
const socket = io('http://localhost:5000');

socket.on('state_update', (data) => {
    // data.status    — {state, target_colour, target_action, range_m}
    // data.detection — closest ball detection or {}
    // data.range_m   — ultrasonic distance
    // data.imu_ypr   — [yaw, pitch, roll]
    // data.pose      — {x, y, yaw}
    // data.voice_log — [{text, time}, ...]
});

// Отправить команду
socket.emit('send_command', { text: 'найди красный мяч' });

// Сбросить симуляцию
socket.emit('reset_sim', {});
```

---

## 4. FSM — Конечный автомат

### Все состояния

| Состояние | Описание |
|-----------|----------|
| `IDLE` | Ожидание команды |
| `SEARCHING` | Поиск цели (вращение 360) |
| `TARGETING` | Центрирование цели в кадре |
| `APPROACHING` | Подъезд к цели |
| `GRABBING` | Захват клешнёй |
| `BURNING` | Выжигание лазером |
| `CALLING` | Вызов второго робота |
| `RETURNING` | Возврат на базу (goal_pose 0,0) |
| `PATROLLING` | Автопатрулирование по точкам |
| `FOLLOWING` | Следование за человеком |
| `PATH_REPLAY` | Воспроизведение записанного маршрута |

### Диаграмма переходов

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    v                                         │
IDLE ──(find)──> SEARCHING ──(spotted)──> TARGETING ──(centered)──> APPROACHING
  │                  │                       │                        │
  │           (360 no ball)           (lost >1.5s)              (range <0.1m)
  │                  │                       │                    /       \
  │                  v                       v                   v         v
  │                IDLE                 SEARCHING           GRABBING   BURNING
  │                                                            │         │
  │                                                  (done)    │   (done)│
  │                                                     v      v         v
  │                                                  RETURNING      SEARCHING
  │
  ├──(patrol)───────> PATROLLING ──(stop)──> IDLE
  ├──(follow)───────> FOLLOWING  ──(stop)──> IDLE
  ├──(replay)───────> PATH_REPLAY──(stop)──> IDLE
  └──(call)─────────> CALLING   ──(done)──> IDLE
```

### Голосовые команды FSM

| Команда (русский) | Действие | Переход |
|-------------------|----------|---------|
| `найди [цвет] мяч` | Найти и захватить | → SEARCHING |
| `получи [цвет] мяч` | Найти и захватить | → SEARCHING |
| `возьми [цвет] мяч` | Найти и захватить | → SEARCHING |
| `сожги [цвет] мяч` | Найти и сжечь лазером | → SEARCHING (burn) |
| `прожги [цвет] мяч` | Найти и сжечь лазером | → SEARCHING (burn) |
| `лазер [цвет]` | Сжечь лазером | → SEARCHING (burn) |
| `стоп` / `остановись` | Экстренная остановка | → IDLE |
| `домой` / `вернись` | Вернуться на базу | → RETURNING |
| `вызови вторую машину` | Мультиагент | → CALLING |
| `патруль` / `обход` | Автопатруль | → PATROLLING |
| `следуй за мной` | Следование | → FOLLOWING |
| `запиши путь` | Запись маршрута | (recording) |
| `воспроизведи путь` | Повторить маршрут | → PATH_REPLAY |

### Цвета

| Русский | English |
|---------|---------|
| красный / красн | red |
| синий / синего | blue |
| зелёный / зелен | green |
| жёлтый / желт | yellow |
| оранжевый / оранж | orange |
| белый / бел | white |
| чёрный / черн | black |

---

## 5. Android приложение

### Режимы подключения

**Simulator Mode (REST API)**:
- Полирует эндпоинты @ 4 Hz
- Камера @ 10 fps, карта @ 3 fps
- Команды через `POST /api/fsm/command`

**Robot Mode (MQTT + REST API)**:
- Голосовые команды: `samurai/{robotId}/voice_command` (MQTT)
- Статус: `samurai/{robotId}/status` (MQTT, retain)
- Dashboard данные: REST API на IP ноутбука
- Vosk распознавание на Android (офлайн)

### Вкладки

1. **Управление** — FSM, быстрые команды, актуаторы, джойстик, голос
2. **Камера** — видеопоток с YOLO детекцией
3. **Карта** — SLAM карта, позиция, зоны, маршрут
4. **Датчики** — ультразвук, IMU, скорость, поза, лог
5. **Авто** — профиль скорости, патруль, следование, запись маршрутов, карты
6. **Настройки** — режим (Simulator/Robot), IP, порт, Robot ID

---

## 6. Web Dashboard

### Dashboard (мониторинг)

**URL:** `http://localhost:5000/`

Камера, карта, FSM, сенсоры, команды.

### Admin Panel (полный контроль, только симулятор)

**URL:** `http://localhost:5000/admin`

Полноценная панель: камера + YOLO, интерактивная карта с зонами, FSM с кнопками переходов, сенсоры, джойстик, актуаторы, детекции, мячи, лог.
