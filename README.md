# Samurai Robot

Автономный гусеничный робот на базе Raspberry Pi 4 с компьютерным зрением (YOLO), SLAM-навигацией, распознаванием речи и Android-управлением.

---

## Содержание

- [Описание проекта](#описание-проекта)
- [Архитектура](#архитектура)
- [Структура проекта](#структура-проекта)
- [Требования](#требования)
- [Установка](#установка)
- [Запуск](#запуск)
- [MQTT-топики](#mqtt-топики)
- [Голосовые команды](#голосовые-команды)
- [API Reference](#api-reference)
- [Отладка](#отладка)

---

## Описание проекта

**Samurai** — интеллектуальный робот с функциями:

- **Голосовое управление** (Vosk, русский язык) — с Android или Pi
- **Компьютерное зрение** (YOLOv8 детекция объектов)
- **SLAM и навигация** (slam_toolbox, Nav2)
- **Удалённое управление** (MQTT, Android приложение)
- **Конечный автомат** (FSM) с 11 состояниями
- **Манипулятор** (сервопривод для захвата)
- **Лазер** (GPIO17, безопасное автоотключение)
- **Камера CSI** (Raspberry Pi Camera Module)
- **Датчики**: ультразвуковой (HC-SR04), IMU (MPU6050), АЦП батареи (ADS7830)

---

## Архитектура

Система разделена на 3 уровня, связанных через **MQTT**:

```
Raspberry Pi (робот)                   Ноутбук (Docker, ROS2)
Pure Python + paho-mqtt                ROS2 Humble + MQTT Bridge
──────────────────────                 ──────────────────────────
motor_node     → PCA9685 I2C          mqtt_bridge_compute → MQTT↔ROS2
imu_node       → MPU6050 I2C          ekf_node            → EKF одометрия
camera_node    → CSI → JPEG (MQTT)    slam_toolbox        → SLAM-картография
ultrasonic_node→ HC-SR04 GPIO         nav2_bringup        → навигация
battery_node   → ADS7830 ADC          yolo_detector_node  → YOLO детекция
temperature_node→ /sys/class/thermal  depth_to_scan_node  → depth→LaserScan
servo_node     → PCA9685 серво        dashboard_node      → FastAPI :5000
laser_node     → GPIO17               patrol_node         → автопатруль
fsm_node       → 11 состояний FSM     follow_me_node      → следование
voice_node     → Vosk ASR (опц.)      path_recorder_node  → запись маршрутов
watchdog_node  → мониторинг           map_manager_node    → управление картами
fallback_nav   → автономность         gesture_node        → жесты
                                      qr_detector_node    → QR-коды
       ↕ MQTT (mosquitto:1883)                ↕
       ↕ samurai/robot1/...                   ↕

                  Android приложение
                  Kotlin + Jetpack Compose
                  ─────────────────────────
                  MQTT + REST API + Socket.IO
                  Vosk офлайн-распознавание
```

**Pi использует чистый Python + MQTT** (без ROS2 и Docker).
**Ноутбук** запускает тяжёлые вычисления (SLAM, Nav2, YOLO) в Docker с ROS2, а `mqtt_bridge_compute` транслирует данные между MQTT и ROS2.

---

## Структура проекта

```
Samurai/
├── start_robot_mqtt.sh          # Запуск Pi (Python + MQTT, основной)
├── start_laptop_robot.sh        # Запуск ноутбука (Docker + ROS2)
├── start_laptop_sim.sh          # Запуск симулятора (Flask, без ROS2)
├── start_robot.sh               # [DEPRECATED] Старый запуск через Docker
│
├── pi_nodes/                    # Все ноды Raspberry Pi (чистый Python)
│   ├── mqtt_node.py             #   Базовый класс MqttNode
│   ├── robot_launcher.py        #   Менеджер процессов (multiprocessing)
│   ├── nodes/
│   │   ├── motor_node.py        #   Моторы + одометрия (20 Hz)
│   │   ├── imu_node.py          #   MPU6050 (50 Hz)
│   │   ├── camera_node.py       #   CSI камера → JPEG (15 fps)
│   │   ├── ultrasonic_node.py   #   HC-SR04 (20 Hz)
│   │   ├── battery_node.py      #   АЦП → напряжение + процент (1 Hz)
│   │   ├── temperature_node.py  #   CPU температура (2 Hz)
│   │   ├── servo_node.py        #   Серво клешни (10 Hz)
│   │   ├── laser_node.py        #   GPIO17 лазер (auto-off 10с)
│   │   ├── fsm_node.py          #   FSM 11 состояний (10 Hz)
│   │   ├── voice_node.py        #   Vosk ASR (опционально)
│   │   ├── watchdog_node.py     #   Мониторинг MQTT (1 Hz)
│   │   └── fallback_nav_node.py #   Автономность без ноутбука
│   └── hardware/
│       ├── pca9685_driver.py    #   PCA9685 I2C driver
│       ├── motor_driver.py      #   Дифференциальный привод
│       └── servo_driver.py      #   Сервопривод
│
├── compute_node/                # Ноды ноутбука
│   ├── mqtt_bridge_compute.py   #   MQTT↔ROS2 мост
│   ├── yolo_detector_node.py    #   YOLO детекция
│   ├── depth_to_scan_node.py    #   Depth → LaserScan
│   ├── dashboard_node.py        #   FastAPI + WebSocket :5000
│   └── simulator.py             #   Автономный симулятор (Flask)
│
├── ros_ws/                      # ROS2 workspace (только ноутбук)
│   └── src/
│       ├── robot_pkg/           #   Python ноды + launch
│       │   ├── launch/
│       │   │   ├── compute_bringup.launch.py  # Запуск ноутбука
│       │   │   └── robot_bringup.launch.py    # [DEPRECATED]
│       │   └── config/          #   YAML конфиги (Nav2, SLAM, EKF)
│       └── robot_pkg_cpp/       #   [DEPRECATED] C++ ноды
│
├── android_app/                 # Android приложение
├── Diagnostic/                  # Диагностика оборудования
├── config.yaml                  # Централизованная конфигурация
├── config_loader.py             # Загрузчик config.yaml
├── Dockerfile                   # Docker образ ноутбука (ROS2)
├── API_REFERENCE.md             # REST API документация
└── README.md                    # Этот файл
```

---

## Требования

### Raspberry Pi (робот)

- **Hardware**:
  - Raspberry Pi 4 (4GB+ RAM)
  - Adeept Robot HAT V3.1 (PCA9685 I2C 0x5F)
  - MPU6050 (IMU, I2C 0x68)
  - ADS7830 (АЦП батареи, I2C 0x48)
  - HC-SR04 (ультразвук, GPIO Trig=27, Echo=22)
  - Raspberry Pi Camera Module (CSI)
  - Лазер (GPIO17)
  - Моторы с драйверами

- **Software**:
  - Debian 13 Trixie / Raspberry Pi OS (64-bit)
  - Python 3.11+
  - Mosquitto MQTT broker
  - **ROS2 НЕ требуется** на Pi

### Ноутбук (вычислительный узел)

- **Software**:
  - Linux (проверено на Arch Linux)
  - Docker + Docker daemon
  - avahi / nss-mdns (для `raspberrypi.local`)

> Docker-образ содержит ROS2 Humble, SLAM Toolbox, Nav2, YOLO, paho-mqtt — ничего устанавливать отдельно не нужно.

### Android приложение

- Android Studio Hedgehog+
- Android 8.0+ (API 26+)

---

## Установка

### 1. Raspberry Pi (робот)

```bash
# Клонировать проект
git clone <repo> ~/Samurai && cd ~/Samurai

# Установить mosquitto
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto

# Всё остальное start_robot_mqtt.sh установит автоматически:
# Python зависимости (paho-mqtt, smbus2, gpiozero, PyYAML)
# I2C включение, avahi-daemon
chmod +x start_robot_mqtt.sh
./start_robot_mqtt.sh
```

> Скрипт автоматически проверит и доустановит все зависимости при первом запуске.

### 2. Ноутбук (вычислительный узел)

```bash
# Docker (Arch Linux)
sudo pacman -S docker
sudo systemctl enable --now docker
sudo usermod -aG docker $USER && newgrp docker

# avahi (mDNS для raspberrypi.local)
sudo pacman -S avahi nss-mdns
sudo systemctl enable --now avahi-daemon

# Запуск — всё остальное скрипт сделает сам:
cd ~/Samurai
chmod +x start_laptop_robot.sh
./start_laptop_robot.sh
```

### 3. Симулятор (без железа)

```bash
pip install flask flask-cors flask-socketio opencv-python numpy paho-mqtt
cd ~/Samurai
./start_laptop_sim.sh
```

### 4. Android приложение

1. Откройте `android_app/` в Android Studio
2. Скачайте модель Vosk:
   ```bash
   cd android_app/app/src/main/assets/
   wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
   unzip vosk-model-small-ru-0.22.zip -d model/
   ```
3. Соберите APK: **Build > Build APK(s)**

---

## Запуск

### Реальный робот (основной сценарий)

**Шаг 1 — Raspberry Pi** (SSH или терминал):
```bash
cd ~/Samurai
./start_robot_mqtt.sh
```

Скрипт автоматически:
- Проверит Python, pip, I2C
- Установит недостающие зависимости
- Запустит mosquitto (если не запущен)
- Запустит avahi-daemon
- Запустит все 13 нод через `robot_launcher.py`

**Шаг 2 — Ноутбук**:
```bash
cd ~/Samurai
./start_laptop_robot.sh
```

Скрипт автоматически:
- Проверит Docker, соберёт образ (первый раз ~15 мин)
- Соберёт ROS2 workspace (colcon build)
- Найдёт Pi через mDNS (`raspberrypi.local`)
- Передаст IP Pi в `mqtt_bridge_compute`
- Запустит контейнер с SLAM, Nav2, YOLO, Dashboard

Опции:
```bash
./start_laptop_robot.sh --pi 192.168.1.50   # IP вручную
./start_laptop_robot.sh --hotspot            # мобильный хотспот
./start_laptop_robot.sh --rebuild            # пересборка образа + workspace
```

Dashboard: **http://localhost:5000**

**Шаг 3 — Android**:
1. Настройки > IP: `raspberrypi.local` > Режим: Robot > Подключить

---

### Симулятор (для разработки)

```bash
./start_laptop_sim.sh
```

Откройте: [http://localhost:5000](http://localhost:5000)

Полная эмуляция: арена 3x3 м, мячи, датчики, FSM, REST API.

---

## MQTT-топики

Все топики имеют префикс `samurai/{robot_id}/` (по умолчанию `samurai/robot1/`).

### Сенсоры (Pi → Ноутбук/Android)

| Топик | Формат | Частота | QoS |
|-------|--------|---------|-----|
| `odom` | `{x, y, theta, vx, vz, ts}` | 20 Hz | 0 |
| `imu` | `{ax, ay, az, gx, gy, gz, ts}` | 50 Hz | 0 |
| `range` | `{range, ts}` | 20 Hz | 0 |
| `camera` | JPEG bytes (binary) | 15 fps | 0 |
| `battery` | `{voltage, percent}` | 1 Hz | 1, retain |
| `temperature` | float | 2 Hz | 0 |
| `claw/state` | float (angle) | 10 Hz | 0 |
| `watchdog` | JSON health | 1 Hz | 0 |
| `status` | JSON FSM state | 1 Hz | 1, retain |

### Команды (Ноутбук/Android → Pi)

| Топик | Формат | QoS |
|-------|--------|-----|
| `cmd_vel` | `{linear_x, angular_z}` | 1 |
| `speed_profile` | `"slow"` / `"normal"` / `"fast"` | 1 |
| `claw/command` | `"open"` / `"close"` / `"angle:N"` | 1 |
| `laser/command` | `true` / `false` | 1 |
| `voice_command` | text | 1 |
| `goal_pose` | `{x, y, theta}` | 1 |
| `ball_detection` | JSON detection | 0 |
| `detections` | JSON all detections | 0 |
| `gesture/command` | text | 1 |
| `call_robot` | JSON | 1 |
| `patrol/command` | `"start"` / `"stop"` | 1 |
| `follow_me/command` | `"start"` / `"stop"` | 1 |
| `path_recorder/command` | `"record"` / `"stop"` / `"replay"` | 1 |

### Примеры работы с MQTT

```bash
# Мониторинг всех топиков
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/#' -v

# Управление моторами
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/cmd_vel' \
  -m '{"linear_x": 0.1, "angular_z": 0.0}'

# Голосовая команда
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/voice_command' \
  -m 'найди красный мяч'

# Открыть клешню
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/claw/command' \
  -m 'open'

# Включить лазер
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/laser/command' \
  -m 'true'

# Стоп
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/cmd_vel' \
  -m '{"linear_x": 0.0, "angular_z": 0.0}'
```

---

## Голосовые команды

Голосовые команды на русском языке. Источник: Android приложение (Vosk) или `voice_node` на Pi.

### Автономные команды

| Команда | Действие | FSM переход |
|---------|----------|-------------|
| `найди [цвет] мяч` | Найти и захватить | → SEARCHING |
| `получи [цвет] мяч` | Найти и захватить | → SEARCHING |
| `возьми [цвет] мяч` | Найти и захватить | → SEARCHING |
| `сожги [цвет] мяч` | Найти и сжечь лазером | → SEARCHING (burn) |
| `прожги [цвет] мяч` | Найти и сжечь лазером | → SEARCHING (burn) |
| `лазер [цвет]` | Сжечь лазером | → SEARCHING (burn) |

### Управление

| Команда | Действие | FSM переход |
|---------|----------|-------------|
| `стоп` / `остановись` | Экстренная остановка | → IDLE |
| `домой` / `вернись` | Вернуться на базу | → RETURNING |
| `вызови вторую машину` | Позвать второго робота | → CALLING → IDLE |
| `патруль` / `обход` | Автопатрулирование | → PATROLLING |
| `следуй за мной` | Следование за человеком | → FOLLOWING |
| `запиши путь` | Начать запись маршрута | (path recording) |
| `воспроизведи путь` | Повторить маршрут | → PATH_REPLAY |

### Жесты (gesture_node)

| Жест | Действие |
|------|----------|
| `stop` | Экстренная остановка |
| `forward` | Движение вперёд |
| `grab` | Начать поиск мяча |
| `follow` | Следование за человеком |
| `point_left` | Повернуть влево |
| `point_right` | Повернуть вправо |

### Цвета

| Русский | English |
|---------|---------|
| красный | red |
| синий | blue |
| зелёный | green |
| жёлтый | yellow |
| оранжевый | orange |
| белый | white |
| чёрный | black |

---

## Конечный автомат (FSM)

11 состояний:

```
IDLE ──(команда)──> SEARCHING ──(мяч найден)──> TARGETING ──(центрирован)──> APPROACHING
  ^                     |                          |                            |
  |              (360° пусто)               (потерян >1.5с)              (range < 0.1м)
  |                     v                          v                       /       \
  |                   IDLE                    SEARCHING              GRABBING   BURNING
  |                                                                     |         |
  +<──────────────────────────────────────────────────────────── RETURNING    SEARCHING
  |
  +──(патруль)──> PATROLLING
  +──(следуй)──> FOLLOWING
  +──(воспроизведи)──> PATH_REPLAY
```

---

## API Reference

Полная документация REST API: [API_REFERENCE.md](API_REFERENCE.md)

Быстрые примеры:

```bash
# Статус робота
curl http://localhost:5000/api/status

# Голосовая команда
curl -X POST http://localhost:5000/api/fsm/command \
  -H "Content-Type: application/json" \
  -d '{"text": "найди красный мяч"}'

# Ультразвук
curl http://localhost:5000/api/sensors/ultrasonic

# Моторы
curl -X POST http://localhost:5000/api/robot/velocity \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.1, "angular": 0.0}'
```

---

## Отладка

### MQTT мониторинг (основной способ)

```bash
# Все топики робота
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/#' -v

# Только статус FSM
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/status' -v

# Только одометрия
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/odom' -v
```

### ROS2 (на ноутбуке, внутри Docker)

```bash
docker exec -it samurai_compute bash
source /opt/ros/humble/setup.bash
source /root/Samurai/ros_ws/install/setup.bash

ros2 topic list
ros2 topic echo /odom
ros2 topic echo /robot_status
```

### Диагностика оборудования

```bash
python3 Diagnostic/diagnostic.py
```

Проверяет: I2C устройства (PCA9685, ADS7830, MPU6050), моторы, серво, ультразвук, камеру.

### I2C сканирование

```bash
i2cdetect -y 1
# Ожидаемые адреса: 0x48 (ADS7830), 0x5F (PCA9685), 0x68 (MPU6050)
```

---

## Лицензия

MIT License

---

## Авторы

Samurai Team
