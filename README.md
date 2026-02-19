# 🤖 Samurai Robot

Автономный гусеничный робот на базе Raspberry Pi 4 с ROS2 Humble, распознаванием речи, компьютерным зрением (YOLO) и навигацией.

---

## 📋 Содержание

- [Описание проекта](#описание-проекта)
- [Структура проекта](#структура-проекта)
- [Требования](#требования)
- [Установка](#установка)
  - [1. Raspberry Pi (робот)](#1-raspberry-pi-робот)
  - [2. Ноутбук (вычислительный узел)](#2-ноутбук-вычислительный-узел)
  - [3. Android приложение](#3-android-приложение)
- [Запуск](#запуск)
  - [Симулятор (для разработки)](#симулятор-для-разработки)
  - [Реальный робот — Raspberry Pi](#реальный-робот--raspberry-pi)
  - [Реальный робот — ноутбук (compute node)](#реальный-робот--ноутбук-compute-node)
  - [Android приложение](#android-приложение)
- [API Reference](#api-reference)
- [Архитектура](#архитектура)

---

## 🎯 Описание проекта

**Samurai** — это интеллектуальный робот с функциями:

- 🎤 **Голосовое управление** (Vosk, русский язык)
- 👁️ **Компьютерное зрение** (YOLOv8/YOLO11 для детекции объектов)
- 🗺️ **SLAM и навигация** (slam_toolbox, nav2)
- 📡 **Удалённое управление** (MQTT, Android приложение)
- 🧠 **Конечный автомат** (FSM) для автономного поведения
- 🔧 **Манипулятор** (сервоприводы для захвата объектов)
- 📷 **Камера CSI** (Raspberry Pi Camera Module)
- 📏 **Датчики**: ультразвуковой, IMU (MPU6050)

---

## 📁 Структура проекта

```
Samurai/
├── start_robot.sh               # 🤖 Запуск всех нод на Raspberry Pi
├── start_laptop_robot.sh        # 💻 Запуск compute node (ноутбук → реальный робот, Docker)
├── start_laptop_sim.sh          # 🎮 Запуск симулятора на ноутбуке (без ROS2)
│
├── ros_ws/                      # ROS2 workspace
│   └── src/robot_pkg/           # Основной ROS2 пакет
│       ├── robot_pkg/           # Python ноды
│       │   ├── hardware/        # Драйверы для PCA9685, моторов, серво
│       │   ├── motor_node.py
│       │   ├── camera_node.py
│       │   ├── voice_node.py
│       │   ├── imu_node.py
│       │   └── mqtt_bridge_node.py
│       ├── launch/              # Launch файлы
│       ├── config/              # Конфигурация nav2, SLAM, localization
│       └── msg/                 # Пользовательские ROS2 сообщения
│
├── compute_node/                # Вычислительные ноды (запускаются на ноутбуке)
│   ├── yolo_detector_node.py   # YOLO детектор (CPU/GPU)
│   ├── depth_to_scan_node.py   # Конвертация depth -> LaserScan
│   ├── geofence_map_publisher.py
│   ├── simulator.py            # 🎮 Симулятор с Flask REST API
│   └── dashboard_node.py       # Web dashboard + REST API
│
├── Diagnostic/                  # Диагностика оборудования
│   ├── diagnostic.py           # Полная проверка железа Raspberry Pi
│   └── setup_robot.sh          # Установка всех зависимостей на Pi
│
├── android_app/                 # Android приложение (Kotlin + Jetpack Compose)
│   └── app/src/main/           # Управление через REST API и MQTT
│
├── Dockerfile                   # Docker образ для ноутбука (ROS2 + SLAM + Nav2 + YOLO)
├── requirements.txt             # Python зависимости
├── API_REFERENCE.md            # Документация REST API
└── README.md                   # Этот файл
```

---

## ⚙️ Требования

### Raspberry Pi (робот)

- **Hardware**:
  - Raspberry Pi 4 (4GB RAM рекомендуется)
  - Adeept Robot HAT V3.1 (PCA9685 для управления моторами и серво)
  - MPU6050 (IMU)
  - HC-SR04 (ультразвуковой датчик)
  - Raspberry Pi Camera Module (CSI)
  - Микрофон USB
  - Моторы с драйверами

- **Software**:
  - Raspberry Pi OS (Debian Bookworm / Trixie, 64-bit)
  - ROS2 Humble (ros-base)
  - Python 3.10+

### Ноутбук (вычислительный узел)

- **Software**:
  - Linux (проверено на Arch Linux)
  - Docker + Docker daemon
  - avahi / nss-mdns (для доступа к Pi по `raspberrypi.local`)

> Docker-образ содержит ROS2 Humble, SLAM Toolbox, Nav2, YOLO и все зависимости — ничего устанавливать отдельно не нужно.

### Android приложение

- Android Studio Hedgehog или новее
- Android 8.0+ (API level 26+)

---

## 🛠️ Установка

### 1. Raspberry Pi (робот)

#### a) Установка зависимостей (одной командой)

```bash
cd ~/Samurai
sudo bash Diagnostic/setup_robot.sh
```

Скрипт установит: ROS2 пакеты, Python библиотеки (gpiozero, smbus2, adafruit-pca9685, vosk, opencv, paho-mqtt), настроит I2C и скачает модель Vosk (~50 МБ).

#### b) Установка ROS2 Humble вручную (если нужно)

```bash
# Добавить ROS2 репозиторий
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Установить ROS2 Humble (base достаточно для Pi)
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions -y

# Настроить окружение
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2. Ноутбук (вычислительный узел)

#### a) Установка Docker (Arch Linux)

```bash
sudo pacman -S docker
sudo systemctl enable --now docker
sudo usermod -aG docker $USER && newgrp docker
```

#### b) Установка avahi (mDNS для raspberrypi.local)

```bash
sudo pacman -S avahi nss-mdns
sudo systemctl enable --now avahi-daemon
```

Добавить `mdns_minimal` в `/etc/nsswitch.conf` строку `hosts`:
```
hosts: mymachines mdns_minimal [NOTFOUND=return] resolve [!UNAVAIL=return] files myhostname dns
```

#### c) Для симулятора (без Docker)

```bash
pip install flask flask-cors flask-socketio opencv-python numpy paho-mqtt
```

---

### 3. Android приложение

1. Откройте `android_app/` в Android Studio
2. Скачайте модель Vosk для Android (локальное распознавание речи):
   ```bash
   cd android_app/app/src/main/assets/
   wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
   unzip vosk-model-small-ru-0.22.zip -d model/
   rm vosk-model-small-ru-0.22.zip
   ```
3. Настройте `local.properties` с путём к Android SDK
4. Соберите APK: **Build → Build Bundle(s) / APK(s) → Build APK(s)**

---

## 🚀 Запуск

### Симулятор (для разработки)

Симулятор — полнофункциональный 2D симулятор робота с REST API. **ROS2 и железо не нужны.**

```bash
cd ~/Samurai
chmod +x start_laptop_sim.sh
./start_laptop_sim.sh
```

Скрипт автоматически проверит Python, установит недостающие зависимости и запустит симулятор.

Откройте браузер: [http://localhost:5000](http://localhost:5000)

**Возможности:**
- Виртуальная арена 3×3 м с мячами разных цветов
- Голосовые команды (найди красный мяч, возьми синий мяч, сожги жёлтый мяч)
- REST API для управления роботом
- Web dashboard с видеопотоком и картой
- Полная эмуляция датчиков (ультразвук, IMU, камера)

Полная документация API: [API_REFERENCE.md](API_REFERENCE.md)

---

### Реальный робот — Raspberry Pi

Запускает все аппаратные ноды: моторы, камеру, IMU, голос (Vosk), MQTT мост, FSM.

```bash
cd ~/Samurai
chmod +x start_robot.sh
./start_robot.sh
```

Скрипт выполнит:
1. Проверку ROS2 Humble
2. Сборку ROS2 workspace (если нужно)
3. Проверку I2C шины (включит автоматически если выключена)
4. Проверку камеры
5. Проверку Python зависимостей
6. Загрузку модели Vosk (если не скачана)
7. Запуск системных служб (pigpiod, avahi-daemon)
8. Выбор режима сети (LAN multicast или мобильный хотспот unicast)
9. Запуск `robot_bringup.launch.py`

**Первый запуск** (установка всех зависимостей):
```bash
sudo bash Diagnostic/setup_robot.sh
```

---

### Реальный робот — ноутбук (compute node)

Запускает SLAM, Nav2, YOLO и Web Dashboard в Docker-контейнере.

```bash
cd ~/Samurai
chmod +x start_laptop_robot.sh
./start_laptop_robot.sh
```

Скрипт выполнит:
1. Проверку и запуск Docker daemon
2. Сборку Docker-образа (первый раз: 10–20 минут)
3. Сборку ROS2 workspace внутри Docker
4. Проверку mDNS (avahi для `raspberrypi.local`)
5. Выбор режима сети (LAN или мобильный хотспот)
6. Проверку доступности Raspberry Pi
7. Запуск контейнера с `compute_bringup.launch.py`

После запуска Dashboard доступен по адресу: **http://localhost:5000**

---

### Android приложение

1. Установите APK на Android устройство
2. Подключитесь к той же Wi-Fi сети, что и робот
3. Откройте вкладку **Настройки**:
   - **Режим**: Симулятор или Реальный робот
   - **IP**: `raspberrypi.local` (или IP адрес) для реального робота, IP ноутбука для симулятора
   - **Порт**: 5000
4. Нажмите **Подключить**

**Вкладки приложения:**
1. **Управление** — FSM состояние, быстрые команды, джойстик, голосовое управление, актуаторы
2. **Камера** — видеопоток с YOLO детекцией
3. **Карта** — SLAM карта, позиция робота, зоны, маршрут
4. **Датчики** — ультразвук, IMU, скорость, поза, лог событий
5. **Авто** — профиль скорости, патруль, следование за человеком, запись маршрутов, управление картами
6. **Настройки** — режим подключения, адрес, Robot ID

---

## 📖 API Reference

Полная документация API: [API_REFERENCE.md](API_REFERENCE.md)

**Примеры:**

```bash
# Получить статус робота
curl http://localhost:5000/api/status

# Отправить команду "найти красный мяч"
curl -X POST http://localhost:5000/api/fsm/command \
  -H "Content-Type: application/json" \
  -d '{"text": "найди красный мяч"}'

# Получить данные с ультразвукового датчика
curl http://localhost:5000/api/sensors/ultrasonic

# Управление моторами
curl -X POST http://localhost:5000/api/robot/velocity \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.1, "angular": 0.0}'
```

---

## 🏗️ Архитектура

### Топология системы

```
Raspberry Pi (Robot)                    Ноутбук (Compute Node, Docker)
├── motor_node      → DRV8833 моторы    ├── ekf_node           → EKF одометрия
├── camera_node     → CSI камера        ├── slam_toolbox       → картографирование
├── voice_node      → Vosk ASR          ├── nav2_bringup       → навигация
├── fsm_node        → конечный автомат  ├── yolo_detector_node → YOLO v8
├── imu_node        → MPU6050           ├── dashboard_node     → REST API :5000
├── mqtt_bridge     → MQTT ↔ Android    ├── patrol_node        → автопатруль
├── battery_node    → заряд батареи     ├── follow_me_node     → следование
├── watchdog_node   → watchdog          ├── path_recorder_node → запись маршрутов
└── Сенсоры: HC-SR04, WS2812, line...  └── map_manager_node   → управление картами

         ↕ ROS2 DDS (Multicast/Unicast по WiFi)

Android приложение
├── RobotApiClient  → HTTP → REST API :5000
├── RobotMqttClient → MQTT ↔ Pi напрямую (режим Robot)
└── VoskRecognizer  → офлайн распознавание речи
```

### ROS2 Топики

| Топик | Тип сообщения | Описание |
|-------|---------------|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Команды скорости для моторов |
| `/camera/image_raw` | `sensor_msgs/Image` | Видеопоток с камеры |
| `/yolo/annotated` | `sensor_msgs/Image` | Видео с YOLO-разметкой |
| `/scan` | `sensor_msgs/LaserScan` | Данные лидара (эмулированные) |
| `/imu/data` | `sensor_msgs/Imu` | Данные IMU (MPU6050) |
| `/range` | `sensor_msgs/Range` | Ультразвуковой датчик |
| `/ball_detection` | `std_msgs/String` | Результаты YOLO детектора |
| `/voice_command` | `std_msgs/String` | Распознанные голосовые команды |
| `/robot_status` | `std_msgs/String` | FSM состояние (JSON) |
| `/odometry/filtered` | `nav_msgs/Odometry` | Поза и скорость (EKF) |
| `/battery_status` | `std_msgs/String` | Заряд батареи (JSON) |
| `/speed_profile/active` | `std_msgs/String` | Текущий профиль скорости |

### Конечный автомат (FSM)

Робот работает в одном из следующих состояний:

```
IDLE → SEARCHING → TARGETING → APPROACHING → GRABBING/BURNING → RETURNING → IDLE
```

- **IDLE**: Ожидание команды
- **SEARCHING**: Поиск цели (вращение 360°)
- **TARGETING**: Центрирование цели в кадре
- **APPROACHING**: Подъезд к цели
- **GRABBING/BURNING**: Захват клешнёй или выжигание лазером
- **RETURNING**: Возврат на базу
- **CALLING**: Вызов второго робота (в многоагентной системе)

---

## 🎮 Управление

### Голосовые команды (русский язык)

#### Автономные команды (поиск объектов)
- `найди красный мяч` — найти и захватить красный мяч
- `получи синий мяч` — найти и захватить синий мяч
- `возьми зелёный мяч` — найти и захватить зелёный мяч
- `сожги жёлтый мяч` — найти и сжечь жёлтый мяч лазером
- `прожги оранжевый мяч` — найти и сжечь оранжевый мяч лазером
- `лазер красный` — найти и сжечь красный мяч лазером

#### Команды управления
- `стоп` / `остановись` — экстренная остановка
- `домой` / `вернись` — вернуться на базу
- `вызови вторую машину` — вызвать второго робота (мультиагент)

#### Ручное управление
- `развернись` / `разворот` — развернуться на 180°
- `повернись 90 градусов` — повернуть на N градусов (влево если положительное, вправо если отрицательное)
- `поверни 45` — короткая форма команды поворота
- `иди вперёд на 50 см` — двигаться вперёд на N сантиметров
- `иди назад на 30 см` — двигаться назад на N сантиметров
- `иди влево на 20 см` — двигаться влево на N сантиметров (с поворотом)
- `иди вправо на 40 см` — двигаться вправо на N сантиметров (с поворотом)

### Цвета

- красный / красн
- синий / синего
- зелёный / зелен
- жёлтый / желт
- оранжевый / оранж

---

## 🐛 Отладка

### Диагностика оборудования

```bash
# Полная проверка всего железа на Raspberry Pi
python3 Diagnostic/diagnostic.py
```

Проверяет: I2C устройства (PCA9685, ADS7830), DC моторы, сервоприводы, ультразвук, line tracking, WS2812 LED, камеру.

### Логи ROS2

```bash
ros2 topic echo /rosout
```

### Просмотр топиков

```bash
# Список всех топиков
ros2 topic list

# Посмотреть сообщения в топике
ros2 topic echo /robot_status
```

### Проверка нод

```bash
ros2 node list
ros2 node info /motor_node
```

---

## 📝 Лицензия

MIT License

---

## 👥 Авторы

Samurai Team

---

## 🙏 Благодарности

- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Vosk Speech Recognition](https://alphacephei.com/vosk/)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [Nav2 Navigation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
