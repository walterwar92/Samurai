# 🤖 Samurai Robot

Автономный гусеничный робот на базе Raspberry Pi 4 с ROS2 Humble, распознаванием речи, компьютерным зрением (YOLO) и навигацией.

---

## 📋 Содержание

- [Описание проекта](#описание-проекта)
- [Структура проекта](#структура-проекта)
- [Требования](#требования)
- [Установка](#установка)
  - [1. Raspberry Pi (робот)](#1-raspberry-pi-робот)
  - [2. Вычислительный узел (опционально)](#2-вычислительный-узел-опционально)
  - [3. Android приложение](#3-android-приложение)
- [Запуск](#запуск)
  - [Симулятор (для разработки)](#симулятор-для-разработки)
  - [Реальный робот](#реальный-робот)
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
├── compute_node/                # Вычислительные ноды (можно запускать отдельно)
│   ├── yolo_detector_node.py   # YOLO детектор (CPU/GPU)
│   ├── depth_to_scan_node.py   # Конвертация depth -> LaserScan
│   ├── geofence_map_publisher.py
│   ├── simulator.py            # 🎮 Симулятор с Flask REST API
│   └── dashboard_node.py       # Web dashboard
│
├── android_app/                 # Android приложение (Kotlin + Jetpack Compose)
│   └── app/src/main/           # Управление через MQTT
│
├── requirements.txt             # Python зависимости
├── API_REFERENCE.md            # Документация REST API симулятора
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
  - Ubuntu 22.04 Server (64-bit) для Raspberry Pi
  - ROS2 Humble Desktop
  - Python 3.10+

### Вычислительный узел (опционально, для YOLO)

- Python 3.10+
- CUDA (для GPU ускорения YOLO, опционально)

### Android приложение

- Android Studio Hedgehog или новее
- Android 8.0+ (API level 26+)

---

## 🛠️ Установка

### 1. Raspberry Pi (робот)

#### a) Установка ROS2 Humble

```bash
# Добавить ROS2 репозиторий
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Установить ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Установить ROS2 пакеты для навигации и SLAM
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-smac-planner \
  ros-humble-cartographer-ros \
  ros-humble-robot-localization \
  ros-humble-cv-bridge \
  ros-humble-image-transport -y

# Настроить окружение
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### b) Установка Python зависимостей

```bash
cd ~/Samurai
pip3 install -r requirements.txt
```

#### c) Сборка ROS2 пакета

```bash
cd ~/Samurai/ros_ws
colcon build --symlink-install
source install/setup.bash

# Добавить в .bashrc для автозагрузки
echo "source ~/Samurai/ros_ws/install/setup.bash" >> ~/.bashrc
```

#### d) Скачать модель Vosk (для распознавания речи)

```bash
cd ~/Samurai
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
unzip vosk-model-small-ru-0.22.zip
rm vosk-model-small-ru-0.22.zip
```

---

### 2. Вычислительный узел (опционально)

Если у вас есть отдельный компьютер с GPU для запуска YOLO:

```bash
cd ~/Samurai
pip3 install -r requirements.txt

# Для GPU поддержки установите PyTorch с CUDA
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

---

### 3. Android приложение

1. Откройте `android_app/` в Android Studio
2. Скачайте модель Vosk для Android (если нужно локальное распознавание):
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

Симулятор — это полнофункциональный 2D симулятор робота с REST API для тестирования алгоритмов без реального железа.

```bash
cd ~/Samurai
python3 compute_node/simulator.py
```

Откройте браузер: [http://localhost:5000](http://localhost:5000)

**Возможности:**
- Виртуальная арена 3×3 м с мячами разных цветов
- Голосовые команды (найди красный мяч, возьми синий мяч, сожги жёлтый мяч)
- REST API для управления роботом
- Web dashboard с видеопотоком и картой
- Полная эмуляция датчиков (ультразвук, IMU, камера)

Полная документация API: [API_REFERENCE.md](API_REFERENCE.md)

---

### Реальный робот

#### a) Запуск всех нод (на Raspberry Pi)

```bash
cd ~/Samurai/ros_ws
source install/setup.bash

# Запустить все ноды робота
ros2 launch robot_pkg robot_bringup.launch.py
```

Это запустит:
- Ноды управления моторами, серво
- Камеру, ультразвуковой датчик, IMU
- MQTT мост для Android приложения
- Голосовой интерфейс (Vosk)

#### b) Запуск вычислительных нод (на отдельном ПК, опционально)

```bash
cd ~/Samurai/compute_node

# YOLO детектор (требует камеры или ROS топика /camera/image_raw)
python3 yolo_detector_node.py

# Depth to LaserScan конвертер
python3 depth_to_scan_node.py

# Geofence publisher
python3 geofence_map_publisher.py
```

#### c) Запуск SLAM (картографирование)

```bash
ros2 launch robot_pkg slam.launch.py
```

#### d) Запуск навигации (Nav2)

```bash
# Сначала загрузите карту
ros2 launch robot_pkg navigation.launch.py map:=/path/to/map.yaml
```

---

### Android приложение

1. Установите APK на Android устройство
2. Подключитесь к той же Wi-Fi сети, что и робот
3. В настройках приложения укажите IP адрес MQTT брокера (адрес Raspberry Pi)
4. Управляйте роботом через джойстик или голосовые команды

**Функции:**
- Виртуальный джойстик для ручного управления
- Голосовые команды
- Просмотр видеопотока с камеры
- Отображение телеметрии (датчики, координаты)

---

## 📖 API Reference

Симулятор предоставляет REST API на порту 5000. Полная документация доступна в [API_REFERENCE.md](API_REFERENCE.md).

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

### ROS2 Топики

| Топик | Тип сообщения | Описание |
|-------|---------------|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Команды скорости для моторов |
| `/camera/image_raw` | `sensor_msgs/Image` | Видеопоток с камеры |
| `/scan` | `sensor_msgs/LaserScan` | Данные лидара (или эмулированные) |
| `/imu/data` | `sensor_msgs/Imu` | Данные IMU (MPU6050) |
| `/ultrasonic` | `sensor_msgs/Range` | Ультразвуковой датчик |
| `/ball_detection` | `robot_pkg/BallDetection` | Результаты YOLO детектора |
| `/voice/command` | `std_msgs/String` | Распознанные голосовые команды |

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

- `найди красный мяч` — найти и захватить красный мяч
- `получи синий мяч` — найти и захватить синий мяч
- `сожги жёлтый мяч` — найти и сжечь жёлтый мяч лазером
- `стоп` — экстренная остановка
- `домой` — вернуться на базу
- `вызови вторую машину` — вызвать второго робота

### Цвета

- красный / красн
- синий / синего
- зелёный / зелен
- жёлтый / желт
- оранжевый / оранж

---

## 🐛 Отладка

### Логи ROS2

```bash
ros2 topic echo /rosout
```

### Просмотр топиков

```bash
# Список всех топиков
ros2 topic list

# Посмотреть сообщения в топике
ros2 topic echo /camera/image_raw
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
