# Samurai Robot — Инструкция первого запуска

> **Версия архитектуры:** Pure Python + MQTT (без ROS2 на Pi)
> **Платформы:** Raspberry Pi 4 + Ноутбук (Arch Linux) + Android

---

## Оглавление

1. [Что понадобится](#1-что-понадобится)
2. [Прошивка и первоначальная настройка Pi](#2-прошивка-и-первоначальная-настройка-pi)
3. [Клонирование проекта на Pi](#3-клонирование-проекта-на-pi)
4. [Установка зависимостей на Pi](#4-установка-зависимостей-на-pi)
5. [Подключение к сети](#5-подключение-к-сети)
6. [Первый запуск робота](#6-первый-запуск-робота)
7. [Запуск ноутбука](#7-запуск-ноутбука)
8. [Подключение Android-приложения](#8-подключение-android-приложения)
9. [Проверка работоспособности](#9-проверка-работоспособности)
10. [Устранение проблем](#10-устранение-проблем)

---

## 1. Что понадобится

### Железо
| Компонент | Требования |
|-----------|-----------|
| Raspberry Pi 4 | 2 GB RAM или больше |
| microSD карта | 16 GB минимум, рекомендуется 32 GB Class 10 |
| Adeept HAT V3.1 | установлен на Pi |
| Ноутбук | Linux (Arch), 8 GB RAM, Docker |
| Android телефон | Android 8.0+ |
| Сеть | Wi-Fi роутер **или** мобильный хотспот |

### Программы на ноутбуке
```bash
# Arch Linux
sudo pacman -S git docker avahi nss-mdns openssh mosquitto-clients
sudo systemctl enable --now docker avahi-daemon
sudo usermod -aG docker $USER   # затем выйти и войти снова
```

---

## 2. Прошивка и первоначальная настройка Pi

### 2.1 Запись образа

1. Скачать **Raspberry Pi Imager**: https://www.raspberrypi.com/software/
2. Выбрать OS: **Raspberry Pi OS (64-bit)** → Debian 13 Trixie
   *(или Bookworm, если Trixie недоступен)*
3. Нажать **⚙ (шестерёнка)** перед записью:
   - ✅ **Hostname:** `raspberrypi`
   - ✅ **Enable SSH** → Use password authentication
   - ✅ **Username:** `pi` | **Password:** задать свой
   - ✅ **Wi-Fi:** имя сети и пароль домашней сети
   - ✅ **Locale:** `Europe/Moscow` (или свой)
4. Записать образ на microSD

### 2.2 Первое подключение по SSH

Вставить microSD в Pi, включить питание, подождать ~60 секунд:

```bash
# С ноутбука:
ssh pi@raspberrypi.local
# Пароль — тот, что задали при записи образа

# Если raspberrypi.local не резолвится — узнать IP через роутер
# или: sudo nmap -sn 192.168.1.0/24 | grep -A1 "Raspberry"
```

### 2.3 Базовая конфигурация Pi

```bash
# Обновить систему
sudo apt update && sudo apt upgrade -y

# Установить git (если нет)
sudo apt install -y git

# Установить hostname (если нужно изменить)
sudo hostnamectl set-hostname raspberrypi
```

---

## 3. Клонирование проекта на Pi

```bash
# На Pi:
cd ~
git clone https://github.com/ТВОЙ_АККАУНТ/Samurai.git
cd Samurai
```

> Если репозиторий приватный:
> ```bash
> git clone https://ТВОЙ_ЛОГИН:ТВОЙ_TOKEN@github.com/ТВОЙ_АККАУНТ/Samurai.git
> ```

---

## 4. Установка зависимостей на Pi

Один скрипт делает всё: apt-пакеты, pip-пакеты, I2C, mosquitto, avahi.

```bash
# На Pi (из папки Samurai):
sudo bash Diagnostic/setup_robot.sh
```

Что установит скрипт:
- `python3`, `pip`, `numpy`, `opencv`, `picamera2`, `smbus2`, `gpiozero`
- `paho-mqtt`, `PyYAML`, `transforms3d`
- `adafruit-circuitpython-pca9685`, `adafruit-motor`, `adafruit-servokit`
- `mosquitto` (MQTT брокер) + конфиг для LAN-доступа
- `avahi-daemon` (mDNS, чтобы Pi был виден как `raspberrypi.local`)
- Включает I2C, SPI, Camera через `raspi-config`

**Ожидаемый результат:**
```
[INFO]  All packages installed successfully!
[INFO]  Mosquitto is running (port 1883)
[INFO]  После перезагрузки запуск робота: cd ~/Samurai && ./start_robot_mqtt.sh
```

После завершения:
```bash
sudo reboot
```

---

## 5. Подключение к сети

### Вариант A: Домашний Wi-Fi роутер (рекомендуется для первого запуска)

Pi и ноутбук в **одной Wi-Fi сети**. Больше ничего настраивать не нужно —
ноутбук найдёт Pi автоматически через `raspberrypi.local`.

### Вариант Б: Мобильный хотспот (для работы вне дома)

1. Включить хотспот на телефоне
2. Подключить к нему Pi (через `raspi-config` → Network → Wi-Fi) и ноутбук
3. На ноутбуке запускать с флагом `--hotspot`:
   ```bash
   ./start_laptop_robot.sh --hotspot
   ```

### Вариант В: Прямое Ethernet-подключение Pi → Ноутбук

```bash
# На ноутбуке: включить IP-sharing (shared connection через NetworkManager)
# В nmtui: Edit Connection → Ethernet → IPv4 Method: Shared to other computers

# Затем узнать IP Pi:
arp -n | grep -i "b8:27\|dc:a6\|e4:5f\|28:cd"  # MAC-адреса Raspberry Pi
```

---

## 6. Первый запуск робота

### 6.1 Запуск на Pi

```bash
# На Pi (из папки ~/Samurai):
./start_robot_mqtt.sh
```

Скрипт автоматически:
1. Проверяет Python и зависимости
2. Проверяет I2C (выводит таблицу устройств)
3. Запускает Mosquitto если не запущен
4. Создаёт конфиг LAN-доступа если нужно
5. Запускает все 11 нод через `robot_launcher.py`

**Ожидаемый вывод:**
```
━━━ Запуск MQTT нод ━━━
  [✓] Broker:   192.168.1.50:1883
  [✓] Robot ID: robot1

[INFO] [launcher] Starting 11 nodes (broker=192.168.1.50:1883, id=robot1)
[INFO] [launcher]   [+] motor (pid=1234)
[INFO] [launcher]   [+] imu (pid=1235)
...
[INFO] [motor] MQTT connected (rc=0)
[INFO] [imu] MQTT connected (rc=0)
```

> **Если нет железа** — ноды запустятся в режиме симуляции. Моторы/IMU/камера
> не будут работать, но MQTT и FSM будут активны.

### 6.2 Автозапуск при старте Pi (опционально)

```bash
# Создать systemd сервис
sudo tee /etc/systemd/system/samurai.service <<'EOF'
[Unit]
Description=Samurai Robot MQTT Nodes
After=network-online.target mosquitto.service
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Samurai
ExecStart=/home/pi/Samurai/start_robot_mqtt.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable samurai
sudo systemctl start samurai

# Просмотр логов:
journalctl -u samurai -f
```

---

## 7. Запуск ноутбука

### 7.1 Клонировать проект на ноутбук

```bash
# На ноутбуке:
git clone https://github.com/ТВОЙ_АККАУНТ/Samurai.git
cd Samurai
chmod +x start_laptop_robot.sh start_laptop_sim.sh
```

### 7.2 Запустить управление реальным роботом

```bash
# Авто-обнаружение Pi через mDNS:
./start_laptop_robot.sh

# Или указать IP вручную:
./start_laptop_robot.sh --pi 192.168.1.50

# Хотспот-режим:
./start_laptop_robot.sh --hotspot

# Первый запуск (пересборка Docker образа ~15 мин):
./start_laptop_robot.sh --rebuild
```

**Что делает скрипт:**
1. Проверяет Docker, собирает образ `samurai` (если нужно)
2. Собирает ROS2 workspace внутри Docker (colcon build)
3. Находит Pi через `raspberrypi.local` или ping
4. Проверяет доступность MQTT порта 1883
5. Запускает Docker контейнер с ROS2 нодами

**Ожидаемый вывод:**
```
  ┌──────────────────────────────────────┐
  │  Dashboard:     http://localhost:5000  │
  │  Android:       http://192.168.1.10:5000
  │  MQTT Broker:   192.168.1.50:1883
  │  ROS_DOMAIN_ID: 42
  └──────────────────────────────────────┘
```

### 7.3 Открыть Dashboard

Открыть в браузере: **http://localhost:5000**

Интерфейс показывает:
- Состояние FSM (IDLE / MANUAL / AUTO / PATROLLING / ...)
- Видеопоток с камеры
- Показания IMU, ультразвука, батареи
- Лог голосовых команд
- Карту SLAM (когда активна)

---

## 8. Подключение Android-приложения

### 8.1 Установка APK

1. Включить на телефоне: **Настройки → Безопасность → Неизвестные источники**
2. Скопировать `android/app-release.apk` на телефон
3. Установить APK

### 8.2 Подключение в приложении

1. Открыть приложение Samurai
2. Нажать **⚙ Настройки**
3. Ввести:
   - **MQTT Broker:** IP ноутбука (например `192.168.1.10`) или IP Pi (`192.168.1.50`)
   - **Port:** `1883`
   - **Robot ID:** `robot1`
4. Нажать **Подключить**

> Приложение подключается **напрямую к брокеру Mosquitto на Pi**.
> Ноутбук для Android необязателен — можно управлять роботом без него.

### 8.3 Основные элементы управления

| Элемент | Действие |
|---------|---------|
| Джойстик | Движение (отправляет `samurai/robot1/cmd_vel`) |
| Кнопка AUTO | Переключить FSM в AUTONOMOUS |
| Кнопка STOP | Экстренная остановка (IDLE) |
| Микрофон | Голосовые команды (через Vosk на Pi) |
| Ползунок серво | Управление клешнёй |
| Лазер ON/OFF | Лазерный указатель |

---

## 9. Проверка работоспособности

### 9.1 Проверить MQTT топики с ноутбука

```bash
# Подписаться на все топики робота:
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/#' -v

# Ожидаемые сообщения каждые ~0.1–1 сек:
# samurai/robot1/imu         {"roll":0.1,"pitch":-0.3,"yaw":45.2,...}
# samurai/robot1/odom        {"x":0.0,"y":0.0,"theta":0.0,...}
# samurai/robot1/battery     {"voltage":7.85,"percent":92}
# samurai/robot1/ultrasonic  {"distance":0.45}
# samurai/robot1/temperature {"cpu_temp":42.3}
# samurai/robot1/fsm_state   {"state":"IDLE"}
```

### 9.2 Проверить ноды на Pi

```bash
# На Pi:
ps aux | grep python3

# Ожидаемо: по одному процессу на каждую ноду
# python3 -m pi_nodes.robot_launcher
# node_motor  (pid=...)
# node_imu    (pid=...)
# node_camera (pid=...)
# ...
```

### 9.3 Тест управления движением

```bash
# С ноутбука — послать команду движения:
mosquitto_pub -h raspberrypi.local \
  -t 'samurai/robot1/cmd_vel' \
  -m '{"linear":0.2,"angular":0.0}'

# Остановить:
mosquitto_pub -h raspberrypi.local \
  -t 'samurai/robot1/cmd_vel' \
  -m '{"linear":0.0,"angular":0.0}'
```

### 9.4 Проверка I2C устройств на Pi

```bash
# На Pi:
i2cdetect -y 1

# Ожидаемая таблица:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 5f   ← PCA9685
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --   ← MPU6050
```

---

## 10. Устранение проблем

### Pi не виден как `raspberrypi.local`

```bash
# Проверить что avahi запущен на Pi:
ssh pi@<IP> "systemctl status avahi-daemon"

# Установить на ноутбуке (Arch):
sudo pacman -S nss-mdns
sudo systemctl enable --now avahi-daemon

# Добавить mdns в /etc/nsswitch.conf (если не сделано автоматически):
# hosts: mymachines mdns_minimal [NOTFOUND=return] resolve [!UNAVAIL=return] files dns
```

### Mosquitto не принимает подключения снаружи

```bash
# На Pi — проверить конфиг:
cat /etc/mosquitto/conf.d/samurai.conf
# Должно быть:
# listener 1883
# allow_anonymous true

# Если нет — создать вручную:
sudo tee /etc/mosquitto/conf.d/samurai.conf <<'EOF'
listener 1883
allow_anonymous true
EOF
sudo systemctl restart mosquitto
```

### Нода вылетает с ImportError

```bash
# На Pi — проверить установку:
python3 -c "import paho.mqtt.client; print('paho OK')"
python3 -c "import yaml; print('PyYAML OK')"

# Переустановить если нужно:
pip3 install --break-system-packages paho-mqtt PyYAML
```

### I2C: устройства не найдены (PCA9685 / MPU6050)

```bash
# Проверить что I2C включён:
ls /dev/i2c*

# Если нет — включить и перезагрузить:
sudo raspi-config nonint do_i2c 0
sudo reboot
```

### Docker образ не собирается на ноутбуке

```bash
# Пересобрать с нуля:
docker rmi samurai 2>/dev/null; ./start_laptop_robot.sh --rebuild

# Проверить логи Docker:
docker build -t samurai . 2>&1 | tail -50
```

### Нет видео с камеры

```bash
# На Pi — проверить что камера включена:
libcamera-hello --timeout 2000

# Включить через raspi-config:
sudo raspi-config nonint do_camera 0
sudo reboot
```

### FSM застрял в неизвестном состоянии

```bash
# Сброс в IDLE через MQTT:
mosquitto_pub -h raspberrypi.local \
  -t 'samurai/robot1/fsm_cmd' \
  -m '{"command":"stop"}'
```

---

## Быстрая шпаргалка (Cheat Sheet)

```bash
# ── Pi ───────────────────────────────────────────
ssh pi@raspberrypi.local          # подключиться к Pi
cd ~/Samurai && ./start_robot_mqtt.sh   # запустить робота
journalctl -u samurai -f          # логи автозапуска

# ── Ноутбук ─────────────────────────────────────
./start_laptop_robot.sh           # запустить управление
./start_laptop_robot.sh --pi IP   # вручную задать IP Pi
./start_laptop_sim.sh             # запустить симулятор

# ── MQTT отладка ─────────────────────────────────
mosquitto_sub -h raspberrypi.local -t 'samurai/#' -v
mosquitto_pub -h raspberrypi.local -t 'samurai/robot1/cmd_vel' \
  -m '{"linear":0.2,"angular":0.0}'

# ── Состояние системы ────────────────────────────
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/fsm_state' -v
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/battery' -v
mosquitto_sub -h raspberrypi.local -t 'samurai/robot1/imu' -v
```

---

## Архитектура системы

```
┌─────────────────────────────────────────────────────────┐
│                   Ваша Wi-Fi сеть                        │
│                                                          │
│  ┌──────────────────┐       MQTT        ┌─────────────┐  │
│  │  Raspberry Pi 4  │◄─────────────────►│   Ноутбук   │  │
│  │                  │   port 1883       │   Docker    │  │
│  │  Mosquitto MQTT  │                   │   ROS2      │  │
│  │  11 Python нод   │                   │   SLAM/Nav2 │  │
│  │  Моторы/IMU/Cam  │                   │   Dashboard │  │
│  └──────────────────┘                   └─────────────┘  │
│           ▲                                    ▲          │
│           │ MQTT                               │ HTTP     │
│           │                                    │ :5000    │
│  ┌──────────────────┐                          │          │
│  │  Android App     │──────────────────────────┘          │
│  │  Управление      │          (опционально)              │
│  └──────────────────┘                                     │
└─────────────────────────────────────────────────────────┘
```
