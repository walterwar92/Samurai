# Диагностика робота Гоша

## Файлы

- `diagnostic.py` — скрипт диагностики железа
- `diagnostic_requirements.txt` — Python-зависимости для диагностики
- `setup_robot.sh` — полная установка зависимостей и сборка C++ нод

## Установка зависимостей и сборка

```bash
cd ~/Samurai
sudo bash Diagnostic/setup_robot.sh
```

Скрипт выполняет:
1. Системные пакеты: `python3-*`, `build-essential`, `cmake`, `i2c-tools`, `portaudio19-dev`
2. I2C / SPI / Camera через raspi-config
3. pip: adafruit-*, smbus2, rpi_ws281x, **webrtcvad**, transforms3d, onnxruntime (опционально)
4. GPIO daemon (pigpiod / lgpio)
5. **Сборка C++ нод** (`robot_pkg_cpp`) через colcon — `imu_node` и `motor_node`
6. Проверка всех пакетов (`import` тест)
7. Сканирование I2C: `0x5F` (PCA9685), `0x68` (MPU6050)

> Только для диагностики (`diagnostic.py`):
> ```bash
> sudo pip3 install --break-system-packages -r Diagnostic/diagnostic_requirements.txt
> sudo python3 Diagnostic/diagnostic.py
> ```

## Ручная сборка C++ нод

Если `setup_robot.sh` пропустил сборку (ROS2 не был установлен на момент запуска):

```bash
source /opt/ros/humble/setup.bash
cd ~/Samurai/ros_ws
colcon build --packages-select robot_pkg_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Бинарники: `ros_ws/install/robot_pkg_cpp/lib/robot_pkg_cpp/`

## Запуск диагностики

```bash
sudo python3 Diagnostic/diagnostic.py
```

`sudo` обязателен — WS2812 LED работает только от root.

## Результаты

Всё сохраняется в `~/samurai_diagnostics/`:

```
~/samurai_diagnostics/
├── diagnostic_20260216_193045.log   # полный текстовый лог
├── diagnostic_20260216_193045.json  # JSON-отчёт со всеми данными
├── camera_snapshot_20260216_193045.jpg  # фото с камеры
└── mic_test_20260216_193045.wav     # 2с запись микрофона
```

## Что делает робот при запуске диагностики

1. Едет **вперёд 1с** → **назад 1с**
2. Поворачивает **влево 1с** → **вправо 1с**
3. Каждый мотор отдельно: вперёд 0.5с, назад 0.5с
4. Сервоприводы: init → тестовые углы → init
5. WS2812: красный → зелёный → синий → бегущий огонь → выкл
