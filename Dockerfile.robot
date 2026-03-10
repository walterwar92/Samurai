# =============================================================
# Dockerfile.robot — Samurai Robot (Raspberry Pi 4 B, arm64)
# =============================================================
#
# Ноды на роботе (robot_bringup.launch.py):
#   motor, ultrasonic, camera, laser, servo, imu,
#   fsm, mqtt_bridge, battery, temperature, watchdog
#
# На ноутбуке (Dockerfile):
#   yolo_detector, dashboard, slam_toolbox, navigation2
#
# -------------------------------------------------------------
# Сборка (на Pi или cross-compile):
#   docker build --platform linux/arm64 -f Dockerfile.robot -t samurai-robot .
#
# Запуск на Pi 4:
#   docker run --rm -it                                          \
#     --privileged                                              \
#     --network host                                            \
#     -v /dev:/dev                                              \
#     -v /run/udev:/run/udev:ro                                 \
#     -v /usr/lib/python3/dist-packages/picamera2:/opt/picamera2:ro  \
#     -v /usr/lib/python3/dist-packages/libcamera:/opt/libcamera:ro  \
#     -v ~/Samurai:/root/Samurai                                \
#     samurai-robot
#
# --privileged  — доступ к GPIO, I2C (/dev/i2c-1), /dev/gpiomem
# --network host — DDS (ROS2) + MQTT без NAT
# /dev           — /dev/i2c-1, /dev/gpiomem, /dev/video*
# /run/udev      — udev для libcamera/picamera2
# picamera2/libcamera — монтируются с хоста (Pi OS Trixie)
# =============================================================

FROM --platform=linux/arm64 ros:humble-ros-base-jammy

# Security patches
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# ROS2 пакеты нужные только роботу (без Nav2/SLAM/YOLO)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-tf2-ros          \
    ros-humble-cv-bridge        \
    python3-pip                 \
    python3-colcon-common-extensions \
    # Системные I2C утилиты
    i2c-tools                   \
    python3-smbus               \
    # OpenCV и NumPy из apt (быстрее чем pip на arm64)
    python3-numpy               \
    python3-opencv              \
    && rm -rf /var/lib/apt/lists/*

# Python пакеты для аппаратных нод
RUN pip3 install --no-cache-dir \
    # PCA9685 — моторы и сервоприводы
    adafruit-circuitpython-pca9685  \
    adafruit-circuitpython-motor    \
    adafruit-circuitpython-servokit \
    # I2C / ADC
    smbus2                          \
    # WS2812 RGB LED
    rpi-ws281x                      \
    # GPIO
    gpiozero                        \
    # MQTT мост (телефон ↔ робот)
    paho-mqtt                       \
    # IMU: вычисление ориентации из кватернионов
    transforms3d

# picamera2 и libcamera НЕ устанавливаются в образ —
# они специфичны для Raspberry Pi OS и требуют нативных .so
# Монтируй с хоста при запуске (см. -v /usr/lib/python3/... выше)
# и добавь в PYTHONPATH:
ENV PYTHONPATH="/opt/picamera2:/opt/libcamera:${PYTHONPATH}"

WORKDIR /root/Samurai/ros_ws
ENV ROS_DOMAIN_ID=42

# При старте: source ROS2, затем workspace (если собран)
ENTRYPOINT ["/bin/bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    if [ -f /root/Samurai/ros_ws/install/setup.bash ]; then \
        source /root/Samurai/ros_ws/install/setup.bash; \
    fi && \
    exec bash"]
