# ros-base вместо humble-desktop → образ ~800MB вместо ~2.5GB (−1.7GB)
# Без GUI компонентов (Rviz, Qt), которые не нужны в headless режиме
#
# Пинируем к конкретной платформе (jammy = Ubuntu 22.04) для
# детерминированных сборок. Не используем floating tag :humble-ros-base,
# который может получать breaking-changes при обновлении базового образа.
FROM ros:humble-ros-base-jammy

# Применяем security-патчи из upstream (часть уязвимостей базового образа
# закрывается обновлением системных пакетов)
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Только нужные ROS2 пакеты
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    python3-pip \
    python3-colcon-common-extensions \
    avahi-daemon \
    avahi-utils \
    libnss-mdns \
    dbus \
    && rm -rf /var/lib/apt/lists/*

# Python зависимости:
#   - onnxruntime вместо torch+torchvision CPU → -500MB RAM, +50% инференс
#   - ultralytics нужен только для экспорта .pt → .onnx (первый запуск)
#   - fastapi+uvicorn вместо flask+socketio → async, меньше CPU
RUN pip3 install --no-cache-dir \
    "ultralytics>=8.0" \
    onnxruntime \
    "fastapi>=0.110" \
    "uvicorn[standard]>=0.27" \
    python-multipart \
    paho-mqtt \
    && pip3 install --no-cache-dir "numpy<2"

# Build ROS2 workspace
WORKDIR /root/Samurai/ros_ws
ENV ROS_DOMAIN_ID=42

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]
