# ros-base вместо humble-desktop → образ ~800MB вместо ~2.5GB (−1.7GB)
# Без GUI компонентов (Rviz, Qt), которые не нужны в headless режиме
#
# Пинируем к конкретной платформе (jammy = Ubuntu 22.04) для
# детерминированных сборок. Не используем floating tag :humble-ros-base,
# который может получать breaking-changes при обновлении базового образа.
FROM ros:humble-ros-base-jammy

# Зеркало apt: yandex быстрее archive.ubuntu.com из RU.
# Отключить (использовать дефолт): --build-arg APT_MIRROR=
ARG APT_MIRROR=mirror.yandex.ru

# Намеренно НЕ запускаем `apt-get upgrade`: security-патчи приходят с
# обновлением базового тега `ros:humble-ros-base-jammy` (docker pull
# раз в неделю — рекомендуемая практика). Upgrade в Dockerfile = +500MB,
# +5-10 минут, невоспроизводимая сборка.
#
# BuildKit cache mounts ускоряют пересборки в разы: индексы и .deb файлы
# переиспользуются, в слой образа не попадают (rm не нужен).
#
# Base image содержит libpng16-16=1.6.37-3ubuntu0.5 (этой версии больше нет
# в jammy-updates/security), а libpng-dev доступен только как -3ubuntu0.4
# и строго пинит libpng16-16 на ту же -3ubuntu0.4. Downgrade libpng16-16
# до -3ubuntu0.4 синхронизирует версии и снимает конфликт при установке
# ROS-пакетов, которые транзитивно тянут libpng-dev.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    if [ -n "${APT_MIRROR}" ]; then \
        sed -i "s|http://archive.ubuntu.com|http://${APT_MIRROR}|g; s|http://security.ubuntu.com|http://${APT_MIRROR}|g" /etc/apt/sources.list; \
    fi && \
    apt-get update && apt-get install -y --no-install-recommends --allow-downgrades \
    libpng16-16=1.6.37-3ubuntu0.4 \
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
    dbus

# Python зависимости:
#   - onnxruntime вместо torch+torchvision CPU → -500MB RAM, +50% инференс
#   - ultralytics нужен только для экспорта .pt → .onnx (первый запуск)
#   - fastapi+uvicorn вместо flask+socketio → async, меньше CPU
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install \
    "numpy<2" \
    "ultralytics>=8.0" \
    onnxruntime \
    "fastapi>=0.110" \
    "uvicorn[standard]>=0.27" \
    "python-socketio[asyncio_client]>=5.11" \
    python-multipart \
    paho-mqtt \
    flask \
    flask-cors \
    flask-socketio \
    timm

# Build ROS2 workspace
WORKDIR /root/Samurai/ros_ws
ENV ROS_DOMAIN_ID=42

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
