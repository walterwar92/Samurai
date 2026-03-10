#!/bin/bash
# =============================================================================
# build_cpp_for_pi.sh — сборка C++ нод для Raspberry Pi 4 на ноутбуке
#
# Что делает:
#   1. Включает QEMU-эмуляцию ARM64 в Docker
#   2. Собирает robot_pkg_cpp под linux/arm64 (ros:humble-ros-base)
#   3. Извлекает бинарники в ./prebuilt/robot_pkg_cpp/
#   4. (Опционально) копирует на Pi по SSH
#
# Использование:
#   ./build_cpp_for_pi.sh                      # только сборка
#   ./build_cpp_for_pi.sh pi@raspberrypi.local  # сборка + деплой на Pi
#
# Требования на ноутбуке:
#   docker, docker buildx (>= 0.8), qemu-user-static
#   Arch: sudo pacman -S docker qemu-user-static binfmt-support
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_TARGET="${1:-}"                          # pi@raspberrypi.local или пусто
IMAGE="samurai-cpp-builder"
CONTAINER="cpp-extract-tmp"
OUT_DIR="$SCRIPT_DIR/prebuilt/robot_pkg_cpp"
PI_DEST_DIR="~/Samurai/prebuilt/robot_pkg_cpp"

# ── Проверки ──────────────────────────────────────────────────────────
command -v docker &>/dev/null || log_error "docker не найден"

# ── QEMU: ARM64 эмуляция ──────────────────────────────────────────────
log_info "Регистрация QEMU handlers для ARM64..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 2>/dev/null || \
    log_warn "qemu-user-static: пропуск (может уже быть настроен)"

# ── Сборка образа ─────────────────────────────────────────────────────
log_info "Сборка Docker образа для ARM64 (linux/arm64)..."
log_info "Это занимает 5–15 минут при первом запуске (скачивается ros:humble-ros-base)."

docker buildx build \
    --platform linux/arm64 \
    --load \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile.cpp_builder" \
    "$SCRIPT_DIR"

log_info "Образ собран: $IMAGE"

# ── Извлечение бинарников ─────────────────────────────────────────────
log_info "Извлечение бинарников..."

docker rm -f "$CONTAINER" 2>/dev/null || true
docker create --name "$CONTAINER" "$IMAGE" bash

mkdir -p "$OUT_DIR"
docker cp "$CONTAINER":/ros_ws/install/robot_pkg_cpp/lib/robot_pkg_cpp/. "$OUT_DIR/"
docker rm "$CONTAINER"

log_info "Бинарники извлечены:"
ls -lh "$OUT_DIR/"

# ── Деплой на Pi (опционально) ────────────────────────────────────────
if [ -n "$PI_TARGET" ]; then
    log_info "Копирование на Pi: $PI_TARGET:$PI_DEST_DIR ..."
    ssh "$PI_TARGET" "mkdir -p $PI_DEST_DIR"
    scp "$OUT_DIR/imu_node"   "$PI_TARGET:$PI_DEST_DIR/"
    scp "$OUT_DIR/motor_node" "$PI_TARGET:$PI_DEST_DIR/"

    log_info "Установка бинарников в ROS2 install-дерево..."
    ROS_LIB_DIR="/home/\$(logname)/Samurai/ros_ws/install/robot_pkg_cpp/lib/robot_pkg_cpp"
    ssh "$PI_TARGET" "
        mkdir -p $ROS_LIB_DIR
        cp $PI_DEST_DIR/imu_node   $ROS_LIB_DIR/
        cp $PI_DEST_DIR/motor_node $ROS_LIB_DIR/
        chmod +x $ROS_LIB_DIR/imu_node $ROS_LIB_DIR/motor_node
        echo 'OK: бинарники установлены'
    "
    log_info "Деплой завершён. На Pi можно запускать:"
    log_info "  source ~/Samurai/ros_ws/install/setup.bash"
    log_info "  ros2 run robot_pkg_cpp imu_node"
    log_info "  ros2 run robot_pkg_cpp motor_node"
else
    echo ""
    log_info "Бинарники готовы в: $OUT_DIR/"
    log_info "Для деплоя на Pi запустите:"
    log_warn "  ./build_cpp_for_pi.sh pi@raspberrypi.local"
    echo ""
    log_info "Или скопируйте вручную:"
    log_info "  scp $OUT_DIR/* pi@raspberrypi.local:~/Samurai/prebuilt/robot_pkg_cpp/"
fi
