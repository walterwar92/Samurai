#!/usr/bin/env bash
# scripts/cmds/build_cpp.sh — кросс-компиляция C++ нод для Pi (arm64) на ноутбуке
#
# Использование:
#   samurai build-cpp                          # только сборка → ./prebuilt/robot_pkg_cpp/
#   samurai build-cpp pi@raspberrypi.local     # сборка + scp на Pi

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"

main() {
    local pi_target="${1:-}"

    if [[ "$pi_target" == "-h" || "$pi_target" == "--help" ]]; then
        cat <<EOF
Использование: samurai build-cpp [pi@host]

Кросс-компилирует robot_pkg_cpp под linux/arm64 в Docker.
Бинарники появятся в: prebuilt/robot_pkg_cpp/

Если указан pi@host — копирует бинарники на Pi через scp.

Требования:
  - docker, docker buildx (>= 0.8)
  - qemu-user-static (Arch: sudo pacman -S qemu-user-static binfmt-support)
EOF
        exit 0
    fi

    print_banner "C R O S S - C O M P I L E" "C++ -> linux/arm64 -> Pi"

    check_docker

    local image="samurai-cpp-builder"
    local container="cpp-extract-tmp"
    local dockerfile="$SAMURAI_ROOT/Dockerfile.cpp_builder"
    local out_dir="$SAMURAI_ROOT/prebuilt/robot_pkg_cpp"
    local pi_dest="~/Samurai/prebuilt/robot_pkg_cpp"

    [[ -f "$dockerfile" ]] || die "Dockerfile.cpp_builder не найден: $dockerfile"

    log_step "QEMU ARM64 emulation"
    if docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 2>/dev/null; then
        log_ok "QEMU handlers зарегистрированы"
    else
        log_warn "QEMU: пропуск (возможно уже настроен)"
    fi

    log_step "Сборка ARM64 образа"
    log_info "Первый запуск: 5-15 минут (скачивает ros:humble-ros-base)"
    docker buildx build \
        --platform linux/arm64 \
        --load \
        -t "$image" \
        -f "$dockerfile" \
        "$SAMURAI_ROOT"
    log_ok "Образ '$image' готов"

    log_step "Извлечение бинарников"
    docker rm -f "$container" 2>/dev/null || true
    docker create --name "$container" "$image" bash >/dev/null

    mkdir -p "$out_dir"
    docker cp "$container":/ros_ws/install/robot_pkg_cpp/lib/robot_pkg_cpp/. "$out_dir/"
    docker rm "$container" >/dev/null

    log_ok "Бинарники в $out_dir:"
    ls -lh "$out_dir/" | sed 's/^/    /'

    if [[ -n "$pi_target" ]]; then
        log_step "Деплой на $pi_target"
        ssh "$pi_target" "mkdir -p $pi_dest"
        scp "$out_dir/imu_node"   "$pi_target:$pi_dest/"
        scp "$out_dir/motor_node" "$pi_target:$pi_dest/"

        local ros_lib_dir='/home/$(logname)/Samurai/ros_ws/install/robot_pkg_cpp/lib/robot_pkg_cpp'
        ssh "$pi_target" "
            mkdir -p $ros_lib_dir
            cp $pi_dest/imu_node   $ros_lib_dir/
            cp $pi_dest/motor_node $ros_lib_dir/
            chmod +x $ros_lib_dir/imu_node $ros_lib_dir/motor_node
        "
        log_ok "Деплой завершён"
        echo ""
        log_info "На Pi запуск:"
        log_info "  source ~/Samurai/ros_ws/install/setup.bash"
        log_info "  ros2 run robot_pkg_cpp imu_node"
        log_info "  ros2 run robot_pkg_cpp motor_node"
    else
        echo ""
        log_info "Для деплоя на Pi: ${YELLOW}samurai build-cpp pi@raspberrypi.local${NC}"
    fi
}

main "$@"
