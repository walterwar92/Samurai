#!/usr/bin/env bash
# =============================================================================
# start_robot.sh — [DEPRECATED] Запуск Samurai Robot через Docker (Dockerfile.robot)
#
# ╔══════════════════════════════════════════════════════════════════╗
# ║  УСТАРЕЛ — используй start_robot_mqtt.sh вместо этого скрипта  ║
# ║  Новый скрипт: чистый Python + MQTT, без Docker / ROS2 на Pi   ║
# ║  Запуск: ./start_robot_mqtt.sh                                 ║
# ╚══════════════════════════════════════════════════════════════════╝
#
# Платформа: Raspberry Pi 4 | Raspberry Pi OS Trixie (arm64)
#
# Использование (старый способ через Docker):
#   chmod +x start_robot.sh
#   ./start_robot.sh
# =============================================================================

set -euo pipefail

# ── Цвета ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log_ok()   { echo -e "${GREEN}  [✓]${NC} $*"; }
log_warn() { echo -e "${YELLOW}  [!]${NC} $*"; }
log_err()  { echo -e "${RED}  [✗]${NC} $*"; }
log_info() { echo -e "${BLUE}  [→]${NC} $*"; }
log_step() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }
die()      { log_err "$*"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_IMAGE="samurai-robot"
DOCKERFILE="$SCRIPT_DIR/Dockerfile.robot"
CONTAINER_NAME="samurai_robot"
ROS_DOMAIN_ID=42

# ── Баннер ───────────────────────────────────────────────────────────────────
print_banner() {
    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "?")

    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║       Raspberry Pi 4  |  Docker Mode         ║"
    echo "  ║  Моторы | Серво | Камера | MQTT | FSM        ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  Hostname: ${BOLD}$(hostname)${NC}  IP: ${BOLD}${my_ip}${NC}"
    echo ""
}

# ─── 1. Docker ───────────────────────────────────────────────────────────────
check_docker() {
    log_step "Проверка Docker"

    if ! command -v docker &>/dev/null; then
        log_err "Docker не установлен."
        echo ""
        echo -e "  Установка Docker на Raspberry Pi OS:"
        echo -e "    ${YELLOW}curl -fsSL https://get.docker.com | sh${NC}"
        echo -e "    ${YELLOW}sudo usermod -aG docker \$USER && newgrp docker${NC}"
        die "Установи Docker и перезапусти скрипт"
    fi
    log_ok "Docker $(docker --version | grep -oP '\d+\.\d+\.\d+' | head -1)"

    if ! docker info &>/dev/null 2>&1; then
        log_warn "Docker daemon не запущен. Пробую запустить..."
        sudo systemctl start docker 2>/dev/null \
            || die "Не удалось запустить Docker. Попробуй: sudo systemctl start docker"
        sleep 2
        docker info &>/dev/null 2>&1 \
            || die "Docker daemon не отвечает. Проверь: sudo journalctl -u docker"
        log_ok "Docker daemon запущен"
    else
        log_ok "Docker daemon активен"
    fi
}

# ─── 2. Docker образ ─────────────────────────────────────────────────────────
check_image() {
    log_step "Docker образ '$DOCKER_IMAGE'"

    [[ -f "$DOCKERFILE" ]] \
        || die "Dockerfile.robot не найден: $DOCKERFILE"

    local need_build=false

    if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null 2>&1; then
        log_warn "Образ не найден — первая сборка займёт 15–25 минут на Pi..."
        need_build=true
    else
        log_ok "Образ '$DOCKER_IMAGE' существует"

        # Если Dockerfile.robot новее образа — предложить пересборку
        local df_mtime img_created
        df_mtime=$(stat -c %Y "$DOCKERFILE" 2>/dev/null || echo 0)
        img_created=$(docker inspect --format='{{.Created}}' "$DOCKER_IMAGE" 2>/dev/null \
            | xargs -I{} date -d "{}" +%s 2>/dev/null || echo 0)

        if [[ "$df_mtime" -gt "$img_created" ]] 2>/dev/null; then
            log_warn "Dockerfile.robot изменился после сборки образа."
            read -rp "    Пересобрать образ? [y/N]: " ans
            [[ "$ans" =~ ^[Yy]$ ]] && need_build=true
        fi
    fi

    if $need_build; then
        log_info "Сборка образа '$DOCKER_IMAGE' (arm64)..."
        docker build \
            --platform linux/arm64 \
            -f "$DOCKERFILE" \
            -t "$DOCKER_IMAGE" \
            "$SCRIPT_DIR" \
            || die "Сборка Docker образа провалилась. Проверь Dockerfile.robot."
        log_ok "Образ '$DOCKER_IMAGE' собран"
    fi
}

# ─── 3. Железо: I2C ──────────────────────────────────────────────────────────
check_i2c() {
    log_step "Интерфейс I2C"

    if [[ ! -e /dev/i2c-1 ]]; then
        log_warn "/dev/i2c-1 не найден. Пробую включить I2C..."
        if command -v raspi-config &>/dev/null; then
            sudo raspi-config nonint do_i2c 0 2>/dev/null || true
            log_warn "I2C включён — может потребоваться перезагрузка"
        fi
        [[ -e /dev/i2c-1 ]] \
            || log_warn "/dev/i2c-1 всё ещё недоступен (без IMU/HAT)"
        return
    fi

    log_ok "I2C доступен (/dev/i2c-1)"

    # Краткое сканирование шины
    if command -v i2cdetect &>/dev/null; then
        log_info "I2C устройства на шине:"
        i2cdetect -y 1 2>/dev/null | while IFS= read -r line; do
            echo "    $line"
        done
    fi
}

# ─── 4. Камера ───────────────────────────────────────────────────────────────
check_camera() {
    log_step "Камера (libcamera)"

    # Проверяем наличие picamera2/libcamera для монтирования в Docker
    local pic2_path="/usr/lib/python3/dist-packages/picamera2"
    local libcam_path="/usr/lib/python3/dist-packages/libcamera"

    if [[ ! -d "$pic2_path" ]]; then
        log_warn "picamera2 не найдена на хосте: $pic2_path"
        echo -e "    Установи: ${YELLOW}sudo apt install -y python3-picamera2${NC}"
        log_warn "camera_node внутри Docker будет работать без камеры"
    else
        log_ok "picamera2 найдена: $pic2_path"
    fi

    if [[ ! -d "$libcam_path" ]]; then
        log_warn "libcamera Python bindings не найдены: $libcam_path"
    else
        log_ok "libcamera найдена: $libcam_path"
    fi

    # Проверка реального детекта камеры
    if command -v libcamera-hello &>/dev/null; then
        if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
            log_ok "Камера обнаружена (libcamera)"
        else
            log_warn "libcamera: камера НЕ обнаружена — проверь CSI-кабель"
        fi
    else
        log_warn "libcamera-hello не найден (пропуск проверки камеры)"
    fi
}

# ─── 5. MQTT брокер (mosquitto) ───────────────────────────────────────────────
check_mqtt() {
    log_step "MQTT брокер (mosquitto)"

    if ! command -v mosquitto &>/dev/null; then
        log_warn "mosquitto не установлен."
        echo -e "    Установи: ${YELLOW}sudo apt install -y mosquitto mosquitto-clients${NC}"
        echo -e "    Запуск:   ${YELLOW}sudo systemctl enable --now mosquitto${NC}"
        read -rp "    Продолжить без MQTT? [y/N]: " ans
        [[ "$ans" =~ ^[Yy]$ ]] || die "Установи mosquitto и перезапусти"
        return
    fi

    if ! systemctl is-active --quiet mosquitto 2>/dev/null; then
        log_warn "mosquitto не запущен. Запускаю..."
        sudo systemctl start mosquitto 2>/dev/null \
            && log_ok "mosquitto запущен" \
            || log_warn "mosquitto не запустился (MQTT-мост с телефоном не будет работать)"
    else
        log_ok "mosquitto активен (порт 1883)"
    fi
}

# ─── 6. avahi (mDNS) ─────────────────────────────────────────────────────────
check_avahi() {
    log_step "mDNS (avahi)"

    if ! command -v avahi-daemon &>/dev/null; then
        log_warn "avahi не установлен — Pi недоступен по $(hostname).local"
        echo -e "    Установи: ${YELLOW}sudo apt install -y avahi-daemon${NC}"
        return
    fi

    if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
        log_warn "avahi-daemon не запущен. Запускаю..."
        sudo systemctl start avahi-daemon 2>/dev/null || true
    fi

    systemctl is-active --quiet avahi-daemon 2>/dev/null \
        && log_ok "avahi активен — Pi доступен по: ${BOLD}$(hostname).local${NC}" \
        || log_warn "avahi не запущен"
}

# ─── 7. ROS2 Workspace ───────────────────────────────────────────────────────
check_workspace() {
    log_step "ROS2 Workspace (сборка внутри Docker)"

    local ws="$SCRIPT_DIR/ros_ws"
    local install_dir="$ws/install"

    build_ws() {
        log_info "Сборка workspace в Docker (colcon build)..."
        docker run --rm \
            --platform linux/arm64 \
            -v "$SCRIPT_DIR:/root/Samurai" \
            "$DOCKER_IMAGE" \
            bash -c "
                set -e
                source /opt/ros/humble/setup.bash
                cd /root/Samurai/ros_ws
                colcon build \
                    --symlink-install \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                    --event-handlers console_cohesion+
            " || die "colcon build провалился"
        log_ok "Workspace собран"
    }

    if [[ ! -d "$install_dir" ]]; then
        log_warn "ros_ws/install не найден — workspace не собран"
        build_ws
    else
        log_ok "Workspace собран (ros_ws/install найден)"

        # Есть ли Python-файлы новее install/?
        local stale
        stale=$(find "$ws/src" -name "*.py" -newer "$install_dir" 2>/dev/null | head -1 || true)
        if [[ -n "$stale" ]]; then
            log_warn "Найдены изменения в исходниках ($(basename "$stale") и др.)"
            read -rp "    Пересобрать workspace? [y/N]: " ans
            [[ "$ans" =~ ^[Yy]$ ]] && build_ws
        fi
    fi
}

# ─── 8. Настройка сети ───────────────────────────────────────────────────────
configure_network() {
    log_step "Настройка сети"

    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "?")

    echo ""
    echo -e "  IP этого Pi: ${BOLD}${my_ip}${NC}"
    echo -e "  mDNS адрес:  ${BOLD}$(hostname).local${NC}"
    echo ""
    echo -e "  ${BOLD}Выбери режим сети:${NC}"
    echo -e "  ${GREEN}1)${NC} LAN / домашний WiFi  — multicast DDS (по умолчанию)"
    echo -e "  ${GREEN}2)${NC} Мобильный хотспот    — unicast DDS (нужен IP ноутбука)"
    echo ""
    read -rp "  Режим [1/2]: " net_mode

    PEER_IP=""
    MQTT_BROKER="$my_ip"

    if [[ "$net_mode" == "2" ]]; then
        echo ""
        log_info "Как узнать IP ноутбука: на ноутбуке выполни: ip route get 1 | awk '{print \$7}'"
        read -rp "  IP ноутбука в сети хотспота (напр. 192.168.43.50): " PEER_IP

        if [[ ! "$PEER_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            die "Неверный IP: '$PEER_IP'"
        fi
        log_ok "Unicast DDS: peer_ip=$PEER_IP"
        log_ok "MQTT broker: $MQTT_BROKER"
    else
        log_ok "Multicast DDS (стандартный режим)"
        log_ok "MQTT broker: $MQTT_BROKER"
    fi
}

# ─── 9. Запуск ───────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск robot_bringup в Docker"

    # Остановить старый контейнер если ещё работает
    if docker ps -q --filter "name=$CONTAINER_NAME" 2>/dev/null | grep -q .; then
        log_warn "Останавливаю предыдущий контейнер '$CONTAINER_NAME'..."
        docker stop "$CONTAINER_NAME" &>/dev/null || true
    fi

    # Пути к picamera2/libcamera на хосте для монтирования в контейнер
    local pic2_vol="" libcam_vol=""
    [[ -d "/usr/lib/python3/dist-packages/picamera2" ]] \
        && pic2_vol="-v /usr/lib/python3/dist-packages/picamera2:/opt/picamera2:ro"
    [[ -d "/usr/lib/python3/dist-packages/libcamera" ]] \
        && libcam_vol="-v /usr/lib/python3/dist-packages/libcamera:/opt/libcamera:ro"

    # Аргументы launch
    local launch_args="mqtt_broker:=$MQTT_BROKER"
    [[ -n "$PEER_IP" ]] && launch_args="$launch_args peer_ip:=$PEER_IP"

    echo ""
    log_ok "Образ:           $DOCKER_IMAGE"
    log_ok "ROS_DOMAIN_ID:   $ROS_DOMAIN_ID"
    log_ok "MQTT broker:     $MQTT_BROKER:1883"
    [[ -n "$PEER_IP" ]] && log_ok "Unicast DDS:     peer_ip=$PEER_IP"
    echo ""
    echo -e "${YELLOW}  ── Запуск ROS2 нод (Ctrl+C для остановки) ──${NC}"
    echo ""

    # shellcheck disable=SC2086
    docker run --rm \
        --name "$CONTAINER_NAME" \
        --platform linux/arm64 \
        --privileged \
        --network host \
        -v /dev:/dev \
        -v /run/udev:/run/udev:ro \
        $pic2_vol \
        $libcam_vol \
        -v "$SCRIPT_DIR:/root/Samurai" \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        "$DOCKER_IMAGE" \
        bash -c "
            source /opt/ros/humble/setup.bash
            source /root/Samurai/ros_ws/install/setup.bash
            exec ros2 launch robot_pkg robot_bringup.launch.py $launch_args
        "
}

# ─── Main ────────────────────────────────────────────────────────────────────
main() {
    print_banner
    check_docker
    check_image
    check_i2c
    check_camera
    check_mqtt
    check_avahi
    check_workspace
    configure_network
    launch
}

main
