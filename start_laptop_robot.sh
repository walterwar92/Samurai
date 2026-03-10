#!/usr/bin/env bash
# =============================================================================
# start_laptop_robot.sh — Запуск ноутбука для управления РЕАЛЬНЫМ роботом
# Платформа: Arch Linux | Метод: Docker (ROS2 Humble + SLAM + Nav2 + YOLO)
#
# Ноутбук запускает:
#   - MQTT ↔ ROS2 bridge (принимает данные Pi через MQTT)
#   - SLAM Toolbox (карта)
#   - Nav2 (навигация)
#   - YOLO detector (распознавание)
#   - Web Dashboard (:5000)
#
# Использование:
#   chmod +x start_laptop_robot.sh
#   ./start_laptop_robot.sh                   # авто-обнаружение Pi
#   ./start_laptop_robot.sh --pi 192.168.1.50 # указать IP вручную
#   ./start_laptop_robot.sh --hotspot         # режим мобильного хотспота
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
DOCKER_IMAGE="samurai"
CONTAINER_NAME="samurai_compute"
ROS_DOMAIN_ID=42

# ── Аргументы командной строки ────────────────────────────────────────────────
PI_IP_ARG=""
HOTSPOT_MODE=false
REBUILD_IMAGE=false
REBUILD_WS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pi)       PI_IP_ARG="$2"; shift 2 ;;
        --hotspot)  HOTSPOT_MODE=true; shift ;;
        --rebuild)  REBUILD_IMAGE=true; REBUILD_WS=true; shift ;;
        --rebuild-image) REBUILD_IMAGE=true; shift ;;
        --rebuild-ws)    REBUILD_WS=true; shift ;;
        -h|--help)
            echo "Использование: $0 [--pi IP] [--hotspot] [--rebuild]"
            echo ""
            echo "  --pi IP       IP Raspberry Pi (вместо авто-обнаружения)"
            echo "  --hotspot     Режим мобильного хотспота (unicast DDS)"
            echo "  --rebuild     Пересобрать Docker образ и workspace"
            echo "  --rebuild-image  Пересобрать только Docker образ"
            echo "  --rebuild-ws     Пересобрать только ROS2 workspace"
            exit 0
            ;;
        *) die "Неизвестный аргумент: $1. Используй --help" ;;
    esac
done

# ── Баннер ───────────────────────────────────────────────────────────────────
print_banner() {
    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "?")

    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║       Ноутбук → Реальный робот               ║"
    echo "  ║  MQTT Bridge | SLAM | Nav2 | YOLO | Docker   ║"
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
        echo -e "    Установка: ${YELLOW}sudo pacman -S docker${NC}"
        echo -e "    Запуск:    ${YELLOW}sudo systemctl enable --now docker${NC}"
        echo -e "    Доступ:    ${YELLOW}sudo usermod -aG docker \$USER && newgrp docker${NC}"
        die "Установи Docker и перезапусти скрипт"
    fi
    log_ok "Docker $(docker --version | grep -oP '\d+\.\d+\.\d+' | head -1)"

    if ! docker info &>/dev/null 2>&1; then
        log_warn "Docker daemon не запущен. Запускаю..."
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

    local need_build=false

    if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null 2>&1; then
        log_warn "Образ не найден — первая сборка займёт 10–20 минут..."
        need_build=true
    elif $REBUILD_IMAGE; then
        log_info "Пересборка образа (--rebuild)"
        need_build=true
    else
        log_ok "Образ '$DOCKER_IMAGE' существует"

        # Dockerfile новее образа?
        local df_mtime img_created
        df_mtime=$(stat -c %Y "$SCRIPT_DIR/Dockerfile" 2>/dev/null || echo 0)
        img_created=$(docker inspect --format='{{.Created}}' "$DOCKER_IMAGE" 2>/dev/null \
            | xargs -I{} date -d "{}" +%s 2>/dev/null || echo 0)

        if [[ "$df_mtime" -gt "$img_created" ]] 2>/dev/null; then
            log_warn "Dockerfile изменился — пересобираю образ..."
            need_build=true
        fi
    fi

    if $need_build; then
        log_info "Сборка образа '$DOCKER_IMAGE'..."
        docker build -t "$DOCKER_IMAGE" "$SCRIPT_DIR" \
            || die "Сборка Docker образа провалилась. Проверь Dockerfile."
        log_ok "Образ собран"
    fi
}

# ─── 3. ROS2 Workspace ───────────────────────────────────────────────────────
check_workspace() {
    log_step "ROS2 Workspace (ros_ws)"

    local ws="$SCRIPT_DIR/ros_ws"
    local install_dir="$ws/install"

    build_ws() {
        log_info "Сборка workspace внутри Docker (colcon build)..."
        docker run --rm \
            -v "$SCRIPT_DIR:/root/Samurai" \
            "$DOCKER_IMAGE" \
            bash -c "
                set -e
                source /opt/ros/humble/setup.bash
                cd /root/Samurai/ros_ws
                colcon build --symlink-install \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                    --event-handlers console_cohesion+
            " || die "colcon build провалился"
        log_ok "Workspace собран"
    }

    if [[ ! -d "$install_dir" ]]; then
        log_warn "ros_ws/install не найден — workspace не собран"
        build_ws
    elif $REBUILD_WS; then
        log_info "Пересборка workspace (--rebuild)"
        build_ws
    else
        log_ok "Workspace собран (ros_ws/install найден)"

        # Есть ли файлы новее install/?
        local stale
        stale=$(find "$ws/src" -name "*.py" -newer "$install_dir" 2>/dev/null | head -1 || true)
        if [[ -n "$stale" ]]; then
            log_warn "Найдены изменения — пересобираю workspace..."
            build_ws
        fi
    fi
}

# ─── 4. mDNS (avahi) ─────────────────────────────────────────────────────────
check_avahi() {
    log_step "mDNS (avahi)"

    if ! command -v avahi-browse &>/dev/null; then
        log_warn "avahi не установлен — авто-обнаружение Pi может не работать"
        echo -e "    Установка: ${YELLOW}sudo pacman -S avahi nss-mdns${NC}"
        echo -e "    Запуск:    ${YELLOW}sudo systemctl enable --now avahi-daemon${NC}"
        return
    fi

    if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
        log_warn "avahi-daemon не запущен. Запускаю..."
        sudo systemctl start avahi-daemon 2>/dev/null || true
    fi

    systemctl is-active --quiet avahi-daemon 2>/dev/null \
        && log_ok "avahi активен (mDNS работает)" \
        || log_warn "avahi не запущен"
}

# ─── 5. Обнаружение Raspberry Pi ─────────────────────────────────────────────
discover_pi() {
    log_step "Обнаружение Raspberry Pi"

    # Если IP указан через --pi, используем его
    if [[ -n "$PI_IP_ARG" ]]; then
        PI_IP="$PI_IP_ARG"
        log_ok "IP Pi (из аргументов): ${BOLD}$PI_IP${NC}"
    else
        # Пробуем mDNS: raspberrypi.local
        log_info "Поиск raspberrypi.local (mDNS)..."
        PI_IP=""

        # Получаем IP через getent (поддерживает nss-mdns)
        local mdns_ip
        mdns_ip=$(getent ahosts raspberrypi.local 2>/dev/null \
            | awk '/STREAM/ {print $1; exit}' || true)

        if [[ -n "$mdns_ip" && "$mdns_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            PI_IP="$mdns_ip"
            log_ok "Pi найден через mDNS: ${BOLD}$PI_IP${NC} (raspberrypi.local)"
        else
            # Пробуем ping
            if ping -c 1 -W 2 raspberrypi.local &>/dev/null; then
                PI_IP=$(ping -c 1 -W 2 raspberrypi.local 2>/dev/null \
                    | grep -oP '\(\K[0-9.]+' | head -1 || true)
                if [[ -n "$PI_IP" ]]; then
                    log_ok "Pi найден через ping: ${BOLD}$PI_IP${NC}"
                fi
            fi
        fi

        if [[ -z "$PI_IP" ]]; then
            log_warn "Pi не найден автоматически."
            echo ""
            read -rp "  Введи IP Raspberry Pi (например 192.168.1.100): " PI_IP
            if [[ ! "$PI_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                die "Неверный IP: '$PI_IP'"
            fi
        fi
    fi

    # Проверка доступности
    log_info "Проверка связи с Pi ($PI_IP)..."
    if ping -c 1 -W 3 "$PI_IP" &>/dev/null; then
        log_ok "Pi доступен: ${BOLD}$PI_IP${NC}"
    else
        log_warn "Pi не отвечает на ping ($PI_IP)"
        log_warn "Убедись что Pi включён и в той же сети"
    fi

    # Проверка MQTT порта
    if command -v nc &>/dev/null; then
        if nc -z -w 2 "$PI_IP" 1883 &>/dev/null; then
            log_ok "MQTT брокер доступен ($PI_IP:1883)"
        else
            log_warn "MQTT порт 1883 не отвечает — убедись что mosquitto запущен на Pi"
        fi
    fi

    MQTT_BROKER="$PI_IP"
}

# ─── 6. Настройка DDS (unicast для хотспота) ─────────────────────────────────
configure_network() {
    log_step "Настройка сети"

    PEER_IP=""

    if $HOTSPOT_MODE; then
        PEER_IP="$PI_IP"
        log_ok "Хотспот: unicast DDS → peer_ip=${BOLD}$PEER_IP${NC}"
    else
        log_ok "LAN: multicast DDS (стандартный режим)"
    fi

    log_ok "MQTT broker: ${BOLD}$MQTT_BROKER:1883${NC}"
}

# ─── 7. Запуск ───────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск compute нод в Docker"

    # Остановить старый контейнер если есть
    if docker ps -q --filter "name=$CONTAINER_NAME" 2>/dev/null | grep -q .; then
        log_warn "Останавливаю предыдущий контейнер $CONTAINER_NAME..."
        docker stop "$CONTAINER_NAME" &>/dev/null || true
    fi

    # Формируем аргументы launch
    local launch_args="mqtt_broker:=$MQTT_BROKER"
    [[ -n "$PEER_IP" ]] && launch_args="$launch_args peer_ip:=$PEER_IP"

    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "127.0.0.1")

    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Dashboard:     ${CYAN}http://localhost:5000${NC}  ${BOLD}│${NC}"
    echo -e "  ${BOLD}│${NC}  Android:       ${CYAN}http://${my_ip}:5000${NC}"
    echo -e "  ${BOLD}│${NC}  MQTT Broker:   ${GREEN}${MQTT_BROKER}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  ROS_DOMAIN_ID: ${GREEN}${ROS_DOMAIN_ID}${NC}"
    [[ -n "$PEER_IP" ]] && \
    echo -e "  ${BOLD}│${NC}  Unicast DDS:   ${GREEN}peer_ip=$PEER_IP${NC}"
    echo -e "  ${BOLD}└──────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Запуск ROS2 нод (Ctrl+C для остановки) ──${NC}"
    echo ""

    # shellcheck disable=SC2086
    docker run --rm \
        --name "$CONTAINER_NAME" \
        --net=host \
        --privileged \
        -v "$SCRIPT_DIR:/root/Samurai" \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        -e DISPLAY="${DISPLAY:-:0}" \
        "$DOCKER_IMAGE" \
        bash -c "
            source /opt/ros/humble/setup.bash
            cd /root/Samurai/ros_ws
            source install/setup.bash
            exec ros2 launch robot_pkg compute_bringup.launch.py $launch_args
        "
}

# ─── Main ────────────────────────────────────────────────────────────────────
main() {
    print_banner
    check_docker
    check_image
    check_workspace
    check_avahi
    discover_pi
    configure_network
    launch
}

main
