#!/usr/bin/env bash
# =============================================================================
# start_laptop_robot.sh — Запуск ноутбука для управления РЕАЛЬНЫМ роботом
# Платформа: Arch Linux | Метод: Docker (ROS2 Humble + SLAM + Nav2 + YOLO)
#
# Использование:
#   chmod +x start_laptop_robot.sh
#   ./start_laptop_robot.sh
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

# ── Баннер ───────────────────────────────────────────────────────────────────
print_banner() {
    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║       Ноутбук → Реальный робот               ║"
    echo "  ║  ROS2 Humble | SLAM | Nav2 | YOLO | Docker   ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
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
        log_warn "Docker daemon не запущен. Пробую запустить..."
        if sudo systemctl start docker 2>/dev/null; then
            sleep 2
            docker info &>/dev/null 2>&1 || die "Не удалось запустить Docker. Попробуй: sudo systemctl start docker"
            log_ok "Docker daemon запущен"
        else
            die "Docker daemon не запускается. Проверь: sudo journalctl -u docker"
        fi
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
    else
        log_ok "Образ '$DOCKER_IMAGE' существует"

        # Проверяем: Dockerfile новее образа?
        local df_mtime img_created
        df_mtime=$(stat -c %Y "$SCRIPT_DIR/Dockerfile" 2>/dev/null || echo 0)
        img_created=$(docker inspect --format='{{.Created}}' "$DOCKER_IMAGE" 2>/dev/null \
            | xargs -I{} date -d "{}" +%s 2>/dev/null || echo 0)

        if [[ "$df_mtime" -gt "$img_created" ]] 2>/dev/null; then
            log_warn "Dockerfile изменился после сборки образа."
            read -rp "    Пересобрать образ? [y/N]: " ans
            [[ "$ans" =~ ^[Yy]$ ]] && need_build=true
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
    else
        log_ok "Workspace собран (ros_ws/install найден)"

        # Есть ли файлы новее install/?
        local stale
        stale=$(find "$ws/src" -name "*.py" -newer "$install_dir" 2>/dev/null | head -1)
        if [[ -n "$stale" ]]; then
            log_warn "Найдены изменения в исходниках ($(basename "$stale") и др.)"
            read -rp "    Пересобрать workspace? [y/N]: " ans
            [[ "$ans" =~ ^[Yy]$ ]] && build_ws
        fi
    fi
}

# ─── 4. mDNS (avahi) ─────────────────────────────────────────────────────────
check_avahi() {
    log_step "mDNS — raspberrypi.local"

    if ! command -v avahi-browse &>/dev/null; then
        log_warn "avahi не установлен — raspberrypi.local не будет работать."
        echo ""
        echo -e "    Установка:"
        echo -e "      ${YELLOW}sudo pacman -S avahi nss-mdns${NC}"
        echo -e "      ${YELLOW}sudo systemctl enable --now avahi-daemon${NC}"
        echo ""
        echo -e "    Настройка /etc/nsswitch.conf (строка hosts):"
        echo -e "      ${YELLOW}hosts: mymachines mdns_minimal [NOTFOUND=return] resolve [!UNAVAIL=return] files myhostname dns${NC}"
        echo ""
        read -rp "    Продолжить без mDNS? [Y/n]: " ans
        [[ "$ans" =~ ^[Nn]$ ]] && exit 0
        return
    fi

    if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
        log_warn "avahi-daemon не запущен"
        read -rp "    Запустить avahi-daemon? [Y/n]: " ans
        if [[ ! "$ans" =~ ^[Nn]$ ]]; then
            sudo systemctl start avahi-daemon
            log_ok "avahi-daemon запущен"
        fi
    else
        log_ok "avahi-daemon активен (raspberrypi.local работает)"
    fi
}

# ─── 5. Настройка сети ───────────────────────────────────────────────────────
configure_network() {
    log_step "Настройка сети"

    echo ""
    echo -e "  ${BOLD}Выбери режим сети:${NC}"
    echo -e "  ${GREEN}1)${NC} LAN / домашний WiFi  — multicast DDS (по умолчанию)"
    echo -e "  ${GREEN}2)${NC} Мобильный хотспот    — unicast DDS (нужен IP Raspberry Pi)"
    echo ""
    read -rp "  Режим [1/2]: " net_mode

    PEER_IP=""
    ROBOT_HOST="raspberrypi.local"

    if [[ "$net_mode" == "2" ]]; then
        echo ""
        log_info "Как узнать IP Pi: подключись по SSH и выполни: hostname -I"
        read -rp "  IP Raspberry Pi в сети хотспота (напр. 192.168.43.100): " PEER_IP

        if [[ ! "$PEER_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            die "Неверный IP: '$PEER_IP'"
        fi
        ROBOT_HOST="$PEER_IP"
        log_ok "Unicast DDS: peer_ip=$PEER_IP"
    else
        log_ok "Multicast DDS (стандартный режим)"
    fi

    # Проверка доступности робота
    echo ""
    log_info "Проверка доступности $ROBOT_HOST ..."
    if ping -c 1 -W 3 "$ROBOT_HOST" &>/dev/null 2>&1; then
        log_ok "Робот доступен ($ROBOT_HOST)"
    else
        log_warn "Робот недоступен по адресу $ROBOT_HOST"
        log_warn "Убедись что Raspberry Pi включён и в той же сети"
        read -rp "  Продолжить всё равно? [y/N]: " ans
        [[ "$ans" =~ ^[Yy]$ ]] || exit 0
    fi
}

# ─── 6. Запуск ───────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск compute node"

    # Остановить старый контейнер если есть
    if docker ps -q --filter "name=$CONTAINER_NAME" 2>/dev/null | grep -q .; then
        log_warn "Останавливаю предыдущий контейнер $CONTAINER_NAME..."
        docker stop "$CONTAINER_NAME" &>/dev/null || true
    fi

    local launch_args=""
    [[ -n "$PEER_IP" ]] && launch_args="peer_ip:=$PEER_IP"

    echo ""
    log_ok "Dashboard:   http://localhost:5000"
    log_ok "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    [[ -n "$PEER_IP" ]] && log_ok "Unicast DDS: peer_ip=$PEER_IP"
    echo ""
    echo -e "${YELLOW}  ── Запуск ROS2 ноды (Ctrl+C для остановки) ──${NC}"
    echo ""

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
    configure_network
    launch
}

main
