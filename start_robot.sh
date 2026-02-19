#!/usr/bin/env bash
# =============================================================================
# start_robot.sh — Запуск всех нод на Raspberry Pi (реальный робот)
# Платформа: Raspberry Pi OS (Debian Bookworm/Trixie) + ROS2 Humble
#
# Использование (скопируй в корень проекта на Pi):
#   chmod +x start_robot.sh
#   ./start_robot.sh
#
# Первый запуск (установка зависимостей):
#   sudo bash Diagnostic/setup_robot.sh
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
ROS_DOMAIN_ID=42
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/ros_ws/install/setup.bash"
VOSK_MODEL_DIR="${HOME}/vosk-model-ru"

# ── Баннер ───────────────────────────────────────────────────────────────────
print_banner() {
    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "?")

    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║          Запуск Raspberry Pi                 ║"
    echo "  ║    Сенсоры | Моторы | MQTT | FSM | Камера    ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  Hostname: ${BOLD}$(hostname)${NC}  IP: ${BOLD}${my_ip}${NC}"
    echo ""
}

# ─── 1. ROS2 Humble ──────────────────────────────────────────────────────────
check_ros2() {
    log_step "Проверка ROS2 Humble"

    if [[ ! -f "$ROS_SETUP" ]]; then
        log_err "ROS2 Humble не найден ($ROS_SETUP)"
        echo ""
        echo "  Установка ROS2 Humble на Raspberry Pi OS:"
        echo -e "  ${YELLOW}sudo apt update && sudo apt install -y software-properties-common${NC}"
        echo -e "  ${YELLOW}sudo add-apt-repository universe${NC}"
        echo -e "  ${YELLOW}sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\"
        echo -e "       -o /usr/share/keyrings/ros-archive-keyring.gpg${NC}"
        echo -e '  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]'
        echo -e '       https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"'"
        echo -e '  | sudo tee /etc/apt/sources.list.d/ros2.list'"
        echo -e "  ${YELLOW}sudo apt update && sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions${NC}"
        die "Установи ROS2 Humble и перезапусти скрипт"
    fi

    # Источник ROS2 в текущем процессе
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    log_ok "ROS2 Humble sourced ($(ros2 --version 2>/dev/null | head -1 || echo 'humble'))"
}

# ─── 2. ROS2 Workspace ───────────────────────────────────────────────────────
check_workspace() {
    log_step "ROS2 Workspace (ros_ws)"

    local ws="$SCRIPT_DIR/ros_ws"

    if ! command -v colcon &>/dev/null; then
        log_warn "colcon не найден"
        echo -e "    Установи: ${YELLOW}sudo apt install -y python3-colcon-common-extensions${NC}"
        die "colcon требуется для сборки"
    fi

    build_ws() {
        log_info "Сборка workspace (colcon build)..."
        (
            source "$ROS_SETUP"
            cd "$ws"
            colcon build --symlink-install \
                --cmake-args -DCMAKE_BUILD_TYPE=Release \
                --event-handlers console_cohesion+
        ) || die "colcon build провалился"
        log_ok "Workspace собран"
    }

    if [[ ! -d "$ws/install" ]]; then
        log_warn "ros_ws/install не найден — workspace не собран"
        build_ws
    else
        log_ok "Workspace собран"

        local stale
        stale=$(find "$ws/src" -name "*.py" -newer "$ws/install" 2>/dev/null | head -1 || true)
        if [[ -n "$stale" ]]; then
            log_warn "Найдены изменения в исходниках ($(basename "$stale") и др.)"
            read -rp "    Пересобрать workspace? [y/N]: " ans
            [[ "$ans" =~ ^[Yy]$ ]] && build_ws
        fi
    fi

    # shellcheck disable=SC1090
    source "$WS_SETUP"
    log_ok "Workspace sourced"
}

# ─── 3. Железо: I2C ──────────────────────────────────────────────────────────
check_i2c() {
    log_step "Интерфейс I2C"

    if ! command -v i2cdetect &>/dev/null; then
        log_warn "i2c-tools не найден"
        echo -e "    Установи: ${YELLOW}sudo apt install -y i2c-tools${NC}"
    fi

    if [[ ! -e /dev/i2c-1 ]]; then
        log_warn "/dev/i2c-1 не найден. Пробую включить I2C..."
        if command -v raspi-config &>/dev/null; then
            sudo raspi-config nonint do_i2c 0 2>/dev/null \
                && log_warn "I2C включён. Может потребоваться перезагрузка."
        fi

        if [[ ! -e /dev/i2c-1 ]]; then
            log_warn "I2C всё ещё недоступен — продолжаю (без IMU/HAT)"
            return
        fi
    fi

    log_ok "I2C доступен (/dev/i2c-1)"

    # Сканируем шину (тихо, просто для инфо)
    if command -v i2cdetect &>/dev/null; then
        log_info "I2C устройства на шине:"
        i2cdetect -y 1 2>/dev/null | while read -r line; do
            echo "    $line"
        done
    fi
}

# ─── 4. Камера ───────────────────────────────────────────────────────────────
check_camera() {
    log_step "Камера"

    # libcamera
    if command -v libcamera-hello &>/dev/null; then
        if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
            log_ok "libcamera: камера найдена"
        else
            log_warn "libcamera установлена, но камера не обнаружена"
            log_warn "Проверь подключение ленточного кабеля CSI"
        fi
    # vcgencmd (старые Pi OS)
    elif command -v vcgencmd &>/dev/null; then
        local cam_status
        cam_status=$(vcgencmd get_camera 2>/dev/null || echo "unsupported")
        if echo "$cam_status" | grep -q "detected=1"; then
            log_ok "Камера обнаружена (vcgencmd)"
        else
            log_warn "Камера не обнаружена: $cam_status"
        fi
    else
        log_warn "Инструменты проверки камеры не найдены (пропуск проверки)"
    fi
}

# ─── 5. Python пакеты (аппаратные) ───────────────────────────────────────────
check_python_deps() {
    log_step "Python зависимости"

    # Формат: "import_name:описание"
    local pkgs=(
        "gpiozero:gpiozero (GPIO управление)"
        "smbus2:smbus2 (I2C/MPU6050)"
        "adafruit_pca9685:adafruit-pca9685 (HAT ШИМ)"
        "adafruit_motor:adafruit-motor (моторы)"
        "vosk:vosk (голосовое распознавание)"
        "cv2:opencv-python (камера)"
        "paho.mqtt.client:paho-mqtt (MQTT)"
    )

    local failed=()
    for entry in "${pkgs[@]}"; do
        local import_name="${entry%%:*}"
        local desc="${entry##*:}"
        if python3 -c "import ${import_name}" &>/dev/null 2>&1; then
            log_ok "$desc"
        else
            log_warn "$desc — НЕ НАЙДЕН"
            failed+=("$desc")
        fi
    done

    if [[ ${#failed[@]} -gt 0 ]]; then
        echo ""
        log_warn "Отсутствующие зависимости: ${#failed[@]} пакет(ов)"
        echo -e "    Запусти: ${YELLOW}sudo bash $SCRIPT_DIR/Diagnostic/setup_robot.sh${NC}"
        read -rp "  Продолжить с отсутствующими пакетами? [y/N]: " ans
        [[ "$ans" =~ ^[Yy]$ ]] || die "Установи зависимости и перезапусти"
    else
        log_ok "Все Python пакеты установлены"
    fi
}

# ─── 6. Vosk модель ──────────────────────────────────────────────────────────
check_vosk_model() {
    log_step "Vosk Russian модель"

    if [[ -d "$VOSK_MODEL_DIR" ]]; then
        log_ok "Модель найдена: $VOSK_MODEL_DIR"
    else
        log_warn "Vosk модель не найдена: $VOSK_MODEL_DIR"
        echo ""
        read -rp "  Скачать сейчас (~50 МБ)? [Y/n]: " ans
        if [[ ! "$ans" =~ ^[Nn]$ ]]; then
            local tmp_zip="/tmp/vosk-model-ru.zip"
            local url="https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip"
            log_info "Загрузка модели..."
            wget -q --show-progress "$url" -O "$tmp_zip" \
                || die "Ошибка загрузки. Проверь интернет-соединение."
            log_info "Распаковка..."
            unzip -q "$tmp_zip" -d "$HOME/" \
                && mv "$HOME/vosk-model-small-ru-0.22" "$VOSK_MODEL_DIR" \
                && rm -f "$tmp_zip"
            log_ok "Vosk модель установлена: $VOSK_MODEL_DIR"
        else
            log_warn "Голосовое управление будет недоступно"
        fi
    fi
}

# ─── 7. Системные службы ─────────────────────────────────────────────────────
check_services() {
    log_step "Системные службы"

    # pigpiod (нужен для HC-SR04 на Bookworm)
    if command -v pigpiod &>/dev/null; then
        if ! pgrep pigpiod &>/dev/null; then
            log_warn "pigpiod не запущен. Запускаю..."
            sudo pigpiod 2>/dev/null || sudo systemctl start pigpiod 2>/dev/null || true
            sleep 1
            if pgrep pigpiod &>/dev/null; then
                log_ok "pigpiod запущен"
            else
                log_warn "pigpiod не запустился (ультразвук может не работать)"
            fi
        else
            log_ok "pigpiod активен"
        fi
    else
        log_info "pigpiod не найден (Trixie/lgpio — ОК)"
    fi

    # avahi-daemon (mDNS — чтобы ноутбук находил Pi по raspberrypi.local)
    if command -v avahi-daemon &>/dev/null; then
        if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
            log_warn "avahi-daemon не запущен. Запускаю..."
            sudo systemctl start avahi-daemon 2>/dev/null || true
            systemctl is-active --quiet avahi-daemon 2>/dev/null \
                && log_ok "avahi-daemon запущен (Pi доступен по: ${BOLD}$(hostname).local${NC})" \
                || log_warn "avahi-daemon не запустился"
        else
            log_ok "avahi-daemon активен (Pi доступен по: ${BOLD}$(hostname).local${NC})"
        fi
    else
        log_warn "avahi-daemon не найден"
        echo -e "    Установи: ${YELLOW}sudo apt install -y avahi-daemon${NC}"
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
        log_info "IP ноутбука: подключись к нему и выполни: ip route get 1 | awk '{print \$7}'"
        read -rp "  IP ноутбука в сети хотспота (напр. 192.168.43.50): " PEER_IP

        if [[ ! "$PEER_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            die "Неверный IP: '$PEER_IP'"
        fi
        MQTT_BROKER="$my_ip"
        log_ok "Unicast DDS: peer_ip=$PEER_IP"
        log_ok "MQTT broker: $MQTT_BROKER"
    else
        log_ok "Multicast DDS (стандартный режим)"
        log_ok "MQTT broker: $MQTT_BROKER"
    fi
}

# ─── 9. Запуск ───────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск robot_bringup"

    local launch_args="mqtt_broker:=$MQTT_BROKER"
    [[ -n "$PEER_IP" ]] && launch_args="$launch_args peer_ip:=$PEER_IP"

    # Путь к vosk модели
    if [[ -d "$VOSK_MODEL_DIR" ]]; then
        launch_args="$launch_args vosk_model:=$VOSK_MODEL_DIR"
    fi

    echo ""
    log_ok "ROS_DOMAIN_ID:   $ROS_DOMAIN_ID"
    log_ok "MQTT broker:     $MQTT_BROKER:1883"
    log_ok "Vosk model:      $VOSK_MODEL_DIR"
    [[ -n "$PEER_IP" ]] && log_ok "Unicast DDS:     peer_ip=$PEER_IP"
    echo ""
    echo -e "${YELLOW}  ── Запуск ROS2 нод (Ctrl+C для остановки) ──${NC}"
    echo ""

    export ROS_DOMAIN_ID
    exec ros2 launch robot_pkg robot_bringup.launch.py $launch_args
}

# ─── Main ────────────────────────────────────────────────────────────────────
main() {
    print_banner
    check_ros2
    check_workspace
    check_i2c
    check_camera
    check_python_deps
    check_vosk_model
    check_services
    configure_network
    launch
}

main
