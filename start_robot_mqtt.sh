#!/usr/bin/env bash
# =============================================================================
# start_robot_mqtt.sh — Запуск Samurai Robot (Pure Python + MQTT, без ROS2)
# Платформа: Raspberry Pi 4 | Debian Trixie (arm64)
#
# Использование:
#   chmod +x start_robot_mqtt.sh
#   ./start_robot_mqtt.sh
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log_ok()   { echo -e "${GREEN}  [✓]${NC} $*"; }
log_warn() { echo -e "${YELLOW}  [!]${NC} $*"; }
log_err()  { echo -e "${RED}  [✗]${NC} $*"; }
log_info() { echo -e "${BLUE}  [→]${NC} $*"; }
log_step() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }
die()      { log_err "$*"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Баннер ────────────────────────────────────────────────────────────────
print_banner() {
    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "?")

    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║       Raspberry Pi 4  |  MQTT Mode           ║"
    echo "  ║  Python + paho-mqtt  |  No ROS2 / Docker     ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  Hostname: ${BOLD}$(hostname)${NC}  IP: ${BOLD}${my_ip}${NC}"
    echo ""
}

# ── 1. Python ──────────────────────────────────────────────────────────────
check_python() {
    log_step "Python"

    if ! command -v python3 &>/dev/null; then
        die "Python3 не найден. Установи: sudo apt install python3 python3-pip"
    fi

    local ver
    ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    log_ok "Python $ver"
}

# ── 2. pip зависимости ─────────────────────────────────────────────────────
check_deps() {
    log_step "Python зависимости"

    local missing=()
    python3 -c "import paho.mqtt.client" 2>/dev/null || missing+=(paho-mqtt)
    python3 -c "import yaml"             2>/dev/null || missing+=(PyYAML)
    python3 -c "import smbus2"           2>/dev/null || missing+=(smbus2)
    python3 -c "import gpiozero"         2>/dev/null || missing+=(gpiozero)

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "Отсутствуют: ${missing[*]}"
        log_info "Устанавливаю..."
        pip3 install --quiet --break-system-packages "${missing[@]}" \
            || pip3 install --quiet "${missing[@]}" \
            || die "pip install провалился. Попробуй: sudo pip3 install ${missing[*]}"
        log_ok "Зависимости установлены"
    else
        log_ok "Все базовые зависимости на месте"
    fi

    # Опциональные (не блокируют запуск)
    python3 -c "import adafruit_pca9685" 2>/dev/null \
        && log_ok "adafruit-pca9685 ✓" \
        || log_warn "adafruit-pca9685 не установлен (моторы в режиме симуляции)"

    python3 -c "import picamera2" 2>/dev/null \
        && log_ok "picamera2 ✓" \
        || log_warn "picamera2 не установлен (камера отключена)"
}

# ── 3. I2C ─────────────────────────────────────────────────────────────────
check_i2c() {
    log_step "Интерфейс I2C"

    if [[ ! -e /dev/i2c-1 ]]; then
        log_warn "/dev/i2c-1 не найден. Пробую включить I2C..."
        if command -v raspi-config &>/dev/null; then
            sudo raspi-config nonint do_i2c 0 2>/dev/null || true
        fi
        sudo modprobe i2c-dev 2>/dev/null || true
        [[ -e /dev/i2c-1 ]] \
            && log_ok "I2C включён" \
            || log_warn "/dev/i2c-1 недоступен (работа без моторов/IMU)"
        return
    fi

    log_ok "I2C доступен (/dev/i2c-1)"

    if command -v i2cdetect &>/dev/null; then
        log_info "I2C устройства:"
        i2cdetect -y 1 2>/dev/null | while IFS= read -r line; do
            echo "    $line"
        done
    fi
}

# ── 4. MQTT (Mosquitto) ───────────────────────────────────────────────────
check_mqtt() {
    log_step "MQTT брокер (mosquitto)"

    if ! command -v mosquitto &>/dev/null; then
        log_warn "mosquitto не установлен."
        echo -e "    Установка: ${YELLOW}sudo apt install -y mosquitto mosquitto-clients${NC}"
        echo -e "    Запуск:    ${YELLOW}sudo systemctl enable --now mosquitto${NC}"
        die "Установи mosquitto и перезапусти скрипт"
    fi

    # Mosquitto 2.0+ blocks remote connections by default.
    # Ensure a listener config exists so laptop/Android can connect.
    local SAMURAI_CONF="/etc/mosquitto/conf.d/samurai.conf"
    local NEED_RESTART=false
    # Always regenerate config to keep limits up-to-date
    local DESIRED_CFG
    read -r -d '' DESIRED_CFG <<'MQTTCFG' || true
# Samurai Robot — allow LAN connections
listener 1883
allow_anonymous true

# ── Anti-freeze limits ──
# Max inflight QoS 1/2 messages per client (default=20, keep low on Pi)
max_inflight_messages 10
# Max queued QoS 1/2 messages per client (prevent RAM bloat)
max_queued_messages 50
# Max message size: 300 KB (enough for 640x480 JPEG, blocks oversized payloads)
message_size_limit 300000
# Memory limit — prevent Mosquitto from eating all Pi RAM
memory_limit 50000000
MQTTCFG
    if [[ ! -f "$SAMURAI_CONF" ]] || ! diff -q <(echo "$DESIRED_CFG") "$SAMURAI_CONF" &>/dev/null; then
        log_warn "Конфиг mosquitto обновляю (anti-freeze лимиты)..."
        echo "$DESIRED_CFG" | sudo tee "$SAMURAI_CONF" >/dev/null
        log_ok "Создан $SAMURAI_CONF (listener 1883, limits)"
        NEED_RESTART=true
    fi
    if $NEED_RESTART; then
        sudo systemctl restart mosquitto 2>/dev/null || true
    fi

    if ! systemctl is-active --quiet mosquitto 2>/dev/null; then
        log_warn "mosquitto не запущен. Запускаю..."
        sudo systemctl start mosquitto 2>/dev/null \
            && log_ok "mosquitto запущен" \
            || die "Не удалось запустить mosquitto"
    else
        log_ok "mosquitto активен (порт 1883)"
    fi
}

# ── 5. avahi ───────────────────────────────────────────────────────────────
check_avahi() {
    log_step "mDNS (avahi)"

    if command -v avahi-daemon &>/dev/null; then
        systemctl is-active --quiet avahi-daemon 2>/dev/null || {
            sudo systemctl start avahi-daemon 2>/dev/null || true
        }
        systemctl is-active --quiet avahi-daemon 2>/dev/null \
            && log_ok "avahi активен — Pi: ${BOLD}$(hostname).local${NC}" \
            || log_warn "avahi не запущен"
    else
        log_warn "avahi не установлен (sudo apt install avahi-daemon)"
    fi
}

# ── 6. Запуск ──────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск MQTT нод"

    local my_ip
    my_ip=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "127.0.0.1")

    echo ""
    log_ok "Broker:   ${BOLD}${my_ip}:1883${NC}"
    log_ok "Robot ID: ${BOLD}robot1${NC}"
    echo ""
    echo -e "${YELLOW}  ── Запуск Python нод (Ctrl+C для остановки) ──${NC}"
    echo ""

    cd "$SCRIPT_DIR"
    exec python3 -m pi_nodes.robot_launcher \
        --broker "$my_ip" \
        --port 1883 \
        --robot-id robot1
}

# ── Main ───────────────────────────────────────────────────────────────────
main() {
    print_banner
    check_python
    check_deps
    check_i2c
    check_mqtt
    check_avahi
    launch
}

main
