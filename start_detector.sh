#!/usr/bin/env bash
# =============================================================================
# start_detector.sh — Запуск детектора объектов на ноутбуке (без ROS2/Docker)
#
# Подключается к MQTT брокеру Raspberry Pi, получает кадры с камеры,
# запускает YOLO (или HSV blob fallback), публикует детекции обратно на Pi.
#
# Использование:
#   chmod +x start_detector.sh
#   ./start_detector.sh                       # авто-поиск Pi через mDNS
#   ./start_detector.sh --pi 192.168.1.50     # указать IP вручную
#   ./start_detector.sh --pi raspberrypi.local
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
PI_IP_ARG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pi) PI_IP_ARG="$2"; shift 2 ;;
        -h|--help)
            echo "Использование: $0 [--pi IP_или_hostname]"
            echo "  --pi ADDR   IP или hostname Raspberry Pi (например raspberrypi.local)"
            exit 0 ;;
        *) die "Неизвестный аргумент: $1" ;;
    esac
done

# ── Баннер ────────────────────────────────────────────────────────────────
echo -e "${CYAN}${BOLD}"
echo "  ╔═══════════════════════════════════════════════╗"
echo "  ║          S A M U R A I   R O B O T           ║"
echo "  ║       Object Detector  |  Ноутбук            ║"
echo "  ║  YOLO + HSV + Дистанция + Карта  |  MQTT     ║"
echo "  ╚═══════════════════════════════════════════════╝"
echo -e "${NC}"

# ── 1. Python ──────────────────────────────────────────────────────────────
log_step "Python"
python3 --version &>/dev/null || die "Python3 не найден"
ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
log_ok "Python $ver"

# ── 2. Зависимости ─────────────────────────────────────────────────────────
log_step "Зависимости"
missing=()
python3 -c "import paho.mqtt.client" 2>/dev/null || missing+=(paho-mqtt)
python3 -c "import cv2"              2>/dev/null || missing+=(opencv-python)
python3 -c "import numpy"            2>/dev/null || missing+=(numpy)

if [[ ${#missing[@]} -gt 0 ]]; then
    log_warn "Отсутствуют: ${missing[*]}"
    log_info "Устанавливаю..."
    pip3 install --quiet "${missing[@]}" || die "pip install провалился"
    log_ok "Зависимости установлены"
else
    log_ok "paho-mqtt, opencv, numpy — OK"
fi

# YOLO (опционально — детектор работает и без него через HSV blob)
if python3 -c "import ultralytics" 2>/dev/null; then
    log_ok "ultralytics (YOLO) — OK"
else
    log_warn "ultralytics не установлен — используется HSV blob детекция"
    log_warn "Установить YOLO: pip3 install ultralytics"
fi

# ── 3. Поиск Pi ────────────────────────────────────────────────────────────
log_step "Raspberry Pi"
if [[ -n "$PI_IP_ARG" ]]; then
    PI_ADDR="$PI_IP_ARG"
    log_ok "Адрес Pi (из аргументов): ${BOLD}$PI_ADDR${NC}"
else
    log_info "Поиск raspberrypi.local (mDNS)..."
    PI_ADDR=""

    if mdns_ip=$(getent ahosts raspberrypi.local 2>/dev/null | awk '/STREAM/ {print $1; exit}') \
       && [[ -n "$mdns_ip" ]]; then
        PI_ADDR="$mdns_ip"
        log_ok "Pi найден: ${BOLD}$PI_ADDR${NC} (raspberrypi.local)"
    elif ping -c 1 -W 2 raspberrypi.local &>/dev/null 2>&1; then
        PI_ADDR=$(ping -c 1 -W 2 raspberrypi.local 2>/dev/null \
            | grep -oP '\(\K[0-9.]+' | head -1 || true)
        [[ -n "$PI_ADDR" ]] && log_ok "Pi найден через ping: ${BOLD}$PI_ADDR${NC}"
    fi

    if [[ -z "$PI_ADDR" ]]; then
        log_warn "Pi не найден автоматически."
        read -rp "  Введи IP или hostname Raspberry Pi: " PI_ADDR
        [[ -z "$PI_ADDR" ]] && die "Адрес не указан"
    fi
fi

# Проверка MQTT
if command -v nc &>/dev/null; then
    if nc -z -w 2 "$PI_ADDR" 1883 &>/dev/null 2>&1; then
        log_ok "MQTT брокер доступен ($PI_ADDR:1883)"
    else
        log_warn "Порт 1883 не отвечает — убедись что mosquitto запущен на Pi"
    fi
fi

# ── 4. Запуск ──────────────────────────────────────────────────────────────
log_step "Запуск детектора"
echo ""
echo -e "  ${BOLD}┌──────────────────────────────────────────┐${NC}"
echo -e "  ${BOLD}│${NC}  Pi MQTT: ${CYAN}${PI_ADDR}:1883${NC}"
echo -e "  ${BOLD}│${NC}  Топики:  ${GREEN}samurai/robot1/camera${NC} → детекция"
echo -e "  ${BOLD}│${NC}           ${GREEN}samurai/robot1/detections${NC} ← результат"
echo -e "  ${BOLD}│${NC}           ${GREEN}samurai/robot1/ball_detection${NC} ← мяч (FSM)"
echo -e "  ${BOLD}│${NC}           ${GREEN}samurai/robot1/detected_frame${NC} ← аннотация"
echo -e "  ${BOLD}└──────────────────────────────────────────┘${NC}"
echo ""
echo -e "${YELLOW}  ── Детектор запущен (Ctrl+C для остановки) ──${NC}"
echo ""

cd "$SCRIPT_DIR"
exec python3 compute_node/object_detector_node.py \
    --broker "$PI_ADDR" \
    --port 1883 \
    --robot-id robot1
