#!/usr/bin/env bash
# =============================================================================
# start_laptop_sim.sh — Запуск симулятора на ноутбуке (без ROS2 и робота)
# Платформа: Arch Linux | Метод: чистый Python (Flask + OpenCV)
#
# Использование:
#   chmod +x start_laptop_sim.sh
#   ./start_laptop_sim.sh
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
SIM_SCRIPT="$SCRIPT_DIR/compute_node/simulator.py"

# Пакеты необходимые для симулятора
# Формат: "import_name:pip_name"
REQUIRED_PKGS=(
    "flask:flask"
    "flask_cors:flask-cors"
    "flask_socketio:flask-socketio"
    "cv2:opencv-python"
    "numpy:numpy"
    "paho.mqtt.client:paho-mqtt"
)

# ── Баннер ───────────────────────────────────────────────────────────────────
print_banner() {
    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║          S A M U R A I   R O B O T           ║"
    echo "  ║           Режим симуляции (Python)           ║"
    echo "  ║    Flask + OpenCV | Без ROS2 и железа        ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ─── 1. Python3 ──────────────────────────────────────────────────────────────
check_python() {
    log_step "Проверка Python"

    if ! command -v python3 &>/dev/null; then
        die "python3 не найден. Установи: sudo pacman -S python"
    fi

    local py_ver
    py_ver=$(python3 --version 2>&1 | grep -oP '\d+\.\d+')
    local major minor
    major=$(echo "$py_ver" | cut -d. -f1)
    minor=$(echo "$py_ver" | cut -d. -f2)

    if [[ "$major" -lt 3 ]] || { [[ "$major" -eq 3 ]] && [[ "$minor" -lt 9 ]]; }; then
        die "Требуется Python 3.9+. Установлен: $(python3 --version)"
    fi
    log_ok "Python $(python3 --version | cut -d' ' -f2)"

    if ! command -v pip3 &>/dev/null && ! python3 -m pip --version &>/dev/null 2>&1; then
        die "pip не найден. Установи: sudo pacman -S python-pip"
    fi
    log_ok "pip доступен"
}

# ─── 2. Python пакеты ────────────────────────────────────────────────────────
check_packages() {
    log_step "Проверка Python зависимостей"

    local missing=()

    for entry in "${REQUIRED_PKGS[@]}"; do
        local import_name="${entry%%:*}"
        local pip_name="${entry##*:}"

        if python3 -c "import ${import_name}" &>/dev/null 2>&1; then
            log_ok "$pip_name"
        else
            log_warn "$pip_name — не установлен"
            missing+=("$pip_name")
        fi
    done

    if [[ ${#missing[@]} -eq 0 ]]; then
        log_ok "Все зависимости установлены"
        return
    fi

    echo ""
    log_warn "Отсутствующие пакеты: ${missing[*]}"
    read -rp "  Установить автоматически? [Y/n]: " ans

    if [[ "$ans" =~ ^[Nn]$ ]]; then
        echo ""
        echo -e "  Установи вручную:"
        echo -e "  ${YELLOW}pip install ${missing[*]}${NC}"
        die "Установи зависимости и перезапусти скрипт"
    fi

    log_info "Устанавливаю: ${missing[*]}"
    python3 -m pip install --user --break-system-packages "${missing[@]}" \
        || die "Ошибка установки пакетов"

    # Повторная проверка
    local still_missing=()
    for entry in "${REQUIRED_PKGS[@]}"; do
        local import_name="${entry%%:*}"
        local pip_name="${entry##*:}"
        if ! python3 -c "import ${import_name}" &>/dev/null 2>&1; then
            still_missing+=("$pip_name")
        fi
    done

    if [[ ${#still_missing[@]} -gt 0 ]]; then
        die "Пакеты всё ещё не доступны: ${still_missing[*]}"
    fi
    log_ok "Все зависимости установлены"
}

# ─── 3. Проверка файла симулятора ────────────────────────────────────────────
check_simulator_file() {
    log_step "Файл симулятора"

    if [[ ! -f "$SIM_SCRIPT" ]]; then
        die "Файл не найден: $SIM_SCRIPT"
    fi
    log_ok "compute_node/simulator.py найден"
}

# ─── 4. Проверка порта 5000 ──────────────────────────────────────────────────
check_port() {
    log_step "Проверка порта 5000"

    if ss -tlnp 2>/dev/null | grep -q ':5000 ' || \
       netstat -tlnp 2>/dev/null | grep -q ':5000 '; then
        log_warn "Порт 5000 уже занят!"
        echo ""
        local pid
        pid=$(ss -tlnp 2>/dev/null | grep ':5000 ' | grep -oP 'pid=\K[0-9]+' | head -1 || echo "?")
        [[ "$pid" != "?" ]] && echo -e "    PID: $pid  (kill -9 $pid)"
        read -rp "  Продолжить всё равно? [y/N]: " ans
        [[ "$ans" =~ ^[Yy]$ ]] || exit 0
    else
        log_ok "Порт 5000 свободен"
    fi
}

# ─── 5. Запуск ───────────────────────────────────────────────────────────────
launch() {
    log_step "Запуск симулятора"

    # Определяем IP для отображения
    local local_ip
    local_ip=$(python3 -c "
import socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    print(s.getsockname()[0])
    s.close()
except:
    print('localhost')
" 2>/dev/null || echo "localhost")

    echo ""
    log_ok "Симулятор стартует..."
    echo ""
    echo -e "  ${BOLD}Открой в браузере:${NC}"
    echo -e "    ${CYAN}http://localhost:5000${NC}        (этот компьютер)"
    echo -e "    ${CYAN}http://${local_ip}:5000${NC}   (Android приложение)"
    echo ""
    echo -e "  ${BOLD}В Android приложении:${NC}"
    echo -e "    Настройки → IP: ${YELLOW}${local_ip}${NC} → Режим: Симулятор → Подключить"
    echo ""
    echo -e "${YELLOW}  ── Запуск (Ctrl+C для остановки) ──${NC}"
    echo ""

    cd "$SCRIPT_DIR/compute_node"
    exec python3 simulator.py
}

# ─── Main ────────────────────────────────────────────────────────────────────
main() {
    print_banner
    check_python
    check_packages
    check_simulator_file
    check_port
    launch
}

main
