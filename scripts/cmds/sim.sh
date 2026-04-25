#!/usr/bin/env bash
# scripts/cmds/sim.sh — симулятор (Flask, без ROS2/железа)
#
# Использование:
#   samurai sim
#   samurai sim --port 5050

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    local port=5000

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --port) port="$2"; shift 2 ;;
            -h|--help)
                cat <<EOF
Использование: samurai sim [опции]

Запускает stand-alone симулятор робота (Flask + OpenCV).
Совместим с тем же фронтендом и Android, что и реальный робот.

Опции:
  --port N    HTTP порт (по умолчанию: 5000)
  -h, --help  Эта справка
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1" ;;
        esac
    done

    acquire_lock sim
    print_banner "S A M U R A I   S I M U L A T O R" "Flask + OpenCV  -  без ROS2/железа"

    check_python 9
    check_pip_packages \
        "flask:flask" \
        "flask_cors:flask-cors" \
        "flask_socketio:flask-socketio" \
        "cv2:opencv-python" \
        "numpy:numpy" \
        "paho.mqtt.client:paho-mqtt"

    local sim_script="$SAMURAI_ROOT/compute_node/simulator.py"
    [[ -f "$sim_script" ]] || die "simulator.py не найден: $sim_script"
    log_ok "compute_node/simulator.py найден"

    check_local_port_free "$port" --ask || true

    log_step "Запуск симулятора"
    local local_ip; local_ip=$(get_my_ip)
    echo ""
    echo -e "  ${BOLD}Открой в браузере:${NC}"
    echo -e "    ${CYAN}http://localhost:${port}${NC}        (этот компьютер)"
    echo -e "    ${CYAN}http://${local_ip}:${port}${NC}   (Android)"
    echo ""
    echo -e "  ${BOLD}В Android:${NC} IP=${YELLOW}${local_ip}${NC}, режим=Симулятор"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    cd "$SAMURAI_ROOT/compute_node"
    exec python3 simulator.py
}

main "$@"
