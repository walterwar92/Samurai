#!/usr/bin/env bash
# scripts/cmds/bridge.sh — Samcan USB bridge (FastAPI :5005)
#
# Использование:
#   samurai bridge                 # auto-detect порта Arduino
#   samurai bridge COM3
#   samurai bridge /dev/ttyUSB0

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    local port_arg=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                cat <<EOF
Использование: samurai bridge [PORT]

Запускает Samcan USB bridge (FastAPI :5005) — мост между фронтом и Arduino Uno.

Опции:
  PORT        COM-порт (COM3 на Windows, /dev/ttyUSB0 на Linux).
              Без аргумента — автоопределение по VID/PID Arduino.

Примеры:
  samurai bridge
  samurai bridge COM3
  samurai bridge /dev/ttyUSB0
EOF
                exit 0
                ;;
            --*) die "Неизвестный флаг: $1" ;;
            *) port_arg="$1"; shift ;;
        esac
    done

    acquire_lock bridge
    print_banner "S A M C A N   U S B   B R I D G E" "Arduino Uno -> FastAPI :5005"

    check_python 9
    check_pip_packages \
        "serial:pyserial" \
        "fastapi:fastapi>=0.110" \
        "uvicorn:uvicorn[standard]>=0.27" \
        "pydantic:pydantic>=2"

    local script="$SAMURAI_ROOT/compute_node/samcan_bridge.py"
    [[ -f "$script" ]] || die "samcan_bridge.py не найден: $script"

    local PY; PY=$(detect_python) || die "Python не найден"

    log_step "Запуск Samcan bridge"
    echo ""
    log_ok "Endpoint:  ${BOLD}http://localhost:5005${NC}"
    log_ok "Скрипт:    $script"
    [[ -n "$port_arg" ]] && log_ok "Порт:      ${BOLD}$port_arg${NC}" || log_ok "Порт:      auto-detect"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    cd "$SAMURAI_ROOT"
    if [[ -n "$port_arg" ]]; then
        exec "$PY" "$script" --port "$port_arg"
    else
        exec "$PY" "$script" --auto
    fi
}

main "$@"
