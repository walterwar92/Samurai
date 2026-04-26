#!/usr/bin/env bash
# scripts/cmds/planner.sh — A* path planner на ноутбуке (#3, 2026-04)
#
# Подписывается на samurai/{robot_id}/slam_map + odom + path_planner/goal,
# планирует A* на occupancy grid (с inflation под радиус робота) и
# публикует path_planner/path. Работает чисто на MQTT (не требует ROS2).
#
# Использование:
#   samurai planner                      # auto-discover Pi
#   samurai planner --pi 192.168.1.50    # явный IP брокера
#   samurai planner --robot-radius 0.20  # 20 см вместо 15 см дефолта
#   samurai planner --no-simplify        # raw cells, без LOS-shortcut

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/discover.sh
source "$LIB_DIR/discover.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    local pi_ip_arg=""
    local robot_id="robot1"
    local robot_radius="0.15"
    local simplify=true

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pi)            pi_ip_arg="$2"; shift 2 ;;
            --robot-id)      robot_id="$2"; shift 2 ;;
            --robot-radius)  robot_radius="$2"; shift 2 ;;
            --no-simplify)   simplify=false; shift ;;
            -h|--help)
                cat <<EOF
Использование: samurai planner [опции]

Запускает compute_node/path_planner — A* планировщик на ноутбуке.
Использует MQTT для коммуникации с Pi (slam_map + odom + path_planner/*).

Опции:
  --pi IP            IP Raspberry Pi (без — mDNS auto-discovery)
  --robot-id ID      MQTT robot_id (default: robot1)
  --robot-radius M   Радиус робота в метрах для inflate (default: 0.15)
  --no-simplify      Не упрощать путь (оставить все ячейки A*)
  -h, --help         Эта справка

Примеры:
  samurai planner                        # auto-discover, default radius
  samurai planner --pi 192.168.1.50      # явный IP
  samurai planner --robot-radius 0.20    # больший inflation
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai planner --help)" ;;
        esac
    done

    acquire_lock planner

    log_step "Поиск Raspberry Pi (MQTT broker)"
    local pi_ip
    pi_ip=$(discover_pi_interactive "$pi_ip_arg")

    print_banner "P A T H   P L A N N E R" "A* on ноутбуке  -  MQTT only"

    check_python 9
    check_pip_packages \
        "paho.mqtt.client:paho-mqtt" \
        "numpy:numpy"

    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    if ! check_tcp_port "$pi_ip" 1883; then
        log_warn "Порт 1883 не отвечает — убедись что mosquitto на Pi"
    fi

    if load_mqtt_creds; then
        log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC}"
    fi

    log_step "Запуск планировщика"
    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Pi MQTT: ${CYAN}${pi_ip}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  Robot:   ${GREEN}${robot_id}${NC} radius=${robot_radius}m"
    echo -e "  ${BOLD}│${NC}  Подписки: ${GREEN}slam_map, odom, path_planner/goal, zones/update${NC}"
    echo -e "  ${BOLD}│${NC}  Топики:  ${GREEN}samurai/${robot_id}/path_planner/{path,status}${NC}"
    echo -e "  ${BOLD}└──────────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    local args=(
        --broker "$pi_ip"
        --port 1883
        --robot-id "$robot_id"
        --robot-radius "$robot_radius"
    )
    $simplify || args+=(--no-simplify)

    cd "$SAMURAI_ROOT"
    exec python3 -m compute_node.path_planner "${args[@]}"
}

main "$@"
