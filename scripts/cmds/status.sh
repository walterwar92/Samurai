#!/usr/bin/env bash
# scripts/cmds/status.sh — показать что запущено через samurai CLI

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    print_banner "S A M U R A I   S T A T U S" "Активные компоненты"

    local components=(robot sim compute detector bridge agent)
    local any_running=false

    printf "  %-12s %-10s %-8s\n" "КОМПОНЕНТ" "СТАТУС" "PID"
    printf "  %-12s %-10s %-8s\n" "──────────" "──────" "───"

    local c pid
    for c in "${components[@]}"; do
        pid=$(is_running "$c" || true)
        if [[ -n "$pid" ]]; then
            printf "  %-12s ${GREEN}%-10s${NC} %-8s\n" "$c" "running" "$pid"
            any_running=true
        else
            printf "  %-12s ${YELLOW}%-10s${NC} %-8s\n" "$c" "stopped" "-"
        fi
    done

    echo ""
    log_step "Системные службы"
    local svc
    for svc in mosquitto avahi-daemon docker; do
        if systemctl is-active --quiet "$svc" 2>/dev/null; then
            log_ok "$svc: active"
        elif systemctl list-unit-files 2>/dev/null | grep -q "^$svc.service"; then
            log_warn "$svc: inactive"
        fi
    done

    echo ""
    log_step "Локальные TCP порты"
    local port
    for port in 1883 5000 5005 5001; do
        if ss -tlnp 2>/dev/null | grep -q ":$port " || netstat -tlnp 2>/dev/null | grep -q ":$port "; then
            log_ok "$port — слушает"
        fi
    done

    echo ""
    log_step "systemd units"
    local unit state enabled
    for unit in samurai-robot samurai-compute samurai-bridge samurai-agent; do
        if systemctl list-unit-files 2>/dev/null | grep -q "^$unit.service"; then
            state=$(systemctl is-active "$unit" 2>/dev/null || echo "inactive")
            enabled=$(systemctl is-enabled "$unit" 2>/dev/null || echo "disabled")
            printf "  %-22s %s / %s\n" "$unit.service" "$state" "$enabled"
        fi
    done

    if ! $any_running; then
        echo ""
        log_info "Ничего не запущено через samurai CLI"
        log_info "Запуск: ${BOLD}./samurai.sh help${NC}"
    fi
}

main "$@"
