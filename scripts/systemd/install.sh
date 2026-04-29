#!/usr/bin/env bash
# scripts/systemd/install.sh — установить samurai-* unit-файлы в /etc/systemd/system/
#
# Требует sudo. Подставляет реальный путь к репо и юзера в шаблоны.
#
# Использование:
#   sudo ./scripts/systemd/install.sh                # все unit'ы
#   sudo ./scripts/systemd/install.sh robot          # только samurai-robot
#   sudo ./scripts/systemd/install.sh robot bridge   # robot и bridge
#   sudo ./scripts/systemd/install.sh --uninstall    # удалить все

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMURAI_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck source=../lib/common.sh
source "$SAMURAI_ROOT/scripts/lib/common.sh"

UNIT_DIR="/etc/systemd/system"
ALL_UNITS=(robot compute bridge agent)

require_root() {
    if [[ $EUID -ne 0 ]]; then
        die "Требуется sudo: ${YELLOW}sudo $0 $*${NC}"
    fi
}

determine_user() {
    # SUDO_USER если запущено через sudo, иначе текущий
    if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
        echo "$SUDO_USER"
    else
        # На Pi обычно есть пользователь pi
        if id -u pi &>/dev/null; then
            echo "pi"
        else
            die "Не могу определить целевого пользователя. Запусти: sudo -u <user> $0 ..."
        fi
    fi
}

uninstall_units() {
    require_root "--uninstall"
    log_step "Удаление samurai-* units"
    local unit
    for unit in "${ALL_UNITS[@]}"; do
        local svc="samurai-${unit}.service"
        if [[ -f "$UNIT_DIR/$svc" ]]; then
            systemctl stop "$svc" 2>/dev/null || true
            systemctl disable "$svc" 2>/dev/null || true
            rm -f "$UNIT_DIR/$svc"
            log_ok "Удалён $svc"
        else
            log_info "$svc не установлен (пропуск)"
        fi
    done
    systemctl daemon-reload
    log_ok "Готово. systemctl daemon-reload выполнен."
}

install_unit() {
    local unit="$1" user="$2"
    local template="$SCRIPT_DIR/samurai-${unit}.service"
    local target="$UNIT_DIR/samurai-${unit}.service"

    [[ -f "$template" ]] || die "Шаблон не найден: $template"

    log_info "Установка samurai-${unit}.service (user=$user, root=$SAMURAI_ROOT)"
    sed \
        -e "s|__SAMURAI_USER__|${user}|g" \
        -e "s|__SAMURAI_ROOT__|${SAMURAI_ROOT}|g" \
        "$template" > "$target"

    chmod 644 "$target"
    log_ok "  → $target"
}

main() {
    if [[ "${1:-}" == "--uninstall" || "${1:-}" == "-u" ]]; then
        uninstall_units
        exit 0
    fi

    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        cat <<EOF
Использование:
  sudo $0                          # все unit'ы (robot, compute, bridge)
  sudo $0 robot                    # только samurai-robot
  sudo $0 robot bridge             # перечислить
  sudo $0 --uninstall              # удалить все

После установки:
  sudo systemctl enable --now samurai-robot     # автозапуск
  systemctl status samurai-robot                # статус
  journalctl -u samurai-robot -f                # live-логи
EOF
        exit 0
    fi

    require_root "$@"

    local user
    user=$(determine_user)

    print_banner "S A M U R A I   S Y S T E M D" "Установка unit-файлов"
    log_ok "Целевой пользователь: ${BOLD}$user${NC}"
    log_ok "Корень репо:          ${BOLD}$SAMURAI_ROOT${NC}"
    log_ok "Unit-директория:      ${BOLD}$UNIT_DIR${NC}"
    echo ""

    # Создать директорию состояния (для логов и lock-файлов)
    mkdir -p /var/lib/samurai/{logs,locks}
    chown -R "$user:$user" /var/lib/samurai
    log_ok "Создана /var/lib/samurai (owner=$user)"

    # Установить logrotate-правило (если logrotate доступен)
    local logrotate_src="$SCRIPT_DIR/samurai.logrotate"
    if [[ -f "$logrotate_src" ]] && command -v logrotate >/dev/null 2>&1; then
        # sed подставит реальный user (по умолчанию шаблон ссылается на pi)
        sed "s/create 0644 pi pi/create 0644 $user $user/" \
            "$logrotate_src" > /etc/logrotate.d/samurai
        chmod 644 /etc/logrotate.d/samurai
        log_ok "Установлен /etc/logrotate.d/samurai (rotate 7d, maxsize 50M)"
    elif [[ -f "$logrotate_src" ]]; then
        log_info "logrotate не установлен — пропуск config (manual-run logs не ротируются)"
    fi

    # Какие unit'ы устанавливать
    local targets=()
    if [[ $# -eq 0 ]]; then
        targets=("${ALL_UNITS[@]}")
    else
        targets=("$@")
    fi

    log_step "Установка"
    local t
    for t in "${targets[@]}"; do
        local found=false
        local v
        for v in "${ALL_UNITS[@]}"; do
            [[ "$t" == "$v" ]] && { found=true; break; }
        done
        $found || die "Неизвестный unit: $t (доступны: ${ALL_UNITS[*]})"
        install_unit "$t" "$user"
    done

    log_step "systemctl daemon-reload"
    systemctl daemon-reload
    log_ok "daemon-reload OK"

    echo ""
    log_step "Что дальше"
    echo ""
    echo -e "  ${BOLD}Включить автозапуск:${NC}"
    for t in "${targets[@]}"; do
        echo -e "    ${YELLOW}sudo systemctl enable --now samurai-${t}${NC}"
    done
    echo ""
    echo -e "  ${BOLD}Проверить статус:${NC}"
    for t in "${targets[@]}"; do
        echo -e "    ${YELLOW}systemctl status samurai-${t}${NC}"
    done
    echo ""
    echo -e "  ${BOLD}Live-логи:${NC}"
    for t in "${targets[@]}"; do
        echo -e "    ${YELLOW}journalctl -u samurai-${t} -f${NC}"
    done
    echo ""
    echo -e "  ${BOLD}Удалить:${NC}"
    echo -e "    ${YELLOW}sudo $0 --uninstall${NC}"
    echo ""
    log_warn "ВАЖНО: установи Python-зависимости ВРУЧНУЮ перед enable"
    log_warn "       (systemd не делает pip install автоматически)"
    log_warn "       Pi:    pip3 install paho-mqtt PyYAML smbus2 gpiozero"
    log_warn "       Ноут:  pip3 install pyserial fastapi uvicorn pydantic"
}

main "$@"
