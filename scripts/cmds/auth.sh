#!/usr/bin/env bash
# scripts/cmds/auth.sh — управление MQTT credentials (~/.samurai/mqtt.passwd)
#
# Подкоманды:
#   samurai auth init [--force]         Создать файл с дефолтными creds (samurai/samurai)
#   samurai auth show                   Показать текущие creds
#   samurai auth set USER PASS          Установить вручную
#   samurai auth disable                Удалить файл (вернуться к anonymous)
#   samurai auth status                 Что настроено сейчас

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"

PASSWD_FILE="$HOME/.samurai/mqtt.passwd"

# ── Helpers ─────────────────────────────────────────────────────────────────
read_passwd() {
    [[ -f "$PASSWD_FILE" ]] || return 1
    local line; line=$(head -n1 "$PASSWD_FILE" 2>/dev/null || echo "")
    [[ "$line" == *":"* ]] || return 1
    echo "$line"
}

write_passwd() {
    local user="$1" pass="$2"
    mkdir -p "$(dirname "$PASSWD_FILE")"
    printf '%s:%s\n' "$user" "$pass" > "$PASSWD_FILE"
    chmod 600 "$PASSWD_FILE"
}

# Если на Pi — обновить mosquitto.conf чтобы требовать auth.
update_mosquitto_if_pi() {
    local user="$1" pass="$2"
    if is_pi; then
        log_info "Обнаружен Pi — обновляю mosquitto.conf с auth"
        check_mosquitto --auth "$user" "$pass"
    else
        log_info "Не Pi — пропускаю настройку mosquitto"
        log_info "Если broker на этой машине, запусти: ./samurai.sh auth init на Pi"
    fi
}

# Если на Pi — пересоздать mosquitto.conf без auth.
disable_mosquitto_auth() {
    if is_pi; then
        log_info "Обнаружен Pi — отключаю auth в mosquitto.conf"
        check_mosquitto
    fi
}

# ── Подкоманды ──────────────────────────────────────────────────────────────
cmd_init() {
    local force=false
    [[ "${1:-}" == "--force" ]] && force=true

    if [[ -f "$PASSWD_FILE" ]] && ! $force; then
        log_warn "Файл уже существует: $PASSWD_FILE"
        local creds; creds=$(read_passwd 2>/dev/null || echo "?:?")
        echo -e "    Текущий user: ${BOLD}${creds%%:*}${NC}"
        echo -e "    Чтобы перезаписать: ${YELLOW}samurai auth init --force${NC}"
        echo -e "    Чтобы посмотреть:   ${YELLOW}samurai auth show${NC}"
        echo -e "    Чтобы изменить:     ${YELLOW}samurai auth set USER PASS${NC}"
        exit 0
    fi

    print_banner "S A M U R A I   A U T H" "Initial setup"

    local user="samurai" pass="samurai"
    log_info "Создаю $PASSWD_FILE с дефолтными credentials"
    write_passwd "$user" "$pass"
    log_ok "Файл создан (chmod 600)"
    log_ok "User:     ${BOLD}$user${NC}"
    log_ok "Password: ${BOLD}$pass${NC}"

    echo ""
    log_warn "Это ДЕФОЛТНЫЕ creds. Для production смени:"
    echo -e "    ${YELLOW}samurai auth set <user> <random-password>${NC}"
    echo ""

    update_mosquitto_if_pi "$user" "$pass"

    echo ""
    log_step "Что дальше"
    echo ""
    echo -e "  ${BOLD}На этой машине:${NC} креды читаются автоматически из $PASSWD_FILE"
    echo ""
    echo -e "  ${BOLD}На других устройствах:${NC} нужно прописать те же creds"
    echo ""
    echo -e "  ${BOLD}Ноутбук (если broker не здесь):${NC}"
    echo -e "    ${YELLOW}./samurai.sh auth set $user $pass${NC}"
    echo ""
    echo -e "  ${BOLD}Android:${NC}"
    echo -e "    Settings → MQTT user: ${YELLOW}$user${NC}"
    echo -e "    Settings → MQTT password: ${YELLOW}$pass${NC}"
    echo ""
    echo -e "  ${BOLD}ESP32 firmware:${NC} в firmware/esp32/src/config.h раскомментировать"
    echo -e "    ${YELLOW}#define MQTT_USER \"$user\"${NC}"
    echo -e "    ${YELLOW}#define MQTT_PASS \"$pass\"${NC}"
    echo ""
    echo -e "  ${BOLD}Перезапусти всё:${NC} ${YELLOW}./samurai.sh stop && ./samurai.sh robot${NC}"
}

cmd_show() {
    if [[ ! -f "$PASSWD_FILE" ]]; then
        log_warn "Файл не существует: $PASSWD_FILE"
        log_info "MQTT auth выключен. Установить: ${YELLOW}samurai auth init${NC}"
        exit 0
    fi
    local creds; creds=$(read_passwd) || die "Файл повреждён: $PASSWD_FILE"
    print_banner "S A M U R A I   A U T H" "Current credentials"
    echo -e "  Файл:     ${BOLD}$PASSWD_FILE${NC}"
    echo -e "  User:     ${BOLD}${creds%%:*}${NC}"
    echo -e "  Password: ${BOLD}${creds#*:}${NC}"
    echo ""
    log_warn "Никому не показывай эти creds. Они дают полный доступ к роботу."
}

cmd_set() {
    local user="${1:-}" pass="${2:-}"
    [[ -z "$user" || -z "$pass" ]] && {
        log_err "Нужно: samurai auth set USER PASSWORD"
        exit 1
    }
    print_banner "S A M U R A I   A U T H" "Set credentials"
    write_passwd "$user" "$pass"
    log_ok "Сохранено в $PASSWD_FILE"
    log_ok "User:     ${BOLD}$user${NC}"
    log_ok "Password: ${BOLD}$pass${NC}"
    update_mosquitto_if_pi "$user" "$pass"
    echo ""
    log_warn "Не забудь обновить creds на других устройствах (ноут, Android, ESP32)"
}

cmd_disable() {
    if [[ ! -f "$PASSWD_FILE" ]]; then
        log_info "$PASSWD_FILE не существует — auth уже отключён"
        exit 0
    fi
    print_banner "S A M U R A I   A U T H" "Disable"
    log_warn "Удалю $PASSWD_FILE — MQTT снова станет anonymous"
    if ! confirm "Продолжить?" n; then
        log_info "Отменено"
        exit 0
    fi
    rm -f "$PASSWD_FILE"
    log_ok "Файл удалён"
    disable_mosquitto_auth
    echo ""
    log_warn "Не забудь убрать creds на ноуте/Android/ESP32"
}

cmd_status() {
    print_banner "S A M U R A I   A U T H" "Status"

    log_step "Файл credentials"
    if [[ -f "$PASSWD_FILE" ]]; then
        log_ok "$PASSWD_FILE существует"
        local creds; creds=$(read_passwd 2>/dev/null || echo "повреждён")
        if [[ "$creds" == "повреждён" ]]; then
            log_err "Формат файла невалидный (ожидается user:password)"
        else
            log_info "User: ${BOLD}${creds%%:*}${NC}"
        fi
    else
        log_info "$PASSWD_FILE не существует (auth выключен)"
    fi

    log_step "ENV переменные"
    if [[ -n "${SAMURAI_MQTT_USER:-}" && -n "${SAMURAI_MQTT_PASS:-}" ]]; then
        log_ok "SAMURAI_MQTT_USER=$SAMURAI_MQTT_USER (имеет приоритет над файлом)"
    else
        log_info "SAMURAI_MQTT_USER / SAMURAI_MQTT_PASS не установлены"
    fi

    log_step "config.yaml"
    if grep -q "^[[:space:]]*username:" "$SAMURAI_ROOT/config.yaml" 2>/dev/null; then
        log_warn "В config.yaml есть mqtt.auth.username (НЕ для production)"
    else
        log_info "В config.yaml auth закомментирован"
    fi

    log_step "Mosquitto (если на Pi)"
    if is_pi && [[ -f /etc/mosquitto/conf.d/samurai.conf ]]; then
        if grep -q "allow_anonymous false" /etc/mosquitto/conf.d/samurai.conf; then
            log_ok "mosquitto: allow_anonymous=false (auth требуется)"
        else
            log_warn "mosquitto: allow_anonymous=true (anonymous разрешён)"
        fi
    fi
}

cmd_help() {
    cat <<EOF
Использование: samurai auth <подкоманда> [args]

Управление MQTT credentials. Файл хранится в ${BOLD}~/.samurai/mqtt.passwd${NC}
(chmod 600, формат "user:password").

Подкоманды:
  ${GREEN}init [--force]${NC}      Создать файл с дефолтом (samurai/samurai)
  ${GREEN}show${NC}                Показать текущие credentials
  ${GREEN}set USER PASS${NC}       Установить вручную
  ${GREEN}disable${NC}             Удалить файл (вернуться к anonymous)
  ${GREEN}status${NC}              Полный статус (файл, ENV, config.yaml, mosquitto)

Приоритет источников credentials в python-клиентах:
  1. ENV: SAMURAI_MQTT_USER + SAMURAI_MQTT_PASS
  2. Файл ~/.samurai/mqtt.passwd
  3. config.yaml: mqtt.auth.{username, password}
  4. None → anonymous (default если ничего не настроено)

Примеры:
  samurai auth init                       # дефолтные creds + настройка mosquitto
  samurai auth set robot1 sup3rs3cr3t     # production-режим
  samurai auth disable                    # вернуться к anonymous
  samurai auth status                     # проверить что настроено
EOF
}

main() {
    local sub="${1:-help}"
    shift || true

    case "$sub" in
        init)    cmd_init "$@" ;;
        show)    cmd_show "$@" ;;
        set)     cmd_set "$@" ;;
        disable) cmd_disable "$@" ;;
        status)  cmd_status "$@" ;;
        help|-h|--help) cmd_help ;;
        *)
            log_err "Неизвестная подкоманда: $sub"
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
