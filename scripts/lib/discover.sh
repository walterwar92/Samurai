# shellcheck shell=bash
# scripts/lib/discover.sh
# Обнаружение Raspberry Pi в сети через mDNS / ping / стандартные IP.

[[ -n "${_SAMURAI_DISCOVER_LOADED:-}" ]] && return 0
_SAMURAI_DISCOVER_LOADED=1

[[ -z "${_SAMURAI_COMMON_LOADED:-}" ]] && {
    # shellcheck source=common.sh
    source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
}

# Обнаружение Pi.
# Использование: PI_IP=$(discover_pi [hint_ip]) || die "Pi не найден"
# Если hint_ip передан — используется как override.
discover_pi() {
    local hint="${1:-}"
    if [[ -n "$hint" ]]; then
        echo "$hint"
        return 0
    fi

    local ip=""

    # 1. mDNS через getent (с nss-mdns)
    ip=$(getent ahosts raspberrypi.local 2>/dev/null \
         | awk '/STREAM/ {print $1; exit}' || true)
    if [[ -n "$ip" && "$ip" =~ ^[0-9.]+$ ]]; then
        echo "$ip"
        return 0
    fi

    # 2. avahi-resolve
    if command -v avahi-resolve &>/dev/null; then
        ip=$(avahi-resolve -n raspberrypi.local 2>/dev/null | awk '{print $2}' | head -1 || true)
        if [[ -n "$ip" && "$ip" =~ ^[0-9.]+$ ]]; then
            echo "$ip"
            return 0
        fi
    fi

    # 3. ping раскрывает имя
    if ping -c 1 -W 2 raspberrypi.local &>/dev/null; then
        ip=$(ping -c 1 -W 2 raspberrypi.local 2>/dev/null \
             | grep -oP '\(\K[0-9.]+' | head -1 || true)
        if [[ -n "$ip" ]]; then
            echo "$ip"
            return 0
        fi
    fi

    # 4. Перебор стандартных IP хотспота
    local candidate
    for candidate in 192.168.1.100 192.168.4.1 192.168.43.1 10.42.0.1; do
        if check_tcp_port "$candidate" 1883; then
            echo "$candidate"
            return 0
        fi
    done

    return 1
}

# Интерактивный fallback — если discover не нашёл, спрашивает у пользователя.
# Использование: PI_IP=$(discover_pi_interactive [hint_ip])
discover_pi_interactive() {
    local hint="${1:-}"
    local ip
    ip=$(discover_pi "$hint" || true)
    if [[ -z "$ip" ]]; then
        log_warn "Pi не найден автоматически" >&2
        read -rp "  Введи IP Raspberry Pi: " ip >&2
        if [[ ! "$ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            die "Неверный IP: '$ip'"
        fi
    fi
    echo "$ip"
}
