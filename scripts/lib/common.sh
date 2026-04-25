# shellcheck shell=bash
# scripts/lib/common.sh
# Общие helper'ы: цвета, логирование, баннер, IP-обнаружение, конфиг.
# Подключается через: source "$SCRIPT_DIR/scripts/lib/common.sh"

# Защита от двойного подключения
[[ -n "${_SAMURAI_COMMON_LOADED:-}" ]] && return 0
_SAMURAI_COMMON_LOADED=1

# ── Цвета ───────────────────────────────────────────────────────────────────
if [[ -t 1 ]]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
    BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; BLUE=''; CYAN=''; BOLD=''; NC=''
fi

# ── Логирование ─────────────────────────────────────────────────────────────
log_ok()   { echo -e "${GREEN}  [✓]${NC} $*"; }
log_warn() { echo -e "${YELLOW}  [!]${NC} $*"; }
log_err()  { echo -e "${RED}  [✗]${NC} $*" >&2; }
log_info() { echo -e "${BLUE}  [→]${NC} $*"; }
log_step() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }
die()      { log_err "$*"; exit 1; }

# ── Корень проекта ──────────────────────────────────────────────────────────
# SAMURAI_ROOT определяется один раз — корень репозитория.
samurai_root() {
    if [[ -n "${SAMURAI_ROOT:-}" ]]; then
        echo "$SAMURAI_ROOT"
        return
    fi
    local self="${BASH_SOURCE[0]}"
    # scripts/lib/common.sh → корень на 2 уровня выше
    cd "$(dirname "$self")/../.." && pwd
}

SAMURAI_ROOT="${SAMURAI_ROOT:-$(samurai_root)}"
export SAMURAI_ROOT

# ── Директории логов и состояния ────────────────────────────────────────────
SAMURAI_STATE_DIR="${SAMURAI_STATE_DIR:-$HOME/.samurai}"
SAMURAI_LOG_DIR="${SAMURAI_LOG_DIR:-$SAMURAI_STATE_DIR/logs}"
SAMURAI_LOCK_DIR="${SAMURAI_LOCK_DIR:-$SAMURAI_STATE_DIR/locks}"

ensure_state_dirs() {
    mkdir -p "$SAMURAI_LOG_DIR" "$SAMURAI_LOCK_DIR" 2>/dev/null || true
}

# ── Баннер ──────────────────────────────────────────────────────────────────
# Использование: print_banner "Заголовок" "Подзаголовок"
print_banner() {
    local title="${1:-S A M U R A I   R O B O T}"
    local subtitle="${2:-}"
    local my_ip
    my_ip=$(get_my_ip)

    echo -e "${CYAN}${BOLD}"
    echo "  ╔═══════════════════════════════════════════════╗"
    printf "  ║  %-43s  ║\n" "$title"
    [[ -n "$subtitle" ]] && printf "  ║  %-43s  ║\n" "$subtitle"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  Hostname: ${BOLD}$(hostname)${NC}  IP: ${BOLD}${my_ip}${NC}"
    echo ""
}

# ── IP / сеть ───────────────────────────────────────────────────────────────
get_my_ip() {
    local ip
    ip=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
    if [[ -z "$ip" ]]; then
        ip=$(ip route get 1.1.1.1 2>/dev/null | awk '{print $7; exit}' || true)
    fi
    echo "${ip:-127.0.0.1}"
}

# Проверка доступности TCP-порта (timeout 2s).
# Использование: check_tcp_port HOST PORT
check_tcp_port() {
    local host="$1" port="$2"
    if command -v nc &>/dev/null; then
        nc -z -w 2 "$host" "$port" &>/dev/null
    else
        timeout 2 bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null
    fi
}

# ── Платформа ───────────────────────────────────────────────────────────────
is_windows() {
    [[ "$(uname -s)" =~ MINGW|MSYS|CYGWIN|NT ]] || [[ "$(uname -o 2>/dev/null)" == "Msys" ]]
}

is_pi() {
    [[ -f /etc/rpi-issue ]] || \
        grep -qi "raspberry pi" /proc/cpuinfo 2>/dev/null
}

is_arch_linux() {
    [[ -f /etc/arch-release ]]
}

# ── Confirm prompt ─────────────────────────────────────────────────────────
# Использование: if confirm "Продолжить?"; then ...; fi
confirm() {
    local prompt="${1:-Продолжить?}"
    local default="${2:-n}"
    local hint="[y/N]"
    [[ "$default" == "y" ]] && hint="[Y/n]"
    local ans
    read -rp "  $prompt $hint: " ans
    ans="${ans:-$default}"
    [[ "$ans" =~ ^[Yy]$ ]]
}

# ── MQTT credentials helper ────────────────────────────────────────────────
# Читает ~/.samurai/mqtt.passwd и экспортирует SAMURAI_MQTT_USER + SAMURAI_MQTT_PASS.
# Если файла нет — переменные не устанавливаются (anonymous).
# Использование (после source common.sh):
#   load_mqtt_creds        # экспортирует ENV если файл есть
#   echo "$SAMURAI_MQTT_USER"
load_mqtt_creds() {
    local passwd_file="${SAMURAI_MQTT_PASSWD:-$HOME/.samurai/mqtt.passwd}"
    [[ -f "$passwd_file" ]] || return 1
    local line
    line=$(head -n1 "$passwd_file" 2>/dev/null || true)
    [[ "$line" == *":"* ]] || return 1
    export SAMURAI_MQTT_USER="${line%%:*}"
    export SAMURAI_MQTT_PASS="${line#*:}"
    return 0
}

# ── Версия CLI ──────────────────────────────────────────────────────────────
SAMURAI_CLI_VERSION="1.0.0"
