#!/usr/bin/env bash
# scripts/cmds/agent.sh — MOIS HTTP-агент: мост сайт ↔ робот
#
# Запускается на ноутбуке (Compute). Polling-based: каждые N секунд
# дёргает Supabase Edge Function, выполняет команды локально через
# dashboard FastAPI :5000, отдаёт результаты + телеметрию.
#
# Использование:
#   samurai agent                              # из ./compute_node/mois_agent/config.json
#   samurai agent --api-url URL --api-token T  # явные креды (бьют файл)
#   samurai agent --dashboard http://127.0.0.1:5000 --robot-id robot1
#   samurai agent --list-commands              # перечислить и выйти
#   samurai agent --log-level DEBUG

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    local api_url=""
    local api_token=""
    local dashboard=""
    local samcan=""
    local robot_id=""
    local poll=""
    local telemetry=""
    local log_level="INFO"
    local config_path=""
    local list_only=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --api-url)         api_url="$2"; shift 2 ;;
            --api-token)       api_token="$2"; shift 2 ;;
            --dashboard)       dashboard="$2"; shift 2 ;;
            --samcan)          samcan="$2"; shift 2 ;;
            --robot-id)        robot_id="$2"; shift 2 ;;
            --poll)            poll="$2"; shift 2 ;;
            --telemetry)       telemetry="$2"; shift 2 ;;
            --config)          config_path="$2"; shift 2 ;;
            --log-level)       log_level="$2"; shift 2 ;;
            --list-commands)   list_only=true; shift ;;
            -h|--help)
                cat <<EOF
Использование: samurai agent [опции]

MOIS HTTP-агент: дёргает Supabase Edge Function (robot-gateway),
выполняет команды локально через dashboard FastAPI :5000.

Опции:
  --api-url URL          Supabase Edge Function URL (или MOIS_API_URL env)
  --api-token TOKEN      Bearer-токен (или MOIS_API_TOKEN env)
  --dashboard URL        Dashboard URL (default: http://127.0.0.1:5000)
  --samcan URL           Прямой URL к Samcan bridge (без — через dashboard прокси)
  --robot-id ID          MQTT robot_id (default: robot1)
  --poll SEC             Poll-интервал (default из config или сервера)
  --telemetry SEC        Интервал телеметрии (default из config)
  --config PATH          Альт. путь к config.json
  --log-level LVL        DEBUG|INFO|WARNING|ERROR
  --list-commands        Распечатать все команды и выйти
  -h, --help             Эта справка

Источники конфига (приоритет ↓):
  1. CLI-флаги
  2. ENV: MOIS_API_URL, MOIS_API_TOKEN, MOIS_DASHBOARD_URL, ...
  3. compute_node/mois_agent/config.json
  4. Дефолты для интервалов и dashboard URL

Примеры:
  samurai agent                                   # из config.json
  samurai agent --list-commands                   # что умеет
  MOIS_API_TOKEN=rt_xxx samurai agent             # токен из ENV
  samurai agent --api-url \$URL --api-token \$T   # явно

Перед запуском:
  cp compute_node/mois_agent/config.json.example \\
     compute_node/mois_agent/config.json
  # отредактируй api_url и api_token, потом:
  samurai agent
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai agent --help)" ;;
        esac
    done

    if ! $list_only; then
        acquire_lock agent
    fi

    print_banner "M O I S   A G E N T" "Site Edge Function  ↔  dashboard :5000"

    check_python 9
    check_pip_packages "httpx:httpx"

    cd "$SAMURAI_ROOT"

    local args=(--log-level "$log_level")
    [[ -n "$api_url"     ]] && args+=(--api-url     "$api_url")
    [[ -n "$api_token"   ]] && args+=(--api-token   "$api_token")
    [[ -n "$dashboard"   ]] && args+=(--dashboard   "$dashboard")
    [[ -n "$samcan"      ]] && args+=(--samcan      "$samcan")
    [[ -n "$robot_id"    ]] && args+=(--robot-id    "$robot_id")
    [[ -n "$poll"        ]] && args+=(--poll        "$poll")
    [[ -n "$telemetry"   ]] && args+=(--telemetry   "$telemetry")
    [[ -n "$config_path" ]] && args+=(--config      "$config_path")
    if $list_only; then
        args+=(--list-commands)
    fi

    if ! $list_only; then
        log_step "Параметры запуска"
        log_ok "Dashboard: ${BOLD}${dashboard:-http://127.0.0.1:5000}${NC}"
        if check_tcp_port "127.0.0.1" 5000; then
            log_ok "Dashboard :5000 слушает"
        else
            log_warn "Dashboard :5000 не отвечает — запусти ./samurai.sh compute"
        fi
        echo ""
        echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
        echo ""
    fi

    exec python3 -m compute_node.mois_agent "${args[@]}"
}

main "$@"
