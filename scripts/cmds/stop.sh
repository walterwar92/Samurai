#!/usr/bin/env bash
# scripts/cmds/stop.sh — остановить компонент (или все)
#
# Использование:
#   samurai stop              # остановить все запущенные
#   samurai stop robot
#   samurai stop sim compute  # несколько

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        cat <<EOF
Использование: samurai stop [target...]

Останавливает samurai-компоненты по имени.
Без аргументов — останавливает все запущенные.

Цели: robot, sim, compute, detector, bridge, agent, all

Примеры:
  samurai stop
  samurai stop robot
  samurai stop sim compute
EOF
        exit 0
    fi

    local valid=(robot sim compute detector bridge agent)
    local targets=()
    local t v found

    if [[ $# -eq 0 ]] || [[ "${1:-}" == "all" ]]; then
        targets=("${valid[@]}")
    else
        for t in "$@"; do
            found=false
            for v in "${valid[@]}"; do
                [[ "$t" == "$v" ]] && { found=true; break; }
            done
            $found || die "Неизвестный target: $t (доступно: ${valid[*]}, all)"
            targets+=("$t")
        done
    fi

    print_banner "S A M U R A I   S T O P" "Остановка компонентов"

    for t in "${targets[@]}"; do
        stop_component "$t"
    done

    log_ok "Готово"
}

main "$@"
