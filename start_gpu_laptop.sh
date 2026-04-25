#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh detector --gpu
#
# Все старые флаги поддерживаются:
#   --broker IP (теперь --pi IP), --port, --robot-id, --model,
#   --conf, --device, --no-annotated, --quality, --install
#
# Если использовался --broker — поменяй на --pi для совместимости с CLI.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh detector --gpu\033[0m вместо $0" >&2

# Конвертируем --broker → --pi (для обратной совместимости)
args=()
for a in "$@"; do
    if [[ "$a" == "--broker" ]]; then
        args+=(--pi)
    else
        args+=("$a")
    fi
done
exec "$SCRIPT_DIR/samurai.sh" detector --gpu "${args[@]}"
