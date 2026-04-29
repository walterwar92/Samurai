#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh sim
# С опциями: ./samurai.sh sim --port 5050
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh sim\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" sim "$@"
