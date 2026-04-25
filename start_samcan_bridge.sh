#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh bridge          # auto-detect
#            ./samurai.sh bridge COM3
#            ./samurai.sh bridge /dev/ttyUSB0
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh bridge\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" bridge "$@"
