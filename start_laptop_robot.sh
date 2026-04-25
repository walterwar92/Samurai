#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh compute [опции]
#
# Все старые флаги поддерживаются:
#   --pi IP, --hotspot, --rebuild, --rebuild-image, --rebuild-ws,
#   --remote-yolo, --no-samcan, --samcan-port P,
#   --no-frontend-build, --rebuild-frontend
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh compute\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" compute "$@"
