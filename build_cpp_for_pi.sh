#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh build-cpp                          # только сборка
#            ./samurai.sh build-cpp pi@raspberrypi.local     # сборка + scp
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh build-cpp\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" build-cpp "$@"
