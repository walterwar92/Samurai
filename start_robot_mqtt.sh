#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh robot
# Или с флагами: ./samurai.sh robot --no-mqtt-restart
#
# Старый скрипт сохранён для обратной совместимости. Будет удалён в будущем.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh robot\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" robot "$@"
