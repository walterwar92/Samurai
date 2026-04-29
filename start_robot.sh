#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh robot --legacy
# Или (рекомендуется): ./samurai.sh robot   (Pure Python + MQTT, быстрее)
#
# Старый Docker+ROS2 путь медленнее на Pi и больше не рекомендуется.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh robot --legacy\033[0m вместо $0" >&2
echo -e "\033[0;36m  [→]\033[0m Совет: попробуй \033[1m./samurai.sh robot\033[0m (Pure Python + MQTT, без Docker)" >&2
exec "$SCRIPT_DIR/samurai.sh" robot --legacy "$@"
