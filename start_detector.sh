#!/usr/bin/env bash
# DEPRECATED — этот скрипт стал тонкой обёрткой.
#
# Используй: ./samurai.sh detector
# С опциями: ./samurai.sh detector --pi 192.168.1.50
# GPU-режим:  ./samurai.sh detector --gpu
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "\033[1;33m  [!]\033[0m DEPRECATED: используй \033[1m./samurai.sh detector\033[0m вместо $0" >&2
exec "$SCRIPT_DIR/samurai.sh" detector "$@"
