#!/usr/bin/env bash
# Vpered USB bridge — мост между frontend и Arduino Uno
#
# Использование:
#   ./start_vpered_bridge.sh                # auto-detect порта
#   ./start_vpered_bridge.sh COM3           # явный COM-порт
#   ./start_vpered_bridge.sh /dev/ttyUSB0   # Linux/macOS

set -e

cd "$(dirname "$0")"

if [ -n "$1" ]; then
    exec python compute_node/vpered_bridge.py --port "$1"
else
    exec python compute_node/vpered_bridge.py --auto
fi
