#!/usr/bin/env bash
# =============================================================================
# samurai.sh — единый CLI запуска компонентов робота Samurai
#
# Заменяет старые start_*.sh скрипты единым интерфейсом с подкомандами.
# Совместимость: старые скрипты сохранены как тонкие обёртки.
#
# Использование:
#   ./samurai.sh                    # справка
#   ./samurai.sh help
#   ./samurai.sh <команда> [опции]
#
# Команды:
#   robot       Pi-сторона (Pure Python + MQTT [по умолчанию] или Docker+ROS2)
#   sim         Симулятор без железа (Flask :5000)
#   compute     Compute-стек (Docker + ROS2 + SLAM + Nav2 + Dashboard :5000)
#   detector    YOLO детектор (отдельный процесс, CPU/HSV или GPU)
#   planner     A* path planner на ноутбуке (#3 — slam_map → goal → path)
#   bridge      Samcan USB bridge (FastAPI :5005)
#   build-cpp   Кросс-компиляция C++ нод для arm64
#   status      Что запущено
#   stop        Остановить компонент(ы)
#   help        Справка
#
# Подробнее: ./samurai.sh help
# =============================================================================

set -euo pipefail

# Корень проекта — туда, где лежит этот скрипт
SAMURAI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SAMURAI_ROOT

CMDS_DIR="$SAMURAI_ROOT/scripts/cmds"

[[ -d "$CMDS_DIR" ]] || {
    echo "ERROR: scripts/cmds/ не найдена. Этот samurai.sh использует scripts/cmds/<name>.sh." >&2
    echo "Возможно ты переименовал/удалил папку. См. git log scripts/" >&2
    exit 1
}

# Без аргументов — показать help
if [[ $# -eq 0 ]]; then
    exec "$CMDS_DIR/help.sh"
fi

cmd="$1"; shift

case "$cmd" in
    -h|--help|help)
        exec "$CMDS_DIR/help.sh"
        ;;
    -v|--version|version)
        # shellcheck source=scripts/lib/common.sh
        source "$SAMURAI_ROOT/scripts/lib/common.sh"
        echo "samurai CLI v${SAMURAI_CLI_VERSION}"
        ;;
    robot|sim|compute|detector|planner|bridge|status|stop|auth)
        exec "$CMDS_DIR/${cmd}.sh" "$@"
        ;;
    build-cpp)
        # Дефис в имени → подчёркивание в файле
        exec "$CMDS_DIR/build_cpp.sh" "$@"
        ;;
    *)
        echo "ERROR: неизвестная команда '$cmd'" >&2
        echo "" >&2
        echo "Доступные команды: robot, sim, compute, detector, planner, bridge, build-cpp, auth, status, stop, help" >&2
        echo "Запусти: $0 help" >&2
        exit 1
        ;;
esac
