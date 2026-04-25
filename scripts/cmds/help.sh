#!/usr/bin/env bash
# scripts/cmds/help.sh — справка по samurai CLI

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"

cat <<EOF
${BOLD}${CYAN}samurai${NC} — единый CLI запуска компонентов робота Samurai (v${SAMURAI_CLI_VERSION})

${BOLD}Использование:${NC}
  ./samurai.sh ${GREEN}<команда>${NC} [опции]

${BOLD}Команды робота:${NC}
  ${GREEN}robot${NC}         Запустить Pi-сторону (Pure Python + MQTT)
                  --legacy   Старый Docker+ROS2 путь (медленно)
  ${GREEN}sim${NC}           Симулятор без железа (Flask :5000)
                  --port N   Альтернативный порт

${BOLD}Команды ноутбука:${NC}
  ${GREEN}compute${NC}       Compute-стек (Docker + ROS2 + SLAM + Nav2 + Dashboard :5000)
                  --pi IP            Явный IP Pi (вместо mDNS)
                  --hotspot          Unicast DDS для мобильного хотспота
                  --rebuild          Пересобрать Docker + ROS2 workspace
                  --rebuild-image    Только Docker
                  --rebuild-ws       Только workspace
                  --remote-yolo      YOLO на отдельном GPU-ноуте
                  --no-samcan        Не запускать Samcan bridge
                  --samcan-port P    Конкретный COM/tty
                  --no-frontend-build  Не пересобирать React
                  --rebuild-frontend   Принудительно пересобрать React
  ${GREEN}detector${NC}      YOLO детектор (отдельный процесс)
                  --gpu              Standalone GPU-ноут (yolo_detector_mqtt)
                  --pi IP            IP MQTT-брокера (Pi)
                  --model PATH       Модель YOLO
                  --conf F           Порог уверенности
                  --device cuda|cpu
  ${GREEN}bridge${NC}        Samcan USB bridge (Arduino Uno → FastAPI :5005)
                  [PORT]             COM3 / /dev/ttyUSB0 (без — auto)
  ${GREEN}build-cpp${NC}     Кросс-компиляция C++ нод для Pi (arm64)
                  [pi@host]          Деплой на Pi через scp

${BOLD}Управление:${NC}
  ${GREEN}status${NC}        Показать запущенные компоненты + системные службы
  ${GREEN}stop${NC} [target]  Остановить компоненты (без аргумента = все)
  ${GREEN}help${NC}          Эта справка

${BOLD}Примеры:${NC}
  ./samurai.sh robot                    # на Pi
  ./samurai.sh sim                      # на ноуте, без железа
  ./samurai.sh compute                  # на ноуте, реальный робот
  ./samurai.sh compute --pi 192.168.1.50 --hotspot
  ./samurai.sh bridge                   # auto-detect Arduino
  ./samurai.sh status                   # что работает?
  ./samurai.sh stop                     # остановить всё

${BOLD}Автозапуск (production):${NC}
  ${YELLOW}sudo ./scripts/systemd/install.sh${NC}      # установить unit-файлы
  ${YELLOW}sudo systemctl enable --now samurai-robot${NC}

${BOLD}Логи и состояние:${NC}
  ${SAMURAI_LOG_DIR:-~/.samurai/logs}/      Логи через systemd: journalctl -u samurai-robot
  ${SAMURAI_LOCK_DIR:-~/.samurai/locks}/    Lock-файлы (PID активных компонентов)

EOF
