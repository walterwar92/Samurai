#!/usr/bin/env bash
# =============================================================================
# start_gpu_laptop.sh — Запуск GPU-ноутбука для YOLO-детекции
#
# GPU-ноутбук работает как отдельный MQTT-клиент:
#   - Получает кадры камеры от Pi через MQTT
#   - Запускает YOLO-детекцию на GPU (CUDA)
#   - Публикует результаты обратно в MQTT
#
# НЕ требует: Docker, ROS2, SLAM, Nav2
# Требует: Python 3.10+, ultralytics, paho-mqtt, opencv-python, onnxruntime-gpu
#
# Все 3 устройства подключены к WiFi-сети робота:
#   Pi (broker)  ←→  GPU-ноутбук (YOLO)
#                ←→  CPU-ноутбук (SLAM/Nav2/dashboard)
#
# Использование:
#   chmod +x start_gpu_laptop.sh
#   ./start_gpu_laptop.sh                            # авто-обнаружение Pi
#   ./start_gpu_laptop.sh --broker 192.168.1.100     # указать IP вручную
#   ./start_gpu_laptop.sh --model yolo11m.pt         # использовать другую модель
#   ./start_gpu_laptop.sh --device cpu               # без GPU (для теста)
# =============================================================================

set -euo pipefail

# ── Цвета ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log_ok()   { echo -e "${GREEN}  [✓]${NC} $*"; }
log_warn() { echo -e "${YELLOW}  [!]${NC} $*"; }
log_err()  { echo -e "${RED}  [✗]${NC} $*"; }
log_info() { echo -e "${BLUE}  [→]${NC} $*"; }
log_step() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }
die()      { log_err "$*"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTOR_SCRIPT="$SCRIPT_DIR/compute_node/yolo_detector_mqtt.py"

# ── Значения по умолчанию ───────────────────────────────────────────────────
BROKER_IP=""
MQTT_PORT=1883
ROBOT_ID="robot1"
MODEL="yolo11n.pt"
CONFIDENCE=0.40
DEVICE="cuda"
NO_ANNOTATED=false
QUALITY=70
INSTALL_DEPS=false

# ── Аргументы командной строки ──────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --broker)       BROKER_IP="$2"; shift 2 ;;
        --port)         MQTT_PORT="$2"; shift 2 ;;
        --robot-id)     ROBOT_ID="$2"; shift 2 ;;
        --model)        MODEL="$2"; shift 2 ;;
        --conf)         CONFIDENCE="$2"; shift 2 ;;
        --device)       DEVICE="$2"; shift 2 ;;
        --no-annotated) NO_ANNOTATED=true; shift ;;
        --quality)      QUALITY="$2"; shift 2 ;;
        --python)       PYTHON_OVERRIDE="$2"; shift 2 ;;
        --install)      INSTALL_DEPS=true; shift ;;
        -h|--help)
            echo "Использование: $0 [опции]"
            echo ""
            echo "  --broker IP       IP MQTT-брокера (Pi). По умолчанию: авто-обнаружение"
            echo "  --port PORT       Порт MQTT (по умолчанию: 1883)"
            echo "  --robot-id ID     ID робота (по умолчанию: robot1)"
            echo "  --model PATH      Модель YOLO (по умолчанию: yolo11n.pt)"
            echo "  --conf FLOAT      Порог уверенности (по умолчанию: 0.40)"
            echo "  --device DEVICE   Устройство: cuda или cpu (по умолчанию: cuda)"
            echo "  --no-annotated    Не публиковать аннотированные кадры"
            echo "  --quality INT     Качество JPEG аннотации (по умолчанию: 70)"
            echo "  --python PATH     Путь к Python (если python3 не тот)"
            echo "  --install         Установить Python-зависимости"
            echo "  -h, --help        Показать эту справку"
            exit 0
            ;;
        *) die "Неизвестный аргумент: $1" ;;
    esac
done

# ── Баннер ──────────────────────────────────────────────────────────────────
echo -e "${BOLD}${CYAN}"
echo "╔════════════════════════════════════════════════════╗"
echo "║       🎯  SAMURAI GPU YOLO DETECTOR               ║"
echo "║       Standalone MQTT • CUDA accelerated          ║"
echo "╚════════════════════════════════════════════════════╝"
echo -e "${NC}"

# ── Установка зависимостей (если --install) ─────────────────────────────────
if $INSTALL_DEPS; then
    log_step "Установка Python-зависимостей"
    pip install --upgrade \
        ultralytics \
        paho-mqtt \
        opencv-python-headless \
        numpy \
        PyYAML
    if [ "$DEVICE" = "cuda" ]; then
        pip install --upgrade onnxruntime-gpu
        log_ok "onnxruntime-gpu установлен"
    else
        pip install --upgrade onnxruntime
        log_ok "onnxruntime (CPU) установлен"
    fi
    log_ok "Зависимости установлены"
fi

# ── Проверка Python ─────────────────────────────────────────────────────────
log_step "Проверка окружения"

if [ -n "${PYTHON_OVERRIDE:-}" ]; then
    PYTHON="$PYTHON_OVERRIDE"
else
    # Предпочитаем python (Windows/conda) → python3 (Linux)
    PYTHON=$(command -v python || command -v python3 || true)
fi
if [ -z "$PYTHON" ]; then
    die "Python не найден! Установите Python 3.10+"
fi
log_ok "Python: $($PYTHON --version)"

# Проверка ключевых модулей
$PYTHON -c "import ultralytics" 2>/dev/null || die "ultralytics не установлен. Запустите: $0 --install"
$PYTHON -c "import paho.mqtt.client" 2>/dev/null || die "paho-mqtt не установлен. Запустите: $0 --install"
$PYTHON -c "import cv2" 2>/dev/null || die "opencv не установлен. Запустите: $0 --install"
log_ok "Python-модули: ultralytics, paho-mqtt, opencv — ОК"

# Проверка CUDA
if [ "$DEVICE" = "cuda" ]; then
    if $PYTHON -c "import onnxruntime as ort; assert 'CUDAExecutionProvider' in ort.get_available_providers()" 2>/dev/null; then
        log_ok "CUDA: доступен через onnxruntime-gpu"
    elif command -v nvidia-smi &>/dev/null; then
        GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
        log_warn "GPU найден ($GPU_NAME), но onnxruntime-gpu может быть не установлен"
        log_info "Для GPU-ускорения: pip install onnxruntime-gpu"
    else
        log_warn "CUDA недоступен — YOLO будет работать на CPU"
        log_info "Для GPU: установите NVIDIA драйверы + pip install onnxruntime-gpu"
    fi
fi

# ── Проверка скрипта детектора ──────────────────────────────────────────────
if [ ! -f "$DETECTOR_SCRIPT" ]; then
    die "Скрипт не найден: $DETECTOR_SCRIPT"
fi
log_ok "Детектор: $DETECTOR_SCRIPT"

# ── Обнаружение Pi (MQTT broker) ───────────────────────────────────────────
log_step "Подключение к MQTT-брокеру"

if [ -z "$BROKER_IP" ]; then
    # Попробовать mDNS
    if command -v avahi-resolve &>/dev/null; then
        BROKER_IP=$(avahi-resolve -n raspberrypi.local 2>/dev/null | awk '{print $2}' | head -1) || true
    fi
    if [ -z "$BROKER_IP" ] && command -v getent &>/dev/null; then
        BROKER_IP=$(getent hosts raspberrypi.local 2>/dev/null | awk '{print $1}' | head -1) || true
    fi
    # Попробовать стандартные IP
    if [ -z "$BROKER_IP" ]; then
        for ip in 192.168.1.100 192.168.4.1 192.168.43.1 10.42.0.1; do
            if timeout 1 bash -c "echo >/dev/tcp/$ip/$MQTT_PORT" 2>/dev/null; then
                BROKER_IP="$ip"
                break
            fi
        done
    fi
    if [ -z "$BROKER_IP" ]; then
        die "Не удалось найти MQTT-брокер. Укажите вручную: $0 --broker <IP>"
    fi
    log_ok "Pi найден: $BROKER_IP (авто)"
else
    log_ok "Pi (ручной): $BROKER_IP"
fi

# Проверка подключения к MQTT порту
if timeout 2 bash -c "echo >/dev/tcp/$BROKER_IP/$MQTT_PORT" 2>/dev/null; then
    log_ok "MQTT порт $MQTT_PORT доступен"
else
    die "Не удалось подключиться к $BROKER_IP:$MQTT_PORT — проверьте сеть и Mosquitto"
fi

# ── Запуск ──────────────────────────────────────────────────────────────────
log_step "Запуск YOLO-детектора"

echo -e "${BOLD}Параметры:${NC}"
echo -e "  Broker:     ${GREEN}$BROKER_IP:$MQTT_PORT${NC}"
echo -e "  Robot ID:   ${GREEN}$ROBOT_ID${NC}"
echo -e "  Model:      ${GREEN}$MODEL${NC}"
echo -e "  Confidence: ${GREEN}$CONFIDENCE${NC}"
echo -e "  Device:     ${GREEN}$DEVICE${NC}"
echo -e "  Annotated:  ${GREEN}$(if $NO_ANNOTATED; then echo 'off'; else echo 'on (quality='$QUALITY')'; fi)${NC}"
echo ""

# Собираем аргументы
ARGS=(
    --broker "$BROKER_IP"
    --port "$MQTT_PORT"
    --robot-id "$ROBOT_ID"
    --model "$MODEL"
    --conf "$CONFIDENCE"
    --device "$DEVICE"
    --quality "$QUALITY"
)
if $NO_ANNOTATED; then
    ARGS+=(--no-annotated)
fi

# Экспортируем для совместимости
export MQTT_BROKER="$BROKER_IP"
export MQTT_PORT="$MQTT_PORT"
export ROBOT_ID="$ROBOT_ID"

log_info "Для остановки нажмите Ctrl+C"
echo ""

exec $PYTHON "$DETECTOR_SCRIPT" "${ARGS[@]}"
