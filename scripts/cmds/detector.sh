#!/usr/bin/env bash
# scripts/cmds/detector.sh — YOLO/HSV детектор как отдельный процесс
#
# Два режима:
#   1. CPU + HSV fallback (по умолчанию) → compute_node/object_detector_node.py
#   2. GPU YOLO (флаг --gpu) → compute_node/yolo_detector_mqtt.py
#
# Использование:
#   samurai detector                       # CPU/HSV, авто-Pi
#   samurai detector --pi 192.168.1.50
#   samurai detector --gpu                 # GPU YOLO
#   samurai detector --gpu --model yolo11m.pt --conf 0.4
#   samurai detector --gpu --device cpu    # GPU-скрипт но без CUDA

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/discover.sh
source "$LIB_DIR/discover.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

# ── CPU/HSV режим ───────────────────────────────────────────────────────────
run_cpu_detector() {
    local pi_ip="$1"
    print_banner "O B J E C T   D E T E C T O R" "CPU + HSV  ->  MQTT"

    check_python 9
    check_pip_packages \
        "paho.mqtt.client:paho-mqtt" \
        "cv2:opencv-python" \
        "numpy:numpy"
    check_pip_optional ultralytics "ultralytics (YOLO)"

    log_step "Pi MQTT broker"
    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    if ! check_tcp_port "$pi_ip" 1883; then
        log_warn "Порт 1883 не отвечает — убедись что mosquitto на Pi"
    fi

    log_step "Запуск детектора"
    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Pi MQTT: ${CYAN}${pi_ip}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  Топики:  ${GREEN}samurai/robot1/camera${NC} → детекция"
    echo -e "  ${BOLD}│${NC}           ${GREEN}samurai/robot1/detections${NC} ← результат"
    echo -e "  ${BOLD}│${NC}           ${GREEN}samurai/robot1/ball_detection${NC} ← FSM"
    echo -e "  ${BOLD}└──────────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    cd "$SAMURAI_ROOT"
    exec python3 compute_node/object_detector_node.py \
        --broker "$pi_ip" \
        --port 1883 \
        --robot-id robot1
}

# ── GPU YOLO режим ──────────────────────────────────────────────────────────
run_gpu_detector() {
    local pi_ip="$1" model="$2" conf="$3" device="$4" no_annotated="$5" quality="$6" install_deps="$7"

    print_banner "G P U   Y O L O   D E T E C T O R" "Standalone MQTT  -  CUDA"

    if [[ "$install_deps" == "true" ]]; then
        log_step "Установка GPU-зависимостей"
        pip install --upgrade ultralytics paho-mqtt opencv-python-headless numpy PyYAML
        if [[ "$device" == "cuda" ]]; then
            pip install --upgrade onnxruntime-gpu && log_ok "onnxruntime-gpu установлен"
        else
            pip install --upgrade onnxruntime && log_ok "onnxruntime (CPU) установлен"
        fi
    fi

    log_step "Окружение"
    local PY; PY=$(detect_python) || die "Python не найден"
    log_ok "Python: $($PY --version)"

    "$PY" -c "import ultralytics" 2>/dev/null \
        || die "ultralytics не установлен. Запустите: samurai detector --gpu --install"
    "$PY" -c "import paho.mqtt.client" 2>/dev/null \
        || die "paho-mqtt не установлен. Запустите: samurai detector --gpu --install"
    "$PY" -c "import cv2" 2>/dev/null \
        || die "opencv не установлен. Запустите: samurai detector --gpu --install"
    log_ok "ultralytics, paho-mqtt, opencv — OK"

    if [[ "$device" == "cuda" ]]; then
        if "$PY" -c "import onnxruntime as ort; assert 'CUDAExecutionProvider' in ort.get_available_providers()" 2>/dev/null; then
            log_ok "CUDA доступен через onnxruntime-gpu"
        elif command -v nvidia-smi &>/dev/null; then
            local gpu_name
            gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
            log_warn "GPU найден ($gpu_name), но onnxruntime-gpu может быть не установлен"
            log_info "samurai detector --gpu --install для GPU-ускорения"
        else
            log_warn "CUDA недоступен — YOLO на CPU"
        fi
    fi

    local script="$SAMURAI_ROOT/compute_node/yolo_detector_mqtt.py"
    [[ -f "$script" ]] || die "Скрипт не найден: $script"

    log_step "MQTT broker"
    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    check_tcp_port "$pi_ip" 1883 || die "Не удалось подключиться к $pi_ip:1883"

    log_step "Запуск YOLO детектора"
    echo -e "${BOLD}Параметры:${NC}"
    echo -e "  Broker:     ${GREEN}$pi_ip:1883${NC}"
    echo -e "  Robot ID:   ${GREEN}robot1${NC}"
    echo -e "  Model:      ${GREEN}$model${NC}"
    echo -e "  Confidence: ${GREEN}$conf${NC}"
    echo -e "  Device:     ${GREEN}$device${NC}"
    if [[ "$no_annotated" == "true" ]]; then
        echo -e "  Annotated:  ${GREEN}off${NC}"
    else
        echo -e "  Annotated:  ${GREEN}on (quality=$quality)${NC}"
    fi
    echo ""
    log_info "Ctrl+C для остановки"
    echo ""

    local args=(
        --broker "$pi_ip"
        --port 1883
        --robot-id robot1
        --model "$model"
        --conf "$conf"
        --device "$device"
        --quality "$quality"
    )
    [[ "$no_annotated" == "true" ]] && args+=(--no-annotated)

    export MQTT_BROKER="$pi_ip" MQTT_PORT=1883 ROBOT_ID=robot1
    exec "$PY" "$script" "${args[@]}"
}

main() {
    local pi_ip_arg=""
    local gpu=false
    local model="yolo11n.pt"
    local conf="0.40"
    local device="cuda"
    local no_annotated=false
    local quality=70
    local install_deps=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pi)           pi_ip_arg="$2"; shift 2 ;;
            --gpu)          gpu=true; shift ;;
            --model)        model="$2"; shift 2 ;;
            --conf)         conf="$2"; shift 2 ;;
            --device)       device="$2"; shift 2 ;;
            --no-annotated) no_annotated=true; shift ;;
            --quality)      quality="$2"; shift 2 ;;
            --install)      install_deps=true; shift ;;
            -h|--help)
                cat <<EOF
Использование: samurai detector [опции]

Два режима:
  Без --gpu (по умолчанию)  → compute_node/object_detector_node.py
                              CPU + HSV fallback, лёгкий
  С --gpu                   → compute_node/yolo_detector_mqtt.py
                              YOLO на CUDA, для отдельного GPU-ноута

Опции:
  --pi IP            IP Raspberry Pi (без аргумента — mDNS)
  --gpu              GPU режим (yolo_detector_mqtt)
  --model PATH       Модель YOLO (default: yolo11n.pt) — только для --gpu
  --conf F           Порог уверенности (default: 0.40) — только для --gpu
  --device cuda|cpu  Устройство (default: cuda) — только для --gpu
  --no-annotated     Не публиковать аннотированные кадры — только для --gpu
  --quality N        JPEG quality аннотации (default: 70) — только для --gpu
  --install          Установить GPU-зависимости (ultralytics, onnxruntime-gpu)
  -h, --help         Эта справка
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai detector --help)" ;;
        esac
    done

    acquire_lock detector

    log_step "Поиск Raspberry Pi"
    local pi_ip
    pi_ip=$(discover_pi_interactive "$pi_ip_arg")

    if $gpu; then
        run_gpu_detector "$pi_ip" "$model" "$conf" "$device" \
                         "$no_annotated" "$quality" "$install_deps"
    else
        run_cpu_detector "$pi_ip"
    fi
}

main "$@"
