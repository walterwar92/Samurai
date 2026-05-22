#!/usr/bin/env bash
# scripts/cmds/detector.sh — YOLO/HSV детектор как отдельный процесс
#
# Использует объединённый compute_node/detector.py
# (заменил собой 3 старых детектора в одном CLI).
#
# Использование:
#   samurai detector                       # CPU + HSV fallback (по умолчанию YOLO если есть)
#   samurai detector --pi 192.168.1.50
#   samurai detector --gpu                 # GPU YOLO
#   samurai detector --gpu --model yolo11m.pt --conf 0.4
#   samurai detector --calibrate red       # HSV калибратор для подстройки

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

main() {
    local pi_ip_arg=""
    local gpu=false
    local backend=""              # auto: yolo если есть, hsv fallback
    local model="yolo11n.pt"
    local conf="0.40"
    local device=""               # auto: cuda если --gpu, иначе cpu
    local no_annotated=false
    local quality=70
    local install_deps=false
    local calibrate=""            # пустой → не калибратор
    local image=""                # для калибратора по фото

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pi)           pi_ip_arg="$2"; shift 2 ;;
            --gpu)          gpu=true; shift ;;
            --backend)      backend="$2"; shift 2 ;;
            --model)        model="$2"; shift 2 ;;
            --conf)         conf="$2"; shift 2 ;;
            --device)       device="$2"; shift 2 ;;
            --no-annotated) no_annotated=true; shift ;;
            --quality)      quality="$2"; shift 2 ;;
            --install)      install_deps=true; shift ;;
            --calibrate)
                # Опциональный позиционный аргумент (red/orange/...)
                if [[ -n "${2:-}" && ! "${2:-}" =~ ^- ]]; then
                    calibrate="$2"; shift 2
                else
                    calibrate="red"; shift
                fi
                ;;
            --image)        image="$2"; shift 2 ;;
            -h|--help)
                cat <<EOF
Использование: samurai detector [опции]

Запускает объединённый compute_node/detector.py (заменил yolo_detector_node,
yolo_detector_mqtt, object_detector_node).

Базовые опции:
  --pi IP            IP Raspberry Pi (без — mDNS auto-discovery)
  --gpu              GPU режим (--device cuda + --backend yolo)
  --backend yolo|hsv Принудительно выбрать backend (default: auto)
  --model PATH       YOLO модель (default: yolo11n.pt)
  --conf F           Порог уверенности (default: 0.40)
  --device cuda|cpu  Принудительно выбрать device

Публикация:
  --no-annotated     Не публиковать аннотированные кадры
  --quality N        JPEG quality аннотации (default: 70)

HSV калибратор:
  --calibrate [COLOUR]   Запустить калибратор (red если без аргумента)
  --image FILE           Использовать статичное фото вместо live с робота

Прочее:
  --install          Установить зависимости (ultralytics, onnxruntime-gpu)
  -h, --help         Эта справка

Примеры:
  samurai detector                              # auto: live с робота, YOLO/HSV
  samurai detector --pi 192.168.1.50            # ручной IP
  samurai detector --gpu --model yolo11m.pt     # GPU + большая модель
  samurai detector --calibrate yellow           # калибровка жёлтого
  samurai detector --calibrate red --image ball.jpg  # из файла
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai detector --help)" ;;
        esac
    done

    # ── Установка зависимостей (опционально) ────────────────────────────
    if $install_deps; then
        log_step "Установка GPU-зависимостей"
        pip install --upgrade ultralytics paho-mqtt opencv-python-headless numpy PyYAML
        if $gpu || [[ "$device" == "cuda" ]]; then
            pip install --upgrade onnxruntime-gpu && log_ok "onnxruntime-gpu установлен"
        else
            pip install --upgrade onnxruntime && log_ok "onnxruntime (CPU) установлен"
        fi
        log_ok "Зависимости установлены"
    fi

    # ── Калибратор: специальный ветка (не требует discover_pi) ──────────
    if [[ -n "$calibrate" ]]; then
        print_banner "H S V   C A L I B R A T O R" "Подстройка цветов под освещение"
        check_python 9
        check_pip_packages "cv2:opencv-python" "numpy:numpy" "yaml:PyYAML"

        if [[ -z "$image" ]]; then
            check_pip_packages "paho.mqtt.client:paho-mqtt"
            log_step "Поиск Pi (для live-источника кадров)"
            local pi_ip; pi_ip=$(discover_pi_interactive "$pi_ip_arg")
            log_ok "Pi: $pi_ip"
            if load_mqtt_creds; then
                log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC}"
            fi
            cd "$SAMURAI_ROOT"
            exec python3 compute_node/detector.py \
                --calibrate "$calibrate" \
                --broker "$pi_ip" --port 1883 --robot-id robot1
        else
            log_step "Калибровка по статичному фото"
            log_ok "Image: $image"
            cd "$SAMURAI_ROOT"
            exec python3 compute_node/detector.py \
                --calibrate "$calibrate" --image "$image"
        fi
    fi

    # ── Обычный детектор: lock + discover Pi ────────────────────────────
    acquire_lock detector

    log_step "Поиск Raspberry Pi"
    local pi_ip
    pi_ip=$(discover_pi_interactive "$pi_ip_arg")

    # Резолв backend / device по флагам
    if [[ -z "$backend" ]]; then
        backend="yolo"  # detector.py сам fallback на hsv если ultralytics нет
    fi
    if [[ -z "$device" ]]; then
        device=$($gpu && echo cuda || echo cpu)
    fi

    if $gpu; then
        print_banner "G P U   Y O L O   D E T E C T O R" "Standalone MQTT - CUDA"
    else
        print_banner "O B J E C T   D E T E C T O R" "MQTT  -  ${backend} on ${device}"
    fi

    # Workarounds для Windows + Git Bash с кириллицей в пути юзера.
    # Должны быть ДО import-проверок!
    #
    # 1) ultralytics: Path.home() ломается с "Could not determine home
    #    directory" если HOME содержит non-ASCII. Указываем ASCII-only
    #    YOLO_CONFIG_DIR — ultralytics использует его вместо ~/.config.
    if [[ -z "${YOLO_CONFIG_DIR:-}" ]]; then
        export YOLO_CONFIG_DIR="C:/yolo_cache"
        mkdir -p "$YOLO_CONFIG_DIR" 2>/dev/null || true
    fi
    # 2) torch._inductor.cache_dir_utils зовёт getpass.getuser(), та
    #    смотрит USERNAME/USER/LOGNAME/LNAME env-vars. В Git Bash они
    #    бывают не выставлены, на Windows нет модуля pwd → OSError
    #    "No username set in the environment". Ставим ASCII-юзера.
    if [[ -z "${USERNAME:-}${USER:-}${LOGNAME:-}${LNAME:-}" ]]; then
        export USERNAME="samurai"
    fi

    check_python 9
    check_pip_packages \
        "paho.mqtt.client:paho-mqtt" \
        "cv2:opencv-python" \
        "numpy:numpy" \
        "yaml:PyYAML" \
        "av:av"
    if [[ "$backend" == "yolo" ]]; then
        check_pip_optional ultralytics "ultralytics (YOLO)"
    fi

    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    if ! check_tcp_port "$pi_ip" 1883; then
        log_warn "Порт 1883 не отвечает — убедись что mosquitto на Pi"
    fi

    if load_mqtt_creds; then
        log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC}"
    fi

    log_step "Запуск детектора"
    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Pi MQTT: ${CYAN}${pi_ip}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  Backend: ${GREEN}${backend}${NC} (${device})"
    [[ "$backend" == "yolo" ]] && \
    echo -e "  ${BOLD}│${NC}  Model:   ${GREEN}${model}${NC} conf=${conf}"
    echo -e "  ${BOLD}│${NC}  Топики:  ${GREEN}samurai/robot1/{ball_detection,detections,yolo/annotated}${NC}"
    echo -e "  ${BOLD}└──────────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    # Собираем аргументы для detector.py.
    # --source h264: с 2026-04 (#9) Pi camera_node шлёт H.264 по TCP,
    # JPEG в MQTT больше не публикует. Detector через H264TCPFrameSource
    # подключается к MQTT discovery → TCP H.264 → PyAV → numpy кадры.
    local args=(
        --source h264
        --backend "$backend"
        --device "$device"
        --model "$model"
        --conf "$conf"
        --quality "$quality"
        --broker "$pi_ip"
        --port 1883
        --robot-id robot1
    )
    $no_annotated && args+=(--no-annotated)

    cd "$SAMURAI_ROOT"
    exec python3 compute_node/detector.py "${args[@]}"
}

main "$@"
