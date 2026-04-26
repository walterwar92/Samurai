#!/usr/bin/env bash
# scripts/cmds/robot.sh — запуск Pi-стороны (samurai robot)
#
# По умолчанию: чистый Python + MQTT (без Docker/ROS2). Это рекомендуемый путь.
# С флагом --legacy: старый Docker+ROS2 (Dockerfile.robot, медленнее на Pi).
#
# Использование:
#   samurai robot
#   samurai robot --legacy
#   samurai robot --no-mqtt-restart   # не пересоздавать конфиг mosquitto

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

# ── Pure Python + MQTT (default, рекомендуется) ─────────────────────────────
cmd_robot_mqtt() {
    local no_mqtt_restart="$1"

    check_python 9
    check_pip_packages \
        "paho.mqtt.client:paho-mqtt" \
        "yaml:PyYAML" \
        "smbus2:smbus2" \
        "gpiozero:gpiozero"

    check_pip_optional adafruit_pca9685 "adafruit-pca9685"
    check_pip_optional picamera2        "picamera2"

    check_i2c || true

    # Загружаем MQTT credentials (если есть файл ~/.samurai/mqtt.passwd).
    # Это экспортирует SAMURAI_MQTT_USER/PASS — pi_nodes.mqtt_node их подхватит.
    if load_mqtt_creds; then
        log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC} (из ~/.samurai/mqtt.passwd)"
        if [[ "$no_mqtt_restart" != "true" ]]; then
            check_mosquitto --auth "$SAMURAI_MQTT_USER" "$SAMURAI_MQTT_PASS"
        fi
    else
        log_info "MQTT auth: anonymous (нет ~/.samurai/mqtt.passwd)"
        if [[ "$no_mqtt_restart" != "true" ]]; then
            check_mosquitto
        fi
    fi
    check_avahi || true

    log_step "Запуск MQTT нод"
    local my_ip; my_ip=$(get_my_ip)
    echo ""
    log_ok "Broker:   ${BOLD}${my_ip}:1883${NC}"
    log_ok "Robot ID: ${BOLD}robot1${NC}"
    echo ""
    echo -e "${YELLOW}  ── Запуск Python нод (Ctrl+C для остановки) ──${NC}"
    echo ""

    cd "$SAMURAI_ROOT"
    # ENV vars SAMURAI_MQTT_USER/PASS унаследуются. robot_launcher их подхватит
    # через config_loader.get_mqtt_credentials().
    exec python3 -m pi_nodes.robot_launcher \
        --broker "$my_ip" \
        --port 1883 \
        --robot-id robot1
}

# ── LEGACY: Docker + ROS2 ───────────────────────────────────────────────────
cmd_robot_legacy() {
    check_docker

    local image="samurai-robot"
    local dockerfile="$SAMURAI_ROOT/Dockerfile.robot"
    local container="samurai_robot"
    local ros_domain_id=42

    [[ -f "$dockerfile" ]] || die "Dockerfile.robot не найден: $dockerfile"

    log_step "Docker образ '$image'"
    if ! docker image inspect "$image" &>/dev/null; then
        log_warn "Образ не найден — сборка 15-25 мин..."
        docker build --platform linux/arm64 -f "$dockerfile" -t "$image" "$SAMURAI_ROOT" \
            || die "docker build провалился"
    fi
    log_ok "Образ '$image' готов"

    check_i2c || true
    check_camera_pi || true
    check_mosquitto
    check_avahi || true
    check_ros_workspace "$image" --platform linux/arm64

    log_step "Сеть"
    local my_ip; my_ip=$(get_my_ip)
    echo ""
    echo -e "  ${BOLD}1)${NC} LAN/WiFi (multicast DDS — стандарт)"
    echo -e "  ${BOLD}2)${NC} Хотспот (unicast DDS — нужен IP ноута)"
    read -rp "  Режим [1/2]: " net_mode
    local peer_ip="" mqtt_broker="$my_ip"
    if [[ "$net_mode" == "2" ]]; then
        read -rp "  IP ноутбука: " peer_ip
        [[ "$peer_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "Неверный IP"
    fi

    log_step "Запуск robot_bringup"
    docker ps -q --filter "name=$container" | grep -q . && docker stop "$container" &>/dev/null

    local pic2_vol="" libcam_vol=""
    [[ -d "/usr/lib/python3/dist-packages/picamera2" ]] \
        && pic2_vol="-v /usr/lib/python3/dist-packages/picamera2:/opt/picamera2:ro"
    [[ -d "/usr/lib/python3/dist-packages/libcamera" ]] \
        && libcam_vol="-v /usr/lib/python3/dist-packages/libcamera:/opt/libcamera:ro"

    local launch_args="mqtt_broker:=$mqtt_broker"
    [[ -n "$peer_ip" ]] && launch_args="$launch_args peer_ip:=$peer_ip"

    # shellcheck disable=SC2086
    exec docker run --rm \
        --name "$container" \
        --platform linux/arm64 \
        --privileged --network host \
        -v /dev:/dev \
        -v /run/udev:/run/udev:ro \
        $pic2_vol $libcam_vol \
        -v "$SAMURAI_ROOT:/root/Samurai" \
        -e ROS_DOMAIN_ID="$ros_domain_id" \
        "$image" \
        bash -c "
            source /opt/ros/humble/setup.bash
            source /root/Samurai/ros_ws/install/setup.bash
            exec ros2 launch robot_pkg robot_bringup.launch.py $launch_args
        "
}

main() {
    local legacy=false
    local no_mqtt_restart=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --legacy)            legacy=true; shift ;;
            --no-mqtt-restart)   no_mqtt_restart=true; shift ;;
            -h|--help)
                cat <<EOF
Использование: samurai robot [опции]

Запускает Pi-сторону Samurai.

Опции:
  --legacy            Через Docker + ROS2 (Dockerfile.robot). DEPRECATED — медленно на Pi.
  --no-mqtt-restart   Не пересоздавать /etc/mosquitto/conf.d/samurai.conf
  -h, --help          Эта справка

По умолчанию запускается pi_nodes.robot_launcher (чистый Python + MQTT).
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai robot --help)" ;;
        esac
    done

    acquire_lock robot

    if $legacy; then
        print_banner "S A M U R A I   R O B O T" "Pi  -  Docker + ROS2 (LEGACY)"
        log_warn "Legacy режим — рекомендуется обычный 'samurai robot' (Pure Python + MQTT)"
        cmd_robot_legacy
    else
        print_banner "S A M U R A I   R O B O T" "Pi  -  Pure Python + MQTT"
        cmd_robot_mqtt "$no_mqtt_restart"
    fi
}

main "$@"
