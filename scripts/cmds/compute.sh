#!/usr/bin/env bash
# scripts/cmds/compute.sh — compute-стек на ноутбуке (Docker + ROS2 + SLAM + Nav2 + Dashboard)
#
# Это ВСЁ что должно крутиться на ноуте при работе с реальным роботом:
#   - MQTT ↔ ROS2 bridge
#   - SLAM Toolbox + Nav2
#   - YOLO detector (или удалённый GPU-ноут через --remote-yolo)
#   - Web Dashboard на :5000 (FastAPI + React фронт)
#   - Samcan USB bridge на :5005 (если подключён Arduino)

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

DOCKER_IMAGE="samurai"
CONTAINER_NAME="samurai_compute"
ROS_DOMAIN_ID=42
SAMCAN_PID=""
SAMCAN_LOG="/tmp/samcan_bridge.log"

# ── Сборка фронта ───────────────────────────────────────────────────────────
build_frontend() {
    local force="$1"
    local fe_dir="$SAMURAI_ROOT/compute_node/frontend"
    local out_index="$SAMURAI_ROOT/compute_node/static/index.html"

    log_step "React frontend"

    [[ -d "$fe_dir" ]] || { log_warn "frontend не найден — пропуск"; return; }

    if ! command -v npm &>/dev/null; then
        log_warn "npm не установлен — фронт не пересобирается"
        if [[ -f "$out_index" ]]; then
            log_info "Используется старый билд: $out_index"
        else
            log_err "Старого билда тоже нет — UI не будет работать"
        fi
        return
    fi

    log_info "vite build (10-30 сек, --no-frontend-build чтобы пропустить)..."
    if [[ ! -d "$fe_dir/node_modules" ]]; then
        log_info "node_modules не найден — npm install (1-3 мин)..."
        (cd "$fe_dir" && npm install --no-audit --no-fund) || {
            log_warn "npm install не удался — пропускаю сборку"; return;
        }
    fi

    rm -rf "$SAMURAI_ROOT/compute_node/static/assets" 2>/dev/null || true

    if (cd "$fe_dir" && npm run build 2>&1 | tail -5); then
        log_ok "Фронт собран → compute_node/static/"
    else
        log_warn "Сборка не удалась — будет использован старый билд"
    fi
}

# ── Samcan USB bridge (фон) ─────────────────────────────────────────────────
start_samcan_bridge() {
    local samcan_port="$1"
    log_step "Samcan USB bridge"

    local PY; PY=$(detect_python) || { log_warn "Python не найден — пропуск Samcan"; return; }

    if ! "$PY" -c "import serial, fastapi, uvicorn" &>/dev/null; then
        log_info "Устанавливаю pyserial/fastapi/uvicorn (auto)..."
        "$PY" -m pip install --quiet --user pyserial 'fastapi>=0.110' 'uvicorn[standard]>=0.27' 'pydantic>=2' 2>&1 | tail -3 \
            || { log_warn "pip install не удался — пропуск Samcan"; return; }
    fi

    if pgrep -f "samcan_bridge.py" &>/dev/null; then
        log_warn "Старый samcan_bridge.py обнаружен — убиваю"
        pkill -f "samcan_bridge.py" || true
        sleep 0.5
    fi

    local args
    if [[ -n "$samcan_port" ]]; then
        args="--port $samcan_port"
        log_info "Указан порт: $samcan_port"
    else
        args="--auto"
        log_info "Авто-поиск Arduino"
    fi

    log_info "Старт samcan_bridge.py → :5005 (лог: $SAMCAN_LOG)"
    # shellcheck disable=SC2086
    nohup "$PY" "$SAMURAI_ROOT/compute_node/samcan_bridge.py" $args > "$SAMCAN_LOG" 2>&1 &
    SAMCAN_PID=$!

    sleep 1.2
    if kill -0 "$SAMCAN_PID" 2>/dev/null; then
        log_ok "Samcan bridge запущен (PID $SAMCAN_PID, http://localhost:5005)"
    else
        log_warn "Bridge упал. Лог:"
        tail -10 "$SAMCAN_LOG" | sed 's/^/    /'
        SAMCAN_PID=""
    fi
}

stop_samcan_bridge() {
    if [[ -n "${SAMCAN_PID:-}" ]] && kill -0 "$SAMCAN_PID" 2>/dev/null; then
        log_info "Останавливаю Samcan bridge (PID $SAMCAN_PID)..."
        kill "$SAMCAN_PID" 2>/dev/null || true
        wait "$SAMCAN_PID" 2>/dev/null || true
    fi
    pkill -f "samcan_bridge.py" 2>/dev/null || true
}

# ── Docker образ ────────────────────────────────────────────────────────────
ensure_docker_image() {
    local rebuild="$1"
    log_step "Docker образ '$DOCKER_IMAGE'"

    if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null; then
        log_warn "Образ не найден — первая сборка (10-20 мин)..."
        docker build -t "$DOCKER_IMAGE" "$SAMURAI_ROOT" || die "docker build провалился"
    elif [[ "$rebuild" == "true" ]]; then
        log_info "Пересборка образа (--rebuild)"
        docker build -t "$DOCKER_IMAGE" "$SAMURAI_ROOT" || die "docker build провалился"
    else
        log_ok "Образ '$DOCKER_IMAGE' существует"
        local df_mtime img_created
        df_mtime=$(stat -c %Y "$SAMURAI_ROOT/Dockerfile" 2>/dev/null || echo 0)
        img_created=$(docker inspect --format='{{.Created}}' "$DOCKER_IMAGE" 2>/dev/null \
            | xargs -I{} date -d "{}" +%s 2>/dev/null || echo 0)
        if [[ "$df_mtime" -gt "$img_created" ]] 2>/dev/null; then
            log_warn "Dockerfile новее образа — пересобираю..."
            docker build -t "$DOCKER_IMAGE" "$SAMURAI_ROOT" || die "docker build провалился"
        fi
    fi
}

# ── Docker запуск compute_bringup ───────────────────────────────────────────
launch_docker() {
    local pi_ip="$1" peer_ip="$2" remote_yolo="$3"

    log_step "Запуск compute нод в Docker"
    docker ps -q --filter "name=$CONTAINER_NAME" | grep -q . && {
        log_warn "Останавливаю предыдущий контейнер $CONTAINER_NAME..."
        docker stop "$CONTAINER_NAME" &>/dev/null || true
    }

    local launch_args="mqtt_broker:=$pi_ip"
    [[ -n "$peer_ip" ]] && launch_args="$launch_args peer_ip:=$peer_ip"
    [[ "$remote_yolo" == "true" ]] && launch_args="$launch_args remote_yolo:=true"

    local my_ip; my_ip=$(get_my_ip)
    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Dashboard:     ${CYAN}http://localhost:5000${NC}"
    echo -e "  ${BOLD}│${NC}  Android:       ${CYAN}http://${my_ip}:5000${NC}"
    echo -e "  ${BOLD}│${NC}  MQTT Broker:   ${GREEN}${pi_ip}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  ROS_DOMAIN_ID: ${GREEN}${ROS_DOMAIN_ID}${NC}"
    [[ -n "$peer_ip" ]] && \
    echo -e "  ${BOLD}│${NC}  Unicast DDS:   ${GREEN}peer_ip=$peer_ip${NC}"
    [[ -n "$SAMCAN_PID" ]] && \
    echo -e "  ${BOLD}│${NC}  Samcan USB:    ${GREEN}http://localhost:5005${NC} (PID $SAMCAN_PID)"
    echo -e "  ${BOLD}└──────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    # Docker Desktop (Win/Mac) vs Linux: --net=host работает только на Linux.
    local docker_net_args samcan_url="http://localhost:5005"
    if is_windows; then
        docker_net_args="-p 5000:5000 --add-host=host.docker.internal:host-gateway"
        samcan_url="http://host.docker.internal:5005"
        log_ok "Windows: -p 5000:5000, Samcan → $samcan_url"
    else
        docker_net_args="--net=host"
    fi

    # MQTT credentials (если файл существует) — пробрасываем в Docker через ENV
    local mqtt_auth_args=()
    if load_mqtt_creds; then
        log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC}"
        mqtt_auth_args=(-e "SAMURAI_MQTT_USER=$SAMURAI_MQTT_USER" \
                        -e "SAMURAI_MQTT_PASS=$SAMURAI_MQTT_PASS")
    else
        log_info "MQTT auth: anonymous (нет ~/.samurai/mqtt.passwd)"
    fi

    # shellcheck disable=SC2086
    MSYS_NO_PATHCONV=1 docker run --rm \
        --name "$CONTAINER_NAME" \
        $docker_net_args \
        --privileged \
        -v "$SAMURAI_ROOT:/root/Samurai" \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e MQTT_BROKER="$pi_ip" \
        -e MQTT_PORT="1883" \
        -e ROBOT_ID="robot1" \
        -e CAMERA_FLIP="-1" \
        -e SAMCAN_BRIDGE_URL="$samcan_url" \
        "${mqtt_auth_args[@]}" \
        "$DOCKER_IMAGE" \
        bash -c "
            source /opt/ros/humble/setup.bash
            cd /root/Samurai/ros_ws
            source install/setup.bash
            exec ros2 launch robot_pkg compute_bringup.launch.py $launch_args
        "
}

# ── Резолв параметров деплоя с приоритетом CLI > env > default ─────────────
resolve_pi_user() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    if [[ -n "${SAMURAI_PI_USER:-}" ]]; then echo "$SAMURAI_PI_USER"; return; fi
    echo "pi"
}

resolve_pi_path() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    if [[ -n "${SAMURAI_PI_PATH:-}" ]]; then echo "$SAMURAI_PI_PATH"; return; fi
    echo "~/Samurai"
}

resolve_ssh_key() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    echo "${SAMURAI_PI_SSH_KEY:-}"
}

# ── Деплой кода на Pi через rsync + рестарт samurai-robot ──────────────────
# Использование: deploy_to_pi <pi_ip> <pi_user> <pi_path> [ssh_key]
# Завершается через die при любой ошибке. На успехе — log_ok.
deploy_to_pi() {
    local pi_ip="$1"
    local pi_user="$2"
    local pi_path="$3"
    local ssh_key="${4:-}"

    local ssh_opts=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new -o BatchMode=yes)
    [[ -n "$ssh_key" ]] && ssh_opts+=(-i "$ssh_key")
    local ssh_target="$pi_user@$pi_ip"

    log_step "Деплой кода на Pi ($ssh_target:$pi_path)"

    # 1. Pre-flight: SSH доступен?
    if ! ssh "${ssh_opts[@]}" "$ssh_target" true 2>/dev/null; then
        die "SSH до $ssh_target недоступен. Проверь: Pi включён, ключ в ~/.ssh/authorized_keys на Pi, sshd работает."
    fi

    # 2. Pre-flight: установлен ли systemd-юнит?
    if ! ssh "${ssh_opts[@]}" "$ssh_target" \
            'systemctl list-unit-files samurai-robot.service --no-pager' 2>/dev/null \
            | grep -q '^samurai-robot\.service'; then
        die "samurai-robot.service не установлен на Pi. Запусти: ssh $ssh_target 'cd Samurai && sudo ./scripts/bootstrap_pi.sh'"
    fi

    # 3. rsync.
    log_info "rsync (.deployignore применён)..."
    # Собираем -e ssh строку с теми же опциями.
    # Array→string flatten: каждый элемент ssh_opts должен быть single-token
    # (без пробелов внутри). Если когда-то понадобится -o с ProxyCommand или
    # подобным — переделать на массив через printf -v.
    local ssh_cmd="ssh ${ssh_opts[*]}"
    if ! rsync -az --delete \
            --exclude-from="$SAMURAI_ROOT/.deployignore" \
            -e "$ssh_cmd" \
            "$SAMURAI_ROOT/" "$ssh_target:$pi_path/"; then
        die "rsync на $ssh_target провалился"
    fi

    # 4. Рестарт сервиса.
    log_info "Рестарт samurai-robot..."
    if ! ssh "${ssh_opts[@]}" "$ssh_target" 'sudo systemctl restart samurai-robot'; then
        die "systemctl restart провалился. NOPASSWD настроен? Запусти на Pi: sudo ./scripts/systemd/install.sh --with-remote-deploy"
    fi

    # 5. Verify — дать пару секунд и проверить is-active.
    sleep 2
    local status
    status=$(ssh "${ssh_opts[@]}" "$ssh_target" \
             'systemctl is-active samurai-robot' 2>/dev/null || true)
    if [[ "$status" != "active" ]]; then
        log_err "samurai-robot не active (status=$status). Логи:"
        ssh "${ssh_opts[@]}" "$ssh_target" \
            'journalctl -u samurai-robot -n 30 --no-pager' >&2 || true
        die "Робот не стартовал — compute-стек не поднимаю"
    fi
    log_ok "samurai-robot запущен на Pi"
}

main() {
    local pi_ip_arg=""
    local hotspot=false
    local rebuild_image=false
    local rebuild_ws=false
    local remote_yolo=false
    local no_samcan=false
    local samcan_port=""
    local no_frontend_build=false
    local force_frontend_build=false
    local no_deploy=false
    local pi_user_arg=""
    local pi_path_arg=""
    local ssh_key_arg=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pi)              pi_ip_arg="$2"; shift 2 ;;
            --no-deploy)       no_deploy=true; shift ;;
            --pi-user)         pi_user_arg="$2"; shift 2 ;;
            --pi-path)         pi_path_arg="$2"; shift 2 ;;
            --ssh-key)         ssh_key_arg="$2"; shift 2 ;;
            --hotspot)         hotspot=true; shift ;;
            --rebuild)         rebuild_image=true; rebuild_ws=true; shift ;;
            --rebuild-image)   rebuild_image=true; shift ;;
            --rebuild-ws)      rebuild_ws=true; shift ;;
            --remote-yolo)     remote_yolo=true; shift ;;
            --no-samcan)       no_samcan=true; shift ;;
            --samcan-port)     samcan_port="$2"; shift 2 ;;
            --no-frontend-build) no_frontend_build=true; shift ;;
            --rebuild-frontend)  force_frontend_build=true; shift ;;
            -h|--help)
                cat <<EOF
Использование: samurai compute [опции]

Запускает compute-стек на ноутбуке (Docker + ROS2 + SLAM + Nav2 + Dashboard).

Опции:
  --pi IP             IP Raspberry Pi (без аргумента — авто mDNS)
  --no-deploy         Не деплоить код на Pi (только поднять compute-стек)
  --pi-user USER      SSH-юзер на Pi (default: pi; env: SAMURAI_PI_USER)
  --pi-path PATH      Путь репо на Pi (default: ~/Samurai; env: SAMURAI_PI_PATH)
  --ssh-key FILE      SSH ключ для аутентификации (env: SAMURAI_PI_SSH_KEY)
  --hotspot           Unicast DDS режим (мобильный хотспот)
  --rebuild           Пересобрать Docker + ROS2 workspace
  --rebuild-image     Только Docker
  --rebuild-ws        Только workspace
  --remote-yolo       YOLO на отдельном GPU-ноуте (через MQTT)
  --no-samcan         Не запускать Samcan USB bridge
  --samcan-port P     Конкретный COM-порт Arduino
  --no-frontend-build Не пересобирать React (использовать существующий билд)
  --rebuild-frontend  Принудительно пересобрать React
  -h, --help          Эта справка
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai compute --help)" ;;
        esac
    done

    acquire_lock compute
    print_banner "S A M U R A I   C O M P U T E" "Ноутбук  -  ROS2 + SLAM + Nav2 + YOLO"

    check_docker
    ensure_docker_image "$rebuild_image"
    if $rebuild_ws; then
        check_ros_workspace "$DOCKER_IMAGE" --rebuild
    else
        check_ros_workspace "$DOCKER_IMAGE"
    fi
    check_avahi || true

    log_step "Поиск Raspberry Pi"
    local pi_ip
    pi_ip=$(discover_pi_interactive "$pi_ip_arg")
    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    if check_tcp_port "$pi_ip" 1883; then
        log_ok "MQTT доступен ($pi_ip:1883)"
    else
        log_warn "MQTT 1883 не отвечает — убедись что mosquitto на Pi запущен"
    fi

    local peer_ip=""
    if $hotspot; then
        peer_ip="$pi_ip"
        log_ok "Хотспот: unicast DDS, peer_ip=$peer_ip"
    fi

    # ── Автодеплой кода на Pi (если задан --pi и не --no-deploy) ──────────────
    if [[ -n "$pi_ip" && "$no_deploy" != "true" ]]; then
        local resolved_user resolved_path resolved_key
        resolved_user=$(resolve_pi_user "$pi_user_arg")
        resolved_path=$(resolve_pi_path "$pi_path_arg")
        resolved_key=$(resolve_ssh_key "$ssh_key_arg")
        deploy_to_pi "$pi_ip" "$resolved_user" "$resolved_path" "$resolved_key"
    elif [[ "$no_deploy" == "true" ]]; then
        log_info "Деплой пропущен (--no-deploy)"
    fi

    if $force_frontend_build; then
        build_frontend "true"
    elif ! $no_frontend_build; then
        build_frontend "false"
    fi

    if ! $no_samcan; then
        # cleanup при выходе
        trap stop_samcan_bridge EXIT INT TERM
        start_samcan_bridge "$samcan_port"
    fi

    launch_docker "$pi_ip" "$peer_ip" "$remote_yolo"
}

# Запускать main только при прямом исполнении, не при source (для тестов).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
