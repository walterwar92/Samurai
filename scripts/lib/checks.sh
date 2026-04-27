# shellcheck shell=bash
# scripts/lib/checks.sh
# Универсальные проверки окружения: Python, pip, Docker, I2C, MQTT, mDNS, камера.

[[ -n "${_SAMURAI_CHECKS_LOADED:-}" ]] && return 0
_SAMURAI_CHECKS_LOADED=1

# Зависит от common.sh
[[ -z "${_SAMURAI_COMMON_LOADED:-}" ]] && {
    # shellcheck source=common.sh
    source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
}

# ── Python ──────────────────────────────────────────────────────────────────
# Проверка Python 3 + версия. По умолчанию минимум 3.9.
# Использование: check_python [min_minor]
check_python() {
    local min_minor="${1:-9}"
    log_step "Python"

    if ! command -v python3 &>/dev/null; then
        die "python3 не найден. Установи: sudo apt install python3 python3-pip (Pi) или sudo pacman -S python python-pip (Arch)"
    fi

    local major minor
    major=$(python3 -c 'import sys; print(sys.version_info.major)')
    minor=$(python3 -c 'import sys; print(sys.version_info.minor)')

    if [[ "$major" -lt 3 ]] || { [[ "$major" -eq 3 ]] && [[ "$minor" -lt "$min_minor" ]]; }; then
        die "Требуется Python 3.${min_minor}+. Установлен: $(python3 --version)"
    fi
    log_ok "Python $(python3 --version | cut -d' ' -f2)"
}

# Возвращает имя бинарника python (для Windows/conda иногда python, не python3).
# Использование: PY=$(detect_python)
detect_python() {
    if command -v python3 &>/dev/null; then
        echo python3
    elif command -v python &>/dev/null; then
        echo python
    else
        return 1
    fi
}

# ── pip пакеты ──────────────────────────────────────────────────────────────
# Проверка списка pip-пакетов. Авто-устанавливает отсутствующие.
# Использование: check_pip_packages "import_name:pip_name" "yaml:PyYAML" ...
# При не-интерактивном запуске (systemd) — устанавливает молча.
check_pip_packages() {
    log_step "Python зависимости"
    local missing=()
    local entry import_name pip_name

    for entry in "$@"; do
        import_name="${entry%%:*}"
        pip_name="${entry##*:}"
        if python3 -c "import ${import_name}" &>/dev/null 2>&1; then
            log_ok "$pip_name"
        else
            log_warn "$pip_name — нет"
            missing+=("$pip_name")
        fi
    done

    [[ ${#missing[@]} -eq 0 ]] && { log_ok "Все зависимости на месте"; return 0; }

    # SAMURAI_SKIP_PIP_INSTALL=1 — для systemd (pip недоступен под service-юзером).
    # В этом режиме отсутствующие пакеты считаются ошибкой.
    if [[ "${SAMURAI_SKIP_PIP_INSTALL:-0}" == "1" ]]; then
        die "Отсутствуют пакеты: ${missing[*]}. Установи вручную: pip install ${missing[*]}"
    fi

    log_info "Устанавливаю: ${missing[*]}"
    # Пробуем три варианта pip install (системный → user → break-system-packages)
    pip3 install --quiet "${missing[@]}" 2>/dev/null \
        || pip3 install --quiet --user "${missing[@]}" 2>/dev/null \
        || pip3 install --quiet --break-system-packages "${missing[@]}" \
        || die "pip install провалился: ${missing[*]}"
    log_ok "Зависимости установлены"
}

# Опциональный пакет — не блокирует если отсутствует.
# Использование: check_pip_optional import_name "Описание для warning"
check_pip_optional() {
    local import_name="$1"
    local label="${2:-$1}"
    if python3 -c "import ${import_name}" &>/dev/null 2>&1; then
        log_ok "$label ✓"
    else
        log_warn "$label не установлен (некоторые функции отключены)"
    fi
}

# ── Docker ──────────────────────────────────────────────────────────────────
check_docker() {
    log_step "Docker"
    if ! command -v docker &>/dev/null; then
        log_err "Docker не установлен"
        if is_arch_linux; then
            echo -e "    ${YELLOW}sudo pacman -S docker${NC}"
        elif is_pi; then
            echo -e "    ${YELLOW}curl -fsSL https://get.docker.com | sh${NC}"
        fi
        echo -e "    ${YELLOW}sudo systemctl enable --now docker${NC}"
        echo -e "    ${YELLOW}sudo usermod -aG docker \$USER && newgrp docker${NC}"
        die "Установи Docker и перезапусти"
    fi
    log_ok "Docker $(docker --version | grep -oP '\d+\.\d+\.\d+' | head -1)"

    if ! docker info &>/dev/null 2>&1; then
        log_warn "Docker daemon не запущен. Запускаю..."
        sudo systemctl start docker 2>/dev/null \
            || die "Не удалось запустить Docker. sudo systemctl start docker"
        sleep 2
        docker info &>/dev/null || die "Docker daemon не отвечает"
        log_ok "Docker daemon запущен"
    else
        log_ok "Docker daemon активен"
    fi
}

# ── I2C ─────────────────────────────────────────────────────────────────────
check_i2c() {
    log_step "I2C"

    if [[ ! -e /dev/i2c-1 ]]; then
        log_warn "/dev/i2c-1 не найден. Пробую включить..."
        if command -v raspi-config &>/dev/null; then
            sudo raspi-config nonint do_i2c 0 2>/dev/null || true
        fi
        sudo modprobe i2c-dev 2>/dev/null || true
        if [[ ! -e /dev/i2c-1 ]]; then
            log_warn "/dev/i2c-1 недоступен (работа без моторов/IMU)"
            return 1
        fi
    fi
    log_ok "I2C доступен (/dev/i2c-1)"

    if command -v i2cdetect &>/dev/null; then
        log_info "I2C устройства:"
        i2cdetect -y 1 2>/dev/null | sed 's/^/    /'
    fi
}

# ── MQTT (mosquitto) ────────────────────────────────────────────────────────
# Проверка/установка/конфиг mosquitto на Pi.
# Гарантирует listener 1883 + anti-freeze limits + auth-by-default policy.
#
# Policy (#75, post-#10):
#   - --auth USER PASS     → создаёт/обновляет passwd, allow_anonymous=false.
#   - no flags + есть passwd file → сохраняет auth включённым (не сносит до anon).
#   - no flags + нет passwd → SECURITY WARNING. Чтобы продолжить без auth,
#                             надо явно передать --allow-anonymous.
#   - --allow-anonymous    → принудительно открытый брокер (только для dev).
#
# Использование: check_mosquitto [--auth user pass | --allow-anonymous]
check_mosquitto() {
    log_step "MQTT брокер (mosquitto)"

    local with_auth=false mqtt_user="" mqtt_pass=""
    local allow_anon_explicit=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --auth) with_auth=true; mqtt_user="$2"; mqtt_pass="$3"; shift 3 ;;
            --allow-anonymous) allow_anon_explicit=true; shift ;;
            *) shift ;;
        esac
    done

    if ! command -v mosquitto &>/dev/null; then
        log_warn "mosquitto не установлен"
        echo -e "    Pi:   ${YELLOW}sudo apt install -y mosquitto mosquitto-clients${NC}"
        echo -e "    Arch: ${YELLOW}sudo pacman -S mosquitto${NC}"
        die "Установи mosquitto и перезапусти"
    fi

    local pwd_file="/etc/mosquitto/samurai.passwd"
    local conf="/etc/mosquitto/conf.d/samurai.conf"
    local has_existing_passwd=false
    [[ -f "$pwd_file" ]] && has_existing_passwd=true

    # Detect whether we should keep auth on. Order:
    # 1. Explicit --auth → yes.
    # 2. Explicit --allow-anonymous → no.
    # 3. Existing passwd file → yes (preserve previously-secured state).
    # 4. Otherwise (first-time setup) → no, but warn loudly.
    local enforce_auth=false
    if $with_auth; then
        enforce_auth=true
    elif $allow_anon_explicit; then
        enforce_auth=false
    elif $has_existing_passwd; then
        enforce_auth=true
    else
        enforce_auth=false
    fi

    local anon_setting="true"
    $enforce_auth && anon_setting="false"

    local desired
    desired=$(cat <<MQTTCFG
# Samurai Robot — generated by samurai.sh
listener 1883
allow_anonymous ${anon_setting}

# Anti-freeze limits (Pi-friendly)
max_inflight_messages 10
max_queued_messages 50
message_size_limit 300000
memory_limit 50000000
MQTTCFG
    )

    if $with_auth; then
        log_info "Настраиваю аутентификацию (user=$mqtt_user)..."
        sudo touch "$pwd_file"
        sudo mosquitto_passwd -b "$pwd_file" "$mqtt_user" "$mqtt_pass" \
            || die "mosquitto_passwd провалился"
        sudo chmod 600 "$pwd_file"
        sudo chown mosquitto:mosquitto "$pwd_file" 2>/dev/null || true
        desired="${desired}
password_file ${pwd_file}"
    elif $enforce_auth; then
        # No new creds, but a passwd file exists from a previous setup.
        # Keep referring to it so the broker stays locked down.
        desired="${desired}
password_file ${pwd_file}"
        log_ok "Auth сохранён (используется существующий ${pwd_file})"
    elif $allow_anon_explicit; then
        log_warn "${BOLD}--allow-anonymous${NC} активен — брокер открыт всей LAN"
    else
        log_warn "${BOLD}MQTT auth НЕ настроен${NC} — брокер принимает анонимные подключения"
        echo -e "    ${YELLOW}Защити брокер: ${BOLD}./samurai.sh auth init${NC}"
        echo -e "    ${YELLOW}Для явного разрешения анонимного режима используй --allow-anonymous${NC}"
    fi

    local need_restart=false
    if [[ ! -f "$conf" ]] || ! diff -q <(echo "$desired") "$conf" &>/dev/null; then
        log_warn "Обновляю $conf..."
        echo "$desired" | sudo tee "$conf" >/dev/null
        log_ok "Конфиг обновлён"
        need_restart=true
    fi

    if $need_restart || ! systemctl is-active --quiet mosquitto 2>/dev/null; then
        sudo systemctl restart mosquitto 2>/dev/null \
            || sudo systemctl start mosquitto 2>/dev/null \
            || die "Не удалось запустить mosquitto"
    fi

    log_ok "mosquitto активен (порт 1883)"
}

# ── Avahi (mDNS) ────────────────────────────────────────────────────────────
check_avahi() {
    log_step "mDNS (avahi)"
    if ! command -v avahi-daemon &>/dev/null && ! command -v avahi-browse &>/dev/null; then
        log_warn "avahi не установлен — Pi/ноутбук не виден по hostname.local"
        if is_pi; then
            echo -e "    ${YELLOW}sudo apt install avahi-daemon${NC}"
        elif is_arch_linux; then
            echo -e "    ${YELLOW}sudo pacman -S avahi nss-mdns${NC}"
        fi
        return 1
    fi

    if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
        sudo systemctl start avahi-daemon 2>/dev/null || true
    fi
    if systemctl is-active --quiet avahi-daemon 2>/dev/null; then
        log_ok "avahi активен — $(hostname).local"
    else
        log_warn "avahi не запущен"
        return 1
    fi
}

# ── Камера (Pi) ─────────────────────────────────────────────────────────────
check_camera_pi() {
    log_step "Камера (libcamera/picamera2)"
    local pic2="/usr/lib/python3/dist-packages/picamera2"
    local libcam="/usr/lib/python3/dist-packages/libcamera"

    [[ -d "$pic2" ]]   && log_ok "picamera2 ✓"   || log_warn "picamera2 не установлен (sudo apt install python3-picamera2)"
    [[ -d "$libcam" ]] && log_ok "libcamera ✓"   || log_warn "libcamera Python bindings не найдены"

    if command -v libcamera-hello &>/dev/null; then
        if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
            log_ok "Камера обнаружена (CSI)"
        else
            log_warn "Камера не обнаружена — проверь CSI-кабель"
        fi
    fi
}

# ── Порт ────────────────────────────────────────────────────────────────────
# Проверка свободен ли локальный TCP-порт.
# Использование: check_local_port_free 5000 [--ask]
check_local_port_free() {
    local port="$1" ask=false
    [[ "${2:-}" == "--ask" ]] && ask=true

    log_step "Порт $port"
    local busy=false
    if ss -tlnp 2>/dev/null | grep -q ":$port " || \
       netstat -tlnp 2>/dev/null | grep -q ":$port "; then
        busy=true
    fi

    if $busy; then
        log_warn "Порт $port занят"
        local pid
        pid=$(ss -tlnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | head -1 || echo "?")
        [[ "$pid" != "?" ]] && echo -e "    PID: $pid  (kill $pid чтобы освободить)"
        if $ask; then
            confirm "Продолжить всё равно?" || exit 0
        fi
        return 1
    fi
    log_ok "Порт $port свободен"
    return 0
}

# ── ROS2 workspace ──────────────────────────────────────────────────────────
# Проверка/сборка ros_ws через Docker.
# Использование: check_ros_workspace IMAGE [--rebuild] [--platform linux/arm64]
check_ros_workspace() {
    log_step "ROS2 Workspace (ros_ws)"
    local image="$1"; shift
    local rebuild=false platform=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --rebuild) rebuild=true; shift ;;
            --platform) platform="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    local ws="$SAMURAI_ROOT/ros_ws"
    local install_dir="$ws/install"

    _build_ws() {
        log_info "colcon build внутри Docker..."
        local plat_arg=""
        [[ -n "$platform" ]] && plat_arg="--platform $platform"
        # shellcheck disable=SC2086
        MSYS_NO_PATHCONV=1 docker run --rm $plat_arg \
            -v "$SAMURAI_ROOT:/root/Samurai" \
            "$image" \
            bash -c "
                set -e
                source /opt/ros/humble/setup.bash
                cd /root/Samurai/ros_ws
                colcon build --symlink-install \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                    --event-handlers console_cohesion+
            " || die "colcon build провалился"
        log_ok "Workspace собран"
    }

    if [[ ! -d "$install_dir" ]] || $rebuild; then
        log_warn "ros_ws/install отсутствует или --rebuild"
        _build_ws
        return
    fi

    log_ok "ros_ws/install найден"
    local stale
    stale=$(find "$ws/src" -name "*.py" -newer "$install_dir" 2>/dev/null | head -1 || true)
    if [[ -n "$stale" ]]; then
        log_warn "Найдены изменения в src/ — пересобираю"
        _build_ws
    fi
}
