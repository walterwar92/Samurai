# shellcheck shell=bash
# scripts/lib/locking.sh
# Lock-файлы для идемпотентности подкоманд.
# Если samurai robot уже запущен — повторный запуск не создаст второй процесс.

[[ -n "${_SAMURAI_LOCKING_LOADED:-}" ]] && return 0
_SAMURAI_LOCKING_LOADED=1

[[ -z "${_SAMURAI_COMMON_LOADED:-}" ]] && {
    # shellcheck source=common.sh
    source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
}

# Захват lock-файла. Если процесс с PID жив — выходим.
# Использование: acquire_lock <name>
# name = robot|sim|compute|detector|bridge
acquire_lock() {
    local name="$1"
    ensure_state_dirs
    local lock="$SAMURAI_LOCK_DIR/$name.lock"

    if [[ -f "$lock" ]]; then
        local old_pid
        old_pid=$(cat "$lock" 2>/dev/null || echo "")
        if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
            log_err "samurai $name уже запущен (PID $old_pid)"
            log_info "Останови его: ./samurai.sh stop $name"
            log_info "Или проверь статус: ./samurai.sh status"
            exit 1
        else
            log_warn "Stale lock-файл $lock (PID $old_pid мёртв) — удаляю"
            rm -f "$lock"
        fi
    fi

    echo "$$" > "$lock"
    # Cleanup при выходе скрипта
    # shellcheck disable=SC2064
    trap "release_lock $name" EXIT INT TERM
}

release_lock() {
    local name="$1"
    rm -f "$SAMURAI_LOCK_DIR/$name.lock" 2>/dev/null || true
}

# Проверка живости. Возвращает 0 если процесс жив, 1 иначе.
# Использование: if is_running <name>; then ...; fi
# Печатает PID в stdout если процесс жив.
is_running() {
    local name="$1"
    local lock="$SAMURAI_LOCK_DIR/$name.lock"
    [[ -f "$lock" ]] || return 1
    local pid
    pid=$(cat "$lock" 2>/dev/null || echo "")
    [[ -n "$pid" ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    echo "$pid"
}

# Маппинг samurai-компонента на systemd unit (если установлен).
# Возвращает имя unit'а в stdout или пусто если для компонента нет unit'а.
_systemd_unit_for() {
    case "$1" in
        robot)   echo "samurai-robot.service" ;;
        compute) echo "samurai-compute.service" ;;
        bridge)  echo "samurai-bridge.service" ;;
        agent)   echo "samurai-agent.service" ;;
        *)       echo "" ;;
    esac
}

# Проверка: установлен ли unit и активен ли.
# Возврат: 0 если active, 1 иначе. Также 1 если systemctl недоступен.
_systemd_unit_active() {
    local unit="$1"
    [[ -n "$unit" ]] || return 1
    command -v systemctl &>/dev/null || return 1
    systemctl is-active --quiet "$unit" 2>/dev/null
}

# Остановка через systemctl. Если уже root — напрямую, иначе через sudo -n
# (требует правила NOPASSWD в /etc/sudoers.d/samurai-robot — ставится через
# `sudo ./scripts/systemd/install.sh --with-remote-deploy`).
# Возврат: 0 при успехе, 1 при ошибке.
_systemctl_stop() {
    local unit="$1"
    if [[ "$EUID" -eq 0 ]]; then
        systemctl stop "$unit"
    else
        # Не пытаемся `sudo -n true` — оно проходит даже если конкретное
        # правило для stop отсутствует. Сразу пробуем целевую команду:
        # успех → NOPASSWD есть, ошибка → fallback на PID-kill.
        sudo -n systemctl stop "$unit" 2>/dev/null
    fi
}

# Послать сигнал в process group если launcher — лидер группы, иначе в PID.
# kill -<SIG> -- -<PGID> шлёт сигнал всей группе (всем потомкам).
_kill_tree() {
    local sig="$1" pid="$2"
    local pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
    if [[ -n "$pgid" && "$pgid" == "$pid" ]]; then
        # PID — лидер группы → бьём по всей группе
        kill "-$sig" -- "-$pgid" 2>/dev/null || true
    else
        # Не лидер группы — fallback на одиночный PID
        kill "-$sig" "$pid" 2>/dev/null || true
    fi
}

# Остановить компонент. Стратегия:
#   1. Если systemd unit установлен и active → systemctl stop (cgroup-wide kill).
#      Это надёжнее всего: systemd убивает все процессы в cgroup и не даёт
#      Restart=on-failure поднять сервис заново.
#   2. Иначе — по lock-файлу. Сигналим всей process group (детям launcher
#      тоже), а не одиночному PID, чтобы не оставлять multiprocessing.Process
#      сиротами при SIGKILL родителя.
# Использование: stop_component <name>
stop_component() {
    local name="$1"
    local unit
    unit=$(_systemd_unit_for "$name")

    # Путь 1: systemd-managed сервис.
    if _systemd_unit_active "$unit"; then
        log_info "Останавливаю $name через systemctl ($unit)..."
        if _systemctl_stop "$unit" >/dev/null 2>&1; then
            release_lock "$name"
            log_ok "$name остановлен (systemd)"
            return 0
        else
            log_warn "systemctl stop $unit не удался — fallback на PID"
        fi
    fi

    # Путь 2: locally-launched через samurai CLI.
    local pid
    pid=$(is_running "$name" || true)
    if [[ -z "$pid" ]]; then
        log_info "$name не запущен"
        return 0
    fi
    log_info "Останавливаю $name (PID $pid, process group)..."
    _kill_tree TERM "$pid"

    # Даём время на graceful shutdown. Launcher на Pi с 14+ нодами и
    # последовательным p.join(timeout=5) может реально занимать до ~15с
    # в плохом сценарии (зависший MQTT loop, медленный I2C cleanup).
    # 15с TERM перед KILL — компромисс: не зависаем надолго, но и не
    # рвём по живому при штатном завершении.
    local i=0
    while kill -0 "$pid" 2>/dev/null && [[ $i -lt 150 ]]; do
        sleep 0.1
        ((i++))
    done
    if kill -0 "$pid" 2>/dev/null; then
        log_warn "$name не отвечает на SIGTERM за 15с — SIGKILL"
        _kill_tree KILL "$pid"
    fi
    release_lock "$name"
    log_ok "$name остановлен"
}
