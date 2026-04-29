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

# Остановить компонент по lock-файлу. Посылает SIGTERM, ждёт 5с, потом SIGKILL.
# Использование: stop_component <name>
stop_component() {
    local name="$1"
    local pid
    pid=$(is_running "$name" || true)
    if [[ -z "$pid" ]]; then
        log_info "$name не запущен"
        return 0
    fi
    log_info "Останавливаю $name (PID $pid)..."
    kill -TERM "$pid" 2>/dev/null || true
    local i=0
    while kill -0 "$pid" 2>/dev/null && [[ $i -lt 50 ]]; do
        sleep 0.1
        ((i++))
    done
    if kill -0 "$pid" 2>/dev/null; then
        log_warn "$name не отвечает на SIGTERM — SIGKILL"
        kill -KILL "$pid" 2>/dev/null || true
    fi
    release_lock "$name"
    log_ok "$name остановлен"
}
