#!/usr/bin/env bash
# tests/shell/test_stop_component.sh
# Тест нового поведения scripts/lib/locking.sh::stop_component:
#   1. _systemd_unit_for — маппинг компонента → unit
#   2. _systemd_unit_active — детект через фейк systemctl
#   3. stop_component вызывает systemctl stop когда unit active
#   4. stop_component fallthrough на PID-kill когда unit отсутствует/неактивен
#   5. _kill_tree использует process group, а не одиночный PID
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

# Изолированное состояние
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT
export SAMURAI_STATE_DIR="$TMP_DIR/state"
export SAMURAI_LOG_DIR="$TMP_DIR/state/logs"
export SAMURAI_LOCK_DIR="$TMP_DIR/state/locks"
mkdir -p "$SAMURAI_LOCK_DIR"

# Фейк systemctl
FAKE_BIN="$TMP_DIR/bin"
mkdir -p "$FAKE_BIN"
cat > "$FAKE_BIN/systemctl" <<'FAKE_EOF'
#!/usr/bin/env bash
echo "systemctl $*" >> "$FAKE_SYSTEMCTL_LOG"
all="$*"
case "$all" in
    "is-active --quiet "*)
        unit="${all##* }"
        [[ "${FAKE_ACTIVE_UNITS:-}" == *":$unit:"* ]] && exit 0 || exit 3 ;;
    "stop "*)
        unit="${all##* }"
        FAKE_ACTIVE_UNITS="${FAKE_ACTIVE_UNITS//:$unit:/:}"
        exit "${FAKE_SYSTEMCTL_STOP_EXIT:-0}" ;;
    *) exit 0 ;;
esac
FAKE_EOF
chmod +x "$FAKE_BIN/systemctl"

# Фейк sudo: просто пропускает аргументы дальше (sudo -n cmd → cmd)
cat > "$FAKE_BIN/sudo" <<'FAKE_EOF'
#!/usr/bin/env bash
# Пропускаем флаги (-n), запускаем оставшееся
while [[ "${1:-}" == -* ]]; do shift; done
exec "$@"
FAKE_EOF
chmod +x "$FAKE_BIN/sudo"

export PATH="$FAKE_BIN:$PATH"
export FAKE_SYSTEMCTL_LOG="$TMP_DIR/systemctl.log"
: > "$FAKE_SYSTEMCTL_LOG"

# shellcheck source=../../scripts/lib/common.sh
source "$REPO_ROOT/scripts/lib/common.sh"
# shellcheck source=../../scripts/lib/locking.sh
source "$REPO_ROOT/scripts/lib/locking.sh"

# ── 1. _systemd_unit_for ────────────────────────────────────────────────────
test_start "_systemd_unit_for: маппинг компонент → unit"
assert_eq "samurai-robot.service"   "$(_systemd_unit_for robot)"   "robot"
assert_eq "samurai-compute.service" "$(_systemd_unit_for compute)" "compute"
assert_eq "samurai-bridge.service"  "$(_systemd_unit_for bridge)"  "bridge"
assert_eq "samurai-agent.service"   "$(_systemd_unit_for agent)"   "agent"
assert_eq ""                        "$(_systemd_unit_for sim)"     "sim (no systemd unit)"
assert_eq ""                        "$(_systemd_unit_for detector)" "detector (no systemd unit)"

# ── 2. _systemd_unit_active с фейком ────────────────────────────────────────
test_start "_systemd_unit_active: фейк detect"
export FAKE_ACTIVE_UNITS=":samurai-robot.service:"
if _systemd_unit_active "samurai-robot.service"; then
    assert_eq "ok" "ok" "robot active → 0"
else
    assert_eq "ok" "fail" "robot active → should be 0"
fi
if _systemd_unit_active "samurai-bridge.service"; then
    assert_eq "ok" "fail" "bridge inactive → should be 1"
else
    assert_eq "ok" "ok" "bridge inactive → 1"
fi

# ── 3. stop_component через systemctl ───────────────────────────────────────
test_start "stop_component: active unit → systemctl stop"
export FAKE_ACTIVE_UNITS=":samurai-robot.service:"
echo "12345" > "$SAMURAI_LOCK_DIR/robot.lock"  # симулируем stale lock
: > "$FAKE_SYSTEMCTL_LOG"
out=$(stop_component robot 2>&1)
log_content=$(cat "$FAKE_SYSTEMCTL_LOG")
assert_contains "$log_content" "stop samurai-robot.service" "systemctl stop вызван"
assert_contains "$out" "systemd" "лог говорит про systemd path"
assert_eq "false" "$([[ -f "$SAMURAI_LOCK_DIR/robot.lock" ]] && echo true || echo false)" \
    "lock-файл удалён"

# ── 4. stop_component fallthrough на PID-kill ──────────────────────────────
test_start "stop_component: unit inactive + no lock → 'не запущен'"
export FAKE_ACTIVE_UNITS=":"
: > "$FAKE_SYSTEMCTL_LOG"
rm -f "$SAMURAI_LOCK_DIR/robot.lock"
out=$(stop_component robot 2>&1)
assert_contains "$out" "не запущен" "нет процесса → 'не запущен'"
# systemctl is-active может быть вызван, но stop — нет
log_content=$(cat "$FAKE_SYSTEMCTL_LOG")
assert_not_contains "$log_content" "stop samurai" "systemctl stop НЕ вызван"

# ── 5. stop_component реально убивает локальный процесс ────────────────────
test_start "stop_component: kill локального процесса через group"
export FAKE_ACTIVE_UNITS=":"  # unit inactive → PID path
# Запускаем долгий процесс в новой process group через setsid (если есть)
if command -v setsid &>/dev/null; then
    setsid sleep 60 &
    bg_pid=$!
    # Дать setsid время форкнуться
    sleep 0.2
    # Найти настоящий PID (setsid форкает потомка)
    real_pid=$(pgrep -P $bg_pid sleep 2>/dev/null | head -1 || echo "$bg_pid")
    # Проверим что он жив
    if kill -0 "$real_pid" 2>/dev/null; then
        echo "$real_pid" > "$SAMURAI_LOCK_DIR/sim.lock"
        # sim → нет systemd unit → пойдём по PID path
        stop_component sim >/dev/null 2>&1 || true
        # Подождать
        sleep 0.3
        if kill -0 "$real_pid" 2>/dev/null; then
            assert_eq "killed" "alive" "процесс должен быть убит"
            kill -KILL "$real_pid" 2>/dev/null || true
        else
            assert_eq "killed" "killed" "процесс убит"
        fi
    else
        echo "  [skip] setsid sleep не запустился"
    fi
else
    echo "  [skip] setsid недоступен"
fi

tests_summary
