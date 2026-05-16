#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

# Подмена PATH: фейки впереди.
export PATH="$SCRIPT_DIR/lib/fakes:$PATH"

# Логи фейков.
export FAKE_SSH_LOG=$(mktemp)
export FAKE_RSYNC_LOG=$(mktemp)
export FAKE_IS_ACTIVE="active"

trap 'rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"' EXIT

# Загружаем compute.sh — он должен предоставлять deploy_to_pi().
# common.sh нужен для log_*/die.
export SAMURAI_ROOT="$REPO_ROOT"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/common.sh"
# shellcheck disable=SC1091
set +e
source "$REPO_ROOT/scripts/cmds/compute.sh"
# NB: errexit OFF deliberately — тест ассертит на exit-кодах через $?,
#     re-enable сломал бы failure-сценарии.

test_start "deploy_to_pi happy path"

# Вызов: deploy_to_pi PI_IP PI_USER PI_PATH SSH_KEY
deploy_to_pi "192.168.4.1" "pi" "/home/pi/Samurai" "" >/dev/null 2>&1
exit_code=$?

assert_eq "0" "$exit_code" "deploy_to_pi exits 0 on happy path"

ssh_calls=$(cat "$FAKE_SSH_LOG")
rsync_calls=$(cat "$FAKE_RSYNC_LOG")

# Pre-flight SSH.
assert_contains "$ssh_calls" "ConnectTimeout=5" "ConnectTimeout set"
assert_contains "$ssh_calls" "pi@192.168.4.1 true" "pre-flight ssh true"

# Pre-flight: проверка наличия юнита.
assert_contains "$ssh_calls" "systemctl list-unit-files samurai-robot.service" "unit check"

# rsync с правильным флагом и таргетом.
assert_contains "$rsync_calls" "-az --delete" "rsync flags"
assert_contains "$rsync_calls" "--exclude-from=$REPO_ROOT/.deployignore" "exclude-from"
assert_contains "$rsync_calls" "$REPO_ROOT/ pi@192.168.4.1:/home/pi/Samurai/" "rsync src/dst"

# Рестарт.
assert_contains "$ssh_calls" "sudo systemctl restart samurai-robot" "restart issued"

# Verify.
assert_contains "$ssh_calls" "systemctl is-active samurai-robot" "is-active check"

tests_summary
