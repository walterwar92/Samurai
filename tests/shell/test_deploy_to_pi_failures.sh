#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

export PATH="$SCRIPT_DIR/lib/fakes:$PATH"
export SAMURAI_ROOT="$REPO_ROOT"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/common.sh"
set +e  # compute.sh содержит set -e — отключаем для тестов
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/cmds/compute.sh"

# ── Test 1: SSH недоступен ─────────────────────────────────────────────────
test_start "fail when SSH unreachable"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_SSH_FAIL_ON_MATCH="true"  # фейк падает на pre-flight 'ssh ... true'

ec=0
output=$(deploy_to_pi "10.0.0.99" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "$ec" "exits non-zero on SSH fail"
assert_contains "$output" "SSH до pi@10.0.0.99 недоступен" "error message"
unset FAKE_SSH_FAIL_ON_MATCH
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

# ── Test 2: systemd unit не установлен ─────────────────────────────────────
test_start "fail when unit not installed"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_SSH_UNIT_MISSING=1

ec=0
output=$(deploy_to_pi "192.168.4.1" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "$ec" "exits non-zero on missing unit"
assert_contains "$output" "samurai-robot.service не установлен" "error message"
assert_contains "$output" "bootstrap_pi.sh" "hints bootstrap script"
unset FAKE_SSH_UNIT_MISSING
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

# ── Test 3: rsync падает ───────────────────────────────────────────────────
test_start "fail when rsync fails"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_RSYNC_EXIT=23  # rsync code 23 = partial transfer

ec=0
output=$(deploy_to_pi "192.168.4.1" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "$ec" "exits non-zero on rsync fail"
assert_contains "$output" "rsync на pi@192.168.4.1 провалился" "error message"
unset FAKE_RSYNC_EXIT
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

# ── Test 4: systemctl restart падает (NOPASSWD не настроен) ────────────────
test_start "fail when restart fails"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_SSH_FAIL_ON_MATCH="sudo systemctl restart"

ec=0
output=$(deploy_to_pi "192.168.4.1" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "$ec" "exits non-zero on restart fail"
assert_contains "$output" "systemctl restart провалился" "error message"
assert_contains "$output" "with-remote-deploy" "hints install.sh flag"
unset FAKE_SSH_FAIL_ON_MATCH
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

# ── Test 5: is-active возвращает не active ─────────────────────────────────
test_start "fail when service not active after restart"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_IS_ACTIVE="failed"

ec=0
output=$(deploy_to_pi "192.168.4.1" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "$ec" "exits non-zero when not active"
assert_contains "$output" "не active" "error message"
assert_contains "$output" "fake journalctl line" "dumps journalctl"
unset FAKE_IS_ACTIVE
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

tests_summary
