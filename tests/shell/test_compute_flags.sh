#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

# Извлекаем функцию resolve_pi_user из compute.sh для проверки приоритета источников.
export SAMURAI_ROOT="$REPO_ROOT"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/common.sh"
set +e
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/cmds/compute.sh"

# ── Test 1: CLI флаг побеждает env ─────────────────────────────────────────
test_start "CLI --pi-user beats env"
result=$(SAMURAI_PI_USER=env_user resolve_pi_user "cli_user")
assert_eq "cli_user" "$result"

# ── Test 2: env используется когда CLI пуст ────────────────────────────────
test_start "env used when CLI empty"
result=$(SAMURAI_PI_USER=env_user resolve_pi_user "")
assert_eq "env_user" "$result"

# ── Test 3: hardcoded default когда оба пусты ──────────────────────────────
test_start "default 'pi' when nothing set"
result=$(unset SAMURAI_PI_USER; resolve_pi_user "")
assert_eq "pi" "$result"

# ── Test 4: resolve_pi_path ────────────────────────────────────────────────
test_start "resolve_pi_path priorities"
result=$(SAMURAI_PI_PATH=/env/path resolve_pi_path "/cli/path")
assert_eq "/cli/path" "$result"
result=$(SAMURAI_PI_PATH=/env/path resolve_pi_path "")
assert_eq "/env/path" "$result"
result=$(unset SAMURAI_PI_PATH; resolve_pi_path "")
assert_eq "~/Samurai" "$result"

# ── Test 5: resolve_ssh_key ────────────────────────────────────────────────
test_start "resolve_ssh_key priorities"
result=$(SAMURAI_PI_SSH_KEY=/env/key resolve_ssh_key "/cli/key")
assert_eq "/cli/key" "$result"
result=$(SAMURAI_PI_SSH_KEY=/env/key resolve_ssh_key "")
assert_eq "/env/key" "$result"
result=$(unset SAMURAI_PI_SSH_KEY; resolve_ssh_key "")
assert_eq "" "$result"

tests_summary
