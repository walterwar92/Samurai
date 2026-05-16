#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

# Изолируем sudoers: переменная SUDOERS_DIR override.
TMP_SUDOERS=$(mktemp -d)
trap 'rm -rf "$TMP_SUDOERS"' EXIT

export SAMURAI_ROOT="$REPO_ROOT"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/common.sh"
set +e
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/systemd/install.sh"  # должен экспортить install_sudoers_remote_deploy()

test_start "sudoers file generated correctly"

# Вызов: install_sudoers_remote_deploy <user> <sudoers_dir>
install_sudoers_remote_deploy "myuser" "$TMP_SUDOERS" >/dev/null

sudoers_file="$TMP_SUDOERS/samurai-robot"
assert_eq "true" "$([[ -f "$sudoers_file" ]] && echo true || echo false)" "file created"

content=$(cat "$sudoers_file")
assert_contains "$content" "myuser ALL=(root) NOPASSWD" "user prefix"
assert_contains "$content" "/bin/systemctl restart samurai-robot" "restart cmd"
assert_contains "$content" "/bin/systemctl is-active samurai-robot" "is-active cmd"
assert_contains "$content" "/bin/systemctl status samurai-robot" "status cmd"
assert_contains "$content" "/bin/journalctl -u samurai-robot" "journalctl cmd"

# Права 0440.
perm=$(stat -c '%a' "$sudoers_file" 2>/dev/null || stat -f '%A' "$sudoers_file" 2>/dev/null)
assert_eq "440" "$perm" "permissions 0440"

# visudo валидация (skip если visudo нет).
if command -v visudo &>/dev/null; then
    if visudo -cf "$sudoers_file" &>/dev/null; then
        echo "  [✓] visudo accepts the file"
        _TESTS_PASSED=$((_TESTS_PASSED+1))
    else
        echo "  [✗] visudo rejects the file"
        _TESTS_FAILED=$((_TESTS_FAILED+1))
    fi
else
    echo "  [skip] visudo not available"
fi

tests_summary
