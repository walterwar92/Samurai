#!/usr/bin/env bash
# Проверяем что .deployignore корректно исключает зоны не для Pi.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

cd "$REPO_ROOT" || exit 1

test_start ".deployignore исключает компонент ноута"

# Dry-run rsync — печатает список файлов которые БЫ скопировались.
listing=$(rsync -a --dry-run --out-format='%n' \
    --exclude-from=".deployignore" \
    ./ /tmp/__deploy_test_dst__/ 2>&1)
rsync_ec=$?
assert_eq "0" "$rsync_ec" "rsync dry-run succeeded"

# Должны быть исключены:
assert_not_contains "$listing" ".git/"           "git history excluded"
assert_not_contains "$listing" "compute_node/"   "compute_node excluded"
assert_not_contains "$listing" "ros_ws/"         "ros_ws excluded"
assert_not_contains "$listing" "android_app/"    "android_app excluded"
assert_not_contains "$listing" "docs/"           "docs excluded"
assert_not_contains "$listing" "node_modules"    "node_modules excluded"
assert_not_contains "$listing" "__pycache__"     "pycache excluded"
assert_not_contains "$listing" ".pyc"            "pyc excluded"
assert_not_contains "$listing" "tests/shell/lib/fakes" "test fakes excluded"
assert_not_contains "$listing" "latex_doc/"      "latex_doc excluded"

# Должны быть включены:
assert_contains "$listing" "pi_nodes/"           "pi_nodes included"
assert_contains "$listing" "scripts/"            "scripts included"
assert_contains "$listing" "samurai.sh"          "samurai.sh included"

tests_summary
