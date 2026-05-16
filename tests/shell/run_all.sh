#!/usr/bin/env bash
# Раннер всех bash-тестов в этом каталоге.
# Usage: bash tests/shell/run_all.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
total_failed=0
total_files=0

for test in "$SCRIPT_DIR"/test_*.sh; do
    [[ -f "$test" ]] || continue
    total_files=$((total_files + 1))
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo " $(basename "$test")"
    echo "════════════════════════════════════════════════════════════════"
    if ! bash "$test"; then
        total_failed=$((total_failed + 1))
    fi
done

echo ""
echo "════════════════════════════════════════════════════════════════"
echo " Total test files: $total_files"
echo " Failed:           $total_failed"
echo "════════════════════════════════════════════════════════════════"

[[ $total_failed -eq 0 ]] || exit 1
