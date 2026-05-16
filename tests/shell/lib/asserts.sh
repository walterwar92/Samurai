# shellcheck shell=bash
# Простые assert-хелперы для bash-тестов.
# Использование: source tests/shell/lib/asserts.sh

_TESTS_PASSED=0
_TESTS_FAILED=0
_TEST_NAME=""

test_start() { _TEST_NAME="$1"; echo "─── $_TEST_NAME"; }

assert_eq() {
    local expected="$1" actual="$2" msg="${3:-}"
    if [[ "$expected" == "$actual" ]]; then
        _TESTS_PASSED=$((_TESTS_PASSED+1))
        echo "  [✓] ${msg:-$_TEST_NAME}"
    else
        _TESTS_FAILED=$((_TESTS_FAILED+1))
        echo "  [✗] ${msg:-$_TEST_NAME}"
        echo "      expected: $expected"
        echo "      actual:   $actual"
    fi
}

assert_contains() {
    local haystack="$1" needle="$2" msg="${3:-}"
    if [[ "$haystack" == *"$needle"* ]]; then
        _TESTS_PASSED=$((_TESTS_PASSED+1))
        echo "  [✓] contains '$needle' ${msg:+— $msg}"
    else
        _TESTS_FAILED=$((_TESTS_FAILED+1))
        echo "  [✗] missing '$needle' ${msg:+— $msg}"
        echo "      haystack: $haystack"
    fi
}

assert_not_contains() {
    local haystack="$1" needle="$2" msg="${3:-}"
    if [[ "$haystack" != *"$needle"* ]]; then
        _TESTS_PASSED=$((_TESTS_PASSED+1))
        echo "  [✓] does not contain '$needle' ${msg:+— $msg}"
    else
        _TESTS_FAILED=$((_TESTS_FAILED+1))
        echo "  [✗] should not contain '$needle' ${msg:+— $msg}"
        echo "      haystack: $haystack"
    fi
}

assert_exit() {
    local expected_code="$1" actual_code="$2" msg="${3:-}"
    assert_eq "$expected_code" "$actual_code" "${msg:-exit code}"
}

tests_summary() {
    echo ""
    echo "Passed: $_TESTS_PASSED  Failed: $_TESTS_FAILED"
    [[ $_TESTS_FAILED -eq 0 ]] || exit 1
    exit 0
}
