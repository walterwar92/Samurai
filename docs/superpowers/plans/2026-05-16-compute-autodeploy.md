# Compute Autodeploy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Сделать `./samurai.sh compute --pi IP` автоматически синхронизирующим код с ноута на Pi через rsync и рестартующим `samurai-robot.service`, чтобы Pi не нужен был интернет.

**Architecture:** Inline функция `deploy_to_pi()` в `scripts/cmds/compute.sh` (rsync через SSH + `systemctl restart` + verify). Файл `.deployignore` в корне репо для исключений rsync. Расширение `scripts/systemd/install.sh` опцией `--with-remote-deploy` для генерации `/etc/sudoers.d/samurai-robot` (NOPASSWD). Новый `scripts/bootstrap_pi.sh` для одноразовой первичной настройки Pi.

**Tech Stack:** bash 5.x, rsync, OpenSSH, systemd, sudoers, raspi-config. Тесты — plain bash в `tests/shell/` с PATH-override фейками для `ssh`/`rsync`/`systemctl`.

**Spec:** [`docs/superpowers/specs/2026-05-16-compute-autodeploy-design.md`](../specs/2026-05-16-compute-autodeploy-design.md)

**Suggested branch:** `feat/compute-autodeploy` (текущая ветка `snapshot/thursday-2026-05-14`). Реализатор может перейти на feat-ветку до Task 1.

---

## File Structure

| Файл | Действие | Ответственность |
|---|---|---|
| `.deployignore` | Create | Корневой список исключений rsync — что НЕ отправлять на Pi |
| `tests/shell/lib/asserts.sh` | Create | Шеринговые assert-хелперы для bash-тестов |
| `tests/shell/lib/fakes/` | Create dir | PATH-override фейки `ssh`, `rsync`, `systemctl`, `journalctl`, `sudo` |
| `tests/shell/test_deployignore.sh` | Create | Проверка что `.deployignore` исключает нужное через `rsync --dry-run` |
| `tests/shell/test_deploy_to_pi_happy.sh` | Create | Happy path `deploy_to_pi()` — все вызовы ssh/rsync корректны |
| `tests/shell/test_deploy_to_pi_failures.sh` | Create | Pre-flight, rsync, restart, verify фейлы |
| `tests/shell/test_compute_flags.sh` | Create | Парсинг `--no-deploy`, `--pi-user`, `--pi-path`, `--ssh-key` |
| `tests/shell/test_install_sudoers.sh` | Create | `install.sh --with-remote-deploy` пишет валидный sudoers |
| `tests/shell/run_all.sh` | Create | Раннер всех bash-тестов |
| `scripts/cmds/compute.sh` | Modify | Добавить `deploy_to_pi()` + новые флаги + точку вызова |
| `scripts/systemd/install.sh` | Modify | Флаг `--with-remote-deploy` + генерация sudoers.d |
| `scripts/bootstrap_pi.sh` | Create | Одноразовая первичная настройка Pi |
| `README.md` | Modify | Раздел «Деплой на Pi одной командой» |
| `memory/MEMORY.md` | Modify | Обновить блок Startup |

---

## Task 1: `.deployignore` file

**Files:**
- Create: `tests/shell/lib/asserts.sh`
- Create: `tests/shell/test_deployignore.sh`
- Create: `.deployignore`

- [ ] **Step 1: Создать shared asserts**

Файл `tests/shell/lib/asserts.sh`:
```bash
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
```

- [ ] **Step 2: Написать failing-тест на `.deployignore`**

Файл `tests/shell/test_deployignore.sh` (исполняемый, `chmod +x`):
```bash
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
    ./ /tmp/__deploy_test_dst__/ 2>&1 || true)

# Должны быть исключены:
assert_not_contains "$listing" ".git/"           "git history excluded"
assert_not_contains "$listing" "compute_node/"   "compute_node excluded"
assert_not_contains "$listing" "ros_ws/"         "ros_ws excluded"
assert_not_contains "$listing" "android/"        "android excluded"
assert_not_contains "$listing" "docs/"           "docs excluded"
assert_not_contains "$listing" "node_modules"    "node_modules excluded"
assert_not_contains "$listing" "__pycache__"     "pycache excluded"
assert_not_contains "$listing" ".pyc"            "pyc excluded"

# Должны быть включены:
assert_contains "$listing" "pi_nodes/"           "pi_nodes included"
assert_contains "$listing" "scripts/"            "scripts included"
assert_contains "$listing" "samurai.sh"          "samurai.sh included"

tests_summary
```

- [ ] **Step 3: Запустить тест — должен упасть (нет файла)**

```bash
chmod +x tests/shell/test_deployignore.sh
bash tests/shell/test_deployignore.sh
```
Expected: тест падает потому что `.deployignore` не существует (rsync создаст dst dir но не отфильтрует ничего → `.git/`, `compute_node/` будут в листинге).

- [ ] **Step 4: Создать `.deployignore`**

Файл `.deployignore` в корне репо:
```gitignore
# Что НЕ синхронизировать на Pi через ./samurai.sh compute --pi.
# Используется rsync --exclude-from. Синтаксис gitignore-подобный.

# Git / CI
.git/
.github/

# Зоны которые крутятся ТОЛЬКО на ноуте
compute_node/
ros_ws/
android_app/
matlab/
docs/

# Локальные артефакты
node_modules/
__pycache__/
*.pyc
*.pyo
.venv/
venv/
.mypy_cache/
.pytest_cache/
.ruff_cache/
dist/
build/
*.egg-info/

# IDE / OS мусор
.idea/
.vscode/
.DS_Store
Thumbs.db

# MATLAB
slprj/
*.slxc
*.asv

# Логи и временные
*.log
*.tmp
```

- [ ] **Step 5: Запустить тест — должен пройти**

```bash
bash tests/shell/test_deployignore.sh
rm -rf /tmp/__deploy_test_dst__
```
Expected: `Passed: 11  Failed: 0`.

- [ ] **Step 6: Commit**

```bash
git add tests/shell/lib/asserts.sh tests/shell/test_deployignore.sh .deployignore
git commit -m "feat(deploy): .deployignore + dry-run rsync test

Корневой список исключений для rsync с ноута на Pi:
исключены .git, compute_node, ros_ws, android, matlab, docs,
node_modules, __pycache__, IDE-мусор, MATLAB slprj.

Тест tests/shell/test_deployignore.sh проверяет dry-run что
исключения работают и pi_nodes/scripts/samurai.sh остаются."
```

---

## Task 2: `deploy_to_pi()` happy path — фейки и юнит-тест

**Files:**
- Create: `tests/shell/lib/fakes/ssh`
- Create: `tests/shell/lib/fakes/rsync`
- Create: `tests/shell/test_deploy_to_pi_happy.sh`
- Modify: `scripts/cmds/compute.sh` (add `deploy_to_pi` function)

- [ ] **Step 1: Создать фейк `ssh`**

Файл `tests/shell/lib/fakes/ssh` (исполняемый):
```bash
#!/usr/bin/env bash
# Фейк ssh. Пишет полный argv в $FAKE_SSH_LOG (одна строка на вызов).
# Возвращает stdout по содержимому $FAKE_SSH_STDOUT (если задан),
# код возврата по $FAKE_SSH_EXIT (default 0).
# Если в команде есть подстрока из $FAKE_SSH_FAIL_ON_MATCH — exit 1.

echo "ssh $*" >> "${FAKE_SSH_LOG:-/dev/null}"

# Поведение по аргументам — для разных проверок в одном тесте.
cmd_str="$*"

if [[ -n "${FAKE_SSH_FAIL_ON_MATCH:-}" && "$cmd_str" == *"$FAKE_SSH_FAIL_ON_MATCH"* ]]; then
    exit 1
fi

# Симуляция конкретных команд:
case "$cmd_str" in
    *"systemctl list-unit-files samurai-robot.service"*)
        echo "samurai-robot.service enabled enabled"
        ;;
    *"systemctl is-active samurai-robot"*)
        echo "${FAKE_IS_ACTIVE:-active}"
        ;;
    *"journalctl -u samurai-robot"*)
        echo "fake journalctl line 1"
        echo "fake journalctl line 2"
        ;;
    *)
        [[ -n "${FAKE_SSH_STDOUT:-}" ]] && echo "$FAKE_SSH_STDOUT"
        ;;
esac

exit "${FAKE_SSH_EXIT:-0}"
```

- [ ] **Step 2: Создать фейк `rsync`**

Файл `tests/shell/lib/fakes/rsync` (исполняемый):
```bash
#!/usr/bin/env bash
# Фейк rsync. Пишет полный argv в $FAKE_RSYNC_LOG.
# Возвращает $FAKE_RSYNC_EXIT (default 0).
echo "rsync $*" >> "${FAKE_RSYNC_LOG:-/dev/null}"
exit "${FAKE_RSYNC_EXIT:-0}"
```

```bash
chmod +x tests/shell/lib/fakes/ssh tests/shell/lib/fakes/rsync
```

- [ ] **Step 3: Написать failing-тест happy path**

Файл `tests/shell/test_deploy_to_pi_happy.sh`:
```bash
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
source "$REPO_ROOT/scripts/cmds/compute.sh"

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
```

- [ ] **Step 4: Запустить тест — должен упасть**

```bash
chmod +x tests/shell/test_deploy_to_pi_happy.sh
bash tests/shell/test_deploy_to_pi_happy.sh
```
Expected: падает с ошибкой что `deploy_to_pi` не определена.

- [ ] **Step 5: Добавить source-guard в конец `compute.sh`**

Сейчас `scripts/cmds/compute.sh` заканчивается `main "$@"`. Если тест делает `source` — выполнится main без аргументов и упадёт. Перед `main "$@"` заменить на:
```bash
# Запускать main только при прямом исполнении, не при source (для тестов).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
```

- [ ] **Step 6: Добавить `deploy_to_pi()` в `scripts/cmds/compute.sh`**

Найти в `scripts/cmds/compute.sh` блок с функциями (после `build_frontend`, до `start_samcan_bridge` или в любом подходящем месте до `cmd_compute`). Вставить функцию:

```bash
# ── Деплой кода на Pi через rsync + рестарт samurai-robot ──────────────────
# Использование: deploy_to_pi <pi_ip> <pi_user> <pi_path> [ssh_key]
# Завершается через die при любой ошибке. На успехе — log_ok.
deploy_to_pi() {
    local pi_ip="$1"
    local pi_user="$2"
    local pi_path="$3"
    local ssh_key="${4:-}"

    local ssh_opts=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new -o BatchMode=yes)
    [[ -n "$ssh_key" ]] && ssh_opts+=(-i "$ssh_key")
    local ssh_target="$pi_user@$pi_ip"

    log_step "Деплой кода на Pi ($ssh_target:$pi_path)"

    # 1. Pre-flight: SSH доступен?
    if ! ssh "${ssh_opts[@]}" "$ssh_target" true 2>/dev/null; then
        die "SSH до $ssh_target недоступен. Проверь: Pi включён, ключ в ~/.ssh/authorized_keys на Pi, sshd работает."
    fi

    # 2. Pre-flight: установлен ли systemd-юнит?
    if ! ssh "${ssh_opts[@]}" "$ssh_target" \
            'systemctl list-unit-files samurai-robot.service --no-pager' 2>/dev/null \
            | grep -q samurai-robot; then
        die "samurai-robot.service не установлен на Pi. Запусти: ssh $ssh_target 'cd Samurai && sudo ./scripts/bootstrap_pi.sh'"
    fi

    # 3. rsync.
    log_info "rsync (.deployignore применён)..."
    # Собираем -e ssh строку с теми же опциями.
    local ssh_cmd="ssh ${ssh_opts[*]}"
    if ! rsync -az --delete \
            --exclude-from="$SAMURAI_ROOT/.deployignore" \
            -e "$ssh_cmd" \
            "$SAMURAI_ROOT/" "$ssh_target:$pi_path/"; then
        die "rsync на $ssh_target провалился"
    fi

    # 4. Рестарт сервиса.
    log_info "Рестарт samurai-robot..."
    if ! ssh "${ssh_opts[@]}" "$ssh_target" 'sudo systemctl restart samurai-robot'; then
        die "systemctl restart провалился. NOPASSWD настроен? Запусти на Pi: sudo ./scripts/systemd/install.sh --with-remote-deploy"
    fi

    # 5. Verify — дать пару секунд и проверить is-active.
    sleep 2
    local status
    status=$(ssh "${ssh_opts[@]}" "$ssh_target" \
             'systemctl is-active samurai-robot' 2>/dev/null || true)
    if [[ "$status" != "active" ]]; then
        log_err "samurai-robot не active (status=$status). Логи:"
        ssh "${ssh_opts[@]}" "$ssh_target" \
            'journalctl -u samurai-robot -n 30 --no-pager' >&2 || true
        die "Робот не стартовал — compute-стек не поднимаю"
    fi
    log_ok "samurai-robot запущен на Pi"
}
```

- [ ] **Step 7: Запустить тест — должен пройти**

```bash
bash tests/shell/test_deploy_to_pi_happy.sh
```
Expected: `Passed: 8  Failed: 0`.

- [ ] **Step 8: Commit**

```bash
git add tests/shell/lib/fakes/ssh tests/shell/lib/fakes/rsync \
        tests/shell/test_deploy_to_pi_happy.sh scripts/cmds/compute.sh
git commit -m "feat(deploy): deploy_to_pi() happy path

Функция в scripts/cmds/compute.sh для синка кода на Pi и
рестарта samurai-robot.service. Используется в compute.sh
при флаге --pi (см. Task 4).

Юнит-тест через PATH-override фейки ssh/rsync проверяет:
- pre-flight ssh с ConnectTimeout=5
- проверка наличия systemd-юнита
- rsync -az --delete --exclude-from=.deployignore
- systemctl restart samurai-robot через sudo
- verify через systemctl is-active"
```

---

## Task 3: `deploy_to_pi()` failure paths

**Files:**
- Create: `tests/shell/test_deploy_to_pi_failures.sh`

- [ ] **Step 1: Написать тесты на все failure paths**

Файл `tests/shell/test_deploy_to_pi_failures.sh`:
```bash
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
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/cmds/compute.sh"

# ── Test 1: SSH недоступен ─────────────────────────────────────────────────
test_start "fail when SSH unreachable"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
export FAKE_SSH_FAIL_ON_MATCH="true"  # фейк падает на pre-flight 'ssh ... true'

output=$(deploy_to_pi "10.0.0.99" "pi" "/home/pi/Samurai" "" 2>&1) || ec=$?
assert_eq "1" "${ec:-0}" "exits non-zero on SSH fail"
assert_contains "$output" "SSH до pi@10.0.0.99 недоступен" "error message"
unset FAKE_SSH_FAIL_ON_MATCH
rm -f "$FAKE_SSH_LOG" "$FAKE_RSYNC_LOG"

# ── Test 2: systemd unit не установлен ─────────────────────────────────────
test_start "fail when unit not installed"
FAKE_SSH_LOG=$(mktemp)
FAKE_RSYNC_LOG=$(mktemp)
export FAKE_SSH_LOG FAKE_RSYNC_LOG
# Кастомизируем фейк — он печатает строку с unit-files, но без samurai-robot.
# В фейке case ставит "samurai-robot.service enabled enabled" — нужно перебить.
# Используем FAKE_SSH_FAIL_ON_MATCH для list-unit-files строки чтобы фейк вернул пусто и exit 1 → grep -q даст fail.
# Проще: задаём FAKE_SSH_EXIT=1 только для команды list-unit-files. Но фейк это не умеет.
# Альтернатива: перепишем фейк добавив FAKE_SSH_UNIT_MISSING=1 переменную.
# Для этого теста требуется доработка фейка ssh — добавим.

# В этом шаге УБРАТЬ из фейка ssh кейс с list-unit-files и заменить на проверку FAKE_SSH_UNIT_MISSING.
# Если FAKE_SSH_UNIT_MISSING=1 — фейк ssh для list-unit-files печатает только заголовок без unit.

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
```

- [ ] **Step 2: Расширить фейк `ssh` поддержкой `FAKE_SSH_UNIT_MISSING`**

Открыть `tests/shell/lib/fakes/ssh`, заменить блок `case "$cmd_str"`:
```bash
case "$cmd_str" in
    *"systemctl list-unit-files samurai-robot.service"*)
        if [[ "${FAKE_SSH_UNIT_MISSING:-0}" == "1" ]]; then
            echo "UNIT FILE                          STATE   PRESET"
            # без строки samurai-robot — grep -q даст fail
        else
            echo "samurai-robot.service enabled enabled"
        fi
        ;;
    *"systemctl is-active samurai-robot"*)
        echo "${FAKE_IS_ACTIVE:-active}"
        ;;
    *"journalctl -u samurai-robot"*)
        echo "fake journalctl line 1"
        echo "fake journalctl line 2"
        ;;
    *)
        [[ -n "${FAKE_SSH_STDOUT:-}" ]] && echo "$FAKE_SSH_STDOUT"
        ;;
esac
```

- [ ] **Step 3: Запустить failure тесты**

```bash
chmod +x tests/shell/test_deploy_to_pi_failures.sh
bash tests/shell/test_deploy_to_pi_failures.sh
```
Expected: `Passed: 10  Failed: 0` (5 тестов × ~2 assert каждый).

- [ ] **Step 4: Запустить и happy-path для регрессии**

```bash
bash tests/shell/test_deploy_to_pi_happy.sh
```
Expected: всё ещё проходит.

- [ ] **Step 5: Commit**

```bash
git add tests/shell/test_deploy_to_pi_failures.sh tests/shell/lib/fakes/ssh
git commit -m "test(deploy): failure-path тесты для deploy_to_pi

Пять сценариев:
1. SSH недоступен → правильное сообщение
2. samurai-robot.service не установлен → подсказка про bootstrap_pi.sh
3. rsync exits non-zero → сообщение про rsync
4. systemctl restart падает → подсказка про --with-remote-deploy
5. is-active != active → дамп journalctl + non-zero exit"
```

---

## Task 4: Wire `deploy_to_pi` into `cmd_compute` + new flags

**Files:**
- Modify: `scripts/cmds/compute.sh` (arg parsing + call site)
- Create: `tests/shell/test_compute_flags.sh`

- [ ] **Step 1: Написать тесты на новые флаги**

Файл `tests/shell/test_compute_flags.sh`:
```bash
#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck source=lib/asserts.sh
source "$SCRIPT_DIR/lib/asserts.sh"

# Извлекаем функцию resolve_pi_user из compute.sh для проверки приоритета источников.
# Подход: source compute.sh, потом тестируем функции напрямую без вызова cmd_compute.
export SAMURAI_ROOT="$REPO_ROOT"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/common.sh"
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

tests_summary
```

- [ ] **Step 2: Запустить тест — должен упасть**

```bash
chmod +x tests/shell/test_compute_flags.sh
bash tests/shell/test_compute_flags.sh
```
Expected: падает потому что `resolve_pi_user`/`resolve_pi_path` не определены.

- [ ] **Step 3: Добавить helper-функции в `compute.sh`**

Вставить ПЕРЕД `deploy_to_pi()` в `scripts/cmds/compute.sh`:
```bash
# ── Резолв параметров деплоя с приоритетом CLI > env > default ─────────────
resolve_pi_user() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    if [[ -n "${SAMURAI_PI_USER:-}" ]]; then echo "$SAMURAI_PI_USER"; return; fi
    echo "pi"
}

resolve_pi_path() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    if [[ -n "${SAMURAI_PI_PATH:-}" ]]; then echo "$SAMURAI_PI_PATH"; return; fi
    echo "~/Samurai"
}

resolve_ssh_key() {
    local cli_val="${1:-}"
    if [[ -n "$cli_val" ]]; then echo "$cli_val"; return; fi
    echo "${SAMURAI_PI_SSH_KEY:-}"
}
```

- [ ] **Step 4: Запустить — все 6 ассертов должны пройти**

```bash
bash tests/shell/test_compute_flags.sh
```
Expected: `Passed: 6  Failed: 0`.

- [ ] **Step 5: Добавить парсинг новых флагов в `cmd_compute`**

Найти в `scripts/cmds/compute.sh` блок `while [[ $# -gt 0 ]]; do case "$1" in` (около строки 226). Добавить в верх блока (перед существующими кейсами) объявления переменных и в case-statement — новые опции:

```bash
# В блоке локальных переменных cmd_compute (поищи "local pi_ip_arg=" или начало функции):
local no_deploy=false
local pi_user_arg=""
local pi_path_arg=""
local ssh_key_arg=""
```

В `case "$1" in` добавить (можно перед `--pi)`):
```bash
            --no-deploy)       no_deploy=true; shift ;;
            --pi-user)         pi_user_arg="$2"; shift 2 ;;
            --pi-path)         pi_path_arg="$2"; shift 2 ;;
            --ssh-key)         ssh_key_arg="$2"; shift 2 ;;
```

- [ ] **Step 6: Вставить вызов `deploy_to_pi` в `cmd_compute`**

Найти место где `cmd_compute` уже вызвал `discover_pi` или подобное (где `pi_ip` уже резолвится). Прямо после получения `pi_ip`, до старта Docker/frontend — добавить:

```bash
# ── Автодеплой кода на Pi (если задан --pi и не --no-deploy) ──────────────
if [[ -n "$pi_ip" && "$no_deploy" != "true" ]]; then
    local resolved_user resolved_path resolved_key
    resolved_user=$(resolve_pi_user "$pi_user_arg")
    resolved_path=$(resolve_pi_path "$pi_path_arg")
    resolved_key=$(resolve_ssh_key "$ssh_key_arg")
    deploy_to_pi "$pi_ip" "$resolved_user" "$resolved_path" "$resolved_key"
elif [[ "$no_deploy" == "true" ]]; then
    log_info "Деплой пропущен (--no-deploy)"
fi
```

- [ ] **Step 7: Обновить help-секцию compute.sh**

Найти секцию help (строка с `--pi IP             IP Raspberry Pi`) и добавить ниже:
```
  --no-deploy         Не деплоить код на Pi (только поднять compute-стек)
  --pi-user USER      SSH-юзер на Pi (default: pi; env: SAMURAI_PI_USER)
  --pi-path PATH      Путь репо на Pi (default: ~/Samurai; env: SAMURAI_PI_PATH)
  --ssh-key FILE      SSH ключ для аутентификации (env: SAMURAI_PI_SSH_KEY)
```

- [ ] **Step 8: Проверить регрессии — все тесты вместе**

```bash
bash tests/shell/test_deployignore.sh
bash tests/shell/test_deploy_to_pi_happy.sh
bash tests/shell/test_deploy_to_pi_failures.sh
bash tests/shell/test_compute_flags.sh
```
Expected: все четыре `Failed: 0`.

- [ ] **Step 9: Commit**

```bash
git add tests/shell/test_compute_flags.sh scripts/cmds/compute.sh
git commit -m "feat(deploy): compute --pi auto-deploys to robot

Новые флаги в ./samurai.sh compute:
  --no-deploy            пропустить деплой
  --pi-user USER         SSH юзер (default pi, env SAMURAI_PI_USER)
  --pi-path PATH         путь репо на Pi (default ~/Samurai)
  --ssh-key FILE         альтернативный приватный ключ

Логика: после discover_pi, если pi_ip задан и нет --no-deploy,
вызывается deploy_to_pi() с резолвом параметров (CLI > env > default).

Юнит-тесты проверяют priority resolver-функций."
```

---

## Task 5: `install.sh --with-remote-deploy` flag

**Files:**
- Modify: `scripts/systemd/install.sh`
- Create: `tests/shell/test_install_sudoers.sh`

- [ ] **Step 1: Тест что генерируется валидный sudoers**

Файл `tests/shell/test_install_sudoers.sh`:
```bash
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
```

- [ ] **Step 2: Запустить — должен упасть**

```bash
chmod +x tests/shell/test_install_sudoers.sh
bash tests/shell/test_install_sudoers.sh
```
Expected: падает — функция `install_sudoers_remote_deploy` не существует.

- [ ] **Step 3: Добавить source-guard в конец `install.sh`**

Заменить последнюю строку файла `main "$@"` на:
```bash
# Запускать main только при прямом исполнении, не при source (для тестов).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
```

- [ ] **Step 4: Добавить функцию `install_sudoers_remote_deploy()` в `install.sh`**

Вставить перед функцией `main()` (после `install_unit()`):

```bash
# ── Генерация /etc/sudoers.d/samurai-robot для NOPASSWD-рестарта с ноута ───
# Использование: install_sudoers_remote_deploy <user> [sudoers_dir]
# sudoers_dir override-ится только для тестов (default /etc/sudoers.d).
install_sudoers_remote_deploy() {
    local user="$1"
    local sudoers_dir="${2:-/etc/sudoers.d}"
    local target="$sudoers_dir/samurai-robot"
    local tmpfile
    tmpfile=$(mktemp)

    cat > "$tmpfile" <<EOF
# Generated by ./scripts/systemd/install.sh --with-remote-deploy
# Allows $user to restart/inspect samurai-robot.service without password,
# so ./samurai.sh compute --pi can ssh-restart the service over LAN.
$user ALL=(root) NOPASSWD: /bin/systemctl restart samurai-robot, \\
                          /bin/systemctl is-active samurai-robot, \\
                          /bin/systemctl status samurai-robot, \\
                          /bin/journalctl -u samurai-robot *
EOF

    # visudo -cf обязателен — битый файл может залочить sudo.
    if command -v visudo &>/dev/null; then
        if ! visudo -cf "$tmpfile" &>/dev/null; then
            rm -f "$tmpfile"
            die "Сгенерированный sudoers не прошёл visudo -cf"
        fi
    fi

    # Атомарная установка с правильными правами.
    # Для тестов (non-root, override sudoers_dir) — fallback на install -m без -o.
    install -m 0440 -o root -g root "$tmpfile" "$target" 2>/dev/null \
        || install -m 0440 "$tmpfile" "$target"
    rm -f "$tmpfile"

    log_ok "Установлен $target (NOPASSWD для $user)"
}
```

- [ ] **Step 5: Рефакторнуть `main()` под proper arg parsing**

Текущий `main()` в `install.sh` обрабатывает только `--uninstall`/`--help`/positional unit names. Нужно добавить флаги `--user USER` и `--with-remote-deploy`. Заменить начало `main()` целиком (примерно строки 79-104):

```bash
main() {
    local with_remote_deploy=false
    local user_arg=""
    local positional=()

    # Парсинг аргументов.
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --uninstall|-u)
                uninstall_units; exit 0 ;;
            -h|--help)
                cat <<EOF
Использование:
  sudo $0 [OPTIONS] [unit1 unit2 ...]   # установить (default: все)
  sudo $0 --uninstall                    # удалить все

Опции:
  --user USER             Целевой юзер (default: \$SUDO_USER или 'pi')
  --with-remote-deploy    Также установить /etc/sudoers.d/samurai-robot
                          с NOPASSWD для удалённого рестарта с ноута
                          (./samurai.sh compute --pi)

После установки:
  sudo systemctl enable --now samurai-robot     # автозапуск
  systemctl status samurai-robot                # статус
  journalctl -u samurai-robot -f                # live-логи
EOF
                exit 0 ;;
            --user)
                user_arg="$2"; shift 2 ;;
            --with-remote-deploy)
                with_remote_deploy=true; shift ;;
            -*)
                die "Неизвестный флаг: $1 (см. --help)" ;;
            *)
                positional+=("$1"); shift ;;
        esac
    done

    require_root

    local user
    if [[ -n "$user_arg" ]]; then
        user="$user_arg"
    else
        user=$(determine_user)
    fi

    print_banner "S A M U R A I   S Y S T E M D" "Установка unit-файлов"
    log_ok "Целевой пользователь: ${BOLD}$user${NC}"
    log_ok "Корень репо:          ${BOLD}$SAMURAI_ROOT${NC}"
    log_ok "Unit-директория:      ${BOLD}$UNIT_DIR${NC}"
    echo ""
```

(Дальше всё что было от `# Создать директорию состояния` до конца `main()` остаётся БЕЗ изменений, но `"$@"` → используем `"${positional[@]}"` в блоке targets, т.е. заменить:)

```bash
    # Какие unit'ы устанавливать
    local targets=()
    if [[ ${#positional[@]} -eq 0 ]]; then
        targets=("${ALL_UNITS[@]}")
    else
        targets=("${positional[@]}")
    fi
```

И в самом конце `main()` (перед закрывающей `}`) добавить вызов sudoers-функции:
```bash
    if [[ "$with_remote_deploy" == "true" ]]; then
        log_step "Установка /etc/sudoers.d/samurai-robot"
        install_sudoers_remote_deploy "$user"
    fi
```

- [ ] **Step 6: Запустить sudoers тест**

```bash
bash tests/shell/test_install_sudoers.sh
```
Expected: `Passed: 7  Failed: 0` (или 8 если visudo доступен).

- [ ] **Step 7: Commit**

```bash
git add tests/shell/test_install_sudoers.sh scripts/systemd/install.sh
git commit -m "feat(deploy): install.sh --with-remote-deploy → sudoers.d

Опция генерирует /etc/sudoers.d/samurai-robot с NOPASSWD для
четырёх команд (restart, is-active, status, journalctl), нужных
для ssh-рестарта с ноута.

Генерация через mktemp + visudo -cf + install -m 0440 — атомарно
и валидируется до записи (битый sudoers ломает sudo всей машины)."
```

---

## Task 6: `scripts/bootstrap_pi.sh`

**Files:**
- Create: `scripts/bootstrap_pi.sh`

Этот скрипт не покрывается юнит-тестами полностью (apt/raspi-config требуют реальный Pi). Делаем `--dry-run` для smoke-теста и максимально декларативный код.

- [ ] **Step 1: Создать `scripts/bootstrap_pi.sh`**

```bash
#!/usr/bin/env bash
# =============================================================================
# scripts/bootstrap_pi.sh — одноразовая первичная настройка Raspberry Pi
# для работы с ./samurai.sh compute --pi (автодеплоем с ноута).
#
# Запуск (на Pi, под sudo):
#   ssh <user>@raspberrypi.local 'cd Samurai && sudo ./scripts/bootstrap_pi.sh'
#
# Опции:
#   --dry-run    Только напечатать команды, не выполнять
#   --user USER  Целевой юзер (default: $SUDO_USER или $USER)
#
# Идемпотентный — можно запускать повторно.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

# ── Парсинг аргументов ──────────────────────────────────────────────────────
DRY_RUN=false
TARGET_USER="${SUDO_USER:-${USER:-pi}}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=true; shift ;;
        --user)    TARGET_USER="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,15p' "$0"
            exit 0
            ;;
        *) die "Unknown arg: $1 (см. --help)" ;;
    esac
done

# ── run — обёртка для dry-run ───────────────────────────────────────────────
run() {
    if [[ "$DRY_RUN" == "true" ]]; then
        echo "  [dry-run] $*"
    else
        log_info "$*"
        "$@"
    fi
}

# ── Проверка что мы под root (или dry-run) ──────────────────────────────────
if [[ "$DRY_RUN" != "true" && $EUID -ne 0 ]]; then
    die "Скрипт нужно запускать под sudo (или с --dry-run для проверки)"
fi

log_step "Bootstrap Pi для samurai (user=$TARGET_USER, dry_run=$DRY_RUN)"

# 1. APT-пакеты ──────────────────────────────────────────────────────────────
log_step "1/6 APT-пакеты"
run apt-get update
run apt-get install -y \
    mosquitto mosquitto-clients \
    python3-pip python3-picamera2 \
    i2c-tools avahi-daemon \
    rsync openssh-server

# 2. Pip-зависимости под целевым юзером ──────────────────────────────────────
log_step "2/6 Pip-зависимости из requirements.txt"
if [[ -f "$REPO_ROOT/requirements.txt" ]]; then
    run sudo -u "$TARGET_USER" pip3 install --user -r "$REPO_ROOT/requirements.txt"
else
    log_warn "requirements.txt не найден — пропускаю pip"
fi

# 3. Группы (gpio/i2c/spi) ───────────────────────────────────────────────────
log_step "3/6 Группы hardware-доступа"
run usermod -aG gpio,i2c,spi "$TARGET_USER"
log_info "После добавления юзер должен перелогиниться чтобы группы применились"

# 4. raspi-config — I2C / camera ────────────────────────────────────────────
log_step "4/6 raspi-config (I2C + camera)"
if command -v raspi-config &>/dev/null; then
    run raspi-config nonint do_i2c 0
    run raspi-config nonint do_camera 0
else
    log_warn "raspi-config не найден — пропускаю (не на Pi?)"
fi

# 5. Сервисы (mosquitto, avahi, ssh) ─────────────────────────────────────────
log_step "5/6 Включение системных сервисов"
run systemctl enable --now mosquitto
run systemctl enable --now avahi-daemon
run systemctl enable --now ssh

# 6. Установка systemd-юнита + sudoers ──────────────────────────────────────
log_step "6/6 samurai-robot.service + sudoers (--with-remote-deploy)"
if [[ -f "$SCRIPT_DIR/systemd/install.sh" ]]; then
    run "$SCRIPT_DIR/systemd/install.sh" --user "$TARGET_USER" --with-remote-deploy
else
    die "$SCRIPT_DIR/systemd/install.sh не найден"
fi

# ── Финальная инструкция ────────────────────────────────────────────────────
log_step "✓ Готово"
cat <<EOF

Следующий шаг — на НОУТБУКЕ:

  1. Скопируй свой SSH-ключ на Pi:
       ssh-copy-id $TARGET_USER@raspberrypi.local

  2. Проверь что compute --pi работает:
       ./samurai.sh compute --pi raspberrypi.local

Если NOPASSWD не сработает — перелогинься в SSH-сессии (sudoers подхватятся).
EOF
```

```bash
chmod +x scripts/bootstrap_pi.sh
```

- [ ] **Step 2: Smoke-тест через `--dry-run`**

```bash
bash scripts/bootstrap_pi.sh --dry-run
```
Expected: скрипт печатает все 6 шагов с `[dry-run]` префиксом и не пытается ничего реально установить. Exit 0.

Если выводится ошибка `install.sh: Неизвестный флаг: --user` — значит Task 5 Step 5 (refactor `main()` с `--user`/`--with-remote-deploy` парсингом) был пропущен. Вернись и сделай.

- [ ] **Step 3: Smoke-тест через несуществующего юзера**

```bash
bash scripts/bootstrap_pi.sh --dry-run --user nobody
```
Expected: все шаги печатаются с `user=nobody`. Без реального запуска опасностей нет.

- [ ] **Step 4: Commit**

```bash
git add scripts/bootstrap_pi.sh
git commit -m "feat(deploy): bootstrap_pi.sh — одноразовая настройка Pi

Шесть шагов:
1. apt install mosquitto python3-picamera2 i2c-tools avahi rsync ssh
2. pip install -r requirements.txt (под $TARGET_USER, не root)
3. usermod -aG gpio,i2c,spi
4. raspi-config nonint do_i2c 0, do_camera 0
5. systemctl enable --now mosquitto avahi-daemon ssh
6. scripts/systemd/install.sh --with-remote-deploy

Идемпотентный, имеет --dry-run для smoke-теста, определяет
TARGET_USER через \$SUDO_USER || \$USER || pi."
```

---

## Task 7: Runner и интеграция в CI

**Files:**
- Create: `tests/shell/run_all.sh`
- Modify: `tests/conftest.py` (если есть, для pytest-интеграции) **ИЛИ** не трогать pytest, добавить отдельный CI-шаг

- [ ] **Step 1: Создать раннер всех bash-тестов**

Файл `tests/shell/run_all.sh`:
```bash
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
```

```bash
chmod +x tests/shell/run_all.sh
```

- [ ] **Step 2: Прогнать все**

```bash
bash tests/shell/run_all.sh
```
Expected: все файлы `Failed: 0`, итог `Total failed: 0`.

- [ ] **Step 3: Найти существующий CI-конфиг и добавить шаг**

```bash
ls .github/workflows/ 2>/dev/null
```

Если есть workflow (например `.github/workflows/tests.yml`) — добавить job или step:

```yaml
  shell-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install rsync
        run: sudo apt-get update && sudo apt-get install -y rsync
      - name: Run shell tests
        run: bash tests/shell/run_all.sh
```

Если CI-конфигов нет — просто пропустить этот шаг, run_all.sh используется локально перед коммитом.

- [ ] **Step 4: Commit**

```bash
git add tests/shell/run_all.sh
# и .github/workflows/*.yml если был добавлен step
git commit -m "test(deploy): shell-test runner + CI integration

tests/shell/run_all.sh запускает все test_*.sh и агрегирует
результат. Добавлен step в GitHub Actions (если workflow существовал)."
```

---

## Task 8: Documentation

**Files:**
- Modify: `README.md`
- Modify: `memory/MEMORY.md`

- [ ] **Step 1: Найти секцию про запуск в README.md**

```bash
grep -n "samurai.sh compute\|## Запуск\|## Quickstart\|## Startup" README.md | head -10
```

- [ ] **Step 2: Добавить раздел в README.md**

Вставить после существующей секции про `compute` (или в Quickstart):

```markdown
### Деплой на робота одной командой (compute --pi)

После первичной настройки Pi (один раз) автодеплой с ноута работает по LAN:

**Первичная настройка Pi (один раз):**
1. Склонировать репо на Pi.
2. Запустить `sudo ./scripts/bootstrap_pi.sh` — поставит apt-пакеты, pip-зависимости,
   включит I2C/camera, поднимет mosquitto/ssh, установит `samurai-robot.service`,
   настроит sudoers.d для NOPASSWD рестарта.
3. С ноута: `ssh-copy-id <user>@raspberrypi.local` чтобы пробросить SSH-ключ.

**Каждый день:**
```bash
./samurai.sh compute --pi raspberrypi.local
```
Эта команда:
- rsync рабочего дерева ноута на Pi (исключения в `.deployignore`).
- `sudo systemctl restart samurai-robot` через SSH.
- Проверяет `systemctl is-active samurai-robot`, при фейле — дамп `journalctl`.
- Поднимает ноутбучный стек (Docker, dashboard, ROS2).

Pi не нуждается в интернете — только в LAN-видимости с ноута.

**Доступные флаги:**
- `--no-deploy` — не пушить код (только compute-стек).
- `--pi-user USER` — SSH-юзер (default `pi`, env `SAMURAI_PI_USER`).
- `--pi-path PATH` — путь репо на Pi (default `~/Samurai`).
- `--ssh-key FILE` — альтернативный приватный ключ.

См. также: [`docs/superpowers/specs/2026-05-16-compute-autodeploy-design.md`](docs/superpowers/specs/2026-05-16-compute-autodeploy-design.md).
```

- [ ] **Step 3: Обновить `memory/MEMORY.md` секцию Startup**

Найти в `memory/MEMORY.md` блок `## Startup — единый CLI ./samurai.sh`. После строки про `compute` добавить заметку:

```markdown
- **Автодеплой:** `compute --pi` теперь синкает код на Pi (rsync) и рестартует
  `samurai-robot.service` через SSH перед поднятием compute-стека. Pi не нужен
  интернет. Однократная настройка: `sudo ./scripts/bootstrap_pi.sh` на Pi +
  `ssh-copy-id` с ноута. Подробности: `docs/superpowers/specs/2026-05-16-compute-autodeploy-design.md`.
```

- [ ] **Step 4: Verify markdown рендерится**

```bash
# Если установлен markdownlint:
command -v markdownlint && markdownlint README.md docs/superpowers/specs/2026-05-16-compute-autodeploy-design.md || true
# Или просто визуально — что блоки и таблицы не сломаны:
head -100 README.md
```

- [ ] **Step 5: Commit**

```bash
git add README.md memory/MEMORY.md
git commit -m "docs(deploy): README + MEMORY обновлены про compute --pi автодеплой

Описаны:
- первичная настройка Pi через bootstrap_pi.sh + ssh-copy-id
- ежедневный сценарий ./samurai.sh compute --pi
- новые флаги (--no-deploy, --pi-user, --pi-path, --ssh-key)
- ссылка на дизайн-спеку"
```

---

## Финальная проверка

- [ ] **Step 1: Прогнать все shell-тесты**

```bash
bash tests/shell/run_all.sh
```
Expected: `Total failed: 0`.

- [ ] **Step 2: Прогнать pytest (регрессия)**

```bash
pytest tests/ -x --tb=short
```
Expected: все Python тесты проходят (этот PR не должен их трогать).

- [ ] **Step 3: Lint shell-скриптов**

```bash
command -v shellcheck && shellcheck \
    scripts/cmds/compute.sh \
    scripts/systemd/install.sh \
    scripts/bootstrap_pi.sh \
    tests/shell/lib/asserts.sh \
    tests/shell/*.sh \
    || echo "shellcheck не установлен — пропуск"
```
Expected: либо `shellcheck` молчит (success), либо warning'и приемлемы (style, не bugs).

- [ ] **Step 4: Манual smoke (опционально, требует Pi)**

Если есть доступ к настоящему Pi:
1. На Pi: `cd ~/Samurai && sudo ./scripts/bootstrap_pi.sh`
2. На ноуте: `ssh-copy-id pi@raspberrypi.local`
3. На ноуте: `./samurai.sh compute --pi raspberrypi.local`
4. Проверить: `ssh pi@raspberrypi.local 'systemctl is-active samurai-robot'` → `active`.
5. Внести правку локально (например `echo "# test" >> pi_nodes/__init__.py`).
6. Снова `./samurai.sh compute --pi raspberrypi.local` — деплой должен быть быстрым (rsync инкремент).
7. Проверить что правка попала: `ssh pi@raspberrypi.local 'tail -1 ~/Samurai/pi_nodes/__init__.py'`.

---

## Self-Review Checklist (заполняется автором плана при сдаче)

**Spec coverage:**
- [x] Проблема и цель → описаны во Введении плана.
- [x] Триггер `compute` автодеплоит → Task 4 (точка вызова `deploy_to_pi`).
- [x] Рабочее дерево как есть → rsync без `git archive` (Task 2).
- [x] `.deployignore` → Task 1.
- [x] Systemd рестарт → Task 2 (deploy_to_pi step 4).
- [x] Failure mode (логи, не rollback) → Task 2 step 5 + Task 3.
- [x] Inline в `compute.sh` → Task 2/4.
- [x] `install.sh --with-remote-deploy` → Task 5.
- [x] `bootstrap_pi.sh` → Task 6.
- [x] Конфигурация (CLI > env > default) → Task 4 steps 1-4.
- [x] Файл `~/.samurai/deploy.conf` → ОТЛОЖЕН (помечено в спеке как «опционально»). YAGNI — env-vars покрывают 90% случаев.
- [x] Тестирование → Tasks 1-5 включают юнит-тесты, Task 7 раннер.
- [x] Edge cases (hotspot, --no-deploy, конкурентный деплой) → конкурентный deploy через flock **НЕ реализован** (v1 single dev — упомянуто как future work в спеке).

**Placeholder scan:** TBD/TODO в плане отсутствуют (проверено grep'ом по самому документу).

**Type consistency:** Имена функций (`deploy_to_pi`, `resolve_pi_user`, `resolve_pi_path`, `resolve_ssh_key`, `install_sudoers_remote_deploy`) одинаковы во всех тасках где упомянуты. Переменные `pi_user_arg`/`pi_path_arg`/`ssh_key_arg`/`no_deploy` — одинаково именованы в Task 4 steps 5 и 6.
