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
# На Debian 13 Trixie / RPi OS Bookworm+ системный Python — PEP 668 (externally
# managed). Pip отказывается ставить без флага --break-system-packages.
# Это легитимный override для контролируемого Pi-окружения (альтернатива —
# venv, но systemd-юнит ожидает user-site пакеты, см. SAMURAI_SKIP_PIP_INSTALL).
log_step "2/6 Pip-зависимости из requirements.txt"
if [[ -f "$REPO_ROOT/requirements.txt" ]]; then
    run sudo -u "$TARGET_USER" pip3 install --user --break-system-packages \
        -r "$REPO_ROOT/requirements.txt"
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
