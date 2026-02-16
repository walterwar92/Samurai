#!/bin/bash
# =============================================================================
# Run with: sudo bash setup_robot.sh
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [ "$EUID" -ne 0 ]; then
    log_error "Run as root: sudo bash setup_robot.sh"
    exit 1
fi

# ── Phase 1: System packages ──────────────────────────────────────────
log_info "Updating package lists..."
apt-get update

log_info "Installing system dependencies..."
apt-get install -y \
    python3-full \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-opencv \
    python3-picamera2 \
    python3-libcamera \
    python3-pyaudio \
    python3-smbus \
    libcap-dev \
    pigpio \
    python3-pigpio \
    i2c-tools \
    libasound2-dev \
    portaudio19-dev

# ── Phase 2: Enable interfaces (with I2C validation) ────────────────
log_info "Checking I2C status..."

I2C_STATUS=$(raspi-config nonint get_i2c 2>/dev/null || echo "unknown")
if [ "$I2C_STATUS" = "1" ]; then
    log_warn "I2C is DISABLED — enabling now..."
    raspi-config nonint do_i2c 0    # 0 = enable
    log_info "I2C enabled via raspi-config"
elif [ "$I2C_STATUS" = "0" ]; then
    log_info "I2C is already enabled"
else
    log_warn "Could not detect I2C status — forcing enable..."
    raspi-config nonint do_i2c 0
fi

# Load I2C kernel module if not loaded
if ! lsmod | grep -q i2c_dev; then
    log_warn "i2c-dev kernel module not loaded — loading..."
    modprobe i2c-dev
    log_info "i2c-dev module loaded"
else
    log_info "i2c-dev kernel module already loaded"
fi

# Verify /dev/i2c-1 exists
if [ -e /dev/i2c-1 ]; then
    log_info "I2C bus /dev/i2c-1 — OK"
else
    log_warn "I2C bus /dev/i2c-1 not found — will be available after reboot"
fi

log_info "Enabling SPI and Camera interfaces..."
raspi-config nonint do_spi 0
raspi-config nonint do_camera 0 2>/dev/null || true

# ── Phase 3: Pip packages (with --break-system-packages) ─────────────
log_info "Installing pip packages (hardware drivers)..."
pip3 install --break-system-packages \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-servokit \
    smbus2 \
    rpi_ws281x \
    gpiozero

# ── Phase 4: Vosk (speech recognition) ───────────────────────────────
log_info "Installing Vosk..."
pip3 install --break-system-packages vosk

VOSK_MODEL_DIR="/home/pi/vosk-model-ru"
if [ ! -d "$VOSK_MODEL_DIR" ]; then
    log_info "Downloading Vosk Russian model..."
    VOSK_URL="https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip"
    cd /tmp
    wget -q "$VOSK_URL" -O vosk-model-ru.zip
    unzip -q vosk-model-ru.zip -d /home/pi/
    mv /home/pi/vosk-model-small-ru-0.22 "$VOSK_MODEL_DIR"
    rm vosk-model-ru.zip
    chown -R pi:pi "$VOSK_MODEL_DIR"
    log_info "Vosk model installed to $VOSK_MODEL_DIR"
else
    log_info "Vosk model already exists at $VOSK_MODEL_DIR"
fi

# ── Phase 5: Pigpio daemon (for better HC-SR04 accuracy) ─────────────
log_info "Enabling pigpio daemon..."
systemctl enable pigpiod
systemctl start pigpiod

# ── Phase 6: Verify installation ─────────────────────────────────────
log_info "Verifying installed packages..."
FAILED=0

check_package() {
    python3 -c "import $1" 2>/dev/null
    if [ $? -eq 0 ]; then
        log_info "  $1 — OK"
    else
        log_warn "  $1 — FAILED"
        FAILED=$((FAILED + 1))
    fi
}

check_package "picamera2"
check_package "cv2"
check_package "numpy"
check_package "pyaudio"
check_package "vosk"
check_package "smbus2"
check_package "gpiozero"
check_package "board"
check_package "busio"
check_package "adafruit_pca9685"
check_package "adafruit_motor"
check_package "rpi_ws281x"

echo ""
if [ $FAILED -eq 0 ]; then
    log_info "All packages installed successfully!"
else
    log_warn "$FAILED package(s) failed verification. Check logs above."
fi

# ── Phase 7: I2C device check ────────────────────────────────────────
log_info "Scanning I2C bus..."
i2cdetect -y 1 2>/dev/null || log_warn "I2C scan failed — reboot may be required"

echo ""
log_info "Setup complete! Reboot recommended: sudo reboot"
