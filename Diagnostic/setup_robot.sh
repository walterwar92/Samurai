#!/bin/bash
# =============================================================================
# Run with: sudo bash setup_robot.sh
# Совместимость: Raspberry Pi OS Trixie (Debian 13) / Bookworm (Debian 12)
# =============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Установить apt-пакет с обработкой ошибки
apt_install() {
    if apt-get install -y "$1" 2>/dev/null; then
        log_info "  $1 — установлен"
    else
        log_error "  $1 — НЕ установлен, пропуск"
        INSTALL_FAILED=$((INSTALL_FAILED + 1))
    fi
}

# Установить pip-пакет с обработкой ошибки
pip_install() {
    if pip3 install --break-system-packages "$1"; then
        log_info "  $1 — установлен"
    else
        log_error "  $1 — НЕ установлен, пропуск"
        INSTALL_FAILED=$((INSTALL_FAILED + 1))
    fi
}

INSTALL_FAILED=0

if [ "$EUID" -ne 0 ]; then
    log_error "Run as root: sudo bash setup_robot.sh"
    exit 1
fi

# Определяем реального пользователя (не root)
REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo pi)}"
REAL_HOME=$(eval echo "~$REAL_USER")
log_info "Пользователь: $REAL_USER, домашняя директория: $REAL_HOME"

# Путь к проекту (скрипт лежит в Diagnostic/, workspace — на уровень выше)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_DIR/ros_ws"

# ── Phase 1: System packages ──────────────────────────────────────────
log_info "Updating package lists..."
apt-get update

log_info "Installing system dependencies..."
for pkg in \
    python3-full \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-smbus \
    libcap-dev \
    i2c-tools \
    python3-opencv \
    python3-picamera2 \
    python3-libcamera \
    python3-lgpio \
    python3-gpiozero \
    build-essential \
    cmake \
    linux-libc-dev \
    python3-colcon-common-extensions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs; do
    apt_install "$pkg"
done

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

# ── Phase 3: Pip packages ─────────────────────────────────────────────
log_info "Installing pip packages (hardware drivers)..."
for pkg in \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-servokit \
    smbus2 \
    rpi_ws281x \
    gpiozero; do
    pip_install "$pkg"
done

log_info "Installing pip packages (ROS2 node dependencies)..."
# transforms3d: для вычислений ориентации (ekf, nav)
# Голосовое управление — на телефоне (VoskRecognizer.kt) через MQTT
for pkg in \
    transforms3d; do
    pip_install "$pkg"
done

# onnxruntime на Pi — ARM-сборка (опционально, если нет PyTorch)
if pip3 install --break-system-packages "onnxruntime" 2>/dev/null; then
    log_info "  onnxruntime — установлен (YOLO без PyTorch)"
else
    log_warn "  onnxruntime — ARM-сборка не найдена, пропуск"
    log_warn "  YOLO будет использовать PyTorch (если установлен)"
fi

# ── Phase 4: GPIO daemon ──────────────────────────────────────────────
# pigpiod — на Bookworm, lgpio — на Trixie
if command -v pigpiod &>/dev/null; then
    log_info "Enabling pigpio daemon..."
    systemctl enable pigpiod
    systemctl start pigpiod
else
    log_info "pigpiod не найден (Trixie использует lgpio) — пропуск"
fi

# ── Phase 5: Build C++ ROS2 nodes (robot_pkg_cpp) ─────────────────────
# Компилирует imu_node и motor_node — прямой доступ к I2C без Python/GIL
# Требует: ROS2 Humble, build-essential, cmake (установлены выше)
log_info "Building ROS2 C++ package (robot_pkg_cpp)..."

ROS_SETUP="/opt/ros/humble/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    log_warn "ROS2 Humble не найден ($ROS_SETUP) — пропуск сборки C++ нод"
    log_warn "После установки ROS2 выполните:"
    log_warn "  source /opt/ros/humble/setup.bash"
    log_warn "  cd $ROS_WS && colcon build --packages-select robot_pkg_cpp"
elif [ ! -d "$ROS_WS/src/robot_pkg_cpp" ]; then
    log_warn "Пакет robot_pkg_cpp не найден в $ROS_WS/src/ — пропуск"
else
    log_info "Сборка robot_pkg_cpp в $ROS_WS ..."
    sudo -u "$REAL_USER" bash -c "
        set -e
        source '$ROS_SETUP'
        cd '$ROS_WS'
        colcon build \
            --packages-select robot_pkg_cpp \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            2>&1
    "
    BUILD_EXIT=$?
    if [ $BUILD_EXIT -eq 0 ]; then
        log_info "robot_pkg_cpp собран успешно"
        log_info "Бинарники: $ROS_WS/install/robot_pkg_cpp/lib/robot_pkg_cpp/"
        log_info "  imu_node   — MPU6050 via direct I2C ioctl @ 50 Hz"
        log_info "  motor_node — PCA9685 via direct I2C ioctl @ 20 Hz"
    else
        log_warn "Сборка robot_pkg_cpp завершилась с ошибкой (код $BUILD_EXIT)"
        log_warn "Попробуйте вручную:"
        log_warn "  source /opt/ros/humble/setup.bash"
        log_warn "  cd $ROS_WS && colcon build --packages-select robot_pkg_cpp"
        INSTALL_FAILED=$((INSTALL_FAILED + 1))
    fi
fi

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
check_package "smbus2"
check_package "gpiozero"
check_package "board"
check_package "busio"
check_package "adafruit_pca9685"
check_package "adafruit_motor"
check_package "rpi_ws281x"

# onnxruntime — опционально
python3 -c "import onnxruntime" 2>/dev/null && \
    log_info "  onnxruntime — OK" || \
    log_warn "  onnxruntime — не установлен (опционально)"

echo ""
if [ $INSTALL_FAILED -gt 0 ]; then
    log_warn "$INSTALL_FAILED пакет(ов) не удалось установить. Смотри [ERROR] выше."
fi

if [ $FAILED -eq 0 ]; then
    log_info "All packages installed successfully!"
else
    log_warn "$FAILED package(s) failed verification. Check logs above."
fi

# ── Phase 7: I2C device check ────────────────────────────────────────
log_info "Scanning I2C bus..."
i2cdetect -y 1 2>/dev/null || log_warn "I2C scan failed — reboot may be required"
log_info "  Expected: 0x5F (PCA9685 моторы/серво), 0x68 (MPU6050 IMU)"

echo ""
log_info "Setup complete! Reboot recommended: sudo reboot"
