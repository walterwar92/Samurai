# SAMURAI Robot - Diagnostic Report

**Date:** 2026-02-16
**Platform:** Raspberry Pi 4 Model B Rev 1.5 (aarch64)
**OS:** Linux 6.12.62+rpt-rpi-v8, Python 3.13.5
**HAT:** Adeept Robot HAT V3.1

---

## Summary

Two diagnostic runs were performed:
1. **Run 1** (I2C/SPI disabled): 8 OK, 3 WARN, **6 ERROR** — I2C bus not found, cascade failures
2. **Run 2** (I2C/SPI enabled): 14 OK, 2 WARN, **1 ERROR** — most hardware operational

After enabling I2C/SPI interfaces in `raspi-config`, the majority of hardware works correctly.

---

## Hardware Status (Run 2 - with I2C enabled)

| Module | Status | Details |
|--------|--------|---------|
| system_info | OK | RPi 4B, 42.8C, 3.8GB RAM, 8.67GB free |
| python_packages | OK | Core packages installed |
| gpio | OK | All pins accessible |
| i2c_scan | OK | Found: 0x48 (ADS7830), 0x5F (PCA9685), 0x70 (unknown) |
| ads7830 | OK | 8-channel ADC, all channels reading |
| pca9685 | OK | PWM @ 50Hz, 16 channels |
| drv8833 | OK | Dual H-Bridge motor drivers |
| lm324 | OK | Op-amp (passive, indirect check) |
| motors | OK | 4 DC motors tested (forward, back, left, right) |
| servos_ad002 | OK | 6 servos tested (0-180 degree range) |
| ultrasonic | OK | HC-SR04 operational (GPIO23/24) |
| line_tracking | OK | 3-channel IR sensors (GPIO16/19/20) |
| ws2812_led | OK | 16 RGB LEDs, all colours tested |
| **camera** | **ERR** | `No module named 'picamera2'` |
| **microphone** | **WARN** | 0 input devices found (no USB mic) |
| **vosk** | **WARN** | Vosk not installed |

---

## Issues Found

### CRITICAL: Missing Python Packages

| Package | Required By | Status |
|---------|------------|--------|
| `picamera2` | camera_node.py | NOT INSTALLED |
| `cv2` (opencv) | camera_node.py, compute nodes | NOT INSTALLED |
| `numpy` | camera_node.py, compute nodes | NOT INSTALLED |
| `vosk` | voice_node.py | NOT INSTALLED |
| `rclpy` | ALL ROS2 nodes | NOT INSTALLED |
| `sensor_msgs` | camera, ultrasonic, imu nodes | NOT INSTALLED |
| `geometry_msgs` | motor, fsm nodes | NOT INSTALLED |
| `std_msgs` | voice, servo, laser, fsm nodes | NOT INSTALLED |
| `cv_bridge` | camera_node.py | NOT INSTALLED |

### CRITICAL: Package Installation Blocked (PEP 668)

```
error: externally-managed-environment
This environment is externally managed
```

`pip3 install` fails on this system. Packages must be installed via:
- `sudo apt install python3-xyz` (for apt-available packages)
- Virtual environment (`python3 -m venv`)
- `pip3 install --break-system-packages` (not recommended)

Also missing: `libcap-dev` headers required for building some wheels.

### HIGH: No Graceful Degradation in Critical Nodes

These nodes will **crash immediately** if hardware/packages are unavailable:

| Node | Missing Fallback | Impact |
|------|-----------------|--------|
| `camera_node.py` | No try-except for picamera2/cv2/numpy | Node crashes, no camera |
| `motor_node.py` | No try-except for hardware init | Node crashes, no movement |
| `motor_driver.py` | No fallback for PCA9685/adafruit_motor | Cascade crash |
| `servo_driver.py` | No fallback for PCA9685/adafruit_motor | Cascade crash |
| `servo_node.py` | No fallback for ServoDriver | Node crashes, no claw |
| `pca9685_driver.py` | No fallback for board/busio/I2C | Cascade crash |
| `ultrasonic_node.py` | No fallback for gpiozero | Node crashes, no ranging |

**Already well-handled (reference patterns):**
- `voice_node.py` — `_HW` flag with graceful degradation
- `laser_node.py` — `_HW` flag with graceful degradation
- `imu_node.py` — `_HW` flag with graceful degradation

### MEDIUM: USB Microphone Not Detected

- 4 audio devices found, all output-only (0 input channels)
- No USB microphone connected or not recognized
- ALSA configuration has many unknown PCM entries
- JACK server not running

### LOW: HC-SR04 Accuracy Warning

```
PWMSoftwareFallback: For more accurate readings, use the pigpio pin factory.
```

gpiozero pin factory is `None` — using software PWM fallback for ultrasonic sensor.

---

## Fix Plan

### Phase 1: Code Resilience (graceful degradation)

Add `_HW` flag pattern (like voice_node/laser_node/imu_node) to all nodes that lack it:

1. **pca9685_driver.py** — wrap board/busio/adafruit imports in try-except
2. **motor_driver.py** — wrap adafruit_motor import, add simulation mode
3. **servo_driver.py** — wrap adafruit_motor import, add simulation mode
4. **camera_node.py** — wrap picamera2/cv2/numpy imports, add simulation mode
5. **motor_node.py** — handle MotorDriver init failure
6. **servo_node.py** — handle ServoDriver init failure
7. **ultrasonic_node.py** — wrap gpiozero import, add simulation mode

### Phase 2: Package Installation (on RPi)

Create a setup script (`setup_robot.sh`) that:
1. Installs system packages via apt: `python3-opencv`, `python3-numpy`, `python3-picamera2`
2. Installs ROS2 Humble/Jazzy packages
3. Installs vosk in a venv or via `--break-system-packages`
4. Installs `libcap-dev` for building wheels
5. Verifies installation

### Phase 3: Hardware (manual actions)

1. Connect USB microphone
2. Install pigpio for better HC-SR04 accuracy: `sudo apt install pigpio python3-pigpio`
3. Investigate unknown I2C device at 0x70
