#!/usr/bin/env python3
"""
hardware_presets — Load/save hardware configuration presets.

Presets define the mapping of physical ports/pins to functional blocks
(motors, servos, camera, IMU, range sensor, LEDs) allowing the same
software to run on different robot platforms (RPi+PCA9685, Arduino, ESP32).

Each preset is a separate YAML file in hardware_presets/ directory.
Active preset name is tracked in hardware_presets/_active.txt.
"""

import os
import re
import threading
from datetime import datetime, timezone

import yaml

_ROOT = os.path.dirname(os.path.abspath(__file__))
_PRESETS_DIR = os.path.join(_ROOT, 'hardware_presets')
_ACTIVE_FILE = os.path.join(_PRESETS_DIR, '_active.txt')
_lock = threading.Lock()


def _ensure_dir():
    """Create presets directory if missing; generate default preset."""
    if not os.path.isdir(_PRESETS_DIR):
        os.makedirs(_PRESETS_DIR, exist_ok=True)
    # Auto-generate default preset if empty
    yamls = [f for f in os.listdir(_PRESETS_DIR) if f.endswith('.yaml')]
    if not yamls:
        save_preset(generate_default())
        set_active('Samurai Default')


def _slugify(name: str) -> str:
    """Convert preset name to safe filename."""
    slug = name.strip().lower()
    slug = re.sub(r'[^a-z0-9а-яё]+', '_', slug, flags=re.UNICODE)
    slug = slug.strip('_')
    return slug or 'preset'


def _filename(name: str) -> str:
    return os.path.join(_PRESETS_DIR, _slugify(name) + '.yaml')


# -- CRUD ------------------------------------------------------------------

def list_presets() -> list:
    """Return list of preset summaries: [{name, platform, description, modified}]."""
    _ensure_dir()
    result = []
    for fname in sorted(os.listdir(_PRESETS_DIR)):
        if not fname.endswith('.yaml'):
            continue
        path = os.path.join(_PRESETS_DIR, fname)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            result.append({
                'name': data.get('name', fname.replace('.yaml', '')),
                'platform': data.get('platform', 'custom'),
                'description': data.get('description', ''),
                'modified': data.get('modified', ''),
            })
        except Exception:
            continue
    return result


def get_preset(name: str) -> dict | None:
    """Load full preset by name. Tries slug match first, then scans."""
    _ensure_dir()
    # Direct slug match
    path = _filename(name)
    if os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    # Scan by name field
    for fname in os.listdir(_PRESETS_DIR):
        if not fname.endswith('.yaml'):
            continue
        fpath = os.path.join(_PRESETS_DIR, fname)
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if data.get('name') == name:
                return data
        except Exception:
            continue
    return None


def save_preset(data: dict) -> str:
    """Save preset. data must contain 'name'. Returns filename."""
    _ensure_dir()
    name = data.get('name', 'Unnamed')
    now = datetime.now(timezone.utc).isoformat(timespec='seconds')
    data.setdefault('created', now)
    data['modified'] = now
    fname = _slugify(name) + '.yaml'
    path = os.path.join(_PRESETS_DIR, fname)
    with _lock:
        with open(path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False,
                      sort_keys=False)
    return fname


def delete_preset(name: str) -> bool:
    """Delete preset file. Returns True if existed."""
    _ensure_dir()
    path = _filename(name)
    if os.path.isfile(path):
        with _lock:
            os.remove(path)
        return True
    # Scan by name
    for fname in os.listdir(_PRESETS_DIR):
        if not fname.endswith('.yaml'):
            continue
        fpath = os.path.join(_PRESETS_DIR, fname)
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if data.get('name') == name:
                with _lock:
                    os.remove(fpath)
                return True
        except Exception:
            continue
    return False


def get_active() -> str:
    """Return currently active preset name."""
    _ensure_dir()
    try:
        with open(_ACTIVE_FILE, 'r', encoding='utf-8') as f:
            return f.read().strip()
    except FileNotFoundError:
        return ''


def set_active(name: str):
    """Set the active preset name."""
    _ensure_dir()
    with _lock:
        with open(_ACTIVE_FILE, 'w', encoding='utf-8') as f:
            f.write(name)


# -- Default preset generation ---------------------------------------------

def generate_default() -> dict:
    """Build default preset from current hardcoded values."""
    return {
        'name': 'Samurai Default',
        'platform': 'raspberry_pi_pca9685',
        'description': 'Adeept Robot HAT V3.1 — 4 мотора, 5 серво, WS2812B',

        'motors': {
            'driver': 'pca9685',
            'count': 4,
            'channels': {
                'M1': {'in1': 15, 'in2': 14, 'label': 'Правый-передний'},
                'M2': {'in1': 12, 'in2': 13, 'label': 'Левый-передний'},
                'M3': {'in1': 11, 'in2': 10, 'label': 'Левый-задний'},
                'M4': {'in1': 8,  'in2': 9,  'label': 'Правый-задний'},
            },
        },

        'servos': {
            'driver': 'pca9685',
            'head': {
                'channel': 4,
                'home': 90,
                'min': 0,
                'max': 180,
            },
            'arm': {
                'channels': [0, 1, 2, 3],
                'home_angles': [0, 120, 0, 0],
                'min_angles': [0, 0, 0, 0],
                'max_angles': [120, 145, 180, 180],
                'labels': ['Основание', 'Сустав 1', 'Сустав 2', 'Клешня'],
            },
        },

        'imu': {
            'type': 'mpu6050',
            'address': '0x68',
        },

        'camera': {
            'type': 'usb',
            'device': '/dev/video0',
            'width': 640,
            'height': 480,
        },

        'range_sensor': {
            'type': 'hc_sr04',
            'trigger_pin': 23,
            'echo_pin': 24,
        },

        'leds': {
            'type': 'ws2812b',
            'gpio_pin': 10,
            'count': 12,
            'brightness': 0.3,
        },

        'i2c': {
            'pca9685_address': '0x5F',
            'pca9685_frequency': 50,
        },
    }
