#!/usr/bin/env python3
"""
calibration_profiles — Load/save wheel calibration profiles.

Profiles store 3 coefficients that depend on the driving surface:
  - scale_fwd:   wheel scale forward  (real / odometric distance)
  - scale_bwd:   wheel scale backward (real / odometric distance)
  - motor_trim:  angular trim %       (compensate motor asymmetry)

Profiles are persisted in calibration_profiles.yaml at project root.
"""

import os
import threading

import yaml

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROFILES_PATH = os.path.join(_ROOT, 'calibration_profiles.yaml')
_lock = threading.Lock()


def _load_all():
    """Load the profiles file. Returns (active_name, profiles_dict)."""
    try:
        with open(_PROFILES_PATH, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (FileNotFoundError, yaml.YAMLError):
        data = {}
    return data.get('active', 'default'), data.get('profiles', {})


def _save_all(active, profiles):
    data = {'active': active, 'profiles': profiles}
    with _lock:
        with open(_PROFILES_PATH, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False,
                      sort_keys=False)


def list_profiles():
    """Return {name: {scale_fwd, scale_bwd, motor_trim, description}, ...}."""
    _, profiles = _load_all()
    return profiles


def get_active():
    """Return (active_name, coefficients_dict or None)."""
    active, profiles = _load_all()
    return active, profiles.get(active)


def get_profile(name):
    """Return profile dict or None."""
    _, profiles = _load_all()
    return profiles.get(name)


def save_profile(name, scale_fwd, scale_bwd, motor_trim, description=''):
    """Save or overwrite a profile."""
    active, profiles = _load_all()
    profiles[name] = {
        'scale_fwd': round(float(scale_fwd), 4),
        'scale_bwd': round(float(scale_bwd), 4),
        'motor_trim': round(float(motor_trim), 3),
        'description': str(description),
    }
    _save_all(active, profiles)


def set_active(name):
    """Set the active profile name. Returns True if profile exists."""
    active, profiles = _load_all()
    if name not in profiles:
        return False
    _save_all(name, profiles)
    return True


def delete_profile(name):
    """Delete a profile. Returns True if existed."""
    active, profiles = _load_all()
    if name not in profiles:
        return False
    del profiles[name]
    if active == name:
        active = next(iter(profiles), 'default')
    _save_all(active, profiles)
    return True
