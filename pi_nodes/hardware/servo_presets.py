"""
Servo preset manager — save/load named positions to JSON file.

Presets are stored in servo_presets.json at the project root.
Format:
{
  "arm": {
    "grab": [0, 120, 90, 180],
    "rest": [0, 120, 0, 0]
  },
  "head": {
    "forward": 90,
    "left": 0
  }
}
"""

import json
import logging
import os
import threading

_log = logging.getLogger(__name__)

# Default path: project_root/servo_presets.json
_DEFAULT_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'servo_presets.json')


class ServoPresets:
    """Thread-safe preset manager for servo positions."""

    def __init__(self, path: str | None = None):
        self._path = os.path.abspath(path or _DEFAULT_PATH)
        self._lock = threading.Lock()
        self._data: dict = self._load()

    def _load(self) -> dict:
        try:
            with open(self._path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def _save(self):
        with open(self._path, 'w', encoding='utf-8') as f:
            json.dump(self._data, f, indent=2, ensure_ascii=False)

    def save_preset(self, group: str, name: str, value):
        """Save a preset. value = list of angles (arm) or float (head)."""
        with self._lock:
            self._data.setdefault(group, {})[name] = value
            self._save()
        _log.info('Preset saved: %s/%s = %s', group, name, value)

    def load_preset(self, group: str, name: str):
        """Load a preset. Returns value or None if not found."""
        with self._lock:
            return self._data.get(group, {}).get(name)

    def delete_preset(self, group: str, name: str) -> bool:
        """Delete a preset. Returns True if found and deleted."""
        with self._lock:
            grp = self._data.get(group, {})
            if name in grp:
                del grp[name]
                self._save()
                _log.info('Preset deleted: %s/%s', group, name)
                return True
            return False

    def list_presets(self, group: str) -> list[str]:
        """List preset names for a group."""
        with self._lock:
            return list(self._data.get(group, {}).keys())
