"""
config_loader — loads config.yaml relative to the project root.

Usage:
    from config_loader import cfg
    timeout = cfg('network.ip_detection_timeout', default=2.0)
"""

import os
import yaml

# Project root is the directory containing this file
_ROOT = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_ROOT, 'config.yaml')

_data: dict = {}


def _load():
    global _data
    if _data:
        return
    try:
        with open(_CONFIG_PATH, 'r', encoding='utf-8') as f:
            _data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        pass  # all callers use defaults
    except yaml.YAMLError:
        pass


def cfg(key: str, default=None):
    """
    Dot-notation lookup, e.g. cfg('network.dashboard_port', 5000).
    Falls back to *default* if config.yaml is missing or key not found.
    """
    _load()
    parts = key.split('.')
    node = _data
    for p in parts:
        if not isinstance(node, dict) or p not in node:
            return default
        node = node[p]
    return node
