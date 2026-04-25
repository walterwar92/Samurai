"""
config_loader — loads config.yaml relative to the project root.

Usage:
    from config_loader import cfg, get_mqtt_credentials
    timeout = cfg('network.ip_detection_timeout', default=2.0)
    user, pwd = get_mqtt_credentials()  # None, None если auth выключен
"""

import os
from typing import Optional, Tuple

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


# ── MQTT credentials resolver ────────────────────────────────────────────────
# Приоритет (от высшего к низшему):
#   1. Аргументы, явно переданные в MqttNode(username=..., password=...)
#   2. ENV vars SAMURAI_MQTT_USER / SAMURAI_MQTT_PASS
#   3. Файл ~/.samurai/mqtt.passwd (формат: "username:password" в одной строке)
#   4. config.yaml секция mqtt.auth.{username, password}
#   5. None, None → anonymous подключение (default)
#
# Файл (1.) и ENV (2.) — для production. config.yaml (4.) — для dev.
# Если включить mqtt.auth.enabled=false в config — auth выключается даже если
# файл существует (override для отладки).

_MQTT_PASSWD_FILE = os.path.expanduser('~/.samurai/mqtt.passwd')


def _read_passwd_file(path: str = _MQTT_PASSWD_FILE) -> Optional[Tuple[str, str]]:
    """Читает файл формата 'user:password\\n'. None если нет/невалидный."""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            line = f.readline().strip()
        if ':' not in line:
            return None
        user, _, pwd = line.partition(':')
        if not user or not pwd:
            return None
        return user, pwd
    except (FileNotFoundError, PermissionError, OSError):
        return None


def get_mqtt_credentials() -> Tuple[Optional[str], Optional[str]]:
    """
    Возвращает (username, password) для MQTT-подключения.
    Если auth отключён или не настроен — (None, None).

    Использование (на стороне клиента):
        user, pwd = get_mqtt_credentials()
        if user is not None:
            client.username_pw_set(user, pwd)
    """
    # Жёсткий выключатель — позволяет принудительно отключить auth, не удаляя
    # файл/ENV (для отладки).
    if cfg('mqtt.auth.enabled', None) is False:
        return None, None

    # 2. ENV
    env_user = os.environ.get('SAMURAI_MQTT_USER', '').strip()
    env_pwd = os.environ.get('SAMURAI_MQTT_PASS', '')
    if env_user and env_pwd:
        return env_user, env_pwd

    # 3. Файл ~/.samurai/mqtt.passwd
    file_creds = _read_passwd_file()
    if file_creds:
        return file_creds

    # 4. config.yaml
    cfg_user = cfg('mqtt.auth.username', None)
    cfg_pwd = cfg('mqtt.auth.password', None)
    if cfg_user and cfg_pwd:
        return str(cfg_user), str(cfg_pwd)

    return None, None
