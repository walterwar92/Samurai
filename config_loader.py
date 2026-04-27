"""
config_loader — loads config.yaml relative to the project root.

Usage:
    from config_loader import cfg, get_mqtt_credentials
    timeout = cfg('network.ip_detection_timeout', default=2.0)
    user, pwd = get_mqtt_credentials()  # None, None если auth выключен

Validation:
    Pydantic schema (validate_config) checks the most error-prone fields
    on startup — port numbers, frequencies, common typo targets. Failures
    are logged but don't crash; callers keep using defaults via cfg().
    Run `python config_loader.py --validate` to surface issues manually.
"""

import logging
import os
from typing import Optional, Tuple

import yaml

# Project root is the directory containing this file
_ROOT = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_ROOT, 'config.yaml')

_log = logging.getLogger('config_loader')
_data: dict = {}
_validation_done = False


def _load():
    global _data
    if _data:
        return
    try:
        with open(_CONFIG_PATH, 'r', encoding='utf-8') as f:
            _data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        pass  # all callers use defaults
    except yaml.YAMLError as exc:
        _log.error('config.yaml is not valid YAML: %s', exc)
        return
    # Validate after load. Issues are logged once, not raised — existing
    # cfg() callers fall back to their defaults if a value is missing/bad.
    _maybe_validate()


# ── Pydantic schema ──────────────────────────────────────────────────────────
# Partial schema: covers the fields where wrong-typed values have caused real
# bugs (numeric ports/rates, MQTT broker addresses). Other sections are not
# enforced — Pydantic accepts unknown fields silently. The tradeoff: full
# coverage would mean ~200 lines of models for 100+ fields, most of which
# never go wrong. Partial schema catches the high-value mistakes and avoids
# the maintenance tax of mirroring every YAML key.

try:
    from pydantic import BaseModel, ConfigDict, Field, ValidationError
    _HAS_PYDANTIC = True
except ImportError:
    _HAS_PYDANTIC = False
    BaseModel = object  # type: ignore[assignment,misc]
    ValidationError = Exception  # type: ignore[assignment,misc]


if _HAS_PYDANTIC:
    class _NetworkSchema(BaseModel):
        model_config = ConfigDict(extra='allow')
        ip_detection_timeout: float = Field(default=2.0, gt=0, le=30)
        dashboard_port: int = Field(default=5000, gt=0, lt=65536)
        mqtt_port: int = Field(default=1883, gt=0, lt=65536)
        mqtt_robot_id: str = Field(default='robot1', min_length=1, max_length=64)
        ros_domain_id: int = Field(default=42, ge=0, le=232)

    class _MqttSchema(BaseModel):
        model_config = ConfigDict(extra='allow')
        broker: str = Field(default='127.0.0.1', min_length=1)
        port: int = Field(default=1883, gt=0, lt=65536)
        robot_id: str = Field(default='robot1', min_length=1, max_length=64)
        camera_fps: int = Field(default=20, gt=0, le=120)
        camera_h264_port: int = Field(default=8554, gt=0, lt=65536)
        camera_h264_bitrate: int = Field(default=2_000_000, gt=0)
        keepalive: int = Field(default=15, gt=0, lt=600)

    class _VoiceSchema(BaseModel):
        model_config = ConfigDict(extra='allow')
        sample_rate: int = Field(default=16000, gt=0)
        chunk_size: int = Field(default=4000, gt=0)
        vad_aggressiveness: int = Field(default=2, ge=0, le=3)
        max_command_length: int = Field(default=200, gt=0, le=10_000)

    class _DashboardSchema(BaseModel):
        model_config = ConfigDict(extra='allow')
        state_push_hz: float = Field(default=10.0, gt=0, le=120)
        mjpeg_fps: float = Field(default=30.0, gt=0, le=120)
        voice_log_size: int = Field(default=20, ge=0, le=10_000)
        event_log_size: int = Field(default=100, ge=0, le=10_000)

    class _WatchdogSchema(BaseModel):
        model_config = ConfigDict(extra='allow')
        startup_grace_sec: float = Field(default=15.0, ge=0, le=600)
        topic_timeout_sec: float = Field(default=3.0, gt=0, le=600)
        report_interval: float = Field(default=0.5, gt=0, le=60)

    class _ConfigSchema(BaseModel):
        # extra='allow' keeps the rest of config.yaml untouched — we only
        # validate sections that have a defined schema.
        model_config = ConfigDict(extra='allow')
        network: _NetworkSchema = Field(default_factory=_NetworkSchema)
        mqtt: _MqttSchema = Field(default_factory=_MqttSchema)
        voice: _VoiceSchema = Field(default_factory=_VoiceSchema)
        dashboard: _DashboardSchema = Field(default_factory=_DashboardSchema)
        watchdog: _WatchdogSchema = Field(default_factory=_WatchdogSchema)


def _maybe_validate() -> bool:
    """Validate _data once. Returns True if no issues found."""
    global _validation_done
    if _validation_done or not _data:
        return True
    _validation_done = True
    if not _HAS_PYDANTIC:
        return True
    try:
        _ConfigSchema(**_data)
        return True
    except ValidationError as exc:
        # Log each issue with the dotted path — easy to grep against config.yaml.
        for err in exc.errors():
            loc = '.'.join(str(p) for p in err.get('loc', ()))
            msg = err.get('msg', 'invalid value')
            _log.error('config.yaml: %s — %s', loc or '<root>', msg)
        return False


def validate_config() -> bool:
    """Public entry point — validate the loaded config and return True on success."""
    _load()
    global _validation_done
    _validation_done = False  # force re-validation
    return _maybe_validate()


def cfg(key: str, default=None):
    """
    Dot-notation lookup, e.g. cfg('network.dashboard_port', 5000).
    Falls back to *default* if config.yaml is missing or key not found.

    Environment-variable override (#55):
        Any setting can be overridden by an env var built from the dotted key:
            mqtt.broker → SAMURAI_MQTT_BROKER
            network.dashboard_port → SAMURAI_NETWORK_DASHBOARD_PORT
            voice.vad_aggressiveness → SAMURAI_VOICE_VAD_AGGRESSIVENESS
        Useful for one-off overrides at startup (`SAMURAI_MQTT_PORT=1884
        ./samurai.sh robot`) and for systemd unit overrides without
        editing config.yaml.

    Type coercion:
        The ENV value is a string. We try int → float → bool → str in that
        order. Booleans accept true/false/yes/no/on/off (case-insensitive).
    """
    env_key = 'SAMURAI_' + key.upper().replace('.', '_').replace('-', '_')
    env_val = os.environ.get(env_key)
    if env_val is not None:
        return _coerce_env_value(env_val)
    _load()
    parts = key.split('.')
    node = _data
    for p in parts:
        if not isinstance(node, dict) or p not in node:
            return default
        node = node[p]
    return node


def _coerce_env_value(raw: str):
    """Best-effort coercion of an ENV string into int/float/bool/str."""
    s = raw.strip()
    if s == '':
        return s
    # int
    try:
        return int(s)
    except ValueError:
        pass
    # float
    try:
        return float(s)
    except ValueError:
        pass
    # bool
    lower = s.lower()
    if lower in ('true', 'yes', 'on'):
        return True
    if lower in ('false', 'no', 'off'):
        return False
    # fallback — string as-is
    return s


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


if __name__ == '__main__':
    import argparse
    import sys

    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)s %(name)s: %(message)s',
    )
    parser = argparse.ArgumentParser(description='config.yaml utilities')
    parser.add_argument('--validate', action='store_true',
                        help='Validate config.yaml against the Pydantic schema '
                             'and exit non-zero if any issues are found.')
    args = parser.parse_args()

    if args.validate:
        ok = validate_config()
        if ok:
            print('config.yaml: OK')
            sys.exit(0)
        else:
            sys.exit(1)
    else:
        parser.print_help()
