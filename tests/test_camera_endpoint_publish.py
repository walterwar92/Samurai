"""
Regression tests for camera_node._publish_discovery.

Context: Pi в AP-режиме без default-route может выдать get_local_ip() →
"127.0.0.1", если interface enumeration по какой-то причине не сработала
(старый код, OSError на ioctl, и т.п.). До фикса camera_node слепо публиковал
этот мусорный host в retained MQTT, и dashboard ноута пытался коннектиться
к собственному localhost:8554 — Errno 111. Теперь camera_node:
  • отказывается публиковать loopback host (лучше пусто);
  • кричит WARNING в лог с подсказкой про SAMURAI_PI_IP env override.

Тесты дёргают unbound метод `CameraNode._publish_discovery` с MagicMock'ом
вместо self — это позволяет проверить логику без реального MQTT-коннекта,
picamera2 или running TCP-сервера.
"""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

# Импорт модуля камеры безопасен и без picamera2 (там try/except → _HW=False),
# но тянет paho-mqtt через MqttNode. Если paho не установлен — скипаем.
pytest.importorskip('paho.mqtt.client')

from pi_nodes.nodes import camera_node as cam_mod  # noqa: E402
from pi_nodes.nodes.camera_node import CameraNode  # noqa: E402


def _make_fake_self(host_returned: str) -> MagicMock:
    """Минимальный mock self для CameraNode._publish_discovery."""
    fake = MagicMock(spec=CameraNode)
    fake._mqtt_connected = True
    fake._tcp = MagicMock()
    fake._tcp.client_count = 3
    fake._broker = '127.0.0.1'
    fake._h264_port = 8554
    fake._w = 640
    fake._h = 480
    fake._fps = 20
    fake._bitrate = 2_000_000
    fake._iperiod = 30
    fake._discovery_topic = 'camera/endpoint'
    return fake


def test_loopback_host_not_published(monkeypatch):
    """Регрессия: get_local_ip() = 127.0.0.1 → publish НЕ вызывается."""
    fake = _make_fake_self('127.0.0.1')
    with patch.object(cam_mod, 'get_local_ip', return_value='127.0.0.1'):
        CameraNode._publish_discovery(fake)

    fake.publish.assert_not_called()
    fake.log_warn.assert_called_once()
    msg = fake.log_warn.call_args[0][0]
    assert 'loopback' in msg.lower()
    assert 'SAMURAI_PI_IP' in msg


def test_other_loopback_octet_also_blocked(monkeypatch):
    """Любой 127.x.x.x — это loopback, не только 127.0.0.1."""
    fake = _make_fake_self('127.0.1.1')
    with patch.object(cam_mod, 'get_local_ip', return_value='127.0.1.1'):
        CameraNode._publish_discovery(fake)

    fake.publish.assert_not_called()
    fake.log_warn.assert_called_once()


def test_real_ip_is_published_with_correct_payload():
    """Happy path: get_local_ip() = 192.168.4.1 → publish с этим host."""
    fake = _make_fake_self('192.168.4.1')
    with patch.object(cam_mod, 'get_local_ip', return_value='192.168.4.1'):
        CameraNode._publish_discovery(fake)

    fake.publish.assert_called_once()
    args, kwargs = fake.publish.call_args
    topic, payload = args[0], args[1]
    assert topic == 'camera/endpoint'
    assert payload['host'] == '192.168.4.1'
    assert payload['port'] == 8554
    assert payload['protocol'] == 'tcp'
    assert payload['codec'] == 'h264'
    assert payload['clients'] == 3
    assert kwargs.get('retain') is True
    assert kwargs.get('qos') == 1
    fake.log_warn.assert_not_called()
    fake.log_info.assert_called_once()


def test_skipped_when_mqtt_not_connected():
    """До MQTT connect не делаем ничего — get_local_ip даже не дёргается."""
    fake = _make_fake_self('192.168.4.1')
    fake._mqtt_connected = False
    with patch.object(cam_mod, 'get_local_ip') as mock_ip:
        CameraNode._publish_discovery(fake)
        mock_ip.assert_not_called()
    fake.publish.assert_not_called()


def test_skipped_when_tcp_server_missing():
    """Камера не запустилась (picamera2 missing) → _tcp = None → skip."""
    fake = _make_fake_self('192.168.4.1')
    fake._tcp = None
    with patch.object(cam_mod, 'get_local_ip') as mock_ip:
        CameraNode._publish_discovery(fake)
        mock_ip.assert_not_called()
    fake.publish.assert_not_called()
