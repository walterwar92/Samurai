"""
Регрессия для camera_node._StreamFileWrapper.

Контекст: новый picamera2 (Debian Trixie, libcamera 0.7) в FileOutput делает
строгий isinstance(file, io.BufferedIOBase) и иначе падает с
RuntimeError('Must pass io.BufferedIOBase'). Старый _StreamFileWrapper был
просто duck-typed классом с write/flush/close — на новом picamera2 это валило
_start_camera ещё до start_recording: камера не запускалась, _cam = None.
Теперь _StreamFileWrapper наследует io.BufferedIOBase.
"""
from __future__ import annotations

import io
from unittest.mock import MagicMock

import pytest

# Импорт camera_node безопасен без picamera2 (try/except → _HW=False),
# но тянет paho-mqtt через MqttNode. Нет paho — скип.
pytest.importorskip('paho.mqtt.client')

from pi_nodes.nodes.camera_node import _StreamFileWrapper  # noqa: E402


def test_wrapper_is_buffered_iobase():
    """picamera2 FileOutput требует именно io.BufferedIOBase, не duck-typing."""
    wrapper = _StreamFileWrapper(MagicMock())
    assert isinstance(wrapper, io.BufferedIOBase)
    assert wrapper.writable() is True


def test_write_forwards_bytes_and_returns_length():
    """write() отдаёт байты в TCPStreamServer.write_frame и возвращает длину."""
    server = MagicMock()
    wrapper = _StreamFileWrapper(server)
    payload = b'\x00\x00\x01\x67spspayload'
    n = wrapper.write(payload)
    server.write_frame.assert_called_once_with(payload)
    assert n == len(payload)


def test_write_accepts_memoryview():
    """picamera2 может отдать memoryview/bytearray — должен принять как bytes."""
    server = MagicMock()
    wrapper = _StreamFileWrapper(server)
    wrapper.write(memoryview(b'nalunit'))
    server.write_frame.assert_called_once_with(b'nalunit')


def test_close_does_not_touch_server():
    """close() от FileOutput на stop_recording НЕ должен рвать TCP-сервер."""
    server = MagicMock()
    wrapper = _StreamFileWrapper(server)
    wrapper.close()
    server.stop.assert_not_called()
    server.write_frame.assert_not_called()
