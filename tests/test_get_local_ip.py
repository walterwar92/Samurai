"""
Tests for get_local_ip — hotspot-safe Pi IP detection.

Regression context: Pi running in WiFi-AP (hotspot) mode without a default
route caused get_local_ip() to silently fall back to 127.0.0.1 (because the
8.8.8.8 UDP-trick raises ENETUNREACH). camera_node then published a retained
endpoint with host=127.0.0.1, and the dashboard tried to connect to its OWN
loopback :8554 — getting Errno 111 Connection refused.

These tests pin behaviour for:
  - SAMURAI_PI_IP env override
  - broker_hint as primary probe target (works on local subnets w/o internet)
  - fallback to interface enumeration (priority: wlan/eth > others)
  - last-resort 127.0.0.1 only when literally nothing usable exists
"""
from __future__ import annotations

import socket
import sys
from unittest.mock import MagicMock

import pytest

from pi_nodes.mqtt_node import get_local_ip


# ---------- helpers ---------------------------------------------------------

def _make_fake_socket(getsockname_ip: str | None = None,
                     connect_raises: BaseException | None = None):
    """Return a fake socket factory installable via monkeypatch."""
    fake = MagicMock()
    if connect_raises is not None:
        fake.connect.side_effect = connect_raises
    if getsockname_ip is not None:
        fake.getsockname.return_value = (getsockname_ip, 12345)
    fake.__enter__ = lambda self: fake
    fake.__exit__ = lambda *a: False
    return lambda *a, **k: fake


def _ifaddr_response(ip: str) -> bytes:
    """Build a fake SIOCGIFADDR ioctl response (sockaddr_in is at offset 20)."""
    # struct ifreq is 32 bytes on Linux: 16 ifr_name + 16 ifr_addr.
    # sockaddr_in: 2 sin_family + 2 sin_port + 4 sin_addr + 8 padding.
    # IP bytes live at offset 20..24 in the full buffer.
    return b'\x00' * 20 + socket.inet_aton(ip) + b'\x00' * 8


def _install_fake_fcntl(monkeypatch, ip_map: dict[str, str]):
    """Replace fcntl module with one whose ioctl returns IPs from ip_map."""
    fake = MagicMock()

    def ioctl(fd, req, packed):
        name = packed[:15].rstrip(b'\x00').decode()
        if name not in ip_map:
            raise OSError(19, 'No such device')
        return _ifaddr_response(ip_map[name])

    fake.ioctl.side_effect = ioctl
    monkeypatch.setitem(sys.modules, 'fcntl', fake)


# ---------- tests ----------------------------------------------------------

def test_env_override_wins(monkeypatch):
    monkeypatch.setenv('SAMURAI_PI_IP', '10.20.30.40')
    assert get_local_ip() == '10.20.30.40'


def test_env_override_ignored_if_empty(monkeypatch):
    monkeypatch.setenv('SAMURAI_PI_IP', '')
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(getsockname_ip='1.2.3.4'))
    assert get_local_ip() == '1.2.3.4'


def test_broker_hint_used_when_provided(monkeypatch):
    """If broker_hint is the local subnet IP, getsockname picks the right NIC.

    This is the primary fix path: Pi in hotspot mode has no internet route,
    but DOES have a route to its own broker IP — using that as probe target
    yields the right interface IP.
    """
    fake = MagicMock()
    fake.getsockname.return_value = ('192.168.4.1', 12345)
    fake.__enter__ = lambda self: fake
    fake.__exit__ = lambda *a: False
    monkeypatch.setattr(socket, 'socket', lambda *a, **k: fake)

    assert get_local_ip(broker_hint='192.168.4.1') == '192.168.4.1'
    fake.connect.assert_called_once()
    # The first arg of connect() should be the broker_hint, not 8.8.8.8.
    target_host = fake.connect.call_args[0][0][0]
    assert target_host == '192.168.4.1'


def test_loopback_broker_hint_skipped(monkeypatch):
    """broker_hint='127.0.0.1' or 'localhost' must NOT be used as probe target."""
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(getsockname_ip='5.6.7.8'))
    # Should fall back to 8.8.8.8 path; getsockname returns 5.6.7.8 anyway.
    ip = get_local_ip(broker_hint='127.0.0.1')
    assert ip == '5.6.7.8'


def test_no_default_route_falls_back_to_interfaces(monkeypatch):
    """Regression: H1 bug — hotspot Pi with no default route must NOT return 127.0.0.1."""
    monkeypatch.delenv('SAMURAI_PI_IP', raising=False)

    # All UDP probes fail with ENETUNREACH (real hotspot scenario).
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(connect_raises=OSError(101, 'Network is unreachable')))
    monkeypatch.setattr(socket, 'if_nameindex',
                        lambda: [(1, 'lo'), (2, 'wlan0')])
    _install_fake_fcntl(monkeypatch, {'wlan0': '192.168.4.1'})

    ip = get_local_ip()
    assert ip == '192.168.4.1', f"H1 regression: expected wlan0 IP, got {ip!r}"


def test_wlan_priority_over_zerotier_and_tailscale(monkeypatch):
    """Real Pi setup: wlan0=192.168.4.1, ztsjsmwlfe=10.26.x, tailscale0.

    The IP that actually reaches the laptop is wlan0 (same subnet).
    ZeroTier/Tailscale must NOT be picked over real LAN/WiFi.
    """
    monkeypatch.delenv('SAMURAI_PI_IP', raising=False)
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(connect_raises=OSError(101, 'Network is unreachable')))
    monkeypatch.setattr(socket, 'if_nameindex', lambda: [
        (1, 'lo'),
        (2, 'tailscale0'),
        (3, 'ztsjsmwlfe'),
        (4, 'wlan0'),
        (5, 'eth0'),
    ])
    _install_fake_fcntl(monkeypatch, {
        'wlan0': '192.168.4.1',
        'eth0': '10.0.0.5',
        'ztsjsmwlfe': '10.26.136.179',
        'tailscale0': '100.64.0.5',
    })

    assert get_local_ip() == '192.168.4.1'


def test_eth_used_when_no_wlan(monkeypatch):
    monkeypatch.delenv('SAMURAI_PI_IP', raising=False)
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(connect_raises=OSError(101, 'Network is unreachable')))
    monkeypatch.setattr(socket, 'if_nameindex', lambda: [
        (1, 'lo'), (2, 'tailscale0'), (3, 'eth0'),
    ])
    _install_fake_fcntl(monkeypatch, {
        'eth0': '192.168.1.42',
        'tailscale0': '100.64.0.5',
    })
    assert get_local_ip() == '192.168.1.42'


def test_only_loopback_returns_loopback(monkeypatch):
    """If literally no usable interface — last resort 127.0.0.1."""
    monkeypatch.delenv('SAMURAI_PI_IP', raising=False)
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(connect_raises=OSError(101, 'Network is unreachable')))
    monkeypatch.setattr(socket, 'if_nameindex', lambda: [(1, 'lo')])
    _install_fake_fcntl(monkeypatch, {})
    assert get_local_ip() == '127.0.0.1'


def test_loopback_ip_from_iface_skipped(monkeypatch):
    """If wlan0 somehow has 127.x.x.x assigned — skip and try next iface."""
    monkeypatch.delenv('SAMURAI_PI_IP', raising=False)
    monkeypatch.setattr(socket, 'socket',
                        _make_fake_socket(connect_raises=OSError(101, 'Network is unreachable')))
    monkeypatch.setattr(socket, 'if_nameindex', lambda: [
        (1, 'lo'), (2, 'wlan0'), (3, 'eth0'),
    ])
    _install_fake_fcntl(monkeypatch, {
        'wlan0': '127.0.0.99',  # weird but possible misconfig
        'eth0': '192.168.1.10',
    })
    assert get_local_ip() == '192.168.1.10'
