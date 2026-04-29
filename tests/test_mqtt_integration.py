"""
MQTT integration tests against a real Mosquitto broker (#52).

Boots Mosquitto in a Docker container via testcontainers — covers behaviour
that pure-Python unit tests can't: actual TCP reconnect, QoS guarantees,
retain semantics, last-will publishing, message ordering under load.

Skips automatically when Docker / testcontainers / paho-mqtt is unavailable
(typical for a fresh dev checkout on Windows). On CI (Ubuntu runner has
Docker built-in) all tests run.

Run locally:
    pip install testcontainers paho-mqtt
    pytest tests/test_mqtt_integration.py -v

Run in CI: see .github/workflows/ci.yml :: mqtt-integration job.
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional

import pytest

# Skip the whole module if any dep is missing — keeps `pytest tests/` clean
# on dev machines that don't have Docker installed.
testcontainers = pytest.importorskip('testcontainers.core.container')
paho = pytest.importorskip('paho.mqtt.client')

try:
    # testcontainers split into many sub-packages; the generic API works for
    # arbitrary images.
    from testcontainers.core.container import DockerContainer
    from testcontainers.core.waiting_utils import wait_for_logs
except ImportError:
    pytest.skip('testcontainers not available', allow_module_level=True)


def _docker_available() -> bool:
    try:
        import docker
        docker.from_env().ping()
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(
    not _docker_available(),
    reason='Docker not available — skipping MQTT integration tests',
)


@pytest.fixture(scope='module')
def mosquitto():
    """Spin up a Mosquitto broker in a container, yield (host, port)."""
    container = (
        DockerContainer('eclipse-mosquitto:2')
        .with_command('mosquitto -c /mosquitto-no-auth.conf')
        .with_exposed_ports(1883)
    )
    container.start()
    try:
        wait_for_logs(container, 'mosquitto version', timeout=15)
        host = container.get_container_host_ip()
        port = int(container.get_exposed_port(1883))
        # Smoke-check the listener is actually accepting connections
        deadline = time.time() + 10
        while time.time() < deadline:
            client = paho.Client(client_id='warmup')
            try:
                client.connect(host, port, keepalive=5)
                client.disconnect()
                break
            except Exception:
                time.sleep(0.2)
        yield host, port
    finally:
        container.stop()


def _make_client(client_id: str) -> paho.Client:
    c = paho.Client(client_id=client_id, protocol=paho.MQTTv311)
    c.reconnect_delay_set(min_delay=0.2, max_delay=2)
    return c


# ── Tests ─────────────────────────────────────────────────────────────────


def test_publish_subscribe_roundtrip(mosquitto):
    """Basic loopback: subscriber receives publisher's message."""
    host, port = mosquitto
    received = []
    done = threading.Event()

    sub = _make_client('sub-roundtrip')
    sub.on_message = lambda c, u, m: (received.append(m.payload), done.set())
    sub.connect(host, port)
    sub.loop_start()
    sub.subscribe('samurai/test/cmd_vel', qos=1)
    time.sleep(0.2)   # let SUBACK settle

    pub = _make_client('pub-roundtrip')
    pub.connect(host, port)
    payload = json.dumps({'linear_x': 0.5, 'angular_z': 0.0}).encode()
    pub.publish('samurai/test/cmd_vel', payload, qos=1)

    assert done.wait(5), 'message never arrived'
    assert json.loads(received[0]) == {'linear_x': 0.5, 'angular_z': 0.0}

    sub.loop_stop()
    sub.disconnect()
    pub.disconnect()


def test_qos1_delivers_after_late_subscribe(mosquitto):
    """QoS 1 with retained=True: a subscriber connecting AFTER the publish
    still gets the latest value. This is the contract camera_endpoint and
    other discovery topics rely on."""
    host, port = mosquitto

    pub = _make_client('pub-retain')
    pub.connect(host, port)
    pub.publish('samurai/test/retained', b'hello-retained', qos=1, retain=True)
    time.sleep(0.2)
    pub.disconnect()

    received = []
    done = threading.Event()
    sub = _make_client('sub-retain')
    sub.on_message = lambda c, u, m: (received.append(m.payload), done.set())
    sub.connect(host, port)
    sub.loop_start()
    sub.subscribe('samurai/test/retained', qos=1)

    assert done.wait(5), 'retained message never arrived'
    assert received[0] == b'hello-retained'

    sub.loop_stop()
    sub.disconnect()
    # Cleanup retained for next test (publish empty retained)
    cleanup = _make_client('pub-cleanup')
    cleanup.connect(host, port)
    cleanup.publish('samurai/test/retained', b'', qos=1, retain=True)
    cleanup.disconnect()


def test_will_message_on_unclean_disconnect(mosquitto):
    """LWT (Last Will & Testament): broker publishes the will when a client
    drops without DISCONNECT. mqtt_node uses this for the
    `samurai/{id}/{node}/online` heartbeat — verifying the contract here."""
    host, port = mosquitto

    received = []
    done = threading.Event()
    sub = _make_client('sub-will')
    sub.on_message = lambda c, u, m: (received.append(m.payload), done.set())
    sub.connect(host, port)
    sub.loop_start()
    sub.subscribe('samurai/test/online', qos=0)
    time.sleep(0.3)

    will_pub = _make_client('pub-will')
    will_pub.will_set('samurai/test/online', b'offline', qos=0, retain=False)
    will_pub.connect(host, port, keepalive=2)
    will_pub.loop_start()
    time.sleep(0.5)

    # Simulate ungraceful disconnect: kill the loop without calling
    # disconnect(). The broker only fires LWT after the keepalive expires
    # (2s here) without a PINGREQ.
    will_pub.loop_stop()
    will_pub._sock_close()  # type: ignore[attr-defined]

    assert done.wait(8), 'LWT message never arrived'
    assert received[0] == b'offline'

    sub.loop_stop()
    sub.disconnect()


def test_message_ordering_within_topic(mosquitto):
    """Single publisher → single subscriber on the same topic: order
    must be preserved at QoS 1. mqtt_node depends on this for the odom
    stream where a stale frame after a fresh one would corrupt position."""
    host, port = mosquitto

    received: list[int] = []
    target = 50
    done = threading.Event()

    def _on_msg(c, u, m):
        received.append(int(m.payload))
        if len(received) == target:
            done.set()

    sub = _make_client('sub-order')
    sub.on_message = _on_msg
    sub.connect(host, port)
    sub.loop_start()
    sub.subscribe('samurai/test/seq', qos=1)
    time.sleep(0.2)

    pub = _make_client('pub-order')
    pub.connect(host, port)
    pub.loop_start()
    for i in range(target):
        pub.publish('samurai/test/seq', str(i).encode(), qos=1)

    assert done.wait(15), f'only {len(received)}/{target} messages arrived'
    assert received == list(range(target)), \
        f'out-of-order: first 10 = {received[:10]}'

    pub.loop_stop()
    pub.disconnect()
    sub.loop_stop()
    sub.disconnect()
