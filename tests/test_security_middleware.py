"""Unit tests for dashboard security middleware:
security headers + opt-in Bearer auth.

Each test creates a fresh FastAPI app with a controlled env, because
the middleware is wired at create_app() time (env vars are read once).
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Skip silently on environments where FastAPI isn't installed (Pi-only setups).
fastapi = pytest.importorskip('fastapi')
from fastapi.testclient import TestClient  # noqa: E402

from compute_node.dashboard.app import create_app  # noqa: E402
from compute_node.dashboard.state import DashboardState  # noqa: E402


@pytest.fixture
def clean_env(monkeypatch):
    """Wipe security-related env vars so each test gets a known baseline."""
    for k in (
        'SAMURAI_DASHBOARD_TOKEN',
        'SAMURAI_SECURITY_HEADERS',
        'SAMURAI_CSP',
        'SAMURAI_HSTS',
    ):
        monkeypatch.delenv(k, raising=False)
    yield monkeypatch


def _client(env: dict[str, str]) -> TestClient:
    """Build a fresh TestClient with the given env vars in place."""
    for k, v in env.items():
        os.environ[k] = v
    app = create_app(DashboardState(), mqtt=None, ros2=None,
                     enable_socketio=False)
    return TestClient(app)


# ── security headers ─────────────────────────────────────────────────


def test_security_headers_present_by_default(clean_env):
    c = _client({})
    r = c.get('/api/v1/status')
    assert r.status_code == 200
    assert r.headers['x-content-type-options'] == 'nosniff'
    assert r.headers['x-frame-options'] == 'DENY'
    assert r.headers['referrer-policy'] == 'strict-origin-when-cross-origin'
    assert "default-src 'self'" in r.headers['content-security-policy']
    assert "frame-ancestors 'none'" in r.headers['content-security-policy']


def test_security_headers_disabled_via_env(clean_env):
    c = _client({'SAMURAI_SECURITY_HEADERS': 'off'})
    r = c.get('/api/v1/status')
    assert r.status_code == 200
    assert 'content-security-policy' not in {k.lower() for k in r.headers}
    assert 'x-frame-options' not in {k.lower() for k in r.headers}


def test_custom_csp_via_env(clean_env):
    c = _client({'SAMURAI_CSP': "default-src 'none'"})
    r = c.get('/api/v1/status')
    assert r.headers['content-security-policy'] == "default-src 'none'"


def test_hsts_opt_in(clean_env):
    c = _client({'SAMURAI_HSTS': 'on'})
    r = c.get('/api/v1/status')
    assert 'max-age=31536000' in r.headers['strict-transport-security']


def test_hsts_off_by_default(clean_env):
    c = _client({})
    r = c.get('/api/v1/status')
    assert 'strict-transport-security' not in {k.lower() for k in r.headers}


# ── Bearer auth ──────────────────────────────────────────────────────


TOKEN = 'super_secret_token_for_tests'


def test_auth_disabled_when_token_unset(clean_env):
    c = _client({})
    r = c.get('/api/v1/status')
    assert r.status_code == 200, "no token → unrestricted access"


def test_auth_required_when_token_set(clean_env):
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})

    # No Authorization at all
    r = c.get('/api/v1/status')
    assert r.status_code == 401
    assert r.headers.get('www-authenticate', '').startswith('Bearer')


def test_auth_accepts_valid_bearer(clean_env):
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})
    r = c.get('/api/v1/status', headers={'Authorization': f'Bearer {TOKEN}'})
    assert r.status_code == 200


@pytest.mark.parametrize('bad_header', [
    '',                              # empty
    'Bearer',                        # no token
    'Bearer ',                       # whitespace only
    f'Bearer wrong_{TOKEN}',         # close but wrong
    f'Bearer  {TOKEN}',              # CRITICAL — double space (regression vs MOIS)
    f'Bearer {TOKEN} ',              # trailing space
    f'Bearer {TOKEN}\n',             # trailing newline
    f'Bearer {TOKEN}\t',             # trailing tab
    f'Basic {TOKEN}',                # wrong scheme
    f'bearer {TOKEN}',               # lowercase scheme
    f'Token {TOKEN}',                # alt scheme
])
def test_auth_rejects_malformed_bearer(clean_env, bad_header):
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})
    r = c.get('/api/v1/status', headers={'Authorization': bad_header})
    assert r.status_code == 401, f"{bad_header!r} should NOT bypass auth"


def test_auth_rejects_alternate_channels(clean_env):
    """`apikey: <token>` and `?token=<token>` must NOT auth — that
    closes the alt_auth_channel finding in the MOIS probe."""
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})
    r = c.get('/api/v1/status', headers={'apikey': TOKEN})
    assert r.status_code == 401
    r = c.get(f'/api/v1/status?token={TOKEN}')
    assert r.status_code == 401


def test_auth_exempts_static_and_docs(clean_env):
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})
    for path in ('/openapi.json', '/docs', '/'):
        r = c.get(path)
        assert r.status_code == 200, f"{path} must remain reachable"


def test_auth_wrong_token_still_gets_security_headers(clean_env):
    """A 401 from auth must still carry the defence-in-depth headers —
    otherwise an attacker can probe API surface without ever seeing CSP."""
    c = _client({'SAMURAI_DASHBOARD_TOKEN': TOKEN})
    r = c.get('/api/v1/status', headers={'Authorization': 'Bearer wrong'})
    assert r.status_code == 401
    assert r.headers['x-content-type-options'] == 'nosniff'
    assert r.headers['x-frame-options'] == 'DENY'
