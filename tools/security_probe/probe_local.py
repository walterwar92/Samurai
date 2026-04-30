"""Security probe для локальных сервисов робота:

- Dashboard FastAPI (default :5000)
- Samcan USB-Serial bridge (default :5005)

Read-only / non-destructive — то же ограничение что и в `probe.py`:
никаких реальных команд (`cmd_vel`, `claw`, `laser`, `fsm/command`),
никаких write-операций на пресеты/сценарии Samcan, никакого DoS.

Зачем нужен отдельный скрипт от `probe.py`:
MOIS-probe бьёт по внешнему Supabase Edge Function и ожидает ответы
формата `{"requests": [...], "commands": [...]}`. Локальный probe
бьёт по другому API surface (REST FastAPI) и проверяет другой набор
гарантий (rate-limit middleware, idempotency-key, security headers,
opt-in Bearer auth) — слепить их в один большой скрипт ради DRY дало
бы нечитаемые ветвления.

Запуск:

    # без auth (default — SAMURAI_DASHBOARD_TOKEN не задан)
    python tools/security_probe/probe_local.py

    # с включённым Bearer на dashboard
    export SAMURAI_DASHBOARD_TOKEN=<token>
    python tools/security_probe/probe_local.py --token "$SAMURAI_DASHBOARD_TOKEN"

    # против другого хоста (например, Pi через VPN)
    python tools/security_probe/probe_local.py --dashboard-url http://10.0.0.5:5000

    # выгрузить findings в JSON
    python tools/security_probe/probe_local.py --out-json findings_local.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

import httpx


# Безопасные read-only endpoints (заведомо не двигают робота).
DASHBOARD_SAFE_GET = '/api/v1/status'
SAMCAN_SAFE_GET = '/api/samcan/state'

# 404-ный путь для проверки middleware (rate-limit, idempotency)
# без срабатывания handler-ов: middleware на /api/* отрабатывает до
# матча роута, а сам путь возвращает 404 — никаких побочных эффектов.
DASHBOARD_PROBE_PING = '/api/v1/_probe_ping'


@dataclass
class Probe:
    name: str
    severity: str  # info, low, medium, high, critical
    finding: str
    evidence: dict


def hdr(title: str) -> None:
    print(f"\n{'━' * 72}\n{title}\n{'━' * 72}")


def show(label: str, response: httpx.Response, max_body: int = 200) -> None:
    body = response.text
    if len(body) > max_body:
        body = body[:max_body] + f"... [{len(response.text)} bytes total]"
    print(f"  [{label}] {response.status_code} {response.reason_phrase}")
    print(f"     ↳ body: {body!r}")


def probe_connectivity(client: httpx.Client, dashboard_url: str,
                       samcan_url: Optional[str]) -> tuple[list[Probe], dict]:
    """Сначала убедиться, что таргеты вообще живы. Если dashboard
    недоступен — нет смысла запускать остальные пробы."""
    findings: list[Probe] = []
    state = {'dashboard_alive': False, 'samcan_alive': False}
    hdr("PROBE 0 — Connectivity")

    try:
        r = client.get(f"{dashboard_url}{DASHBOARD_SAFE_GET}")
        show(f"GET {dashboard_url}{DASHBOARD_SAFE_GET}", r)
        if r.status_code in (200, 401, 403):  # 401/403 — auth on, тоже живой
            state['dashboard_alive'] = True
    except httpx.RequestError as e:
        print(f"  Dashboard недоступен: {e}")

    if samcan_url:
        try:
            r = client.get(f"{samcan_url}{SAMCAN_SAFE_GET}")
            show(f"GET {samcan_url}{SAMCAN_SAFE_GET}", r)
            if r.status_code == 200:
                state['samcan_alive'] = True
        except httpx.RequestError as e:
            print(f"  Samcan-bridge недоступен: {e} (ок если не запущен)")

    return findings, state


def probe_security_headers(client: httpx.Client, url: str,
                           token: Optional[str]) -> list[Probe]:
    """Проверить наличие defence-in-depth заголовков."""
    findings: list[Probe] = []
    hdr(f"PROBE 1 — Security headers @ {url}")

    headers = {'Authorization': f'Bearer {token}'} if token else {}
    try:
        r = client.get(f"{url}{DASHBOARD_SAFE_GET}", headers=headers)
    except httpx.RequestError as e:
        print(f"  ERROR: {e}")
        return findings
    show("GET status", r)

    expected = {
        'x-content-type-options':  ('nosniff', 'low'),
        'x-frame-options':         (None,      'medium'),
        'referrer-policy':         (None,      'low'),
        'content-security-policy': (None,      'medium'),
    }
    actual_lower = {k.lower(): v for k, v in r.headers.items()}
    missing: list[str] = []
    wrong_value: list[tuple[str, str, str]] = []
    for hkey, (expected_value, _sev) in expected.items():
        if hkey not in actual_lower:
            missing.append(hkey)
        elif expected_value is not None and \
                actual_lower[hkey].lower() != expected_value.lower():
            wrong_value.append((hkey, expected_value, actual_lower[hkey]))

    if missing:
        worst = max((expected[h][1] for h in missing),
                    key=lambda s: ['info', 'low', 'medium', 'high',
                                   'critical'].index(s))
        findings.append(Probe(
            'missing_security_headers',
            worst,
            f"Не отдаются: {', '.join(missing)}",
            {'missing': missing,
             'present': sorted(actual_lower.keys())},
        ))
    for hkey, want, got in wrong_value:
        findings.append(Probe(
            'wrong_security_header',
            'low',
            f"{hkey}: ожидается {want!r}, получено {got!r}",
            {'header': hkey, 'expected': want, 'actual': got},
        ))
    return findings


def probe_cors(client: httpx.Client, url: str) -> list[Probe]:
    """OPTIONS preflight с подозрительных Origin."""
    findings: list[Probe] = []
    hdr(f"PROBE 2 — CORS preflight @ {url}")

    origins = [
        'https://evil.example',
        'null',
        'http://127.0.0.1:8080',
    ]
    for origin in origins:
        try:
            r = client.options(
                f"{url}{DASHBOARD_SAFE_GET}",
                headers={
                    'Origin': origin,
                    'Access-Control-Request-Method': 'GET',
                    'Access-Control-Request-Headers': 'authorization, content-type',
                },
            )
        except httpx.RequestError as e:
            print(f"  [Origin={origin}] ERROR: {e}")
            continue
        ac_origin = r.headers.get('access-control-allow-origin', '')
        ac_creds = r.headers.get('access-control-allow-credentials', '')
        print(f"  [Origin={origin}] {r.status_code}  ACAO={ac_origin!r}  ACAC={ac_creds!r}")

        if ac_origin == '*' and ac_creds.lower() == 'true':
            findings.append(Probe(
                'cors_creds_wildcard',
                'high',
                'Allow-Origin=* + Allow-Credentials=true (CSRF-vector)',
                {'origin': origin, 'ac_origin': ac_origin, 'ac_creds': ac_creds},
            ))
        elif ac_origin == origin and origin != 'null':
            findings.append(Probe(
                'cors_origin_reflect',
                'medium',
                f"ACAO отражает Origin={origin!r} — фактически any-origin",
                {'origin': origin, 'ac_origin': ac_origin},
            ))
        elif ac_origin == '*':
            findings.append(Probe(
                'cors_wildcard',
                'low',
                "Allow-Origin=* — ожидаемо для dev, в prod — сужать",
                {'origin': origin, 'ac_origin': ac_origin},
            ))
    return findings


def probe_auth(client: httpx.Client, url: str, token: Optional[str]) -> list[Probe]:
    """Если SAMURAI_DASHBOARD_TOKEN задан — проверить, что он действительно
    обязателен и регресс из MOIS-probe (`Bearer  ` с двумя пробелами)
    не повторяется."""
    findings: list[Probe] = []
    hdr(f"PROBE 3 — Bearer auth @ {url}")

    if not token:
        print("  SAMURAI_DASHBOARD_TOKEN не задан — auth-пробы пропущены.")
        print("  (Если ты ожидаешь, что dashboard защищён — задай --token.)")
        return findings

    cases = [
        ('без Authorization',           {},                                                  401),
        ('пустой Authorization',        {'Authorization': ''},                               401),
        ('Bearer пустой',               {'Authorization': 'Bearer '},                        401),
        ('Bearer мусор',                {'Authorization': 'Bearer NOT_A_REAL_TOKEN'},        401),
        ('Bearer SQL-проба',            {'Authorization': "Bearer ' OR '1'='1"},             401),
        ('Bearer + лишний пробел',      {'Authorization': f'Bearer  {token}'},               401),  # regression
        ('Bearer + хвостовой \\n',      {'Authorization': f'Bearer {token}\n'},              401),
        ('apikey вместо Bearer',        {'apikey': token},                                   401),
        ('Bearer верный (sanity)',      {'Authorization': f'Bearer {token}'},                200),
    ]

    for label, headers, want_status in cases:
        try:
            r = client.get(f"{url}{DASHBOARD_SAFE_GET}", headers=headers)
        except httpx.RequestError as e:
            print(f"  [{label}] ERROR: {e}")
            continue
        ok = (r.status_code == want_status)
        marker = '✓' if ok else '✗'
        print(f"  {marker} [{label}] got={r.status_code}, want={want_status}")

        if not ok:
            # Особый случай — sanity-чек упал: токен сам неверный.
            if label == 'Bearer верный (sanity)':
                findings.append(Probe(
                    'auth_token_invalid',
                    'info',
                    "Передан --token, но валидный Bearer не получает 200 — "
                    "токен не совпадает с SAMURAI_DASHBOARD_TOKEN на сервере?",
                    {'status': r.status_code, 'body': r.text[:200]},
                ))
                continue
            sev = 'critical' if r.status_code == 200 else 'medium'
            findings.append(Probe(
                'auth_unexpected_status' if sev == 'medium' else 'auth_bypass',
                sev,
                f"Кейс {label!r}: status={r.status_code}, ожидался {want_status}. "
                f"200 без валидного Bearer = bypass.",
                {'label': label, 'status': r.status_code, 'body': r.text[:200]},
            ))
    return findings


def probe_rate_limit(client: httpx.Client, url: str,
                     token: Optional[str], n: int = 35) -> list[Probe]:
    """Дашборд имеет MovingWindow rate-limit (default 30/min) на write-verbs
    в /api/*. Бьём POST на 404-ный путь `/api/v1/_probe_ping` — middleware
    срабатывает before роутинга, реальный handler не задействован."""
    findings: list[Probe] = []
    hdr(f"PROBE 4 — Rate limit (POST x{n} on /api/v1/_probe_ping)")

    headers = {'Authorization': f'Bearer {token}'} if token else {}
    statuses: list[int] = []
    got_429 = False
    for i in range(n):
        try:
            r = client.post(f"{url}{DASHBOARD_PROBE_PING}", headers=headers, json={})
        except httpx.RequestError as e:
            print(f"  [{i}] ERROR: {e}")
            continue
        statuses.append(r.status_code)
        if r.status_code == 429:
            got_429 = True
            print(f"  ✓ 429 на запросе #{i + 1}: {r.headers.get('retry-after', '?')}s "
                  f"({r.text[:80]!r})")
            break

    print(f"  Статусы: {statuses[:10]}{'...' if len(statuses) > 10 else ''} "
          f"(всего {len(statuses)})")

    if not got_429:
        # Если SAMURAI_RATELIMIT=off — это ожидаемо. Иначе — high risk.
        ratelimit_env = os.environ.get('SAMURAI_RATELIMIT', 'on').lower()
        if ratelimit_env != 'off':
            findings.append(Probe(
                'rate_limit_inactive',
                'high',
                f"После {n} POST на /api/* сервер не вернул 429. "
                "Либо middleware отсутствует, либо лимит слишком высокий.",
                {'samples': statuses[:20], 'count': len(statuses)},
            ))
        else:
            print("  SAMURAI_RATELIMIT=off — 429 не ожидается.")
    return findings


def probe_idempotency(client: httpx.Client, url: str,
                      token: Optional[str]) -> list[Probe]:
    """Dashboard кеширует ответы POST с Idempotency-Key на 60s.
    Отправляем тот же POST дважды — на втором ответе должен быть
    `Idempotency-Replay: true`."""
    findings: list[Probe] = []
    hdr(f"PROBE 5 — Idempotency-Key @ {url}")

    headers_base = {'Authorization': f'Bearer {token}'} if token else {}

    # Используем endpoint, который реально принимает POST и возвращает 2xx —
    # /api/v1/log принимает произвольное событие. Если его нет, отвалится 404,
    # но проверим оба варианта.
    endpoint = '/api/v1/log/event'
    key = f'probe-idem-{int(time.time())}'
    payload = {'level': 'info', 'message': 'security_probe heartbeat'}

    headers = {**headers_base, 'Idempotency-Key': key}
    try:
        r1 = client.post(f"{url}{endpoint}", headers=headers, json=payload)
    except httpx.RequestError as e:
        print(f"  POST #1 ERROR: {e}")
        return findings
    show("POST #1", r1, max_body=120)

    if not (200 <= r1.status_code < 300):
        print(f"  POST #1 не 2xx ({r1.status_code}) — endpoint {endpoint} "
              f"возможно не существует, тест Idempotency пропускается.")
        return findings

    try:
        r2 = client.post(f"{url}{endpoint}", headers=headers, json=payload)
    except httpx.RequestError as e:
        print(f"  POST #2 ERROR: {e}")
        return findings
    show("POST #2", r2, max_body=120)

    replay = r2.headers.get('Idempotency-Replay', '').lower() == 'true'
    if replay:
        print(f"  ✓ Idempotency-Replay: true на втором POST — middleware работает.")
    else:
        findings.append(Probe(
            'idempotency_inactive',
            'medium',
            "Второй POST с тем же Idempotency-Key не помечен Idempotency-Replay. "
            "Кэш либо отсутствует, либо не работает — повторные сетевые ретраи "
            "выполнят команду дважды.",
            {'status_1': r1.status_code, 'status_2': r2.status_code,
             'replay_header': r2.headers.get('Idempotency-Replay', '<absent>')},
        ))
    return findings


def probe_method_spoofing(client: httpx.Client, url: str,
                          token: Optional[str]) -> list[Probe]:
    """Что если на GET-only endpoint прислать DELETE? На POST-only — GET?
    Что если запросить /api/v1/_admin?"""
    findings: list[Probe] = []
    hdr(f"PROBE 6 — Method/path spoofing @ {url}")

    headers = {'Authorization': f'Bearer {token}'} if token else {}
    cases = [
        ('DELETE /api/v1/status (GET-only)',     'DELETE', f'{url}/api/v1/status', None),
        ('PUT /api/v1/status (GET-only)',        'PUT',    f'{url}/api/v1/status', {}),
        ('GET /api/v1/_admin',                   'GET',    f'{url}/api/v1/_admin', None),
        ('GET /api/v1/.env',                     'GET',    f'{url}/api/v1/.env', None),
        ('GET /api/v1/../etc/passwd',            'GET',    f'{url}/api/v1/../etc/passwd', None),
        ('GET /api/v1/path_recorder/../../boot', 'GET',    f'{url}/api/v1/path_recorder/../../boot', None),
    ]
    for label, method, u, body in cases:
        try:
            r = client.request(method, u, headers=headers, json=body)
        except httpx.RequestError as e:
            print(f"  [{label}] ERROR: {e}")
            continue
        print(f"  [{label}] {r.status_code} {r.reason_phrase}")
        # Любой 200 на этих кейсах = реальная находка.
        if r.status_code == 200:
            findings.append(Probe(
                'unexpected_200',
                'high',
                f"{label} вернул 200 — нужно проверить вручную, что отдаётся.",
                {'method': method, 'url': u, 'body': r.text[:300]},
            ))
    return findings


def main(argv: Optional[list[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace')
        except (AttributeError, OSError):
            pass

    p = argparse.ArgumentParser(
        description='Local Samurai dashboard / samcan-bridge security probe')
    p.add_argument('--dashboard-url', default='http://127.0.0.1:5000',
                   help='Dashboard FastAPI base URL (default: %(default)s)')
    p.add_argument('--samcan-url', default='http://127.0.0.1:5005',
                   help='Samcan bridge base URL (default: %(default)s; '
                        'передай "" чтобы пропустить).')
    p.add_argument('--token', default=os.environ.get('SAMURAI_DASHBOARD_TOKEN'),
                   help='Bearer token для dashboard (или env SAMURAI_DASHBOARD_TOKEN)')
    p.add_argument('--out-json', help='Записать findings в JSON')
    p.add_argument('--skip-rate-limit', action='store_true',
                   help='Пропустить probe 4 — он отправляет ~30 POST подряд')
    args = p.parse_args(argv)

    samcan_url = args.samcan_url.strip() or None
    print(f"Dashboard: {args.dashboard_url}")
    print(f"Samcan:    {samcan_url or '<пропущен>'}")
    print(f"Token:     {'<задан>' if args.token else '<НЕТ — auth-пробы skip>'}")

    findings: list[Probe] = []
    with httpx.Client(timeout=10.0, follow_redirects=False,
                      headers={'User-Agent': 'samurai-security-probe/1.0'}) as client:
        conn_findings, state = probe_connectivity(client, args.dashboard_url,
                                                  samcan_url)
        findings += conn_findings
        if not state['dashboard_alive']:
            print("\nDashboard недоступен — остальные пробы пропущены. "
                  "Запусти `./samurai.sh compute` или `./samurai.sh sim`.")
            return 1

        findings += probe_security_headers(client, args.dashboard_url, args.token)
        findings += probe_cors(client, args.dashboard_url)
        findings += probe_auth(client, args.dashboard_url, args.token)
        if not args.skip_rate_limit:
            findings += probe_rate_limit(client, args.dashboard_url, args.token)
        findings += probe_idempotency(client, args.dashboard_url, args.token)
        findings += probe_method_spoofing(client, args.dashboard_url, args.token)

    hdr("РЕЗЮМЕ FINDINGS (local)")
    if not findings:
        print("  Чисто. Дашборд проходит read-only пробы.")
    else:
        order = {'critical': 0, 'high': 1, 'medium': 2, 'low': 3, 'info': 4}
        findings.sort(key=lambda f: order.get(f.severity, 99))
        for f in findings:
            print(f"  [{f.severity.upper():8}] {f.name}: {f.finding}")

    if args.out_json:
        with open(args.out_json, 'w', encoding='utf-8') as fh:
            json.dump([f.__dict__ for f in findings], fh,
                      ensure_ascii=False, indent=2)
        print(f"\nJSON отчёт: {args.out_json}")

    # Exit code: 0 если только info/low, 1 если есть medium+, 2 если critical.
    severities = {f.severity for f in findings}
    if 'critical' in severities:
        return 2
    if severities & {'high', 'medium'}:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
