"""Security probe для MOIS robot-gateway Edge Function (Supabase).

Read-only / non-destructive проверки. Запускается под валидным студенческим
токеном (export MOIS_API_TOKEN=rt_xxx; export MOIS_API_URL=https://...).

Этот скрипт НИКОГДА:
- не делает DoS / спам
- не делает брут-форс токенов
- не пишет команды в очередь, которые робот реально исполнит
- не отправляет XSS-маркеры без явного --xss флага

При обнаружении находок — печатает компактные заметки в формате [SEV] FINDING.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Optional

import httpx


@dataclass
class Probe:
    name: str
    severity: str  # info, low, medium, high, critical
    finding: str
    evidence: dict


def hdr(title: str) -> None:
    print(f"\n{'━' * 72}\n{title}\n{'━' * 72}")


def show(label: str, response: httpx.Response, max_body: int = 300) -> None:
    body = response.text
    if len(body) > max_body:
        body = body[:max_body] + f"... [{len(response.text)} bytes total]"
    print(f"  [{label}] {response.status_code} {response.reason_phrase}")
    print(f"     ↳ body: {body!r}")
    interesting = [
        "content-type",
        "content-length",
        "strict-transport-security",
        "x-content-type-options",
        "x-frame-options",
        "content-security-policy",
        "access-control-allow-origin",
        "access-control-allow-credentials",
        "access-control-allow-methods",
        "access-control-allow-headers",
        "x-served-by",
        "server",
        "via",
        "x-deno-deploy-id",
        "set-cookie",
        "www-authenticate",
    ]
    for k, v in response.headers.items():
        if k.lower() in interesting:
            print(f"     ↳ {k}: {v}")


def probe_auth(client: httpx.Client, url: str, real_token: str) -> list[Probe]:
    """Какие токены проходят/не проходят на ?action=poll."""
    findings: list[Probe] = []
    hdr("PROBE 1 — Auth: какие токены принимаются?")

    cases = [
        ("без Authorization",      {}),
        ("пустой Authorization",   {"Authorization": ""}),
        ("Bearer пустой",          {"Authorization": "Bearer "}),
        ("Bearer мусор",           {"Authorization": "Bearer rt_NOT_A_REAL_TOKEN_xxxxxxxxxxxx"}),
        ("Bearer случайный",       {"Authorization": "Bearer notRtPrefix_aaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}),
        ("Bearer SQL-проба",       {"Authorization": "Bearer ' OR '1'='1"}),
        ("apikey вместо Bearer",   {"apikey": real_token}),
        ("Token в query ?token=",  {}),  # отдельный URL, см. ниже
        ("Bearer + лишний пробел", {"Authorization": f"Bearer  {real_token}"}),
        ("Bearer верный (sanity)", {"Authorization": f"Bearer {real_token}"}),
    ]

    poll_url = f"{url}?action=poll"
    timings: dict[str, list[float]] = {}

    for label, headers in cases:
        actual_url = poll_url
        if label.startswith("Token в query"):
            actual_url = f"{poll_url}&token={real_token}"
        try:
            t0 = time.perf_counter()
            r = client.get(actual_url, headers=headers)
            dt = (time.perf_counter() - t0) * 1000
        except httpx.RequestError as e:
            print(f"  [{label}] ERROR: {e}")
            continue
        timings[label] = [dt]
        show(label, r)
        # Эвристика: 200 на чём-то кроме верного токена → CRITICAL
        if r.status_code == 200 and label not in ("Bearer верный (sanity)",):
            findings.append(Probe(
                "auth_bypass",
                "critical",
                f"200 OK без валидного Bearer ({label!r})",
                {"label": label, "status": r.status_code, "body": r.text[:300]},
            ))
        # apikey/query как альтернатива Bearer — потенциальная утечка
        if r.status_code == 200 and label in ("apikey вместо Bearer", "Token в query ?token="):
            findings.append(Probe(
                "alt_auth_channel",
                "high",
                f"Принимает токен через {label} — повышает риск утечки в логах/Referer",
                {"label": label, "status": r.status_code},
            ))

    return findings


def probe_timing(client: httpx.Client, url: str, real_token: str, n: int = 25) -> list[Probe]:
    """Тайминг 401 на разной длины/префикса токенах."""
    findings: list[Probe] = []
    hdr(f"PROBE 2 — Timing на 401 (n={n} samples per case)")

    poll_url = f"{url}?action=poll"
    cases = {
        "случайный без префикса rt_": "x" * 30,
        "rt_ + 26 нулей":             "rt_" + "0" * 26,
        "rt_ + 26 символов 'a'":      "rt_" + "a" * 26,
        "первые 5 байт реального + xxxx": real_token[:5] + ("x" * (len(real_token) - 5)),
        "первые 15 байт реального + xx": real_token[:15] + ("x" * (len(real_token) - 15)),
    }

    samples: dict[str, list[float]] = {k: [] for k in cases}

    for round_idx in range(n):
        for label, tok in cases.items():
            try:
                t0 = time.perf_counter()
                r = client.get(poll_url, headers={"Authorization": f"Bearer {tok}"})
                dt = (time.perf_counter() - t0) * 1000
                samples[label].append(dt)
            except httpx.RequestError:
                continue

    for label, ts in samples.items():
        if not ts:
            continue
        med = statistics.median(ts)
        mn, mx = min(ts), max(ts)
        print(f"  {label:50}  median={med:6.1f}ms  min={mn:6.1f}  max={mx:6.1f}")

    medians = [statistics.median(ts) for ts in samples.values() if ts]
    if medians:
        spread = max(medians) - min(medians)
        if spread > 30:  # порог: 30мс разницы между медианами — подозрительно
            findings.append(Probe(
                "timing_leak_suspect",
                "low",
                f"Разница медиан таймингов 401 = {spread:.1f}мс — потенциальная утечка через timing",
                {"spread_ms": spread, "medians_ms": medians},
            ))
    return findings


def probe_cors(client: httpx.Client, url: str) -> list[Probe]:
    """OPTIONS preflight с разных Origin — увидим CORS-policy."""
    findings: list[Probe] = []
    hdr("PROBE 3 — CORS / preflight")

    origins = [
        "https://localhost",
        "https://evil.example",
        "null",
        "http://127.0.0.1:8080",
    ]
    for origin in origins:
        try:
            r = client.options(
                f"{url}?action=poll",
                headers={
                    "Origin": origin,
                    "Access-Control-Request-Method": "GET",
                    "Access-Control-Request-Headers": "authorization, content-type",
                },
            )
        except httpx.RequestError as e:
            print(f"  [Origin={origin}] ERROR: {e}")
            continue
        show(f"OPTIONS Origin={origin}", r, max_body=80)
        ac_origin = r.headers.get("access-control-allow-origin", "")
        ac_creds = r.headers.get("access-control-allow-credentials", "")
        if ac_origin == "*" and ac_creds.lower() == "true":
            findings.append(Probe(
                "cors_creds_wildcard",
                "high",
                "Access-Control-Allow-Origin=* + Allow-Credentials=true (несовместимо по spec, но если работает — CSRF-vector)",
                {"origin_sent": origin, "ac_origin": ac_origin, "ac_creds": ac_creds},
            ))
        if ac_origin == origin and origin not in ("null",):
            findings.append(Probe(
                "cors_origin_reflect",
                "medium",
                f"Сервер отражает Origin={origin!r} в Access-Control-Allow-Origin — фактически разрешает любой сайт",
                {"origin_sent": origin, "ac_origin": ac_origin},
            ))
    return findings


def probe_headers(client: httpx.Client, url: str, real_token: str) -> list[Probe]:
    """Security headers на нормальном ответе."""
    findings: list[Probe] = []
    hdr("PROBE 4 — Security headers (на валидном poll)")

    r = client.get(f"{url}?action=poll", headers={"Authorization": f"Bearer {real_token}"})
    show("valid poll", r, max_body=400)

    expected = {
        "strict-transport-security": "high",
        "x-content-type-options":    "low",
        "content-security-policy":   "medium",
        "x-frame-options":           "low",
    }
    missing = [h for h in expected if h not in {k.lower() for k in r.headers}]
    if missing:
        sev = max((expected[h] for h in missing), key=lambda s: ["info","low","medium","high","critical"].index(s))
        findings.append(Probe(
            "missing_security_headers",
            sev,
            f"Edge Function отдаёт ответ без: {', '.join(missing)}. Само API это не ломает, но усиливает impact XSS на UI.",
            {"missing": missing, "got": dict(r.headers)},
        ))

    return findings


def probe_method_spoofing(client: httpx.Client, url: str, real_token: str) -> list[Probe]:
    """Что если на capabilities прислать GET? На poll — POST? На неизвестное действие?"""
    findings: list[Probe] = []
    hdr("PROBE 5 — Method/action spoofing")

    headers = {"Authorization": f"Bearer {real_token}"}
    cases = [
        ("GET ?action=capabilities (должен быть POST)", "GET",  f"{url}?action=capabilities", None),
        ("POST ?action=poll (должен быть GET)",         "POST", f"{url}?action=poll", {"foo": "bar"}),
        ("GET ?action=cmd-result (должен быть POST)",   "GET",  f"{url}?action=cmd-result", None),
        ("GET без action",                              "GET",  url, None),
        ("GET ?action=admin",                           "GET",  f"{url}?action=admin", None),
        ("GET ?action=DROP",                            "GET",  f"{url}?action=DROP", None),
        ("GET ?action=poll&debug=1",                    "GET",  f"{url}?action=poll&debug=1", None),
    ]
    for label, method, u, body in cases:
        try:
            r = client.request(method, u, headers=headers, json=body)
        except httpx.RequestError as e:
            print(f"  [{label}] ERROR: {e}")
            continue
        show(label, r, max_body=400)
        if r.status_code == 200 and label.startswith("GET ?action=admin"):
            findings.append(Probe(
                "admin_action_exposed",
                "critical",
                "?action=admin отвечает 200 — проверить вручную содержимое",
                {"body": r.text[:500]},
            ))
    return findings


def main(argv: Optional[list[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass
    p = argparse.ArgumentParser(description="MOIS robot-gateway security probe")
    p.add_argument("--api-url", default=os.environ.get("MOIS_API_URL"))
    p.add_argument("--api-token", default=os.environ.get("MOIS_API_TOKEN"))
    p.add_argument("--out-json", help="Записать findings в JSON")
    args = p.parse_args(argv)

    if not args.api_url or not args.api_token:
        print("Установи MOIS_API_URL и MOIS_API_TOKEN (или --api-url/--api-token)",
              file=sys.stderr)
        return 2

    print(f"Target: {args.api_url}")
    print(f"Token: {args.api_token[:6]}...{args.api_token[-4:]}")

    findings: list[Probe] = []
    with httpx.Client(timeout=15.0, follow_redirects=False,
                      headers={"User-Agent": "samurai-security-probe/1.0"}) as client:
        findings += probe_auth(client, args.api_url, args.api_token)
        findings += probe_timing(client, args.api_url, args.api_token, n=25)
        findings += probe_cors(client, args.api_url)
        findings += probe_headers(client, args.api_url, args.api_token)
        findings += probe_method_spoofing(client, args.api_url, args.api_token)

    hdr("РЕЗЮМЕ FINDINGS")
    if not findings:
        print("  Ничего интересного на read-only пробах. Переходим к активным тестам?")
    else:
        order = {"critical": 0, "high": 1, "medium": 2, "low": 3, "info": 4}
        findings.sort(key=lambda f: order.get(f.severity, 99))
        for f in findings:
            print(f"  [{f.severity.upper():8}] {f.name}: {f.finding}")

    if args.out_json:
        with open(args.out_json, "w", encoding="utf-8") as fh:
            json.dump([f.__dict__ for f in findings], fh, ensure_ascii=False, indent=2)
        print(f"\nJSON отчёт: {args.out_json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
