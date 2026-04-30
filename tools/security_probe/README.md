# security_probe

Read-only / non-destructive проверки для API Samurai-стека.

## Цели

- **`probe.py`** — внешний robot-gateway MOIS (Supabase Edge Function),
  куда Pi пушит телеметрию и забирает команды. Запускается под
  валидным студенческим токеном.
- **`probe_local.py`** — локальный dashboard (FastAPI :5000) и
  Samcan-bridge (FastAPI :5005). Бьёт по `127.0.0.1`, проверяет защиту
  собственного API робота: security headers, CORS preflight,
  опциональный Bearer auth, rate-limit (#58), idempotency (#59),
  method/path spoofing.

## Что НЕ делает probe

- DoS / нагрузочный спам
- брут-форс токенов
- команды, которые реально подвинут робота (`cmd_vel`, `claw`, `laser`)
- XSS-маркеры без явного `--xss` флага

## Запуск

### MOIS (внешний gateway)

    export MOIS_API_URL=https://<project>.supabase.co/functions/v1/robot-gateway
    export MOIS_API_TOKEN=rt_xxx
    python tools/security_probe/probe.py --target mois --out-json findings.json

### Локальный dashboard

    # без аутентификации (default — auth выключен)
    python tools/security_probe/probe_local.py

    # с включённым SAMURAI_DASHBOARD_TOKEN
    export SAMURAI_DASHBOARD_TOKEN=$(cat ~/.samurai/dashboard.token)
    python tools/security_probe/probe_local.py --token "$SAMURAI_DASHBOARD_TOKEN"

    # против Pi через VPN
    python tools/security_probe/probe_local.py --dashboard-url http://10.0.0.5:5000

    # exit-code: 0 = чисто, 1 = есть medium/high, 2 = есть critical
    python tools/security_probe/probe_local.py --out-json findings_local.json

## Severity

| level    | значение                                                    |
|----------|-------------------------------------------------------------|
| critical | прямой обход auth, RCE, leak секретов                       |
| high     | ослабленная политика (CORS-* + creds, CSRF-vector)          |
| medium   | отсутствие defence-in-depth (CSP, X-Frame-Options)          |
| low      | timing-leak, тривиальные info-disclosure                    |
| info     | наблюдение, не тянет на уязвимость                          |

## Findings

См. [`findings.json`](findings.json) — последний прогон. Сводка
актуальных находок и план реакции — в корневом
[`SECURITY.md`](../../SECURITY.md).
