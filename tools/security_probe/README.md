# security_probe

Read-only / non-destructive проверки для API Samurai-стека.

## Цели

- **`mois`** — внешний robot-gateway (Supabase Edge Function), куда Pi
  пушит телеметрию и забирает команды. Запускается под валидным
  студенческим токеном.
- **`local`** — локальный dashboard (FastAPI :5000) и Samcan-bridge
  (FastAPI :5005). Бьёт по `127.0.0.1`, проверяет защиту собственного
  API робота.

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
    python tools/security_probe/probe.py --target local

    # с включённым SAMURAI_DASHBOARD_TOKEN
    export SAMURAI_DASHBOARD_TOKEN=$(cat ~/.samurai/dashboard.token)
    python tools/security_probe/probe.py --target local --token "$SAMURAI_DASHBOARD_TOKEN"

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
