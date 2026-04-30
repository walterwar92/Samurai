# Security policy — Samurai robot

## Reporting

Найдена уязвимость, которая позволяет управлять роботом без авторизации,
читать чужие токены, удалённо выполнять код или вызвать DoS чужому
оператору? Сообщите **в приватном канале**, не открывая публичный issue:

- email: `walterwarda@gmail.com`
- ожидайте подтверждение в течение 7 дней

Открытое раскрытие — после фикса и согласования сроков.

## Что входит в scope

| Компонент                                         | В scope | Комментарий                                         |
|---------------------------------------------------|---------|-----------------------------------------------------|
| `compute_node/dashboard/` (FastAPI :5000)         | да      | основное API робота                                 |
| `compute_node/samcan_bridge.py` (:5005)           | да      | USB-Serial мост для Samcan                          |
| `pi_nodes/` (MQTT-ноды)                           | да      | в первую очередь auth и path/voice валидация        |
| `firmware/` (Arduino, ESP32)                      | да      | сериал-протокол, EEPROM                             |
| `tools/security_probe/`                           | да      | сами пробы тоже могут содержать баги                |
| MOIS robot-gateway (Supabase Edge Function)       | частично | внешний компонент; см. ниже                        |
| `compute_node/frontend/node_modules/**`           | нет     | сторонние deps; зовите `npm audit`                  |

## Out of scope

- DoS через одиночный сетевой канал (1 Гбит/с не наш уровень защиты).
- Атаки требующие физический доступ к Pi (USB, SD-карта, GPIO) — для
  лабораторного робота это норма.
- "Самокатание" робота когда пользователь оставил `cmd_vel` опубликованным
  без auth-токена в открытой сети — см. далее как защититься.

## Защитные средства, которые есть

| Слой           | Что включено                                                                     |
|----------------|----------------------------------------------------------------------------------|
| MQTT           | username/password (`samurai auth init`) — улучшение #10                          |
| Dashboard auth | Opt-in Bearer на `/api/v1/*` через `SAMURAI_DASHBOARD_TOKEN`                     |
| Headers        | CSP, X-Frame-Options, X-Content-Type-Options, Referrer-Policy                    |
| HSTS           | Opt-in (`SAMURAI_HSTS=on`) — включай только за HTTPS-прокси                      |
| Rate limit     | 30/min на write-verbs `/api/*` через slowapi (#58)                               |
| Idempotency    | `Idempotency-Key` на POST/PUT/PATCH/DELETE кешируется 60 секунд (#59)            |
| Voice          | sanitize input — drop control bytes/escape (#18)                                 |
| Path recorder  | regex `^[A-Za-z0-9_-]{1,64}$` + realpath check (#17)                             |
| Watchdog       | edge-triggered alarm на пропавший camera/range (#19)                             |

## Включить защиту dashboard "наружу"

```bash
# 1. Сгенерировать токен
TOKEN=$(python -c "import secrets; print(secrets.token_urlsafe(32))")

# 2. Положить в env (или systemd unit Environment=)
export SAMURAI_DASHBOARD_TOKEN=$TOKEN
export SAMURAI_HSTS=on   # только если фронтит HTTPS-прокси

# 3. Перезапустить
./samurai.sh stop compute && ./samurai.sh compute

# 4. Проверить
python tools/security_probe/probe_local.py --token "$TOKEN"
```

После этого:

- любой запрос на `/api/v1/*` без `Authorization: Bearer $TOKEN` → 401
- сравнение токенов через `hmac.compare_digest` (constant-time)
- `Bearer  <token>` с двойным пробелом → 401 (regression-test зашит)
- `apikey: <token>` или `?token=<...>` → 401 (alt-channel-leak закрыт)
- preflight `OPTIONS` exempt — CORS-клиенты не ломаются

## Известные findings

### MOIS robot-gateway (внешний, не наш код)

Прогон `python tools/security_probe/probe.py` зафиксирован в
[`tools/security_probe/findings.json`](tools/security_probe/findings.json).

| Severity | Finding                  | Status                                                              |
|----------|--------------------------|---------------------------------------------------------------------|
| CRITICAL | `auth_bypass`            | `Bearer  <token>` с двойным пробелом возвращает 200. Сообщено куратору MOIS. На нашей стороне закрыт regression-тестом для собственного `bearer_auth` middleware. |
| MEDIUM   | `missing_security_headers` | MOIS Edge Function не отдаёт CSP / X-Frame-Options. На наш `/api/*` это не влияет — `compute_node/dashboard/app.py` теперь отдаёт всё это сам. |
| LOW      | `timing_leak_suspect`    | 68 мс разброса между токенами разной длины на 401. Скорее всего шум cloudflare → supabase, не реальный leak. Перепроверить через VPN ближе к региону. |

### Самурай-стек (локальный)

Прогон `python tools/security_probe/probe_local.py` — пусто, при условии
что dashboard запущен с включёнными middleware из этой ветки.

## Что НЕ делает probe

- DoS / нагрузочный спам
- брут-форс токенов
- команды, которые реально подвинут робота (`cmd_vel`, `claw`, `laser`)
- XSS-маркеры без явного `--xss` флага
- write-операции на пресеты Samcan / сценарии

Эти ограничения зафиксированы в [`tools/security_probe/README.md`](tools/security_probe/README.md)
и в начале каждого probe-скрипта.
