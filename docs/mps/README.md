# МПС — модуль

Рабочая ветка для разработки модуля МПС.

**Owners:** [@OneAstr0](https://github.com/OneAstr0), [@razdryzg-dev](https://github.com/razdryzg-dev)

**Базируется на:** `main` (включая security-hardening — bearer auth,
security headers, rate-limit, idempotency).

## Скоуп

Здесь описывается, что входит в модуль МПС, какие у него интерфейсы
с остальной системой Samurai, и какие требования к разработке. Этот
документ — точка входа для тех, кто будет смотреть PR.

> TODO(@OneAstr0, @razdryzg-dev): дополнить раздел по факту первого
> работающего скетча.

## Интерфейсы с роботом

- MQTT topic prefix: `samurai/{robot_id}/mps/...` — выбран
  чтобы попадать под единую систему MQTT-auth (#10).
- REST: при необходимости — `/api/v1/mps/*` на dashboard FastAPI.
  Тогда новый router в `compute_node/dashboard/routers/`.
- Конфиг: секция `mps:` в `config.yaml`.

## Локальный запуск

    git checkout feat/mps
    git pull
    # дальше — по специфике модуля, см. issues и TODO.md

## Workflow

Стандартный для проекта: ветка → коммиты → PR в `dev` → ревью →
merge в `dev`. После накопления изменений — `dev` → `main` через PR.

См. корневой [`SECURITY.md`](../../SECURITY.md) для disclosure
policy и [`CLAUDE.md`](../../CLAUDE.md) (если есть) для общих
конвенций проекта.
